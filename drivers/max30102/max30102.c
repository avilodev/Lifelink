/**
 * max30102.c - MAX30102 pulse-oximeter / heart-rate sensor driver
 */

#include "max30102/max30102.h"
#include "../../app/lifelink/vitals_math.h"
#include "config.h"

#include <stdio.h>
#include <string.h> 
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* Register map. */
#define REG_INTR_STATUS_1   0x00u
#define REG_INTR_STATUS_2   0x01u
#define REG_INTR_ENABLE_1   0x02u
#define REG_INTR_ENABLE_2   0x03u
#define REG_FIFO_WR_PTR     0x04u
#define REG_OVF_COUNTER     0x05u
#define REG_FIFO_RD_PTR     0x06u
#define REG_FIFO_DATA       0x07u
#define REG_FIFO_CONFIG     0x08u
#define REG_MODE_CONFIG     0x09u
#define REG_SPO2_CONFIG     0x0Au
#define REG_LED1_PA         0x0Cu   /* Red LED current */
#define REG_LED2_PA         0x0Du   /* IR  LED current */
#define REG_TEMP_INT        0x1Fu
#define REG_TEMP_FRAC       0x20u
#define REG_TEMP_CONFIG     0x21u
#define REG_REV_ID          0xFEu
#define REG_PART_ID         0xFFu

#define EXPECTED_PART_ID    0x15u
#define I2C_TIMEOUT_US      10000
#define INTR_A_FULL         0x80u
#define INTR_PPG_RDY        0x40u

/* 18-bit ADC full-scale; used to size the outlier-rejection window below. */
#define FIFO_SAMPLE_MAX      0x3FFFFu

/* A single glitched I2C read (e.g. from a marginal/flexing wire connection)
 * can hand back a sample tens of thousands of counts away from its
 * neighbors even though the I2C transaction itself reported success - the
 * bus check in reg_read() only confirms the right number of bytes came
 * back, not that they're sane. One such sample sitting in the rolling
 * window can distort ac_pp/min/max for the full window duration.
 * OUTLIER_JUMP_FRAC rejects a sample that jumps further than this fraction
 * of full-scale from the immediately preceding accepted sample, on the
 * assumption that a real pulse signal changes gradually sample-to-sample
 * at this sample rate, while a bus glitch does not. Tune this if genuine
 * fast physiological changes start getting dropped. */
#define OUTLIER_JUMP_FRAC    0.20f

/* Low-level I2C helpers. */

static bool reg_write(i2c_inst_t *i2c, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    int n = i2c_write_timeout_us(i2c, MAX30102_I2C_ADDR, buf, 2, false, I2C_TIMEOUT_US);
    return n == 2;
}

static bool reg_read(i2c_inst_t *i2c, uint8_t reg, uint8_t *out, size_t len) {
    int n = i2c_write_timeout_us(i2c, MAX30102_I2C_ADDR, &reg, 1, true, I2C_TIMEOUT_US);
    if (n != 1) return false;
    n = i2c_read_timeout_us(i2c, MAX30102_I2C_ADDR, out, len, false, I2C_TIMEOUT_US);
    return n == (int)len;
}

/* Outlier-rejection state for the FIFO stream. Reset whenever the sensor
 * (re)initializes so a stale reference from before a disconnect/reconnect
 * can't reject every sample after the sensor comes back online. */
static bool     s_have_last = false;
static uint32_t s_last_ir   = 0;
static uint32_t s_last_red  = 0;

static bool sample_is_outlier(uint32_t last, uint32_t now) {
    uint32_t diff = (now > last) ? (now - last) : (last - now);
    return (float)diff > OUTLIER_JUMP_FRAC * (float)FIFO_SAMPLE_MAX;
}

/* Public API. */

bool max30102_init(i2c_inst_t *i2c) {
    uint8_t part_id = 0;
    if (!reg_read(i2c, REG_PART_ID, &part_id, 1)) {
        printf("[HW]  MAX30102: no I2C ACK at 0x%02X; check wiring, 3V3 power "
               "and SDA/SCL pull-ups\n", MAX30102_I2C_ADDR);
        return false;
    }
    if (part_id != EXPECTED_PART_ID) {
        printf("[CFG] MAX30102: PART_ID 0x%02X, expected 0x%02X; wrong chip at "
               "0x%02X or an address conflict\n",
               part_id, EXPECTED_PART_ID, MAX30102_I2C_ADDR);
        return false;
    }

    /* Soft reset; the chip clears RESET when it is ready. */
    if (!reg_write(i2c, REG_MODE_CONFIG, 0x40u)) goto bus_fault;
    for (int i = 0; i < 100; i++) {
        uint8_t mode = 0;
        if (!reg_read(i2c, REG_MODE_CONFIG, &mode, 1)) goto bus_fault;
        if ((mode & 0x40u) == 0) break;
        sleep_ms(1);
    }

    if (!max30102_clear_interrupts(i2c)) goto bus_fault;
    if (!reg_write(i2c, REG_INTR_ENABLE_1, 0x00u)) goto bus_fault;
    if (!reg_write(i2c, REG_INTR_ENABLE_2, 0x00u)) goto bus_fault;

    /* Reset FIFO pointers before enabling SpO2 sampling. */
    if (!reg_write(i2c, REG_FIFO_WR_PTR, 0x00u)) goto bus_fault;
    if (!reg_write(i2c, REG_OVF_COUNTER, 0x00u)) goto bus_fault;
    if (!reg_write(i2c, REG_FIFO_RD_PTR, 0x00u)) goto bus_fault;

    /* FIFO config:
     *      SMP_AVE = 4 (b010)        - average 4 samples
     *      FIFO_ROLLOVER_EN = 1
     *      FIFO_A_FULL = 0xF (15)    - assert INT when about 17 samples are queued
     *    => 0b010_1_1111 = 0x5F */
    if (!reg_write(i2c, REG_FIFO_CONFIG, 0x5Fu)) goto bus_fault;

    /* Mode = SpO2 (Red + IR). */
    if (!reg_write(i2c, REG_MODE_CONFIG, 0x03u)) goto bus_fault;

    /* SpO2 config:
     *      SPO2_ADC_RGE = b10 (4096 nA)
     *      SPO2_SR      = b001 (100 Hz)
     *      LED_PW       = b11  (411 µs, 18-bit)
     *    => 0b0_10_001_11 = 0x47 */
    if (!reg_write(i2c, REG_SPO2_CONFIG, 0x47u)) goto bus_fault;

    /* LED currents are intentionally conservative for continuous wear. */
    if (!reg_write(i2c, REG_LED1_PA, 0x24u)) goto bus_fault;   /* Red */
    if (!reg_write(i2c, REG_LED2_PA, 0x24u)) goto bus_fault;   /* IR  */

    if (!max30102_clear_interrupts(i2c)) goto bus_fault;
    if (!reg_write(i2c, REG_INTR_ENABLE_1, INTR_A_FULL | INTR_PPG_RDY)) goto bus_fault;
    if (!reg_write(i2c, REG_INTR_ENABLE_2, 0x00u)) goto bus_fault;

    /* Fresh reference for outlier rejection - avoids comparing the first
     * post-(re)init sample against a stale value from before a disconnect. */
    s_have_last = false;

    printf("[OK]  MAX30102: connected at 0x%02X (PART_ID 0x%02X), SpO2 mode\n",
           MAX30102_I2C_ADDR, part_id);
    return true;

bus_fault:
    printf("[HW]  MAX30102: identified but configuration write failed\n");
    return false;
}

bool max30102_read_fifo_checked(i2c_inst_t        *i2c,
                                max30102_sample_t *out,
                                uint32_t           max_samps,
                                uint32_t          *samples_read)
{
    if (samples_read) *samples_read = 0;

    uint8_t wr = 0, rd = 0;
    if (!reg_read(i2c, REG_FIFO_WR_PTR, &wr, 1)) return false;
    if (!reg_read(i2c, REG_FIFO_RD_PTR, &rd, 1)) return false;

    /* Pointers are 5-bit; FIFO depth is 32 entries. */
    int avail = (int)wr - (int)rd;
    if (avail < 0) avail += 32;
    if (avail == 0) return true;
    if ((uint32_t)avail > max_samps) avail = (int)max_samps;

    /* Each FIFO entry in SpO2 mode is 6 bytes: 3 Red + 3 IR. */
    uint32_t count = 0;
    for (int i = 0; i < avail; i++) {
        uint8_t buf[6];
        if (!reg_read(i2c, REG_FIFO_DATA, buf, 6)) return false;
        uint32_t red = (((uint32_t)buf[0] << 16) |
                        ((uint32_t)buf[1] << 8)  |
                         (uint32_t)buf[2]) & 0x3FFFFu;
        uint32_t ir  = (((uint32_t)buf[3] << 16) |
                        ((uint32_t)buf[4] << 8)  |
                         (uint32_t)buf[5]) & 0x3FFFFu;

        /* Drop this sample rather than let a single glitched I2C read
         * (bad byte from a marginal wire connection, still ACKed fine)
         * poison the rolling HR/SpO2 window for the next several seconds.
         * The very first sample after (re)init has no reference yet, so
         * it's always accepted. */
        if (s_have_last &&
            (sample_is_outlier(s_last_ir, ir) || sample_is_outlier(s_last_red, red))) {
            continue;
        }

        s_last_ir  = ir;
        s_last_red = red;
        s_have_last = true;

        out[count].red = red;
        out[count].ir  = ir;
        count++;
    }
    if (samples_read) *samples_read = count;
    return true;
}

uint32_t max30102_read_fifo(i2c_inst_t        *i2c,
                            max30102_sample_t *out,
                            uint32_t           max_samps)
{
    uint32_t samples_read = 0;
    if (!max30102_read_fifo_checked(i2c, out, max_samps, &samples_read)) return 0;
    return samples_read;
}

bool max30102_clear_interrupts(i2c_inst_t *i2c) {
    uint8_t status[2];
    return reg_read(i2c, REG_INTR_STATUS_1, status, sizeof status);
}

bool max30102_read_die_temp(i2c_inst_t *i2c, float *celsius) {
    /* Trigger a one-shot temperature conversion. */
    if (!reg_write(i2c, REG_TEMP_CONFIG, 0x01u)) return false;
    sleep_ms(30);

    uint8_t int_part = 0, frac_part = 0;
    if (!reg_read(i2c, REG_TEMP_INT,  &int_part,  1)) return false;
    if (!reg_read(i2c, REG_TEMP_FRAC, &frac_part, 1)) return false;

    *celsius = (float)((int8_t)int_part) + (float)(frac_part & 0x0Fu) * 0.0625f;
    return true;
}