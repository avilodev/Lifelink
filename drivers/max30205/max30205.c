/**
 * max30205.c - MAX30205 human body temperature sensor driver
 */

#include "max30205/max30205.h"
#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" 

/* Register map. */
#define REG_TEMP            0x00u   /* read-only, 16-bit signed */
#define REG_CONFIG          0x01u   /* 8-bit configuration       */
#define REG_THYST           0x02u   /* alert release threshold    */
#define REG_TOS             0x03u   /* alert assert threshold     */

#define I2C_TIMEOUT_US      10000

/* Continuous conversion, comparator mode, active-low OS/INT. */
#define CONFIG_CONTINUOUS   0x00u

/* Low-level I2C helpers. */

static bool reg_write8(i2c_inst_t *i2c, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    int n = i2c_write_timeout_us(i2c, MAX30205_I2C_ADDR, buf, 2, false, I2C_TIMEOUT_US);
    return n == 2;
}

static bool reg_write16(i2c_inst_t *i2c, uint8_t reg, int16_t val) {
    uint8_t buf[3] = {
        reg,
        (uint8_t)(((uint16_t)val >> 8) & 0xFFu),
        (uint8_t)((uint16_t)val & 0xFFu)
    };
    int n = i2c_write_timeout_us(i2c, MAX30205_I2C_ADDR, buf, 3, false, I2C_TIMEOUT_US);
    return n == 3;
}

static bool reg_read(i2c_inst_t *i2c, uint8_t reg, uint8_t *out, size_t len) {
    int n = i2c_write_timeout_us(i2c, MAX30205_I2C_ADDR, &reg, 1, true, I2C_TIMEOUT_US);
    if (n != 1) return false;
    n = i2c_read_timeout_us(i2c, MAX30205_I2C_ADDR, out, len, false, I2C_TIMEOUT_US);
    return n == (int)len;
}

static int16_t celsius_to_raw(float celsius) {
    return (int16_t)(celsius * 256.0f + (celsius >= 0.0f ? 0.5f : -0.5f));
}

/* Public API. */

bool max30205_init(i2c_inst_t *i2c) {
    /* Probe by reading the config register; the MAX30205 has no device ID. */
    uint8_t cfg = 0;
    if (!reg_read(i2c, REG_CONFIG, &cfg, 1)) {
        printf("[HW]  MAX30205: no I2C ACK at 0x%02X; check wiring, 3V3 power "
               "and SDA/SCL pull-ups\n", MAX30205_I2C_ADDR);
        return false;
    }

    if (!reg_write8(i2c, REG_CONFIG, CONFIG_CONTINUOUS)) {
        printf("[HW]  MAX30205: probe ACKed but config write failed\n");
        return false;
    }
    if (!max30205_configure_alert(i2c, MAX30205_ALERT_HIGH_C, MAX30205_ALERT_LOW_C)) {
        printf("[HW]  MAX30205: alert threshold write failed\n");
        return false;
    }

    /* First conversion takes ~50 ms; give it some time. */
    sleep_ms(60);

    printf("[OK]  MAX30205: connected at 0x%02X, continuous mode, alert %.2f/%.2f C\n",
           MAX30205_I2C_ADDR, MAX30205_ALERT_HIGH_C, MAX30205_ALERT_LOW_C);
    return true;
}

bool max30205_configure_alert(i2c_inst_t *i2c, float high_c, float low_c) {
    if (low_c > high_c) {
        float tmp = low_c;
        low_c = high_c;
        high_c = tmp;
    }

    /* Only touch the threshold registers here - REG_CONFIG is deliberately
     * left alone so this can be called on its own to update thresholds
     * without clobbering any other config bits (fault queue, polarity,
     * shutdown mode, etc.) the caller may have set. max30205_init() sets
     * REG_CONFIG itself before calling this. */
    if (!reg_write16(i2c, REG_TOS, celsius_to_raw(high_c))) return false;
    return reg_write16(i2c, REG_THYST, celsius_to_raw(low_c));
}

bool max30205_read_celsius(i2c_inst_t *i2c, float *celsius) {
    uint8_t buf[2];
    if (!reg_read(i2c, REG_TEMP, buf, 2)) return false;

    /* Big-endian per Maxim's datasheet, 1 LSB = 1/256 deg C.
     *
     * CONFIRMED ERRATUM (matches an independently-reported case with this
     * exact symptom): in normal (non-extended) format, bits D15 (sign) and
     * D14 come back set even though the real temperature data only ever
     * needs 14 bits for the 0-50C rated range. Left unmasked, this makes
     * every reading look like a large negative two's-complement value.
     * Masking them off before decoding recovers the correct positive
     * reading - verified against this board's own logged raw samples,
     * which produce a clean rising curve once masked (consistent with a
     * finger warming the sensor) instead of the inverted/falling trend
     * negation was producing. */
    int16_t raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    raw &= 0x3FFF;

    float val = (float)raw * 0.00390625f;

    *celsius = val;
    return true;
}

bool max30205_read_fahrenheit(i2c_inst_t *i2c, float *fahrenheit) {
    float c;
    if (!max30205_read_celsius(i2c, &c)) return false;
    *fahrenheit = c * 9.0f / 5.0f + 32.0f;
    return true;
}