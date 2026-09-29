/**
 * max30102.h - MAX30102 pulse-oximeter / heart-rate sensor driver
 *
 * Wiring (see config.h for pins):
 *   MAX30102 VCC -> Pico 3.3 V (or 1.8 V if your breakout has a regulator)
 *   MAX30102 GND -> Pico GND
 *   MAX30102 SDA -> Pico MAX30102_I2C_SDA_PIN
 *   MAX30102 SCL -> Pico MAX30102_I2C_SCL_PIN
 *
 * I2C address is fixed at 0x57.  The driver leaves the I2C peripheral
 * configuration up to the application (so it can be shared with other
 * sensors on the same bus).
 */ 
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/* Raw 18-bit photodiode sample pair (one per FIFO entry). */
typedef struct {
    uint32_t red;   /* 18-bit, masked to 0x3FFFF */
    uint32_t ir;    /* 18-bit, masked to 0x3FFFF */
} max30102_sample_t;

/**
 * Probe for the device on the bus and configure it for SpO2 mode
 * (Red + IR LEDs, 100 Hz sample rate, 18-bit ADC, 4096 nA range).
 *
 * @param i2c   I2C peripheral instance (already initialised by caller)
 * @return true if PART_ID matches and configuration succeeded
 */
bool max30102_init(i2c_inst_t *i2c);

/**
 * Drain all pending FIFO samples into the caller's buffer.
 *
 * @param i2c        I2C peripheral instance
 * @param out        Buffer to receive samples
 * @param max_samps  Maximum samples to read (size of out[])
 * @return number of samples actually read (0 if none available)
 */
uint32_t max30102_read_fifo(i2c_inst_t        *i2c,
                            max30102_sample_t *out,
                            uint32_t           max_samps);

/**
 * Drain FIFO samples and report I2C failures separately from an empty FIFO.
 */
bool max30102_read_fifo_checked(i2c_inst_t        *i2c,
                                max30102_sample_t *out,
                                uint32_t           max_samps,
                                uint32_t          *samples_read);

/**
 * Clear latched interrupt status bits. The MAX30102 clears status on read.
 */
bool max30102_clear_interrupts(i2c_inst_t *i2c);

/**
 * Read the on-chip die temperature (deg C).  Used as a sanity check;
 * it is NOT a body-temperature sensor.  Use the MAX30205 for that.
 */
bool max30102_read_die_temp(i2c_inst_t *i2c, float *celsius);
