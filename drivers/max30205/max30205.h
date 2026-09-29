/**
 * max30205.h - MAX30205 human body temperature sensor driver
 *
 * Wiring (see config.h for pins):
 *   MAX30205 VCC -> Pico 3.3 V
 *   MAX30205 GND -> Pico GND
 *   MAX30205 SDA -> Pico MAX30205_I2C_SDA_PIN
 *   MAX30205 SCL -> Pico MAX30205_I2C_SCL_PIN
 *
 * Default I2C address (A0=A1=A2=GND) is 0x48. Change MAX30205_I2C_ADDR
 * in config.h if your breakout has the address pins strapped differently. 
 */
#pragma once 

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

/**
 * Probe for the device on the bus and put it in continuous-conversion mode.
 *
 * @param i2c   I2C peripheral instance (already initialised by caller)
 * @return true if the device acknowledges
 */
bool max30205_init(i2c_inst_t *i2c);

/**
 * Configure the OS/INT pin as an active-low over-temperature comparator.
 *
 * The output asserts at high_c and releases after temperature falls below
 * low_c. This is an alert source; regular temperature reads are still polled.
 */
bool max30205_configure_alert(i2c_inst_t *i2c, float high_c, float low_c);

/**
 * Read the current temperature in degrees Celsius.
 * Resolution is 0.00390625 deg C/LSB (16-bit signed, two's complement).
 *
 * @return true on success, false on bus error
 */
bool max30205_read_celsius(i2c_inst_t *i2c, float *celsius);

/** Convenience wrapper: same reading converted to Fahrenheit. */
bool max30205_read_fahrenheit(i2c_inst_t *i2c, float *fahrenheit);
