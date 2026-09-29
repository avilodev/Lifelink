/**
 * hm10.h - HM-10 BLE module driver (transparent UART bridge)
 *
 * The HM-10 is a Bluetooth-Low-Energy serial bridge. Anything written
 * to its UART becomes an outgoing BLE notification; anything received
 * over BLE arrives on its UART. No protocol framing is required.
 *
 * Wiring (see config.h for pins):
 *   Pico HM10_UART_TX_PIN -> HM-10 RX
 *   Pico HM10_UART_RX_PIN <- HM-10 TX
 *   Common GND, HM-10 VCC -> 3.3 V
 *
 * Factory baud is 9600. AT commands (for example, "AT+NAMELifeLink") may be
 * sent by simply writing to the UART when the module is NOT connected 
 * to a central.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Initialise the configured UART peripheral and pins.
 * Must be called once at startup.
 */
void hm10_init(void);

/**
 * Configure the module's BLE security once, at boot.
 *
 * Sets the pairing PIN (HM10_PAIR_PIN) via AT+PASS and switches the module
 * from the factory-default open mode (AT+TYPE0) to authenticated + bonded
 * (AT+TYPE2), so a central must enter the PIN before it can read the stream.
 *
 * MUST be called only while the module is NOT connected to a central (AT
 * commands are ignored once connected) - i.e. at boot, right after
 * hm10_init(), before anything else touches the UART. MUST NOT be called
 * again from the main loop: re-issuing AT+PASS/AT+TYPE to a connected module
 * can drop the client or leave the module in a bad state.
 *
 * Every read and write is bounded by a timeout, so an absent, unresponsive,
 * or already-connected module never blocks boot.
 *
 * @return true if the module handshook and accepted the security settings;
 *         false if it did not respond or rejected a command (boot continues
 *         either way).
 */
bool hm10_configure_pairing(void);

/** Send a null-terminated string. */
void hm10_write_str(const char *s);

/** Send raw bytes, dropping the tail if the UART remains full. */
void hm10_write(const uint8_t *data, size_t len);

/** True if at least one byte is waiting in the RX FIFO. */
bool hm10_readable(void);

/**
 * Read up to max_len bytes without blocking. Returns the number of
 * bytes actually read.
 */
size_t hm10_read(uint8_t *out, size_t max_len);
