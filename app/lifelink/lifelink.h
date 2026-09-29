/**
 * lifelink.h - Top-level application loop
 *
 * Owns hardware initialisation, sensor sampling, derived metrics
 * (HR + SpO2), and the publish path: USB serial print + HM-10 send.
 */
#pragma once

/** One-shot init: stdio, I2C bus, UART, and all attached sensors. */
void lifelink_init(void);
 
/** Main sample-and-publish loop.  Never returns. */
void lifelink_loop(void);
 
