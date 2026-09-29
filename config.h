/**
 * config.h - Central project configuration
 *
 * Edit this file to match your hardware wiring.
 *
 *   MAX30102  (heart rate / SpO2)   I2C0  SDA=GP4  SCL=GP5   (dedicated bus)
 *   MAX30205  (body temperature)    I2C1  SDA=GP6  SCL=GP7   (dedicated bus)
 *   HM-10     (BLE module)          UART0 TX =GP0  RX =GP1
 *
 * Each sensor owns a separate hardware I2C block, so they share no SDA/SCL
 * pins. The RP2040 has exactly two I2C controllers (i2c0, i2c1) - one per
 * sensor - which also isolates a stuck/held bus on one sensor from the other.
 */
#pragma once

/* MAX30102 heart-rate / SpO2 sensor - dedicated I2C0 bus. */
#define MAX30102_I2C            i2c0
#define MAX30102_I2C_SDA_PIN    4u      /* GP4, I2C0 SDA */
#define MAX30102_I2C_SCL_PIN    5u      /* GP5, I2C0 SCL */
#define MAX30102_I2C_HZ         100000u /* 100 kHz */
#define MAX30102_I2C_ADDR       0x57u
#define MAX30102_INT_PIN        2u      /* GP2, active-low open-drain INT */
/* HR/SpO2 sample window: 100 FIFO samples is about 4 s after SMP_AVE=4. */
#define MAX30102_SAMPLE_WIN     100u

/* MAX30205 body-temperature sensor - dedicated I2C1 bus. */
#define MAX30205_I2C            i2c1
#define MAX30205_I2C_SDA_PIN    6u      /* GP6, I2C1 SDA */
#define MAX30205_I2C_SCL_PIN    7u      /* GP7, I2C1 SCL */
#define MAX30205_I2C_HZ         100000u /* 100 kHz */
#define MAX30205_I2C_ADDR       0x48u   /* A0=A1=A2=GND default */
#define MAX30205_INT_PIN        3u      /* GP3, active-low OS/INT comparator */
#define MAX30205_ALERT_HIGH_C   38.00f
#define MAX30205_ALERT_LOW_C    37.50f

/* HM-10 BLE UART. */
#define HM10_UART               uart0
#define HM10_UART_TX_PIN        0u      /* GP0, UART0 TX to HM-10 RX */
#define HM10_UART_RX_PIN        1u      /* GP1, UART0 RX from HM-10 TX */
#define HM10_BAUD               9600u   /* HM-10 factory default */

/* BLE pairing PIN (6 digits), applied at boot via AT+PASS + AT+TYPE2
   (auth+bond). Inject the real PIN at build time so it never lives in tracked
   source:
       cmake -DHM10_PAIR_PIN=481920 ..
   (CMakeLists forwards that to a -DHM10_PAIR_PIN="481920" compile define).
   The fallback below is an OBVIOUS placeholder - shipping it leaves the vitals
   stream readable by anyone who knows it, so always override it. */
#ifndef HM10_PAIR_PIN
#define HM10_PAIR_PIN           "000000" /* placeholder - override at build time */
#endif

/* Application. */
#define READ_INTERVAL_MS        300u   /* publish reading every 300 ms */
#define IDLE_WAKE_INTERVAL_MS   100u    /* periodic wake for timers/watchdog */
 