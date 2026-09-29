/**
 * lifelink.c - Top-level application loop
 *
 *   1. Initialise USB serial, both sensor I2C buses, and the HM-10 UART
 *   2. Drain the MAX30102 FIFO into a sliding sample window on sensor IRQ
 *   3. On every READ_INTERVAL_MS tick:
 *        a. compute HR (peak detection on IR), smoothed across publishes
 *        b. compute SpO2 (Red/IR AC-DC ratio)
 *        c. read body temperature from the MAX30205
 *        d. print one line to USB serial AND send the same line to HM-10
 *
 * Crash safety:
 *   - Every I2C transaction has a 10 ms timeout; a missing or unplugged
 *     sensor never blocks the loop.
 *   - A sensor that goes silent or stops ACKing is marked offline; we
 *     keep retrying it once every 5 seconds so hot-replug is automatic.
 *   - The hardware watchdog reboots the chip if the loop stalls for
 *     more than 8 seconds.
 *   - The published line always tells the user which sensor is offline.
 */

#include "lifelink/lifelink.h"
#include "lifelink/vitals_math.h"
#include "config.h"

#include "max30102/max30102.h"
#include "max30205/max30205.h"
#include "hm10/hm10.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

/* pico/stdio_usb.h is not exported through this library's include path. */
extern bool stdio_usb_connected(void);

/* Sample buffer. */

#define SAMPLE_RATE_HZ   25.0f
#define FINGER_DC_THRESH 50000u
#define WD_TIMEOUT_MS    8000u
#define RESENSE_MS       5000u                   /* retry dead sensor every 5 s */
#define SILENCE_MS       3000u                   /* MAX30102 dead if no samps  */
#define MAX102_BACKSTOP_MS 500u                  /* recover from a missed edge */

static vitals_window_t window;                   /* IR/Red ring buffer + HR/SpO2 math */

static bool max102_ok = false;
static bool max205_ok = false;
static bool hm10_secured = false;                /* BLE pairing configured at boot */
static volatile bool max102_irq_pending = false;
static volatile bool max205_alert_pending = false;

/* HR smoothing across publishes - each published HR is otherwise an
 * independent ~4s window with no memory between them, so a single noisy
 * window can swing wildly from the one before/after it. This keeps a
 * small history of recent valid (finger-on, non-zero) HR readings and
 * publishes their median instead of the raw single-window value. */
#define HR_SMOOTH_LEN   6

static int      hr_smooth_buf[HR_SMOOTH_LEN];
static int      hr_smooth_n   = 0;
static uint32_t hr_smooth_pos = 0;

static void hr_smooth_reset(void) {
    hr_smooth_n   = 0;
    hr_smooth_pos = 0;
}

static void hr_smooth_push(int bpm) {
    hr_smooth_buf[hr_smooth_pos] = bpm;
    hr_smooth_pos = (hr_smooth_pos + 1) % HR_SMOOTH_LEN;
    if (hr_smooth_n < HR_SMOOTH_LEN) hr_smooth_n++;
}

static int hr_smooth_median(void) {
    int tmp[HR_SMOOTH_LEN];
    for (int i = 0; i < hr_smooth_n; i++) tmp[i] = hr_smooth_buf[i];
    for (int i = 1; i < hr_smooth_n; i++) {
        int key = tmp[i], j = i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
        tmp[j + 1] = key;
    }
    return tmp[hr_smooth_n / 2];
}

/* Hardware bring-up. */

static void i2c_bus_init(void) {
    /* MAX30102 on its own I2C0 bus. */
    i2c_init(MAX30102_I2C, MAX30102_I2C_HZ);
    gpio_set_function(MAX30102_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MAX30102_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MAX30102_I2C_SDA_PIN);
    gpio_pull_up(MAX30102_I2C_SCL_PIN);

    /* MAX30205 on its own I2C1 bus (no shared SDA/SCL with the MAX30102). */
    i2c_init(MAX30205_I2C, MAX30205_I2C_HZ);
    gpio_set_function(MAX30205_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MAX30205_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MAX30205_I2C_SDA_PIN);
    gpio_pull_up(MAX30205_I2C_SCL_PIN);
}

static void sensor_gpio_irq(uint gpio, uint32_t events) {
    if ((events & GPIO_IRQ_EDGE_FALL) == 0) return;
    if (gpio == MAX30102_INT_PIN) {
        max102_irq_pending = true;
    } else if (gpio == MAX30205_INT_PIN) {
        max205_alert_pending = true;
    }
}

static void sensor_irq_init(void) {
    gpio_init(MAX30102_INT_PIN);
    gpio_set_dir(MAX30102_INT_PIN, GPIO_IN);
    gpio_pull_up(MAX30102_INT_PIN);

    gpio_init(MAX30205_INT_PIN);
    gpio_set_dir(MAX30205_INT_PIN, GPIO_IN);
    gpio_pull_up(MAX30205_INT_PIN);

    gpio_set_irq_enabled_with_callback(MAX30102_INT_PIN,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &sensor_gpio_irq);
    gpio_set_irq_enabled(MAX30205_INT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

static void print_banner(void) {
    printf("\n=== LifeLink ===\n");
    printf("UART0  TX=GP%u  RX=GP%u  @ %u baud (HM-10)\n",
           HM10_UART_TX_PIN, HM10_UART_RX_PIN, HM10_BAUD);
    printf("HM-10 pairing: %s\n",
           hm10_secured ? "OK (PIN required)" : "not configured");
    printf("MAX30102 (0x%02X, I2C0 SDA=GP%u SCL=GP%u @ %u Hz, INT=GP%u): %s\n",
           MAX30102_I2C_ADDR, MAX30102_I2C_SDA_PIN, MAX30102_I2C_SCL_PIN,
           MAX30102_I2C_HZ, MAX30102_INT_PIN, max102_ok ? "OK" : "OFFLINE");
    printf("MAX30205 (0x%02X, I2C1 SDA=GP%u SCL=GP%u @ %u Hz, OS/INT=GP%u): %s\n",
           MAX30205_I2C_ADDR, MAX30205_I2C_SDA_PIN, MAX30205_I2C_SCL_PIN,
           MAX30205_I2C_HZ, MAX30205_INT_PIN, max205_ok ? "OK" : "OFFLINE");
    printf("Publishing every %u ms\n", READ_INTERVAL_MS);
    if (watchdog_caused_reboot()) {
        printf("(previous run ended in a watchdog reset)\n");
    }
}

void lifelink_init(void) {
    stdio_init_all();           /* USB CDC serial */

    /* Wait briefly for a host so the boot banner is visible. */
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++) {
        sleep_ms(100);
    }

    i2c_bus_init();
    hm10_init();
    /* Configure BLE pairing once, while the module is not yet connected to a
       central. Bounded/non-blocking; boot continues even if it fails. This is
       the only place it runs - never from lifelink_loop(). */
    hm10_secured = hm10_configure_pairing();
    sensor_irq_init();

    max102_ok = max30102_init(MAX30102_I2C);
    max205_ok = max30205_init(MAX30205_I2C);
    max102_irq_pending = max102_ok;
    max205_alert_pending = max205_ok && !gpio_get(MAX30205_INT_PIN);

    print_banner();

    watchdog_enable(WD_TIMEOUT_MS, true);
}

/* Main loop. */

static bool idle_wake_cb(repeating_timer_t *timer) {
    (void)timer;
    return true;
}

static bool due(absolute_time_t deadline) {
    return absolute_time_diff_us(get_absolute_time(), deadline) <= 0;
}

void lifelink_loop(void) {
    absolute_time_t next_publish    = make_timeout_time_ms(READ_INTERVAL_MS);
    absolute_time_t next_resensor   = make_timeout_time_ms(RESENSE_MS);
    absolute_time_t last_max102_smp = get_absolute_time();
    absolute_time_t last_max102_try = nil_time;
    bool read_max102_fifo = false;
    bool last_usb = false;

    repeating_timer_t idle_timer;
    add_repeating_timer_ms(-(int64_t)IDLE_WAKE_INTERVAL_MS,
                           idle_wake_cb,
                           NULL,
                           &idle_timer);

    for (;;) {
        watchdog_update();

        bool usb_now = stdio_usb_connected();
        if (usb_now && !last_usb) {
            print_banner();
        }
        last_usb = usb_now;

        if (due(next_resensor)) {
            if (!max102_ok && max30102_init(MAX30102_I2C)) {
                max102_ok = true;
                max102_irq_pending = true;
                last_max102_smp = get_absolute_time();
                printf("MAX30102 came online\n");
            }
            if (!max205_ok && max30205_init(MAX30205_I2C)) {
                max205_ok = true;
                max205_alert_pending = !gpio_get(MAX30205_INT_PIN);
                printf("MAX30205 came online\n");
            }
            next_resensor = make_timeout_time_ms(RESENSE_MS);
        }

        if (max102_ok) {
            bool backstop_due =
                is_nil_time(last_max102_try) ||
                absolute_time_diff_us(last_max102_try,
                                      get_absolute_time()) >=
                    (int64_t)MAX102_BACKSTOP_MS * 1000;

            read_max102_fifo = max102_irq_pending || backstop_due;
            max102_irq_pending = false;

            if (read_max102_fifo) {
                last_max102_try = get_absolute_time();
                if (!max30102_clear_interrupts(MAX30102_I2C)) {
                    printf("MAX30102 interrupt clear failed; marking offline\n");
                    max102_ok = false;
                    vitals_reset(&window);
                    read_max102_fifo = false;
                }
            }
        }

        if (max102_ok && read_max102_fifo) {
            read_max102_fifo = false;
            max30102_sample_t scratch[16];
            uint32_t got = 0;
            bool read_ok = max30102_read_fifo_checked(MAX30102_I2C, scratch, 16, &got);
            if (!read_ok) {
                printf("MAX30102 FIFO read failed; marking offline\n");
                max102_ok = false;
                vitals_reset(&window);
            } else if (got > 0) {
                last_max102_smp = get_absolute_time();
                for (uint32_t i = 0; i < got; i++) {
                    vitals_push(&window, scratch[i].ir, scratch[i].red);
                }
            } else {
                int64_t silent_us =
                    absolute_time_diff_us(last_max102_smp, get_absolute_time());
                if (silent_us > (int64_t)SILENCE_MS * 1000) {
                    printf("MAX30102 silent; marking offline\n");
                    max102_ok = false;
                    vitals_reset(&window);
                }
            }
        }

        if (max205_alert_pending) {
            max205_alert_pending = false;
            if (max205_ok) {
                float alert_c = 0.0f;
                if (max30205_read_celsius(MAX30205_I2C, &alert_c)) {
                    printf("MAX30205 alert asserted at %.2fC\n", alert_c);
                } else {
                    printf("MAX30205 alert read failed; marking offline\n");
                    max205_ok = false;
                }
            }
        }

        if (due(next_publish)) {
            int   hr_bpm    = 0;
            int   spo2_pct  = 0;
            float body_f    = 0.0f;
            bool  finger    = false;
            bool  temp_ok   = false;

            if (max205_ok) {
                temp_ok = max30205_read_fahrenheit(MAX30205_I2C, &body_f);
                if (!temp_ok) {
                    printf("MAX30205 read failed; marking offline\n");
                    max205_ok = false;
                }
            }

            if (max102_ok && window.count >= 10) {
                uint32_t dc_ir = vitals_mean_ir_dc(&window);
                finger = dc_ir > FINGER_DC_THRESH;
                if (finger) {
                    hr_bpm   = vitals_compute_hr_bpm(&window, SAMPLE_RATE_HZ);
                    spo2_pct = vitals_compute_spo2_pct(&window);
                }
            }

            /* Smooth HR across publishes rather than trusting a single
             * ~4s window in isolation. Reset the history whenever the
             * finger comes off so a new session doesn't inherit stale
             * readings from the last one. */
            if (!finger) {
                hr_smooth_reset();
            } else if (hr_bpm > 0) {
                hr_smooth_push(hr_bpm);
                hr_bpm = hr_smooth_median();
            } else if (hr_smooth_n > 0) {
                /* This window's peak detection failed outright, but we
                 * still have recent history - reuse it rather than
                 * publishing "--" for one bad window in an otherwise
                 * good session. */
                hr_bpm = hr_smooth_median();
            }

            char hr_str  [8]  = "--"; 
            char spo2_str[8]  = "--";
            char temp_str[12] = "--";
            if (hr_bpm   > 0)              snprintf(hr_str,   sizeof hr_str,   "%d",      hr_bpm);
            if (spo2_pct > 0)              snprintf(spo2_str, sizeof spo2_str, "%d",      spo2_pct);
            if (max205_ok && temp_ok)      snprintf(temp_str, sizeof temp_str, "%.2fF",   body_f);

            const char *status = "";
            if (!max102_ok && !max205_ok) status = "  [no sensors]";
            else if (!max102_ok)          status = "  [MAX30102 offline]";
            else if (!max205_ok)          status = "  [MAX30205 offline]";
            else if (!finger)             status = "  [no finger]";

            char line[128];
            int  len = snprintf(line, sizeof line,
                                "HR=%s SpO2=%s BodyT=%s%s\n",
                                hr_str, spo2_str, temp_str, status);
            if (len < 0) len = 0;
            if (len > (int)sizeof line - 1) len = sizeof line - 1;

            fputs(line, stdout);
            hm10_write((const uint8_t *)line, (size_t)len);

            next_publish = make_timeout_time_ms(READ_INTERVAL_MS);
        }

        if (!max102_irq_pending && !max205_alert_pending) {
            __wfi();
        }
    }
}