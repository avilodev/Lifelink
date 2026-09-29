/**
 * hm10.c - HM-10 BLE module driver (transparent UART bridge)
 */

#include "hm10/hm10.h"
#include "config.h"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include <stdio.h>
#include <string.h>

#define HM10_WRITE_TIMEOUT_US 1000

/* Boot-time AT-configuration timeouts. These are used only by
   hm10_configure_pairing() and never by the streaming write path, so raising
   them cannot affect runtime publish behaviour. */
#define HM10_AT_WRITE_TIMEOUT_US 20000  /* full AT command flushes even if FIFO fills */
#define HM10_AT_RESP_TIMEOUT_MS  500    /* wait this long for the first reply byte */
#define HM10_AT_IDLE_GAP_MS      30     /* reply is complete once RX idles this long */
#define HM10_AT_RESET_SETTLE_MS  600    /* module restart window after AT+RESET */

void hm10_init(void) {
    uart_init(HM10_UART, HM10_BAUD);
    gpio_set_function(HM10_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HM10_UART_RX_PIN, GPIO_FUNC_UART);

    /* HM-10 default frame is 8N1. */
    uart_set_format(HM10_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(HM10_UART, true);
}

/* Boot-time AT configuration (module not yet connected to a central). */

/* Send a NUL-terminated AT command (no CR/LF - genuine HM-10 dialect),
   bounded by HM10_AT_WRITE_TIMEOUT_US so a full command flushes. */
static void hm10_at_write(const char *s) {
    absolute_time_t deadline = make_timeout_time_us(HM10_AT_WRITE_TIMEOUT_US);
    for (size_t i = 0; s[i] != '\0'; i++) {
        while (!uart_is_writable(HM10_UART)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                return;
            }
            tight_loop_contents();
        }
        uart_putc_raw(HM10_UART, s[i]);
    }
}

/* Discard any bytes currently waiting in the RX FIFO. */
static void hm10_flush_rx(void) {
    while (uart_is_readable(HM10_UART)) {
        (void)uart_getc(HM10_UART);
    }
}

/* Read one AT reply into out (always NUL-terminated). Waits up to
   HM10_AT_RESP_TIMEOUT_MS for the first byte, then reads until the RX line
   stays idle for HM10_AT_IDLE_GAP_MS or the buffer fills. HM-10 replies carry
   no line terminator, so a gap-based read is used rather than waiting for CRLF. */
static size_t hm10_at_read_response(char *out, size_t cap) {
    if (cap == 0) return 0;
    out[0] = '\0';

    absolute_time_t first_deadline = make_timeout_time_ms(HM10_AT_RESP_TIMEOUT_MS);
    while (!uart_is_readable(HM10_UART)) {
        if (absolute_time_diff_us(get_absolute_time(), first_deadline) <= 0) {
            return 0;
        }
        tight_loop_contents();
    }

    size_t n = 0;
    absolute_time_t idle_deadline = make_timeout_time_ms(HM10_AT_IDLE_GAP_MS);
    while (n < cap - 1) {
        if (uart_is_readable(HM10_UART)) {
            out[n++] = (char)uart_getc(HM10_UART);
            idle_deadline = make_timeout_time_ms(HM10_AT_IDLE_GAP_MS);
        } else if (absolute_time_diff_us(get_absolute_time(), idle_deadline) <= 0) {
            break;
        } else {
            tight_loop_contents();
        }
    }
    out[n] = '\0';
    return n;
}

/* Send cmd and report whether the reply contains token. */
static bool hm10_at_expect(const char *cmd, const char *token,
                           char *resp, size_t cap) {
    hm10_flush_rx();
    hm10_at_write(cmd);
    hm10_at_read_response(resp, cap);
    return strstr(resp, token) != NULL;
}

bool hm10_configure_pairing(void) {
    char resp[64];

    /* Handshake gate: only proceed if a bare AT returns OK. This keeps us from
       blasting AT+PASS/AT+TYPE at a module that is absent, already connected to
       a central, or a clone with a different AT dialect. */
    if (!hm10_at_expect("AT", "OK", resp, sizeof resp)) {
        printf("[HW]  HM-10: no AT response; leaving security unconfigured "
               "(unplugged, already connected, or incompatible clone)\n");
        return false;
    }

    /* Identify the module. Logged so a human can confirm it is a genuine HM-10
       (e.g. \"HMSoft V54x\") before trusting the AT sequence. Never parsed. */
    hm10_flush_rx();
    hm10_at_write("AT+VERS?");
    hm10_at_read_response(resp, sizeof resp);
    printf("[HW]  HM-10 version: %s\n", resp[0] ? resp : "(no version response)");

    /* Pairing PIN. */
    if (!hm10_at_expect("AT+PASS" HM10_PAIR_PIN, "OK", resp, sizeof resp)) {
        printf("[HW]  HM-10: AT+PASS rejected (resp: \"%s\")\n", resp);
        return false;
    }

    /* Authentication + bonding (factory default is AT+TYPE0 = open). */
    if (!hm10_at_expect("AT+TYPE2", "OK", resp, sizeof resp)) {
        printf("[HW]  HM-10: AT+TYPE2 rejected (resp: \"%s\")\n", resp);
        return false;
    }

    /* Some HM-10 firmware only latches PASS/TYPE after a restart. */
    hm10_flush_rx();
    hm10_at_write("AT+RESET");
    hm10_at_read_response(resp, sizeof resp);   /* typically "OK+RESET" */
    sleep_ms(HM10_AT_RESET_SETTLE_MS);
    hm10_flush_rx();

    printf("[OK]  HM-10: pairing configured (PIN required, auth+bond)\n");
    return true;
}

void hm10_write_str(const char *s) {
    hm10_write((const uint8_t *)s, strlen(s));
}

void hm10_write(const uint8_t *data, size_t len) {
    absolute_time_t deadline = make_timeout_time_us(HM10_WRITE_TIMEOUT_US);
    for (size_t i = 0; i < len; i++) {
        while (!uart_is_writable(HM10_UART)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                return;
            }
            tight_loop_contents();
        }
        uart_putc_raw(HM10_UART, (char)data[i]);
    }
}

bool hm10_readable(void) {
    return uart_is_readable(HM10_UART);
}

size_t hm10_read(uint8_t *out, size_t max_len) {
    size_t n = 0;
    while (n < max_len && uart_is_readable(HM10_UART)) {
        out[n++] = uart_getc(HM10_UART);
    }
    return n;
}
