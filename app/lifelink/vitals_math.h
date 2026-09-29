/**
 * vitals_math.h - HR/SpO2 math over a sliding IR/Red sample window
 *
 * Hardware-independent: no Pico SDK includes, no global state. Callers
 * own a vitals_window_t and pass it in explicitly, which makes this
 * module buildable and testable with a plain host compiler (see
 * tests/test_vitals_math.c) as well as the ARM cross build.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "config.h"

/* Window length matches the MAX30102 sample window in config.h so the
 * two stay in sync automatically. */
#define VITALS_WINDOW_LEN MAX30102_SAMPLE_WIN

typedef struct {
    uint32_t ir  [VITALS_WINDOW_LEN];
    uint32_t red [VITALS_WINDOW_LEN];
    uint32_t head;    /* next write slot */
    uint32_t count;   /* valid samples, 0..VITALS_WINDOW_LEN */
} vitals_window_t;

/** Push one IR/Red sample pair, overwriting the oldest once full. */
void vitals_push(vitals_window_t *w, uint32_t ir, uint32_t red);

/** Empty the window (used when the MAX30102 is marked offline). */
void vitals_reset(vitals_window_t *w);

/** Mean IR DC level over the window; used for finger-presence detection. */
uint32_t vitals_mean_ir_dc(const vitals_window_t *w);

/**
 * Compute heart rate in bpm via peak detection on the IR channel.
 *
 * bpm is derived from the average sample spacing between consecutive
 * detected peaks (not from peak_count / window_seconds), so resolution
 * isn't quantised to multiples of 60/window_seconds.
 *
 * @param sample_rate_hz  FIFO output rate (see config.h / SMP_AVE setting)
 * @return bpm, or 0 if there isn't enough signal/peaks for a confident
 *         reading, or the result falls outside 30-220 bpm.
 */
int vitals_compute_hr_bpm(const vitals_window_t *w, float sample_rate_hz);

/**
 * Compute SpO2 percentage via the Red/IR AC-DC ratio-of-ratios.
 * @return SpO2 clamped to 70-100, or 0 if there isn't enough signal
 *         for a confident reading.
 */
int vitals_compute_spo2_pct(const vitals_window_t *w);
