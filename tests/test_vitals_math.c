/**
 * test_vitals_math.c - host-native unit tests for vitals_math.c
 *
 * No Pico SDK, no hardware: builds and runs directly on the dev machine
 * via tests/CMakeLists.txt (see that file for how to run).
 */

#include "lifelink/vitals_math.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SAMPLE_RATE_HZ 25.0f

/* Fills the window with a sine wave (dc + amplitude*sin) on both
 * channels, sampled at SAMPLE_RATE_HZ, for n samples. */
static void fill_sine(vitals_window_t *w, float bpm, uint32_t n,
                      uint32_t dc, uint32_t amplitude) {
    float freq_hz = bpm / 60.0f;
    for (uint32_t i = 0; i < n; i++) {
        float t = (float)i / SAMPLE_RATE_HZ;
        float s = sinf(2.0f * (float)M_PI * freq_hz * t);
        uint32_t v = (uint32_t)((float)dc + (float)amplitude * s);
        vitals_push(w, v, v);
    }
}

static void test_hr_matches_synthetic_bpm_without_quantization(void) {
    /* Targets are deliberately not multiples of 15: the previous
     * peak_count / window_seconds formula could only ever emit
     * 15, 30, 45, ... A correct implementation should land within a
     * couple bpm of each of these instead. */
    float targets[] = { 50.0f, 68.0f, 92.0f, 113.0f };
    for (size_t k = 0; k < sizeof(targets) / sizeof(targets[0]); k++) {
        vitals_window_t w = {0};
        fill_sine(&w, targets[k], VITALS_WINDOW_LEN, 100000u, 20000u);
        int bpm = vitals_compute_hr_bpm(&w, SAMPLE_RATE_HZ);
        printf("  target=%.0f bpm -> computed=%d bpm\n", targets[k], bpm);
        assert(bpm > 0);
        assert(fabsf((float)bpm - targets[k]) <= 3.0f);
    }
    printf("PASS test_hr_matches_synthetic_bpm_without_quantization\n");
}

static void test_hr_rejects_flat_signal(void) {
    vitals_window_t w = {0};
    for (uint32_t i = 0; i < VITALS_WINDOW_LEN; i++) vitals_push(&w, 100000u, 100000u);
    assert(vitals_compute_hr_bpm(&w, SAMPLE_RATE_HZ) == 0);
    printf("PASS test_hr_rejects_flat_signal\n");
}

static void test_hr_rejects_too_few_samples(void) {
    vitals_window_t w = {0};
    for (uint32_t i = 0; i < 5; i++) vitals_push(&w, 100000u + i * 1000u, 100000u);
    assert(vitals_compute_hr_bpm(&w, SAMPLE_RATE_HZ) == 0);
    printf("PASS test_hr_rejects_too_few_samples\n");
}

static void test_spo2_matches_known_ratio(void) {
    vitals_window_t w = {0};
    /* ac_ir/dc_ir = 20000/100000 = 0.20
     * ac_red/dc_red = 9600/100000 = 0.096
     * ratio = 0.096/0.20 = 0.48 -> spo2 = 110 - 25*0.48 = 98
     * 75 bpm over a 4 s / 100-sample window is an exact 5 cycles, so
     * the sampled mean/peak-to-peak land very close to the analytic
     * values instead of being skewed by a partial trailing cycle. */
    float freq_hz = 75.0f / 60.0f;
    for (uint32_t i = 0; i < VITALS_WINDOW_LEN; i++) {
        float t = (float)i / SAMPLE_RATE_HZ;
        float s = sinf(2.0f * (float)M_PI * freq_hz * t);
        uint32_t ir  = (uint32_t)(100000.0f + 10000.0f * s);
        uint32_t red = (uint32_t)(100000.0f + 4800.0f  * s);
        vitals_push(&w, ir, red);
    }
    int spo2 = vitals_compute_spo2_pct(&w);
    printf("  computed spo2=%d (expected ~98)\n", spo2);
    assert(spo2 >= 96 && spo2 <= 100);
    printf("PASS test_spo2_matches_known_ratio\n");
}

static void test_spo2_clamps_to_range(void) {
    float freq_hz = 75.0f / 60.0f;

    /* No red AC at all -> ratio 0 -> spo2 clamps to 100. */
    vitals_window_t w_high = {0};
    for (uint32_t i = 0; i < VITALS_WINDOW_LEN; i++) {
        float t = (float)i / SAMPLE_RATE_HZ;
        float s = sinf(2.0f * (float)M_PI * freq_hz * t);
        uint32_t ir = (uint32_t)(100000.0f + 10000.0f * s);
        vitals_push(&w_high, ir, 100000u);
    }
    assert(vitals_compute_spo2_pct(&w_high) == 100);

    /* Red AC much larger than IR AC -> large ratio -> spo2 clamps to 70. */
    vitals_window_t w_low = {0};
    for (uint32_t i = 0; i < VITALS_WINDOW_LEN; i++) {
        float t = (float)i / SAMPLE_RATE_HZ;
        float s = sinf(2.0f * (float)M_PI * freq_hz * t);
        uint32_t ir  = (uint32_t)(100000.0f + 2000.0f  * s);
        uint32_t red = (uint32_t)(100000.0f + 60000.0f * s);
        vitals_push(&w_low, ir, red);
    }
    assert(vitals_compute_spo2_pct(&w_low) == 70);

    printf("PASS test_spo2_clamps_to_range\n");
}

static void test_spo2_rejects_no_signal(void) {
    vitals_window_t w = {0};
    for (uint32_t i = 0; i < VITALS_WINDOW_LEN; i++) vitals_push(&w, 100000u, 100000u);
    assert(vitals_compute_spo2_pct(&w) == 0);
    printf("PASS test_spo2_rejects_no_signal\n");
}

static void test_ring_buffer_wraparound(void) {
    vitals_window_t w = {0};
    /* Push 20 more samples than the window holds; the oldest 20
     * (values 0..19) must be evicted, leaving 20..119 as the window. */
    for (uint32_t i = 0; i < VITALS_WINDOW_LEN + 20; i++) {
        vitals_push(&w, i, i);
    }
    assert(w.count == VITALS_WINDOW_LEN);
    /* mean of 20..119 inclusive (100 values) = 6950 / 100 = 69 */
    assert(vitals_mean_ir_dc(&w) == 69);
    printf("PASS test_ring_buffer_wraparound\n");
}

static void test_reset_clears_window(void) {
    vitals_window_t w = {0};
    for (uint32_t i = 0; i < VITALS_WINDOW_LEN; i++) vitals_push(&w, 100000u + i, 100000u);
    vitals_reset(&w);
    assert(w.count == 0);
    assert(w.head == 0);
    assert(vitals_compute_hr_bpm(&w, SAMPLE_RATE_HZ) == 0);
    printf("PASS test_reset_clears_window\n");
}

int main(void) {
    test_hr_matches_synthetic_bpm_without_quantization();
    test_hr_rejects_flat_signal();
    test_hr_rejects_too_few_samples();
    test_spo2_matches_known_ratio();
    test_spo2_clamps_to_range();
    test_spo2_rejects_no_signal();
    test_ring_buffer_wraparound();
    test_reset_clears_window();
    printf("\nAll vitals_math tests passed.\n");
    return 0;
}
