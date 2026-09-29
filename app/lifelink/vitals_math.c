/**
 * vitals_math.c - HR/SpO2 math over a sliding IR/Red sample window
 *
 * HR estimation pipeline (in vitals_compute_hr_bpm):
 *   1. Remove slow baseline drift (finger pressure changes, perfusion
 *      drift, breathing) from the IR channel with a centered moving-
 *      average subtraction, so peak detection responds to the actual
 *      cardiac pulsatile component instead of being swamped by whichever
 *      way the baseline happens to be drifting that window.
 *   2. Peak-detect on the detrended signal with a threshold relative to
 *      the detrended amplitude.
 *   3. Require at least 4 peaks (3 intervals) before trusting a result -
 *      a bpm computed from a single interval has no averaging and a
 *      single misdetected peak swings the answer wildly.
 *   4. Take the MEDIAN interval, then average only the intervals that
 *      are close to that median. A median can't be dragged off by one
 *      outlier interval the way a plain mean can.
 *
 * SpO2 keeps the same AC/DC ratio approach but now measures the AC swing
 * on the same detrended signal, so a slow baseline drift during the
 * window doesn't get counted as pulsatile amplitude.
 */

#include "lifelink/vitals_math.h"

/* Maps a chronological sample index (0 = oldest sample currently in the
 * window) to its physical slot in the ring buffer. */
static inline uint32_t ring_index(const vitals_window_t *w, uint32_t logical_idx) {
    uint32_t oldest = (w->head + VITALS_WINDOW_LEN - w->count) % VITALS_WINDOW_LEN;
    return (oldest + logical_idx) % VITALS_WINDOW_LEN;
}

void vitals_push(vitals_window_t *w, uint32_t ir, uint32_t red) {
    w->ir [w->head] = ir;
    w->red[w->head] = red;
    w->head = (w->head + 1) % VITALS_WINDOW_LEN;
    if (w->count < VITALS_WINDOW_LEN) w->count++;
}

void vitals_reset(vitals_window_t *w) {
    w->head  = 0;
    w->count = 0;
}

uint32_t vitals_mean_ir_dc(const vitals_window_t *w) {
    if (w->count == 0) return 0;
    uint64_t sum = 0;
    for (uint32_t i = 0; i < w->count; i++) sum += w->ir[ring_index(w, i)];
    return (uint32_t)(sum / w->count);
}

/* Centered moving-average baseline, used to detrend a channel before peak
 * detection / AC measurement. half_win is the number of samples on each
 * side of i to average (clipped at the window edges). */
static void compute_baseline(const uint32_t *vals, uint32_t n, uint32_t half_win,
                             float *baseline_out) {
    for (uint32_t i = 0; i < n; i++) {
        uint32_t lo = (i > half_win) ? (i - half_win) : 0;
        uint32_t hi = (i + half_win < n - 1) ? (i + half_win) : (n - 1);
        uint64_t sum = 0;
        for (uint32_t k = lo; k <= hi; k++) sum += vals[k];
        baseline_out[i] = (float)sum / (float)(hi - lo + 1);
    }
}

/* Ascending insertion sort - windows here are small (a few dozen values
 * at most), so this is simpler and plenty fast without pulling in a
 * generic qsort comparator. */
static void sort_ascending(int *vals, int n) {
    for (int i = 1; i < n; i++) {
        int key = vals[i];
        int j = i - 1;
        while (j >= 0 && vals[j] > key) {
            vals[j + 1] = vals[j];
            j--;
        }
        vals[j + 1] = key;
    }
}

static int median_of(int *vals, int n) {
    sort_ascending(vals, n);
    if (n % 2 == 1) return vals[n / 2];
    return (vals[n / 2 - 1] + vals[n / 2]) / 2;
}

int vitals_compute_hr_bpm(const vitals_window_t *w, float sample_rate_hz) {
    uint32_t n = w->count;
    if (n < 10) return 0;

    uint32_t raw[VITALS_WINDOW_LEN];
    for (uint32_t i = 0; i < n; i++) raw[i] = w->ir[ring_index(w, i)];

    /* Detrend: subtract a baseline computed over roughly one-and-a-bit
     * pulse periods, so the cardiac component survives but slower drift
     * (finger pressure, perfusion, breathing) is removed. */
    uint32_t half_win = (uint32_t)(0.6f * sample_rate_hz);
    if (half_win < 2) half_win = 2;
    float baseline[VITALS_WINDOW_LEN];
    compute_baseline(raw, n, half_win, baseline);

    float detr[VITALS_WINDOW_LEN];
    float mn = 0.0f, mx = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        detr[i] = (float)raw[i] - baseline[i];
        if (i == 0 || detr[i] < mn) mn = detr[i];
        if (i == 0 || detr[i] > mx) mx = detr[i];
    }
    float ac_pp = mx - mn;
    if (ac_pp < 40.0f) return 0;   /* lower floor than before - this is now
                                    * genuine pulsatile amplitude, not raw
                                    * signal swamped by baseline drift. */

    float    thresh        = mn + ac_pp * 0.5f;
    int      min_spacing   = (int)(0.4f * sample_rate_hz);
    int      last_peak     = -1000;
    int      intervals[VITALS_WINDOW_LEN];
    int      interval_n    = 0;

    for (uint32_t i = 1; i + 1 < n; i++) {
        float v  = detr[i];
        float vp = detr[i - 1];
        float vn = detr[i + 1];
        if (v > thresh && v >= vp && v > vn &&
            (int)i - last_peak > min_spacing) {
            if (last_peak >= 0) {
                intervals[interval_n++] = (int)i - last_peak;
            }
            last_peak = (int)i;
        }
    }

    /* Require several intervals so a single misdetected/missed peak can't
     * swing the whole estimate - a bpm from just one interval has no
     * averaging behind it at all. */
    if (interval_n < 3) return 0;

    /* Robust combine: take the median interval, then average only the
     * intervals close to it. This is what actually stops one stray or
     * missed peak from producing a wildly different bpm than the reading
     * right before/after it. */
    int median_copy[VITALS_WINDOW_LEN];
    for (int i = 0; i < interval_n; i++) median_copy[i] = intervals[i];
    int median_iv = median_of(median_copy, interval_n);
    if (median_iv <= 0) return 0;

    int64_t inlier_sum = 0;
    int     inlier_n    = 0;
    for (int i = 0; i < interval_n; i++) {
        int diff = intervals[i] - median_iv;
        if (diff < 0) diff = -diff;
        if (diff <= median_iv / 3) {   /* within ~33% of the median */
            inlier_sum += intervals[i];
            inlier_n++;
        }
    }
    /* median_iv is itself one of the intervals, so this can't be empty. */
    float mean_interval_samples = (float)inlier_sum / (float)inlier_n;

    int bpm = (int)(60.0f * sample_rate_hz / mean_interval_samples + 0.5f);
    if (bpm < 30 || bpm > 220) return 0;
    return bpm;
}

int vitals_compute_spo2_pct(const vitals_window_t *w) {
    uint32_t n = w->count;
    if (n < 10) return 0;

    uint32_t raw_r[VITALS_WINDOW_LEN];
    uint32_t raw_i[VITALS_WINDOW_LEN];
    uint64_t sum_r = 0, sum_i = 0;
    for (uint32_t k = 0; k < n; k++) {
        uint32_t idx = ring_index(w, k);
        raw_r[k] = w->red[idx];
        raw_i[k] = w->ir [idx];
        sum_r += raw_r[k];
        sum_i += raw_i[k];
    }
    float dc_r = (float)sum_r / (float)n;
    float dc_i = (float)sum_i / (float)n;

    /* Detrend both channels the same way HR does, so a slow baseline
     * drift during the window isn't counted as pulsatile AC amplitude
     * here either. */
    uint32_t half_win = 12; /* ~0.5s at a typical 25Hz output rate */
    float base_r[VITALS_WINDOW_LEN], base_i[VITALS_WINDOW_LEN];
    compute_baseline(raw_r, n, half_win, base_r);
    compute_baseline(raw_i, n, half_win, base_i);

    float r_mn = 0.0f, r_mx = 0.0f, i_mn = 0.0f, i_mx = 0.0f;
    for (uint32_t k = 0; k < n; k++) {
        float dr = (float)raw_r[k] - base_r[k];
        float di = (float)raw_i[k] - base_i[k];
        if (k == 0 || dr < r_mn) r_mn = dr;
        if (k == 0 || dr > r_mx) r_mx = dr;
        if (k == 0 || di < i_mn) i_mn = di;
        if (k == 0 || di > i_mx) i_mx = di;
    }
    float ac_r = r_mx - r_mn;
    float ac_i = i_mx - i_mn;
    if (dc_r < 1.0f || dc_i < 1.0f || ac_i < 1.0f) return 0;

    float ratio = (ac_r / dc_r) / (ac_i / dc_i);
    int   spo2  = (int)(110.0f - 25.0f * ratio + 0.5f);
    if (spo2 < 70)  return 70;
    if (spo2 > 100) return 100;
    return spo2;
}