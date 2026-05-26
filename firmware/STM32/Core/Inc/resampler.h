/*
 * resampler.h — port of alex6679/teensy-4-spdifIn polyphase resampler
 *
 * Original copyright:
 *   Audio Library for Teensy 3.X
 *   Copyright (c) 2019, Paul Stoffregen, paul@pjrc.com
 *   Algorithm by Alexander Walch — MIT licence (preserved in resampler.c)
 *
 * Bandlimited interpolation (Julius O. Smith / CCRMA), polyphase
 * windowed-sinc FIR with linear interpolation between adjacent phases.
 * Kaiser-windowed prototype, configurable cutoff for both up- and
 * down-sampling. PI controller on the resampling rate, driven by
 * external buffer-fill error (in seconds).
 *
 * Sized for STM32H7 (~165 KB → 41 KB by dropping the polyphase factor
 * 1024 → 256 and the max half-filter length 80 → 40). Quality stays
 * above 16-bit transparent for the SPDIF→SAI use case (1:1 ± ppm,
 * occasional 2:1 / 4:1 downsample for 96k/192k input).
 */

#ifndef RESAMPLER_H
#define RESAMPLER_H

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32H723xx)

/* Tuning constants. Filter table size = MAX_FILTER_SAMPLES floats. */
#define RESAMPLER_OVERSAMPLING       256
#define RESAMPLER_MAX_HALF_FILTER    40
#define RESAMPLER_MAX_FILTER_SAMPLES (RESAMPLER_MAX_HALF_FILTER * RESAMPLER_OVERSAMPLING + 1)
/* = 10241 floats = ~41 KB. Placed in AXI SRAM. */

/* PI controller / step adaption parameters. */
typedef struct {
    double alpha;              /* exp-smoothing on diff slope (settled detect) */
    double maxAdaption;        /* max relative step adjustment (1% default)    */
    double kp;                 /* P gain                                       */
    double ki;                 /* I gain                                       */
    double kpIncreaseThrs;     /* error threshold (s) above which kp boosts    */
    double kaiserBetaDefault;  /* default Kaiser β for upsampling              */
    int32_t periodeLength;     /* output samples between updateIncrement calls */
} ResamplerSettings;

/* Default settings — Teensy defaults toned down for our coarser
 * fill-error signal. Original kp=0.6, ki=0.00012 hunted ±1000 ppm
 * on the H7 because our ring-fill noise (±256-frame bursts) is much
 * larger than Teensy's clean block-level error. */
#define RESAMPLER_SETTINGS_DEFAULT { \
    .alpha = 0.05, \
    .maxAdaption = 0.02, \
    .kp = 0.1, \
    .ki = 0.000005, \
    .kpIncreaseThrs = 100.0e-6, \
    .kaiserBetaDefault = 18.0, \
    .periodeLength = 128, \
}

/* One-time module init: allocates the filter table, clears state.
 * Call once at boot. */
void resampler_init(void);

/* Configure for a (input_fs → output_fs) ratio. Builds the Kaiser-
 * windowed sinc prototype. Slow operation (~ms) — call only on lock
 * acquisition or major rate change. */
void resampler_configure(double input_fs, double output_fs);

/* Reset transient state (delay line, PI accumulator). Does not
 * re-build the filter. Call on lock loss / source switch. */
void resampler_reset(void);

/* Per-block PI controller update. `diff` is the deviation of the
 * input ring buffer fill from target latency, IN SECONDS. Should be
 * low-pass filtered upstream of this call to suppress block-rate
 * jitter. Returns true when the smoothed error slope is "settled". */
bool resampler_update_increment(double diff);

/* Block-based stereo resample.
 *   in_l, in_r     — pointers to deinterleaved input float buffers
 *   in_len         — length of each input buffer (frames)
 *   processed_len  — out: how many input frames were consumed
 *   out_l, out_r   — pointers to deinterleaved output float buffers
 *   out_len        — length of each output buffer (frames)
 *   out_count      — out: how many output frames were produced
 * Maintains an internal delay line for inter-call continuity. */
void resampler_resample(const float *in_l, const float *in_r,
                        int32_t in_len, int32_t *processed_len,
                        float *out_l, float *out_r,
                        int32_t out_len, int32_t *out_count);

/* Status / diagnostic. */
bool   resampler_is_initialised(void);
double resampler_get_step(void);            /* current adapted ratio */
double resampler_get_configured_step(void); /* nominal ratio         */
int32_t resampler_get_half_filter_length(void);

/* Sub-sample position helpers (Teensy compatibility). */
double resampler_get_x_pos(void);
void   resampler_set_pos(double val);       /* 0 ≤ val < 1           */
void   resampler_fix_increment(void);       /* snap step = stepAdapted */

#else /* not STM32H723xx — RP / cross-compile stubs */

typedef struct { int dummy; } ResamplerSettings;
#define RESAMPLER_SETTINGS_DEFAULT { 0 }

static inline void   resampler_init(void)                              { }
static inline void   resampler_configure(double a, double b)           { (void)a; (void)b; }
static inline void   resampler_reset(void)                             { }
static inline bool   resampler_update_increment(double d)              { (void)d; return false; }
static inline void   resampler_resample(const float *a, const float *b,
                                        int32_t c, int32_t *d,
                                        float *e, float *f,
                                        int32_t g, int32_t *h) {
    (void)a; (void)b; (void)c; (void)e; (void)f; (void)g;
    if (d) *d = 0;
    if (h) *h = 0;
}
static inline bool   resampler_is_initialised(void)             { return false; }
static inline double resampler_get_step(void)                   { return 1.0; }
static inline double resampler_get_configured_step(void)        { return 1.0; }
static inline int32_t resampler_get_half_filter_length(void)    { return 0; }
static inline double resampler_get_x_pos(void)                  { return 0.0; }
static inline void   resampler_set_pos(double v)                { (void)v; }
static inline void   resampler_fix_increment(void)              { }

#endif /* STM32H723xx */

#endif /* RESAMPLER_H */
