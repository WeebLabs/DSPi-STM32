/*
 * resampler.c — see resampler.h for theory of operation.
 *
 * Original code (C++):
 *   Audio Library for Teensy 3.X
 *   Copyright (c) 2019, Paul Stoffregen, paul@pjrc.com
 *   Polyphase bandlimited interpolator by Alexander Walch
 *   alex6679/teensy-4-spdifIn, MIT licence
 *
 * Port to C for STM32H7:
 *   - Single instance, file-static state
 *   - Stereo only (no multi-channel template)
 *   - Filter table reduced 1024 → 256 oversampling factor, 80 → 40
 *     max half-length to fit in AXI SRAM with comfortable margin
 *   - Kaiser window table small/static instead of new[]
 *
 * MIT licence terms apply — both ST Stoffregen and Walch copyrights
 * are preserved verbatim in the header above.
 */

#include "main.h"
#include "resampler.h"
#include <math.h>
#include <string.h>

#if defined(STM32H723xx)

/* ---- constants / sizes ---- */
#define NO_KAISER_SAMPLES   1025      /* Kaiser lookup resolution */

/* ---- placement (AXI SRAM, 320 KB at 0x24000000; see audio_out.c map) ----
 * The 1024-phase filter[] table is ~160 KB and runs 0x24014000 ..
 * ~0x2403C004, so the Kaiser scratch and the delay line now sit ABOVE it
 * (they were at 0x2402xxxx when the table was 256-phase / ~41 KB).
 * filter[]:           ~160 KB at 0x24014000
 * kaiserScratch_xSq:  ~4 KB   at 0x24040000  (used only during configure)
 * kaiserScratch_temp: ~4 KB   at 0x24041000  (used only during configure)
 * kaiserScratch_win:  ~4 KB   at 0x24042000  (used only during configure)
 * delay_l/r:          320 B   at 0x24044000  (hot path, in AXI but small)
 * Top of use ~0x24044800 — comfortably under the 0x24050000 ceiling. */
#define FILTER_TABLE_ADDR     0x24014000UL
#define KAISER_XSQ_ADDR       0x24040000UL
#define KAISER_TEMP_ADDR      0x24041000UL
#define KAISER_WIN_ADDR       0x24042000UL
#define DELAY_L_ADDR          0x24044000UL
#define DELAY_R_ADDR          0x24044400UL

static float * const filter =
        (float *)FILTER_TABLE_ADDR;
static const int32_t FILTER_TABLE_CAPACITY = RESAMPLER_MAX_FILTER_SAMPLES;

/* delay line — small but kept in AXI SRAM so as not to bloat DTCM BSS */
static float * const delay_l = (float *)DELAY_L_ADDR;
static float * const delay_r = (float *)DELAY_R_ADDR;
static float *endOfBuffer_l;
static float *endOfBuffer_r;

/* runtime config */
static int32_t halfFilterLength;
static int32_t filterLength;
static int32_t overSamplingFactor;
static bool    initialised = false;
static double  attenuation = 0.0;
static float   targetAttenuation = 100.0f;
static int32_t minHalfFilterLength = 20;
static int32_t maxHalfFilterLength = RESAMPLER_MAX_HALF_FILTER;

/* PI controller state */
static ResamplerSettings settings = RESAMPLER_SETTINGS_DEFAULT;
static double configuredStep;
static double step;
static double stepAdapted;
static double cPos;
static double sum;
static double oldDiffs[2];
static const double SETTLED_THRS = 1e-7;

/* utility */
static inline int32_t imin(int32_t a, int32_t b) { return a < b ? a : b; }
static inline int32_t imax(int32_t a, int32_t b) { return a > b ? a : b; }

/* ---- Kaiser window construction ---- */
/* Computes the Kaiser window samples (NO_KAISER_SAMPLES points,
 * symmetric — only one half is computed and stored). Both scratch
 * arrays live in AXI SRAM at fixed addresses (see above). */
static float * const kaiser_xSq    = (float *)KAISER_XSQ_ADDR;
static float * const kaiser_temp   = (float *)KAISER_TEMP_ADDR;
static float * const kaiser_window = (float *)KAISER_WIN_ADDR;

static void get_kaiser_exact(float *win, double beta) {
    const float thres = 0.00000001f;
    {
        double s = 1.0 / (double)(NO_KAISER_SAMPLES - 1);
        for (int i = 1; i < NO_KAISER_SAMPLES; i++) {
            double x = (double)i * s;
            kaiser_xSq[i - 1] = (float)(1.0 - x * x);
        }
    }

    win[0] = 1.0f;
    for (int i = 1; i < NO_KAISER_SAMPLES; i++) {
        win[i] = 1.0f;
        kaiser_temp[i - 1] = 1.0f;
    }
    float denomLastSummand = 1.0f;
    const float halfBetaSq = (float)(beta * beta * 0.25);
    float denom = 1.0f;
    for (int i = 1; i < 1000; i++) {
        denomLastSummand *= (halfBetaSq / (float)(i * i));
        denom += denomLastSummand;
        for (int j = 1; j < NO_KAISER_SAMPLES; j++) {
            kaiser_temp[j - 1] *= kaiser_xSq[j - 1];
            float summand = denomLastSummand * kaiser_temp[j - 1];
            win[j] += summand;
            if (summand < thres) break;
        }
        if (denomLastSummand < thres) break;
    }
    double inv = 1.0 / (double)denom;
    for (int i = 1; i < NO_KAISER_SAMPLES; i++) {
        win[i] = (float)((double)win[i] * inv);
    }
}

/* Resamples the Kaiser window from NO_KAISER_SAMPLES points to
 * noSamples points and writes into filter[]. */
static void set_kaiser_window(double beta, int32_t noSamples) {
    get_kaiser_exact(kaiser_window, beta);
    double s = (double)(NO_KAISER_SAMPLES - 1) / (double)(noSamples - 1);
    double xPos = s;
    float *fc = filter;
    *fc++ = 1.0f;
    int32_t lower = (int32_t)xPos;
    float *winL = &kaiser_window[lower];
    float *winU = &kaiser_window[lower + 1];
    for (int32_t i = 0; i < noSamples - 2; i++) {
        float lambda = (float)(xPos - (double)lower);
        if (lambda > 1.0f) {
            lambda -= 1.0f;
            winL++; winU++; lower++;
        }
        *fc++ = lambda * (*winU) + (1.0f - lambda) * (*winL);
        xPos += s;
        if (xPos >= NO_KAISER_SAMPLES - 1 || lower >= NO_KAISER_SAMPLES - 1) break;
    }
    *fc = *winU;
}

static void set_filter(int32_t halfFL, int32_t over, double cutoff, double beta) {
    const int32_t noSamples = halfFL * over + 1;
    set_kaiser_window(beta, noSamples);
    float *fc = filter;
    *fc++ = (float)cutoff;
    double s = (double)halfFL / (double)(noSamples - 1);
    double xPos = s;
    double factor = M_PI * cutoff;
    for (int32_t i = 1; i < noSamples; i++) {
        *fc++ *= (float)(sin(xPos * factor) / (xPos * M_PI));
        xPos += s;
    }
}

/* ---- public API ---- */

void resampler_init(void) {
    initialised = false;
    halfFilterLength = 0;
    filterLength = 0;
    overSamplingFactor = RESAMPLER_OVERSAMPLING;
    cPos = 0.0;
    step = 1.0;
    stepAdapted = 1.0;
    sum = 0.0;
    oldDiffs[0] = oldDiffs[1] = 0.0;
    attenuation = 0.0;
    memset(delay_l, 0, RESAMPLER_MAX_HALF_FILTER * 2 * sizeof(float));
    memset(delay_r, 0, RESAMPLER_MAX_HALF_FILTER * 2 * sizeof(float));
    /* endOfBuffer_* gets the correct address in resampler_configure
     * once filterLength is known. Until then, use the MAX as a
     * placeholder so debug snapshots of these globals are sane. */
    endOfBuffer_l = &delay_l[RESAMPLER_MAX_HALF_FILTER * 2];
    endOfBuffer_r = &delay_r[RESAMPLER_MAX_HALF_FILTER * 2];
}

void resampler_reset(void) {
    initialised = false;
    sum = 0.0;
    oldDiffs[0] = oldDiffs[1] = 0.0;
}

void resampler_configure(double fs, double newFs) {
    if (fs <= 0.0 || newFs * 0.5 <= 20000.0) {
        attenuation = 0.0;
        halfFilterLength = 0;
        initialised = false;
        return;
    }
    attenuation = (double)targetAttenuation;
    step = fs / newFs;
    configuredStep = step;
    stepAdapted = step;
    sum = 0.0;
    oldDiffs[0] = oldDiffs[1] = 0.0;
    memset(delay_l, 0, RESAMPLER_MAX_HALF_FILTER * 2 * sizeof(float));
    memset(delay_r, 0, RESAMPLER_MAX_HALF_FILTER * 2 * sizeof(float));

    double kaiserBeta, cutOffFreq;
    overSamplingFactor = RESAMPLER_OVERSAMPLING;
    if (fs <= newFs) {
        /* upsampling: pass full band */
        cutOffFreq = 1.0;
        kaiserBeta = settings.kaiserBetaDefault;
        attenuation = kaiserBeta / 0.1102 + 8.7;
        if (fs * 0.5 > 20000.0) {
            halfFilterLength = minHalfFilterLength;
        } else {
            halfFilterLength = maxHalfFilterLength;
        }
    } else {
        /* downsampling: filter to newFs / fs */
        cutOffFreq = newFs / fs;
        double b = (2.0 * (0.5 * newFs - 20000.0) / fs);
        int32_t hfl = (int32_t)((attenuation - 8.0) / (2.0 * 2.285 * 2.0 * M_PI * b) + 0.5);
        if (hfl >= minHalfFilterLength && hfl <= maxHalfFilterLength) {
            halfFilterLength = hfl;
        } else if (hfl < minHalfFilterLength) {
            halfFilterLength = minHalfFilterLength;
            attenuation = ((2.0 * (double)halfFilterLength + 1.0) - 1.0)
                          * (2.285 * 2.0 * M_PI * b) + 8.0;
        } else {
            halfFilterLength = maxHalfFilterLength;
            attenuation = ((2.0 * (double)halfFilterLength + 1.0) - 1.0)
                          * (2.285 * 2.0 * M_PI * b) + 8.0;
        }
        if (attenuation > 50.0) {
            kaiserBeta = 0.1102 * (attenuation - 8.7);
        } else if (attenuation >= 21.0) {
            kaiserBeta = 0.5842 * pow(attenuation - 21.0, 0.4) +
                         0.07886 * (attenuation - 21.0);
        } else {
            kaiserBeta = 0.0;
        }
        if (newFs * 0.5 - 20000.0 > (fs - newFs) * 0.5 &&
            kaiserBeta < settings.kaiserBetaDefault) {
            double l = ((fs - newFs) * 0.5) / (newFs * 0.5 - 20000.0);
            kaiserBeta = l * kaiserBeta + (1.0 - l) * settings.kaiserBetaDefault;
            attenuation = kaiserBeta / 0.1102 + 8.7;
        }
        int32_t noSamples = halfFilterLength * overSamplingFactor + 1;
        if (noSamples > FILTER_TABLE_CAPACITY) {
            int32_t f = (noSamples - 1) / (FILTER_TABLE_CAPACITY - 1) + 1;
            overSamplingFactor /= f;
        }
    }

    set_filter(halfFilterLength, overSamplingFactor, cutOffFreq, kaiserBeta);
    filterLength = halfFilterLength * 2;
    /* Match Teensy semantics: endOfBuffer = base + filterLength. The
     * wing-pointer math (ipL = endOfBuffer + leftWingIndex etc.) relies
     * on this — using the MAX-array end here lands accesses in the
     * wrong delay slots and produces severe distortion. */
    endOfBuffer_l = &delay_l[filterLength];
    endOfBuffer_r = &delay_r[filterLength];
    cPos = -(double)halfFilterLength;
    initialised = true;
}

bool resampler_is_initialised(void)              { return initialised; }
double resampler_get_step(void)                  { return stepAdapted; }
double resampler_get_configured_step(void)       { return configuredStep; }
int32_t resampler_get_half_filter_length(void)   { return halfFilterLength; }
double resampler_get_x_pos(void)                 { return cPos + (double)halfFilterLength; }
void resampler_set_pos(double val) {
    if (val < 0.0) val = 0.0;
    else if (val >= 1.0) val = 0.999;
    cPos = val - (double)halfFilterLength;
}
void resampler_fix_increment(void) {
    if (!initialised) return;
    step = stepAdapted;
    sum = 0.0;
    oldDiffs[0] = oldDiffs[1] = 0.0;
}

bool resampler_update_increment(double diff) {
    sum += diff;
    double correction = settings.kp * diff + settings.ki * sum;
    double absDiff = diff < 0 ? -diff : diff;
    if (absDiff > 2.0 * settings.kpIncreaseThrs) {
        correction += 2.0 * settings.kp * (diff - settings.kpIncreaseThrs);
    } else if (absDiff > settings.kpIncreaseThrs) {
        correction += settings.kp * (diff - settings.kpIncreaseThrs);
    }
    stepAdapted = step + correction;
    double r = stepAdapted / configuredStep - 1.0;
    if ((r < 0 ? -r : r) > settings.maxAdaption) {
        initialised = false;
        return false;
    }
    bool settled = false;
    oldDiffs[0] = oldDiffs[1];
    oldDiffs[1] = (1.0 - settings.alpha) * oldDiffs[1] + settings.alpha * diff;
    double slope = oldDiffs[1] - oldDiffs[0];
    double absSlope = slope < 0 ? -slope : slope;
    if ((absSlope / (double)settings.periodeLength) < SETTLED_THRS * absDiff &&
        absDiff > 2.0 * 1e-6) {
        settled = true;
    }
    return settled;
}

/* ---- main resample hot path ---- */
void resampler_resample(const float *input0, const float *input1,
                        int32_t inputLength, int32_t *processedLength,
                        float *output0, float *output1,
                        int32_t outputLength, int32_t *outputCount) {
    int32_t outCount = 0;
    int32_t successorIndex = (int32_t)ceil(cPos);

    const int32_t halfFLenUpsampledM1 = (halfFilterLength - 1) * overSamplingFactor;
    const int32_t overSamplingFactorM1 = overSamplingFactor - 1;
    const int32_t hfl = halfFilterLength;

    const float *ipR0, *ipR1, *ipL0, *ipL1;
    float res0, res1;

    while (floor(cPos + (double)hfl) < (double)inputLength && outCount < outputLength) {
        float dist = (float)((double)successorIndex - cPos);
        int32_t rightWingIndex = successorIndex + hfl - 1;
        int32_t leftWingIndex  = successorIndex - hfl;

        if (dist == 0.0f) {
            ipR0 = input0 + rightWingIndex + 1;
            ipR1 = input1 + rightWingIndex + 1;
        } else {
            ipR0 = input0 + rightWingIndex;
            ipR1 = input1 + rightWingIndex;
        }
        res0 = 0.0f;
        res1 = 0.0f;

        if (cPos < 0.0) {
            ipL0 = endOfBuffer_l + leftWingIndex;
            ipL1 = endOfBuffer_r + leftWingIndex;
            leftWingIndex = 1;
        } else if (leftWingIndex < 0) {
            ipL0 = endOfBuffer_l + leftWingIndex;
            ipL1 = endOfBuffer_r + leftWingIndex;
            rightWingIndex = -2;
        } else {
            ipL0 = input0 + leftWingIndex;
            ipL1 = input1 + leftWingIndex;
            rightWingIndex = -2;
            leftWingIndex = 1;
        }

        if (dist == 0.0f) {
            if (cPos >= 0.0 || rightWingIndex != hfl - 1) {
                rightWingIndex++;
            }
            const float *fPtr = filter + hfl * overSamplingFactor;
            for (int32_t i = 0; i < hfl; i++) {
                res0 += (*ipL0++ + *ipR0--) * (*fPtr);
                res1 += (*ipL1++ + *ipR1--) * (*fPtr);
                fPtr -= overSamplingFactor;
                if (i == rightWingIndex) {
                    ipR0 = endOfBuffer_l - 1;
                    ipR1 = endOfBuffer_r - 1;
                } else if (i == (-leftWingIndex - 1)) {
                    ipL0 = input0;
                    ipL1 = input1;
                }
            }
            res0 += (*filter) * (*ipR0);
            res1 += (*filter) * (*ipR1);
            *output0++ = res0;
            *output1++ = res1;
        } else {
            float distScaled = dist * (float)overSamplingFactor;
            int32_t sufR = (int32_t)ceilf(distScaled);
            int32_t sufL = -(sufR - overSamplingFactor - 1);
            float w0 = (float)sufR - distScaled;
            float w1 = 1.0f - w0;
            const float *fPtrR = filter + sufR + halfFLenUpsampledM1;
            const float *fPtrL = filter + sufL + halfFLenUpsampledM1;
            for (int32_t i = 0; i < hfl; i++) {
                float rwCoeff = w1 * (*fPtrR--);
                rwCoeff += w0 * (*fPtrR);
                float lwCoeff = w0 * (*fPtrL--);
                lwCoeff += w1 * (*fPtrL);
                res0 += (*ipL0++) * lwCoeff;
                res0 += (*ipR0--) * rwCoeff;
                res1 += (*ipL1++) * lwCoeff;
                res1 += (*ipR1--) * rwCoeff;
                fPtrR -= overSamplingFactorM1;
                fPtrL -= overSamplingFactorM1;
                if (i == rightWingIndex) {
                    ipR0 = endOfBuffer_l - 1;
                    ipR1 = endOfBuffer_r - 1;
                } else if (i == (-leftWingIndex - 1)) {
                    ipL0 = input0;
                    ipL1 = input1;
                }
            }
            *output0++ = res0;
            *output1++ = res1;
        }
        outCount++;
        cPos += stepAdapted;
        while (cPos > (double)successorIndex) successorIndex++;
    }

    int32_t procLen;
    if (outCount < outputLength) {
        procLen = inputLength;
    } else {
        procLen = imin(inputLength, (int32_t)floor(cPos + (double)hfl));
    }

    /* Save the trailing input window into the delay line for next call. */
    int32_t indexData = procLen - filterLength;
    if (indexData >= 0) {
        const size_t bytes = (size_t)filterLength * sizeof(float);
        memcpy(delay_l, input0 + indexData, bytes);
        memcpy(delay_r, input1 + indexData, bytes);
    } else {
        float *b0 = delay_l;
        float *b1 = delay_r;
        const float *ip0 = endOfBuffer_l + indexData;
        const float *ip1 = endOfBuffer_r + indexData;
        for (int32_t i = 0; i < filterLength; i++) {
            if (ip0 == endOfBuffer_l) {
                ip0 = input0;
                ip1 = input1;
            }
            *b0++ = *ip0++;
            *b1++ = *ip1++;
        }
    }
    cPos -= (double)procLen;
    if (cPos < -(double)hfl) cPos = -(double)hfl;

    *processedLength = procLen;
    *outputCount = outCount;
}

#endif /* STM32H723xx */
