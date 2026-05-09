/**
 * audio_out.c — DSPi STM32H723, M4 + M5 + M6a SAI1_A + SAI1_B I2S TX
 *
 * Brings up SAI1 sub-blocks A and B as I2S Philips master TX.
 *
 *   Sub-block A: asynchronous master, generates BCK/FS/MCLK on
 *                PE2 (MCLK)/PE4 (FS)/PE5 (BCK)/PE6 (SD). DMA1 Stream 0.
 *
 *   Sub-block B: SYNCHRONOUS to A — uses A's BCK and FS clocks via the
 *                internal sister-block sync path; outputs serial data on
 *                PE3 (the only LQFP100 pin available for SAI1_SD_B).
 *                DMA1 Stream 1. Both DMAs read from the SAME ping-pong
 *                buffer in AXI SRAM, so PE3 mirrors PE6 sample-for-sample.
 *
 * Sample alignment (M5 hard constraint): both blocks emit their first
 * data bit on the same BCK edge because B is a clock-slave to A; both
 * DMAs read the same memory at the same rate; the sample on PE3 is
 * literally the same int32_t word as the sample on PE6.
 *
 * Audio source (M6a): the USB UAC1 OUT EP feeds a small ring buffer in
 * usb_audio.c; this file's fill_half drains the ring, converts each
 * 16-bit signed PCM sample to the SAI's 24-bit-right-aligned format
 * (sample << 8 to scale 16→24, NOT to MSB-align in the slot — see
 * lesson 10), and writes it into the half-buffer. If the ring is
 * starving (USB idle, between packets, or alt-0), the slot fills with
 * silence so the SAI keeps clocking and the DAC stays locked.
 *
 * D-cache is still off through M6a; AXI SRAM access is coherent without
 * MPU surgery. Custom linker script + cache enable arrives in M6c.
 */

#include "main.h"
#include "usb_audio.h"
#include "dsp_pipeline.h"  /* M7c: per-channel biquad EQ */
#include "crossfeed.h"     /* M7d: BS2B crossfeed */
#include "leveller.h"      /* M7d: volume leveller */
#include "loudness.h"      /* M7i: loudness compensation */
#include <math.h>          /* M7e: fabsf for level meters */

/* ---- Per-sub-block DMA rings in AXI SRAM (DMA1 cannot reach DTCM) ----
 *
 * Phase 1 (4 independent stereo slots): SAI1_A and SAI1_B each get their
 * own ping-pong buffer so the two physical pin pairs can carry DIFFERENT
 * stereo signals. The earlier build had both DMAs reading the same
 * audio_buf, which collapsed PE6 and PE3 to identical data.
 *
 * AXI SRAM layout (320 KB at 0x24000000):
 *   0x24000000  audio_buf_A (3 KB) — SAI1_A / Out0+1
 *   0x24001000  audio_buf_B (3 KB) — SAI1_B / Out2+3
 *   0x24004000  flash_mirror (48 KB) — preset region (M11)
 * Plenty of room for SAI2 buffers in phase 2 without touching the mirror. */
#define AUDIO_BUFFER_BASE_A  0x24000000UL
#define AUDIO_BUFFER_BASE_B  0x24001000UL

/* 192 stereo frames per ping-pong half × 2 halves × 2 ch = 768 int32 words.
 * 192 frames @ 48 kHz = 4 ms per half — comfortably above any HAL ISR
 * latency. */
#define AUDIO_FRAMES_HALF    192U
#define AUDIO_FRAMES_TOTAL   (AUDIO_FRAMES_HALF * 2)
#define AUDIO_WORDS_TOTAL    (AUDIO_FRAMES_TOTAL * 2)  /* L+R */

static int32_t * const audio_buf_a = (int32_t *)AUDIO_BUFFER_BASE_A;
static int32_t * const audio_buf_b = (int32_t *)AUDIO_BUFFER_BASE_B;

/* Scratch for one half's worth of stereo 16-bit samples popped from the
 * ring; converted to 24-bit-right-aligned in place into audio_buf[]. */
static int16_t pop_scratch[AUDIO_FRAMES_HALF * 2];

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

volatile uint32_t audio_dma_callbacks = 0;
volatile uint32_t audio_underruns     = 0;

/* ---------------------------------------------------------------------- */
/* DMA half-buffer fill — drain USB ring, convert 16-bit → 24-bit         */
/* ---------------------------------------------------------------------- */
/* M7c DSP pipeline (per-sample for now; M7d may switch to block-based):
 *
 *   USB int16 → float [-1, +1]
 *   ↓
 *   Per-input EQ (channels 0/1)              ← Console "USB L/R" sliders
 *   ↓
 *   Matrix mixer (input → output crosspoint
 *     gain × phase, summed per output)       ← Console "Routing" matrix
 *   ↓
 *   Per-output EQ (channels 2..N)            ← Console per-output PEQ
 *   ↓
 *   Per-output mute + linear gain stage      ← Console "Output Gain"
 *   ↓
 *   Float [-1, +1] → int32 right-aligned     → SAI DMA buffer
 *
 * The SAI only physically drives Outputs 0 + 1 in this build (SPDIF/PDM
 * arrive in M8/M10) so we compute just those two outputs. The rest of
 * the matrix is irrelevant — Console can drive it but nothing audibly
 * happens until those outputs have hardware backing them. */

#define INT16_RECIP   (1.0f / 32768.0f)
#define FLOAT_TO_24   8388607.0f      /* 2^23 − 1 */

extern MatrixMixer matrix_mixer;
extern volatile float global_preamp_linear[NUM_INPUT_CHANNELS];
extern volatile float master_volume_linear;
extern volatile bool  bypass_master_eq;
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool            crossfeed_update_pending;
extern volatile LevellerConfig  leveller_config;
extern volatile bool            leveller_update_pending;
extern volatile bool            leveller_reset_pending;

/* M7d: per-output delay lines (defined in dsp_pipeline.c). delay_lines is
 * indexed by output number (0..NUM_DELAY_CHANNELS-1), NOT by global channel
 * index — channel_delays_ms[CH_OUT_1+out] is the user-facing field. */
extern float    delay_lines[NUM_DELAY_CHANNELS][MAX_DELAY_SAMPLES];
extern uint32_t delay_write_idx;
extern int32_t  channel_delay_samples[NUM_DELAY_CHANNELS];
extern bool     any_delay_active;

/* M7e: level meters. global_status.peaks[] is u16 [0..32767] = |sample|*32767.
 * Console polls REQ_GET_STATUS wValue=9 every ~30 ms; we just keep refreshing
 * the latest per-fill-half block peak in place — Console smooths visually. */
extern volatile SystemStatusPacket global_status;

/* M7k: CPU metering via DWT cycle counter.
 *
 * The audio path runs in 192-sample halves at 48 kHz → one fill_half call
 * every 4 ms. At SYSCLK=550 MHz that's a 2,200,000-cycle budget. We snap
 * the cycle counter at the top + bottom of fill_half, accumulate the
 * delta, and every CPU_METER_BLOCKS calls (≈ once per 0.5 s) divide by
 * the total budget to produce an integer % (0..100). The result lives
 * in global_status.cpu0_load and Console pulls it via REQ_GET_STATUS
 * wValue=9. cpu1_load stays 0 (no Core 1 on STM32).
 *
 * Two reasons cycles, not microseconds: (a) DWT cycle count is the
 * cheapest read on M7 (single MRC), zero peripheral dependency; (b) it
 * gives us the metric users actually care about — "what fraction of the
 * core am I using?" — without floating-point time math in the hot path.
 */
#define CPU_METER_BLOCKS         128U  /* ~0.5 s at 192 frames / 48 kHz */
#define CPU_BUDGET_CYCLES_PER_HALF \
        ((uint32_t)((uint64_t)550000000U * AUDIO_FRAMES_HALF / 48000U))

/* M7i: loudness compensation — per-channel SVF state for the 2-biquad
 * shelf cascade. State lives here (not in loudness.c) because it's a
 * stateful run-time object that rolls samples; the imported loudness.c
 * is pure coefficient computation. [0]=L, [1]=R. */
static LoudnessSvfState loudness_state[2][LOUDNESS_BIQUAD_COUNT];
extern volatile bool loudness_enabled;

/* Local DSP state (zeroed on init by Audio_Init). */
static CrossfeedState crossfeed_state;
static LevellerState  leveller_state;
static LevellerCoeffs leveller_coeffs;

/* Block-based per-stage scratch buffers — one half-buffer's worth.
 * Each stage operates on the full block in-place. AXI SRAM-resident
 * via the shared audio_buf placement isn't required here (these are
 * CPU-only) so they live as plain BSS in DTCM (faster, no DMA). */
/* Phase 1: per-stage scratch buffers placed in AXI SRAM via fixed
 * pointers so the DTCM budget isn't blown out by 4× per-output stereo
 * scratch. AXI SRAM map (after audio_buf_a/_b at 0..0x1FFF):
 *   0x24002000  buf_l       (768 B)
 *   0x24002300  buf_r       (768 B)
 *   0x24002600  buf_o0      (768 B)
 *   0x24002900  buf_o1      (768 B)
 *   0x24002C00  buf_o2      (768 B)
 *   0x24002F00  buf_o3      (768 B)
 *   0x24004000  flash_mirror (48 KB) — already there
 * Total scratch = 4.5 KB, fits comfortably below the mirror at 0x4000. */
#define DSP_SCRATCH_BASE  0x24002000UL
static float * const buf_l  = (float *)(DSP_SCRATCH_BASE + 0 * 0x300);
static float * const buf_r  = (float *)(DSP_SCRATCH_BASE + 1 * 0x300);
static float * const buf_o0 = (float *)(DSP_SCRATCH_BASE + 2 * 0x300);
static float * const buf_o1 = (float *)(DSP_SCRATCH_BASE + 3 * 0x300);
static float * const buf_o2 = (float *)(DSP_SCRATCH_BASE + 4 * 0x300);
static float * const buf_o3 = (float *)(DSP_SCRATCH_BASE + 5 * 0x300);

/* Per-output EQ channel index: Out0 → CH 2, Out1 → CH 3, Out2 → CH 4,
 * Out3 → CH 5. (CH_OUT_n already exist as global config defines but
 * they're 1-indexed-by-pin in the project naming.) */
#define EQ_CH_OUT0    2
#define EQ_CH_OUT1    3
#define EQ_CH_OUT2    4
#define EQ_CH_OUT3    5

static void fill_half(int32_t *dst_a, int32_t *dst_b) {
    /* M7k: snap cycle count at entry; computed at exit and accumulated
     * into a per-window total that gets converted to a % every
     * CPU_METER_BLOCKS calls. Single MRC, ~1 cycle. */
    uint32_t cpu_t0 = DWT->CYCCNT;

    uint32_t got = usb_ring_pop_frames(pop_scratch, AUDIO_FRAMES_HALF);
    if (got < AUDIO_FRAMES_HALF) ++audio_underruns;

    /* Snapshot the matrix crosspoints once per buffer-half (≪ 1 µs each)
     * so the inner loop stays branch-light. Per-input gain folds in
     * phase invert. Phase 1: snapshot all FOUR outputs so SAI1_B carries
     * Out2/Out3 instead of mirroring Out0/Out1. */
    #define XP_GAIN(in, out) \
        (matrix_mixer.crosspoints[(in)][(out)].enabled \
            ? (matrix_mixer.crosspoints[(in)][(out)].phase_invert \
                ? -matrix_mixer.crosspoints[(in)][(out)].gain_linear \
                :  matrix_mixer.crosspoints[(in)][(out)].gain_linear) \
            : 0.0f)
    float g_l_to_o0 = XP_GAIN(0, 0);
    float g_r_to_o0 = XP_GAIN(1, 0);
    float g_l_to_o1 = XP_GAIN(0, 1);
    float g_r_to_o1 = XP_GAIN(1, 1);
    float g_l_to_o2 = XP_GAIN(0, 2);
    float g_r_to_o2 = XP_GAIN(1, 2);
    float g_l_to_o3 = XP_GAIN(0, 3);
    float g_r_to_o3 = XP_GAIN(1, 3);
    #undef XP_GAIN

    #define POST_GAIN(out) \
        ((matrix_mixer.outputs[(out)].enabled && !matrix_mixer.outputs[(out)].mute) \
            ? matrix_mixer.outputs[(out)].gain_linear : 0.0f)
    float out0_post_gain = POST_GAIN(0);
    float out1_post_gain = POST_GAIN(1);
    float out2_post_gain = POST_GAIN(2);
    float out3_post_gain = POST_GAIN(3);
    #undef POST_GAIN

    /* M7d: per-input preamp + master volume + bypass snapshots —
     * once per buffer-half so the per-sample loops stay branch-light. */
    float preamp_l = global_preamp_linear[0];
    float preamp_r = global_preamp_linear[1];
    /* M7h: composite output gain = host volume (UAC1) × master volume.
     * audio_state.vol_mul is Q15 (32768 = unity); convert once per buffer
     * -half. audio_state.mute folded in as a hard 0 multiplier — host's
     * mute key always wins over the slider. */
    float host_vol = audio_state.mute
                     ? 0.0f
                     : (float)audio_state.vol_mul * (1.0f / 32768.0f);
    float master   = host_vol * master_volume_linear;
    bool  eq_bypass = bypass_master_eq;
    bool  cf_active = crossfeed_config.enabled;
    bool  lv_active = leveller_config.enabled;

    /* Apply pending crossfeed / leveller coefficient recomputes (set
     * when Console changes any param). Cheap and once per buffer-half. */
    if (crossfeed_update_pending) {
        crossfeed_update_pending = false;
        crossfeed_compute_coefficients(&crossfeed_state,
                                        (CrossfeedConfig *)&crossfeed_config,
                                        48000.0f);
    }
    if (leveller_update_pending) {
        leveller_update_pending = false;
        leveller_compute_coefficients(&leveller_coeffs,
                                       (LevellerConfig *)&leveller_config,
                                       48000.0f);
    }
    if (leveller_reset_pending) {
        leveller_reset_pending = false;
        leveller_reset_state(&leveller_state);
    }

    /* === Stage 1: depacketize USB int16 stereo → buf_l / buf_r float
     *               with per-input preamp folded in. Pad shortfall
     *               with silence. */
    uint32_t i;
    for (i = 0; i < got; ++i) {
        buf_l[i] = (float)pop_scratch[2*i + 0] * INT16_RECIP * preamp_l;
        buf_r[i] = (float)pop_scratch[2*i + 1] * INT16_RECIP * preamp_r;
    }
    for (; i < AUDIO_FRAMES_HALF; ++i) {
        buf_l[i] = 0.0f;
        buf_r[i] = 0.0f;
    }

    /* === Stage 1.5: loudness compensation (volume-aware shelf cascade).
     *               Two SVF biquads per channel: low shelf (~50 Hz boost
     *               at low volumes) + high shelf (~10 kHz boost). Boost
     *               curve scales with how far below loudness_ref_spl the
     *               host volume is. Coefficients re-keyed in
     *               audio_set_volume() — current_loudness_coeffs points
     *               into the pre-computed 61×2 LUT. */
    bool loud_on = loudness_enabled;
    const LoudnessCoeffs *loud_coeffs = current_loudness_coeffs;
    if (loud_on && loud_coeffs) {
        for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
            float ml = buf_l[k];
            float mr = buf_r[k];
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; ++j) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                LoudnessSvfState *st = &loudness_state[0][j];
                float v3 = ml - st->ic2eq;
                float v1 = lc->sva1 * st->ic1eq + lc->sva2 * v3;
                float v2 = st->ic2eq + lc->sva2 * st->ic1eq + lc->sva3 * v3;
                st->ic1eq = 2.0f * v1 - st->ic1eq;
                st->ic2eq = 2.0f * v2 - st->ic2eq;
                ml = lc->svm0 * ml + lc->svm1 * v1 + lc->svm2 * v2;
            }
            for (int j = 0; j < LOUDNESS_BIQUAD_COUNT; ++j) {
                const LoudnessCoeffs *lc = &loud_coeffs[j];
                if (lc->bypass) continue;
                LoudnessSvfState *st = &loudness_state[1][j];
                float v3 = mr - st->ic2eq;
                float v1 = lc->sva1 * st->ic1eq + lc->sva2 * v3;
                float v2 = st->ic2eq + lc->sva2 * st->ic1eq + lc->sva3 * v3;
                st->ic1eq = 2.0f * v1 - st->ic1eq;
                st->ic2eq = 2.0f * v2 - st->ic2eq;
                mr = lc->svm0 * mr + lc->svm1 * v1 + lc->svm2 * v2;
            }
            buf_l[k] = ml;
            buf_r[k] = mr;
        }
    }

    /* === Stage 2: per-input EQ (block-based — uses dsp_process_channel
     *               -block which is faster than per-sample dispatch). */
    if (!eq_bypass) {
        dsp_process_channel_block(filters[0], buf_l, AUDIO_FRAMES_HALF, 0);
        dsp_process_channel_block(filters[1], buf_r, AUDIO_FRAMES_HALF, 1);
    }

    /* === Stage 3: crossfeed (per-sample API — fold into a tight loop). */
    if (cf_active) {
        for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
            crossfeed_process_stereo(&crossfeed_state, &buf_l[k], &buf_r[k]);
        }
    }

    /* === Stage 4: volume leveller (block-based, stereo-linked AGC). */
    if (lv_active) {
        leveller_process_block(&leveller_state, &leveller_coeffs,
                               (LevellerConfig *)&leveller_config,
                               buf_l, buf_r, AUDIO_FRAMES_HALF);
    }

    /* === Stage 5: matrix mixer → per-output buffers (4 stereo slots). */
    for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
        buf_o0[k] = buf_l[k] * g_l_to_o0 + buf_r[k] * g_r_to_o0;
        buf_o1[k] = buf_l[k] * g_l_to_o1 + buf_r[k] * g_r_to_o1;
        buf_o2[k] = buf_l[k] * g_l_to_o2 + buf_r[k] * g_r_to_o2;
        buf_o3[k] = buf_l[k] * g_l_to_o3 + buf_r[k] * g_r_to_o3;
    }

    /* === Stage 6: per-output EQ (block-based, channels 2..5). */
    if (!eq_bypass) {
        dsp_process_channel_block(filters[EQ_CH_OUT0], buf_o0,
                                  AUDIO_FRAMES_HALF, EQ_CH_OUT0);
        dsp_process_channel_block(filters[EQ_CH_OUT1], buf_o1,
                                  AUDIO_FRAMES_HALF, EQ_CH_OUT1);
        dsp_process_channel_block(filters[EQ_CH_OUT2], buf_o2,
                                  AUDIO_FRAMES_HALF, EQ_CH_OUT2);
        dsp_process_channel_block(filters[EQ_CH_OUT3], buf_o3,
                                  AUDIO_FRAMES_HALF, EQ_CH_OUT3);
    }

    /* === Stage 6.5: per-output delay (circular delay-line, mirrors RP
     *                ordering — runs AFTER per-output EQ but BEFORE the
     *                gain stage so latency-correction tracks the EQ-shaped
     *                signal). Each output has its own delay line; both
     *                start from the SAME delay_write_idx, so we snapshot
     *                it, run output 0 with widx0, output 1 with widx1, and
     *                advance the global index ONCE at the end (matching
     *                audio_pipeline.c's pattern). With dly==0 we skip
     *                entirely — leaves the previously-captured tail in
     *                the delay line for re-engagement; the buffer is
     *                large enough (MAX_DELAY_SAMPLES samples) that this
     *                is harmless. */
    if (any_delay_active) {
        /* Phase 1: extend per-output delay to 4 outputs. delay_lines[]
         * already has 9 entries (NUM_DELAY_CHANNELS), so no allocation
         * change. Each output starts from the SAME delay_write_idx; the
         * global index advances ONCE at the end so all four lines stay
         * sample-aligned. */
        float * const buf_outs[4] = { buf_o0, buf_o1, buf_o2, buf_o3 };
        for (int out = 0; out < 4; ++out) {
            int32_t dly = channel_delay_samples[out];
            if (dly <= 0) continue;
            float *dline = delay_lines[out];
            uint32_t widx = delay_write_idx;
            float *bo = buf_outs[out];
            for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
                dline[widx] = bo[k];
                bo[k]       = dline[(widx - dly) & MAX_DELAY_MASK];
                widx = (widx + 1) & MAX_DELAY_MASK;
            }
        }
        delay_write_idx = (delay_write_idx + AUDIO_FRAMES_HALF) & MAX_DELAY_MASK;
    }

    /* === Stage 7: per-output gain + master volume + clamp + 24-bit
     *               quantisation. Phase 1: 4 outputs, written into two
     *               ping-pong buffers (Out0/Out1 -> dst_a interleaved
     *               L+R, Out2/Out3 -> dst_b interleaved L+R). Peaks +
     *               clip detection folded into the same loop. */
    float pk_in_l = 0.0f, pk_in_r = 0.0f;
    float pk_o0 = 0.0f, pk_o1 = 0.0f, pk_o2 = 0.0f, pk_o3 = 0.0f;
    bool  clip0 = false, clip1 = false, clip2 = false, clip3 = false;

    for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
        /* Pre-gain "master/input" peaks (track buf_l/buf_r, post-EQ but
         * before output-routing — closest analogue to the RP master peak
         * so Console's "USB L/R" meter behaves the same). */
        float al = fabsf(buf_l[k]); if (al > pk_in_l) pk_in_l = al;
        float ar = fabsf(buf_r[k]); if (ar > pk_in_r) pk_in_r = ar;

        float o0 = buf_o0[k] * out0_post_gain * master;
        float o1 = buf_o1[k] * out1_post_gain * master;
        float o2 = buf_o2[k] * out2_post_gain * master;
        float o3 = buf_o3[k] * out3_post_gain * master;

        /* Clip detection BEFORE clamp (RP convention — flag if any sample
         * tried to exceed full-scale before the limiter hides it). */
        float ao0 = fabsf(o0); if (ao0 > pk_o0) pk_o0 = ao0;
        float ao1 = fabsf(o1); if (ao1 > pk_o1) pk_o1 = ao1;
        float ao2 = fabsf(o2); if (ao2 > pk_o2) pk_o2 = ao2;
        float ao3 = fabsf(o3); if (ao3 > pk_o3) pk_o3 = ao3;
        if (ao0 > CLIP_THRESH_F) clip0 = true;
        if (ao1 > CLIP_THRESH_F) clip1 = true;
        if (ao2 > CLIP_THRESH_F) clip2 = true;
        if (ao3 > CLIP_THRESH_F) clip3 = true;

        if (o0 >  1.0f) o0 =  1.0f; else if (o0 < -1.0f) o0 = -1.0f;
        if (o1 >  1.0f) o1 =  1.0f; else if (o1 < -1.0f) o1 = -1.0f;
        if (o2 >  1.0f) o2 =  1.0f; else if (o2 < -1.0f) o2 = -1.0f;
        if (o3 >  1.0f) o3 =  1.0f; else if (o3 < -1.0f) o3 = -1.0f;

        dst_a[2*k + 0] = (int32_t)(o0 * FLOAT_TO_24);
        dst_a[2*k + 1] = (int32_t)(o1 * FLOAT_TO_24);
        dst_b[2*k + 0] = (int32_t)(o2 * FLOAT_TO_24);
        dst_b[2*k + 1] = (int32_t)(o3 * FLOAT_TO_24);
    }

    /* Publish to global_status — converted u16 [0..32767]. clip_flags is a
     * sticky bitmask cleared by REQ_CLEAR_CLIPS so brief overshoots stay
     * visible until the user explicitly resets. */
    if (pk_in_l > 1.0f) pk_in_l = 1.0f;
    if (pk_in_r > 1.0f) pk_in_r = 1.0f;
    if (pk_o0   > 1.0f) pk_o0   = 1.0f;
    if (pk_o1   > 1.0f) pk_o1   = 1.0f;
    if (pk_o2   > 1.0f) pk_o2   = 1.0f;
    if (pk_o3   > 1.0f) pk_o3   = 1.0f;
    global_status.peaks[CH_MASTER_LEFT]  = (uint16_t)(pk_in_l * 32767.0f);
    global_status.peaks[CH_MASTER_RIGHT] = (uint16_t)(pk_in_r * 32767.0f);
    global_status.peaks[CH_OUT_1]        = (uint16_t)(pk_o0   * 32767.0f);
    global_status.peaks[CH_OUT_2]        = (uint16_t)(pk_o1   * 32767.0f);
    global_status.peaks[CH_OUT_3]        = (uint16_t)(pk_o2   * 32767.0f);
    global_status.peaks[CH_OUT_4]        = (uint16_t)(pk_o3   * 32767.0f);
    if (clip0) global_status.clip_flags |= (1u << CH_OUT_1);
    if (clip1) global_status.clip_flags |= (1u << CH_OUT_2);
    if (clip2) global_status.clip_flags |= (1u << CH_OUT_3);
    if (clip3) global_status.clip_flags |= (1u << CH_OUT_4);

    /* M7k: cycle-count delta for this call. The DWT counter is 32-bit
     * free-running at SYSCLK; the natural unsigned subtract handles
     * wrap correctly as long as the call took less than 2^32 cycles
     * (≈ 7.8 s at 550 MHz — never going to happen). */
    static uint32_t cpu_cycle_acc   = 0;
    static uint16_t cpu_block_count = 0;
    cpu_cycle_acc   += DWT->CYCCNT - cpu_t0;
    cpu_block_count += 1;
    if (cpu_block_count >= CPU_METER_BLOCKS) {
        /* avg cycles / call * 100 / budget = % load. Integer math:
         *   load_pct = cpu_cycle_acc * 100 / (CPU_METER_BLOCKS * budget).
         * Numerator fits in u32 because cpu_cycle_acc <= 128 * budget
         * (= 128 * 2.2 M = 281 M, well under 2^32). */
        uint32_t pct = (cpu_cycle_acc * 100U) /
                       (CPU_METER_BLOCKS * CPU_BUDGET_CYCLES_PER_HALF);
        if (pct > 100U) pct = 100U;
        global_status.cpu0_load = (uint8_t)pct;
        cpu_cycle_acc   = 0;
        cpu_block_count = 0;
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    /* Only A drives the fill — B's callback fires essentially
     * simultaneously (shared BCK/FS) and would just refill the same
     * halves redundantly. We refill BOTH ping-pong halves (A and B) in
     * one fill_half call so all 4 output channels stay sample-aligned
     * across the two SAI sub-blocks. */
    if (hsai == &hsai_BlockA1) {
        fill_half(&audio_buf_a[0],
                  &audio_buf_b[0]);
    }
    ++audio_dma_callbacks;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        fill_half(&audio_buf_a[AUDIO_FRAMES_HALF * 2],
                  &audio_buf_b[AUDIO_FRAMES_HALF * 2]);
    }
    ++audio_dma_callbacks;
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    (void)hsai;
}

/* ---------------------------------------------------------------------- */
/* DMA IRQ handlers                                                       */
/* ---------------------------------------------------------------------- */
void DMA1_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_sai1_a); }
void DMA1_Stream1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_sai1_b); }

/* ---------------------------------------------------------------------- */
/* Init helpers                                                           */
/* ---------------------------------------------------------------------- */
static void dma_init_common(DMA_HandleTypeDef *h, DMA_Stream_TypeDef *stream,
                            uint32_t request) {
    h->Instance                 = stream;
    h->Init.Request             = request;
    h->Init.Direction           = DMA_MEMORY_TO_PERIPH;
    h->Init.PeriphInc           = DMA_PINC_DISABLE;
    h->Init.MemInc              = DMA_MINC_ENABLE;
    h->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    h->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    h->Init.Mode                = DMA_CIRCULAR;
    h->Init.Priority            = DMA_PRIORITY_HIGH;
    h->Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    h->Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    h->Init.MemBurst            = DMA_MBURST_SINGLE;
    h->Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(h) != HAL_OK) Error_Handler();
}

/* ---------------------------------------------------------------------- */
/* Public init                                                            */
/* ---------------------------------------------------------------------- */
void Audio_Init(void) {
    /* RCC */
    __HAL_RCC_SAI1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* DMAMUX1 has no separate clock-enable on H7. */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* GPIO PE2/3/4/5/6 → AF6 (SAI1)
     *   PE2 = SAI1_MCLK_A   (sub-block A)
     *   PE3 = SAI1_SD_B     (sub-block B — internal slave to A)
     *   PE4 = SAI1_FS_A
     *   PE5 = SAI1_SCK_A
     *   PE6 = SAI1_SD_A
     */
    GPIO_InitTypeDef g = {
        .Pin       = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_NOPULL,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF6_SAI1,
    };
    HAL_GPIO_Init(GPIOE, &g);

    /* DMA1 Stream 0 → SAI1_A. Stream 1 → SAI1_B. The LINKDMA calls happen
     * later, AFTER the SAI handles are configured (B's hsai struct is a
     * post-init clone of A's, which copies A's hdmatx pointer — must be
     * re-linked to its own DMA stream after the clone or both blocks
     * end up pointing at A's stream and B's data never moves). */
    dma_init_common(&hdma_sai1_a, DMA1_Stream0, DMA_REQUEST_SAI1_A);
    dma_init_common(&hdma_sai1_b, DMA1_Stream1, DMA_REQUEST_SAI1_B);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    /* SAI1 Block A — I2S Philips master TX, 24-bit / 32-bit slot, 48 kHz.
     * Generates BCK and FS that B (configured below) reads internally. */
    hsai_BlockA1.Instance              = SAI1_Block_A;
    hsai_BlockA1.Init.AudioMode        = SAI_MODEMASTER_TX;
    hsai_BlockA1.Init.Synchro          = SAI_ASYNCHRONOUS;
    hsai_BlockA1.Init.OutputDrive      = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockA1.Init.NoDivider        = SAI_MASTERDIVIDER_ENABLE;
    hsai_BlockA1.Init.FIFOThreshold    = SAI_FIFOTHRESHOLD_HF;
    hsai_BlockA1.Init.AudioFrequency   = SAI_AUDIO_FREQUENCY_48K;
    hsai_BlockA1.Init.SynchroExt       = SAI_SYNCEXT_DISABLE;
    hsai_BlockA1.Init.MonoStereoMode   = SAI_STEREOMODE;
    hsai_BlockA1.Init.CompandingMode   = SAI_NOCOMPANDING;
    hsai_BlockA1.Init.TriState         = SAI_OUTPUT_NOTRELEASED;
    hsai_BlockA1.Init.Mckdiv           = 0;
    hsai_BlockA1.Init.MckOverSampling  = SAI_MCK_OVERSAMPLING_DISABLE;
    hsai_BlockA1.Init.MckOutput        = SAI_MCK_OUTPUT_ENABLE;
    if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD,
                             SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&hsai_BlockA1, hdmatx, hdma_sai1_a);

    /* SAI1 Block B — clone A's config wholesale, then override only what
     * MUST differ. Cloning ensures identical NoDivider / MckOverSampling
     * / FrameInit / SlotInit values so B's internal slot timing matches
     * A's bit-for-bit; an earlier draft set NoDivider=DISABLE on B to
     * "skip the master divider" and got a distorted output (RM0468
     * §34.6 NODIV=1 changes the bit-clock formula even for synchronous
     * sub-blocks, breaking slot timing). The only differences B truly
     * needs from A are:
     *   - Instance       : the other sub-block
     *   - Synchro        : SYNCHRONOUS instead of ASYNCHRONOUS
     *   - MckOutput      : DISABLE (no MCLK pin bonded to PE3) */
    hsai_BlockB1                   = hsai_BlockA1;  /* whole-struct clone */
    hsai_BlockB1.Instance          = SAI1_Block_B;
    /* AudioMode = SLAVE_TX, NOT MASTER_TX. A SYNCHRONOUS sub-block takes
     * SCK and FS from its sister via the on-chip sync path, so from the
     * clock-generator perspective it is a slave. MASTER_TX with
     * SYNCHRONOUS sets MODE=00 in CR1 (own clock generator) while also
     * setting SYNCEN=01 (use sister clock) — the generator and sync
     * logic fight and the slot timing comes out scrambled, audible as
     * the same kind of distortion the M4 `s << 8` bug produced. */
    hsai_BlockB1.Init.AudioMode    = SAI_MODESLAVE_TX;
    hsai_BlockB1.Init.Synchro      = SAI_SYNCHRONOUS;
    hsai_BlockB1.Init.MckOutput    = SAI_MCK_OUTPUT_DISABLE;
    /* Reset HAL state fields the struct copy carried over from A so
     * HAL_SAI_Init treats this as a fresh sub-block, not a re-init. */
    hsai_BlockB1.Lock              = HAL_UNLOCKED;
    hsai_BlockB1.State             = HAL_SAI_STATE_RESET;
    hsai_BlockB1.ErrorCode         = HAL_SAI_ERROR_NONE;
    if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD,
                             SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK) {
        Error_Handler();
    }
    /* CRITICAL: re-link B's DMA after the clone, otherwise B's hdmatx
     * still points at A's stream and B's buffer never flows. */
    __HAL_LINKDMA(&hsai_BlockB1, hdmatx, hdma_sai1_b);

    /* Pre-fill both buffers with silence so the first BCK edges after
     * Audio_Start ship zero samples until USB starts delivering. */
    for (uint32_t i = 0; i < AUDIO_WORDS_TOTAL; ++i) {
        audio_buf_a[i] = 0;
        audio_buf_b[i] = 0;
    }

    /* M7d: init crossfeed + leveller state + initial coefficients.
     * Default configs are disabled, so the coefficients only take
     * effect once the user enables them in Console. */
    crossfeed_init(&crossfeed_state);
    crossfeed_compute_coefficients(&crossfeed_state,
                                    (CrossfeedConfig *)&crossfeed_config,
                                    48000.0f);
    leveller_reset_state(&leveller_state);
    leveller_compute_coefficients(&leveller_coeffs,
                                  (LevellerConfig *)&leveller_config,
                                  48000.0f);
}

void Audio_Start(void) {
    /* Phase 1: each sub-block streams from its OWN AXI SRAM ring.
     * SAI1_A drains audio_buf_a (Out0/Out1 stereo on PE6); SAI1_B drains
     * audio_buf_b (Out2/Out3 stereo on PE3). Two DMAs targeting separate
     * SRAM addresses — no contention beyond AXI bus bandwidth (trivial
     * at 6 MB/s combined).
     *
     * Start the synchronous slave (B) FIRST so its DMA + SAIEN are
     * armed and waiting on A's BCK/FS edges. Then start A — the moment
     * A's SAIEN goes high, BCK/FS begin toggling and B shifts out its
     * first sample on the same edge as A's, giving sample-aligned
     * output across both pin pairs. */
    if (HAL_SAI_Transmit_DMA(&hsai_BlockB1,
                             (uint8_t *)audio_buf_b,
                             AUDIO_WORDS_TOTAL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA1,
                             (uint8_t *)audio_buf_a,
                             AUDIO_WORDS_TOTAL) != HAL_OK) {
        Error_Handler();
    }
}
