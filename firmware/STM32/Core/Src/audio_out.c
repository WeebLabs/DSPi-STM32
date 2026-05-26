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
#include "audio_input.h"   /* M9: active_input_source */
#include "spdif_input.h"   /* M9: spdif_input_pop_frames */
#include "notify.h"        /* M12 P4: notify_param_write on coerce */
#include "bulk_params.h"   /* M12 P4: WireBulkParams offsetof for notify */
#include <math.h>          /* M7e: fabsf for level meters */
#include <stddef.h>        /* offsetof */

/* ---- Per-sub-block DMA rings in AXI SRAM (DMA1 cannot reach DTCM) ----
 *
 * Phase 1 (4 independent stereo slots): SAI1_A and SAI1_B each get their
 * own ping-pong buffer so the two physical pin pairs can carry DIFFERENT
 * stereo signals. The earlier build had both DMAs reading the same
 * audio_buf, which collapsed PE6 and PE3 to identical data.
 *
 * Memory layout:
 *
 *   AXI SRAM (D1 domain, DMA1-reachable, 320 KB at 0x24000000):
 *     0x24000000  audio_buf_A (3 KB) — SAI1_A / Out0+1 (slot 0)
 *     0x24001000  audio_buf_B (3 KB) — SAI1_B / Out2+3 (slot 1)
 *     0x24004000  flash_mirror (48 KB) — preset region (M11)
 *     0x24010000  DSP scratch (10 × 768 B = 7.5 KB) — buf_l..buf_o7
 *
 *   SRAM4 (D3 domain, BDMA-reachable, 16 KB at 0x38000000):
 *     0x38000000  audio_buf_C (3 KB) — SAI4_A / Out4+5 (slot 2)
 *     0x38001000  audio_buf_D (3 KB) — SAI4_B / Out6+7 (slot 3)
 *
 * SAI4 lives in the D3 power domain and its DMA is BDMA (not DMA1).
 * BDMA can only reach SRAM4, so slots 2/3's ping-pong rings sit there
 * instead of AXI SRAM. The AXI bus matrix on H7 doesn't bridge BDMA to
 * the D1 SRAM region, so a placement mistake fails silently with an
 * empty buffer. */
#define AUDIO_BUFFER_BASE_A  0x24000000UL
#define AUDIO_BUFFER_BASE_B  0x24001000UL
#define AUDIO_BUFFER_BASE_C  0x38000000UL  /* SRAM4 — BDMA-only */
#define AUDIO_BUFFER_BASE_D  0x38001000UL  /* MUST be ≥ C + 0xC00.
                                            * Earlier 0x38000800 stride
                                            * (2 KB) overlapped C's tail
                                            * 1024 bytes deep into D's
                                            * head — slots 2 + 3 ate
                                            * each other's samples.
                                            * 4 KB stride matches AXI
                                            * SRAM (A/B at 0x*0000 /
                                            * 0x*1000) — keeps the
                                            * mental model consistent. */

/* 192 stereo frames per ping-pong half × 2 halves × 2 ch = 768 int32 words.
 * 192 frames @ 48 kHz = 4 ms per half — comfortably above any HAL ISR
 * latency. */
#define AUDIO_FRAMES_HALF    192U
#define AUDIO_FRAMES_TOTAL   (AUDIO_FRAMES_HALF * 2)
#define AUDIO_WORDS_TOTAL    (AUDIO_FRAMES_TOTAL * 2)  /* L+R */

static int32_t * const audio_buf_a = (int32_t *)AUDIO_BUFFER_BASE_A;
static int32_t * const audio_buf_b = (int32_t *)AUDIO_BUFFER_BASE_B;
static int32_t * const audio_buf_c = (int32_t *)AUDIO_BUFFER_BASE_C;
static int32_t * const audio_buf_d = (int32_t *)AUDIO_BUFFER_BASE_D;

/* Scratch for one half's worth of stereo 16-bit samples popped from the
 * ring; converted to 24-bit-right-aligned in place into audio_buf[]. */
static int16_t pop_scratch[AUDIO_FRAMES_HALF * 2];

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
SAI_HandleTypeDef hsai_BlockA4;     /* phase 2: SAI4 sub-block A — slot 2.
                                     * H723 has SAI1 + SAI4 (no SAI2/3) —
                                     * SAI4 is in the D3 power domain and
                                     * its DMA is BDMA, not DMA1. */
SAI_HandleTypeDef hsai_BlockB4;     /* phase 2: SAI4 sub-block B — slot 3 */
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
DMA_HandleTypeDef hdma_sai4_a;      /* DMA_HandleTypeDef shared between
                                     * DMA1/2 (D2 domain) and BDMA (D3) —
                                     * Instance pointer disambiguates. */
DMA_HandleTypeDef hdma_sai4_b;

volatile uint32_t audio_dma_callbacks = 0;
volatile uint32_t audio_underruns      = 0;
volatile uint32_t audio_fill_peak_cycles = 0;

/* Per-stage cycle counters for diagnosing CPU anomalies. Each is an
 * accumulator across fill_half calls; reset by the probe (vendor cmd
 * 0xEC) so each read reports cycles-per-stage since the last read.
 * Indexes match the stages defined just above the cycle-capture
 * macros in fill_half. */
#define STAGE_COUNT 8
volatile uint32_t audio_stage_cycles[STAGE_COUNT];
volatile uint32_t audio_stage_calls;     /* fill_half calls since last reset */
/* CPU budget = cycles available per fill_half call. Exposed so the
 * servo-debug probe can convert peak cycles into a % of budget. */
extern const uint32_t audio_fill_budget_cycles;

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

/* Defined here for reference by the M9 servo-debug probe (vendor 0xF7),
 * so the probe can show "peak cycles / budget = % budget consumed". */
const uint32_t audio_fill_budget_cycles = CPU_BUDGET_CYCLES_PER_HALF;

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
#define DSP_SCRATCH_BASE  0x24010000UL
static float * const buf_l  = (float *)(DSP_SCRATCH_BASE + 0 * 0x300);
static float * const buf_r  = (float *)(DSP_SCRATCH_BASE + 1 * 0x300);
static float * const buf_o0 = (float *)(DSP_SCRATCH_BASE + 2 * 0x300);
static float * const buf_o1 = (float *)(DSP_SCRATCH_BASE + 3 * 0x300);
static float * const buf_o2 = (float *)(DSP_SCRATCH_BASE + 4 * 0x300);
static float * const buf_o3 = (float *)(DSP_SCRATCH_BASE + 5 * 0x300);
/* Phase 2: 4 more output scratch buffers for SAI2 (Out4..7). Same
 * 0x300-byte stride. Block ends at 0x24002C00 + 0x300 = 0x24002F00,
 * still well below audio_buf_C at 0x24003000. */
static float * const buf_o4 = (float *)(DSP_SCRATCH_BASE + 6 * 0x300);
static float * const buf_o5 = (float *)(DSP_SCRATCH_BASE + 7 * 0x300);
static float * const buf_o6 = (float *)(DSP_SCRATCH_BASE + 8 * 0x300);
static float * const buf_o7 = (float *)(DSP_SCRATCH_BASE + 9 * 0x300);

/* Per-output EQ channel index: Out0 → CH 2, Out1 → CH 3, Out2 → CH 4,
 * Out3 → CH 5. (CH_OUT_n already exist as global config defines but
 * they're 1-indexed-by-pin in the project naming.) */
#define EQ_CH_OUT0    2
#define EQ_CH_OUT1    3
#define EQ_CH_OUT2    4
#define EQ_CH_OUT3    5

static void fill_half(int32_t *dst_a, int32_t *dst_b,
                      int32_t *dst_c, int32_t *dst_d) {
    /* M7k: snap cycle count at entry; computed at exit and accumulated
     * into a per-window total that gets converted to a % every
     * CPU_METER_BLOCKS calls. Single MRC, ~1 cycle. */
    uint32_t cpu_t0 = DWT->CYCCNT;

    /* Per-stage timing scaffolding. Each stage captures _ts at start,
     * accumulates (CYCCNT - _ts) into audio_stage_cycles[stage_idx]
     * at end. Macros keep the inline code tidy; total overhead per
     * stage is ~4 cycles (two CYCCNT reads + subtraction + add). */
    audio_stage_calls++;
    uint32_t stage_ts = cpu_t0;
    #define STAGE_END(idx) do { \
        uint32_t now = DWT->CYCCNT; \
        audio_stage_cycles[idx] += (now - stage_ts); \
        stage_ts = now; \
    } while (0)

    /* M9: source samples from the active input. SPDIF path drains
     * the SPDIFRX demux ring; USB path drains the UAC1 OUT ring. The
     * downstream DSP graph is identical either way — same int16
     * stereo format, same fill convention (silence-pad on shortfall),
     * same starvation accounting. */
    uint32_t got;
    if (active_input_source == INPUT_SOURCE_SPDIF) {
        got = spdif_input_pop_frames(pop_scratch, AUDIO_FRAMES_HALF);
    } else {
        got = usb_ring_pop_frames(pop_scratch, AUDIO_FRAMES_HALF);
    }
    if (got < AUDIO_FRAMES_HALF) ++audio_underruns;
    STAGE_END(0);   /* stage 0: input pop (USB ring or SPDIF resampler) */

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
    float g_l_to_o4 = XP_GAIN(0, 4);
    float g_r_to_o4 = XP_GAIN(1, 4);
    float g_l_to_o5 = XP_GAIN(0, 5);
    float g_r_to_o5 = XP_GAIN(1, 5);
    float g_l_to_o6 = XP_GAIN(0, 6);
    float g_r_to_o6 = XP_GAIN(1, 6);
    float g_l_to_o7 = XP_GAIN(0, 7);
    float g_r_to_o7 = XP_GAIN(1, 7);
    #undef XP_GAIN

    #define POST_GAIN(out) \
        ((matrix_mixer.outputs[(out)].enabled && !matrix_mixer.outputs[(out)].mute) \
            ? matrix_mixer.outputs[(out)].gain_linear : 0.0f)
    float out0_post_gain = POST_GAIN(0);
    float out1_post_gain = POST_GAIN(1);
    float out2_post_gain = POST_GAIN(2);
    float out3_post_gain = POST_GAIN(3);
    float out4_post_gain = POST_GAIN(4);
    float out5_post_gain = POST_GAIN(5);
    float out6_post_gain = POST_GAIN(6);
    float out7_post_gain = POST_GAIN(7);
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

    STAGE_END(1);   /* stage 1: preamp / matrix snapshot / param prep */

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

    STAGE_END(2);   /* stage 2: loudness shelves */

    /* === Stage 2: per-input EQ (block-based — uses dsp_process_channel
     *               -block which is faster than per-sample dispatch). */
    if (!eq_bypass) {
        dsp_process_channel_block(filters[0], buf_l, AUDIO_FRAMES_HALF, 0);
        dsp_process_channel_block(filters[1], buf_r, AUDIO_FRAMES_HALF, 1);
    }

    STAGE_END(3);   /* stage 3: per-input PEQ (channels 0, 1) */

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

    STAGE_END(4);   /* stage 4: crossfeed + leveller */

    /* === Stage 5: matrix mixer → per-output buffers (8 outputs / 4
     * stereo slots). */
    for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
        float il = buf_l[k];
        float ir = buf_r[k];
        buf_o0[k] = il * g_l_to_o0 + ir * g_r_to_o0;
        buf_o1[k] = il * g_l_to_o1 + ir * g_r_to_o1;
        buf_o2[k] = il * g_l_to_o2 + ir * g_r_to_o2;
        buf_o3[k] = il * g_l_to_o3 + ir * g_r_to_o3;
        buf_o4[k] = il * g_l_to_o4 + ir * g_r_to_o4;
        buf_o5[k] = il * g_l_to_o5 + ir * g_r_to_o5;
        buf_o6[k] = il * g_l_to_o6 + ir * g_r_to_o6;
        buf_o7[k] = il * g_l_to_o7 + ir * g_r_to_o7;
    }

    STAGE_END(5);   /* stage 5: matrix mixer */

    /* === Stage 6: per-output EQ (block-based, channels 2..9). */
    if (!eq_bypass) {
        dsp_process_channel_block(filters[2], buf_o0, AUDIO_FRAMES_HALF, 2);
        dsp_process_channel_block(filters[3], buf_o1, AUDIO_FRAMES_HALF, 3);
        dsp_process_channel_block(filters[4], buf_o2, AUDIO_FRAMES_HALF, 4);
        dsp_process_channel_block(filters[5], buf_o3, AUDIO_FRAMES_HALF, 5);
        dsp_process_channel_block(filters[6], buf_o4, AUDIO_FRAMES_HALF, 6);
        dsp_process_channel_block(filters[7], buf_o5, AUDIO_FRAMES_HALF, 7);
        dsp_process_channel_block(filters[8], buf_o6, AUDIO_FRAMES_HALF, 8);
        dsp_process_channel_block(filters[9], buf_o7, AUDIO_FRAMES_HALF, 9);
    }

    STAGE_END(6);   /* stage 6: per-output PEQ (channels 2..9) */

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
        /* Phase 2: 8 outputs (was 4). delay_lines[] already has 9 entries
         * (NUM_DELAY_CHANNELS), so no allocation change. Each output
         * starts from the SAME delay_write_idx; the global index
         * advances ONCE at the end so all 8 lines stay sample-aligned. */
        float * const buf_outs[8] = {
            buf_o0, buf_o1, buf_o2, buf_o3,
            buf_o4, buf_o5, buf_o6, buf_o7,
        };
        for (int out = 0; out < 8; ++out) {
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
     *               quantisation. Phase 2: 8 outputs across 4 stereo
     *               slots, each slot interleaves L+R into its own DMA
     *               half (dst_a..dst_d). Peaks + clip detection folded
     *               into the same loop. */
    float pk_in_l = 0.0f, pk_in_r = 0.0f;
    float pk_o[8] = { 0 };
    bool  clip[8] = { false };

    for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
        /* Pre-gain "master/input" peaks (track buf_l/buf_r, post-EQ but
         * before output-routing — closest analogue to the RP master peak
         * so Console's "USB L/R" meter behaves the same). */
        float al = fabsf(buf_l[k]); if (al > pk_in_l) pk_in_l = al;
        float ar = fabsf(buf_r[k]); if (ar > pk_in_r) pk_in_r = ar;

        float o[8] = {
            buf_o0[k] * out0_post_gain * master,
            buf_o1[k] * out1_post_gain * master,
            buf_o2[k] * out2_post_gain * master,
            buf_o3[k] * out3_post_gain * master,
            buf_o4[k] * out4_post_gain * master,
            buf_o5[k] * out5_post_gain * master,
            buf_o6[k] * out6_post_gain * master,
            buf_o7[k] * out7_post_gain * master,
        };

        for (int i = 0; i < 8; ++i) {
            /* Clip detection BEFORE clamp (RP convention — flag if any
             * sample tried to exceed full-scale before the limiter
             * hides it). */
            float a = fabsf(o[i]);
            if (a > pk_o[i]) pk_o[i] = a;
            if (a > CLIP_THRESH_F) clip[i] = true;
            if (o[i] >  1.0f) o[i] =  1.0f;
            else if (o[i] < -1.0f) o[i] = -1.0f;
        }

        /* Mask to 24 bits — for I2S those upper bits are zero-fill
         * (SAI's DataSize=24 ignores DR[31:24]), but in SPDIF mode the
         * SAI hardware reads DR[24]=V, [25]=U, [26]=C as IEC 60958
         * metadata bits. Letting int32_t sign-extension propagate
         * `1` into those bits on negative samples flags every negative
         * sample as Invalid (V=1) and randomises U/C — receiver
         * decodes audio that's audibly distorted on the negative half
         * of every cycle. The mask preserves 24-bit 2's-complement so
         * negative samples decode correctly (e.g. -1 = 0xFFFFFF in
         * 24-bit signed) while clearing V/U/C to zero (valid, no
         * user data, default channel-status block of all zeros which
         * consumer receivers tolerate). P (parity, DR[27]) is auto-
         * generated by the SAI hardware regardless. */
        dst_a[2*k + 0] = (int32_t)(o[0] * FLOAT_TO_24) & 0x00FFFFFF;
        dst_a[2*k + 1] = (int32_t)(o[1] * FLOAT_TO_24) & 0x00FFFFFF;
        dst_b[2*k + 0] = (int32_t)(o[2] * FLOAT_TO_24) & 0x00FFFFFF;
        dst_b[2*k + 1] = (int32_t)(o[3] * FLOAT_TO_24) & 0x00FFFFFF;
        dst_c[2*k + 0] = (int32_t)(o[4] * FLOAT_TO_24) & 0x00FFFFFF;
        dst_c[2*k + 1] = (int32_t)(o[5] * FLOAT_TO_24) & 0x00FFFFFF;
        dst_d[2*k + 0] = (int32_t)(o[6] * FLOAT_TO_24) & 0x00FFFFFF;
        dst_d[2*k + 1] = (int32_t)(o[7] * FLOAT_TO_24) & 0x00FFFFFF;
    }

    /* Publish to global_status — converted u16 [0..32767]. clip_flags is
     * a sticky bitmask cleared by REQ_CLEAR_CLIPS so brief overshoots
     * stay visible until the user explicitly resets. */
    if (pk_in_l > 1.0f) pk_in_l = 1.0f;
    if (pk_in_r > 1.0f) pk_in_r = 1.0f;
    global_status.peaks[CH_MASTER_LEFT]  = (uint16_t)(pk_in_l * 32767.0f);
    global_status.peaks[CH_MASTER_RIGHT] = (uint16_t)(pk_in_r * 32767.0f);
    for (int i = 0; i < 8; ++i) {
        if (pk_o[i] > 1.0f) pk_o[i] = 1.0f;
        global_status.peaks[CH_OUT_1 + i] = (uint16_t)(pk_o[i] * 32767.0f);
        if (clip[i]) global_status.clip_flags |= (1u << (CH_OUT_1 + i));
    }

    /* M7k: cycle-count delta for this call. The DWT counter is 32-bit
     * free-running at SYSCLK; the natural unsigned subtract handles
     * wrap correctly as long as the call took less than 2^32 cycles
     * (≈ 7.8 s at 550 MHz — never going to happen). */
    static uint32_t cpu_cycle_acc   = 0;
    static uint16_t cpu_block_count = 0;
    STAGE_END(7);   /* stage 7: delay + output gain + format convert */
    #undef STAGE_END

    uint32_t cpu_delta = DWT->CYCCNT - cpu_t0;
    cpu_cycle_acc   += cpu_delta;
    cpu_block_count += 1;
    /* Track peak cycles per fill_half across the whole CPU-meter window
     * so we can spot spikes that exceed the 4 ms budget even when the
     * average load looks fine. Exposed via vendor cmd 0xF7 (servo debug
     * packet). Reset to 0 at the end of each averaging window so each
     * report covers a fresh ~0.5 s slice. */
    extern volatile uint32_t audio_fill_peak_cycles;
    if (cpu_delta > audio_fill_peak_cycles) {
        audio_fill_peak_cycles = cpu_delta;
    }
    if (cpu_block_count >= CPU_METER_BLOCKS) {
        /* avg cycles / call * 100 / budget = % load. The intermediate
         * `cpu_cycle_acc * 100` overflows u32 once average-per-call
         * cycles exceed ~336k (= 15% of the 2.2M-cycle budget). Without
         * the uint64 promotion below the wrap produces nonsense
         * percentages that move OPPOSITE to actual CPU. Promote and
         * the math stays honest up to 100%. */
        uint32_t pct = (uint32_t)(((uint64_t)cpu_cycle_acc * 100U) /
                                  (CPU_METER_BLOCKS * CPU_BUDGET_CYCLES_PER_HALF));
        if (pct > 100U) pct = 100U;
        global_status.cpu0_load = (uint8_t)pct;
        cpu_cycle_acc   = 0;
        cpu_block_count = 0;
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    /* Only SAI1_A drives the fill — the other three sub-blocks fire
     * essentially simultaneously (shared BCK/FS via cross-peripheral
     * SyncExt) and would just refill the same halves redundantly. One
     * fill_half call refreshes ALL four ping-pong A-halves so all 8
     * output channels stay sample-aligned across both SAI peripherals. */
    if (hsai == &hsai_BlockA1) {
        fill_half(&audio_buf_a[0],
                  &audio_buf_b[0],
                  &audio_buf_c[0],
                  &audio_buf_d[0]);
    }
    ++audio_dma_callbacks;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        fill_half(&audio_buf_a[AUDIO_FRAMES_HALF * 2],
                  &audio_buf_b[AUDIO_FRAMES_HALF * 2],
                  &audio_buf_c[AUDIO_FRAMES_HALF * 2],
                  &audio_buf_d[AUDIO_FRAMES_HALF * 2]);
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
void BDMA_Channel0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_sai4_a); }
void BDMA_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_sai4_b); }

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

/* BDMA shares HAL_DMA_Init / DMA_HandleTypeDef but rejects FIFO/burst
 * fields and uses different request constants. Same Direction/Mode/etc.
 * The Instance points at a BDMA_Channel rather than a DMA_Stream. */
static void bdma_init_common(DMA_HandleTypeDef *h, BDMA_Channel_TypeDef *ch,
                             uint32_t request) {
    h->Instance                 = (DMA_Stream_TypeDef *)ch;  /* HAL casts internally */
    h->Init.Request             = request;
    h->Init.Direction           = DMA_MEMORY_TO_PERIPH;
    h->Init.PeriphInc           = DMA_PINC_DISABLE;
    h->Init.MemInc              = DMA_MINC_ENABLE;
    h->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    h->Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    h->Init.Mode                = DMA_CIRCULAR;
    h->Init.Priority            = DMA_PRIORITY_HIGH;
    h->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;   /* no FIFO on BDMA */
    if (HAL_DMA_Init(h) != HAL_OK) Error_Handler();
}

/* ---------------------------------------------------------------------- */
/* output_types[] dependency rules                                        */
/* ---------------------------------------------------------------------- */
/* Hardware constraints baked into our pin layout on the WeAct H723VGT6:
 *
 *   PE2 = SAI1_MCLK_A   (the only MCLK pin AF-muxed)
 *   PE5 = SAI1_SCK_A    (the only BCK pin AF-muxed)
 *   PE4 = SAI1_FS_A     (the only LRCLK pin AF-muxed)
 *   PE6 = SAI1_SD_A     slot 0
 *   PE3 = SAI1_SD_B     slot 1
 *   PD11 = SAI4_SD_A    slot 2
 *   PA0  = SAI4_SD_B    slot 3
 *
 * The shared MCLK/BCK/LRCLK pins are SAI1_A's. Any slot configured for
 * I2S needs those clocks present on PE5/PE4 (and optionally PE2). They
 * are present iff SAI1_A is itself in I2S mode — in SPDIF mode the
 * SAI clock generator runs at the biphase rate and the FS/SCK pin
 * outputs do not carry valid I2S timing.
 *
 * Cross-block sync paths (the only way slots 1/2/3 can use SAI1_A's
 * I2S clocks):
 *   slot 1 → SAI1_B, can only sync-internal to SAI1_A within the SAI1
 *            peripheral. No AF-muxed clock pins of its own.
 *   slot 2 → SAI4_A, can only sync-external to SAI1 via the cross-
 *            peripheral mesh. PD12/PD13 (SAI4_FS_A/SCK_A) NOT AF-muxed.
 *   slot 3 → SAI4_B, can only sync-internal to SAI4_A. No clock pins
 *            of its own.
 *
 * Therefore an I2S slot N requires its "clock parent" to also be I2S:
 *   slot 1 I2S → slot 0 must be I2S (sync-internal to SAI1_A)
 *   slot 2 I2S → slot 0 must be I2S (sync-ext to SAI1)
 *   slot 3 I2S → slot 2 must be I2S (sync-internal to SAI4_A)
 *
 * SPDIF slots have no parent dependency: they self-clock from PLL2_P,
 * embed the recovered clock in the biphase output, and don't share
 * pins with anything else.
 *
 * sanitize_output_types coerces invalid I2S choices DOWN to SPDIF
 * (cascade-down) so audio_configure_sais always sees a valid set.
 * Returns a 4-bit mask of slots that were coerced — the caller fires
 * notify_param_write for each so Console reflects the auto-change. */
static uint8_t sanitize_output_types(void) {
    extern uint8_t output_types[];
    uint8_t coerced = 0;
    /* slot 1 depends on slot 0; slot 2 depends on slot 0; slot 3
     * depends on slot 2. Apply rules in dependency order so a single
     * pass settles. */
    if (output_types[1] == OUTPUT_TYPE_I2S
        && output_types[0] == OUTPUT_TYPE_SPDIF) {
        output_types[1] = OUTPUT_TYPE_SPDIF;
        coerced |= (1u << 1);
    }
    if (output_types[2] == OUTPUT_TYPE_I2S
        && output_types[0] == OUTPUT_TYPE_SPDIF) {
        output_types[2] = OUTPUT_TYPE_SPDIF;
        coerced |= (1u << 2);
    }
    /* slot 3's parent is slot 2, which we may have just coerced. */
    if (output_types[3] == OUTPUT_TYPE_I2S
        && output_types[2] == OUTPUT_TYPE_SPDIF) {
        output_types[3] = OUTPUT_TYPE_SPDIF;
        coerced |= (1u << 3);
    }
    return coerced;
}

/* ---------------------------------------------------------------------- */
/* SAI sub-block (re)configuration — type-driven                          */
/* ---------------------------------------------------------------------- */
/* Extracted from the original Audio_Init body so Phase 4 can call it
 * twice — once at boot and again whenever a slot's output type changes
 * at runtime. Honours the current `output_types[]` array and applies
 * the all-I2S-vs-mixed sync rule from Phase 3. Caller is responsible
 * for ensuring all four sub-blocks are in HAL_SAI_STATE_RESET (i.e.,
 * teardown before re-configure). */
static void audio_configure_sais(void) {
    extern uint8_t output_types[];
    bool s0_spdif  = output_types[0] == OUTPUT_TYPE_SPDIF;
    bool s1_spdif  = output_types[1] == OUTPUT_TYPE_SPDIF;
    bool s2_spdif  = output_types[2] == OUTPUT_TYPE_SPDIF;
    bool s3_spdif  = output_types[3] == OUTPUT_TYPE_SPDIF;
    /* Per-slot sync rule (post-sanitize): an I2S child syncs to its
     * I2S parent. SPDIF slots are always their own master.
     *   slot 1 I2S → sync-internal to SAI1_A
     *   slot 2 I2S → sync-ext to SAI1
     *   slot 3 I2S → sync-internal to SAI4_A
     * sanitize_output_types guarantees that whenever the child is I2S,
     * the parent is also I2S, so these always make sense. */
    bool s1_sync_to_a1 = !s1_spdif;
    bool s2_sync_to_s1 = !s2_spdif;
    bool s3_sync_to_a4 = !s3_spdif;
    /* SAI1_A broadcasts on the cross-peripheral mesh whenever slot 0
     * is I2S — that's the path SAI4_A subscribes to when slot 2 is
     * also I2S. Always-on when slot 0 is I2S is harmless and saves
     * the configure logic from caring about subscribers. */
    bool a1_broadcast = !s0_spdif;

    /* Common shape for every sub-block — fields that don't depend on
     * type or sync role. Per-block init starts with this and then
     * overrides Instance / Protocol / Mode / Sync / NoDivider. */
    SAI_HandleTypeDef hsai_template = { 0 };
    hsai_template.Init.AudioMode       = SAI_MODEMASTER_TX;
    hsai_template.Init.Synchro         = SAI_ASYNCHRONOUS;
    hsai_template.Init.OutputDrive     = SAI_OUTPUTDRIVE_DISABLE;
    hsai_template.Init.NoDivider       = SAI_MASTERDIVIDER_ENABLE;
    hsai_template.Init.FIFOThreshold   = SAI_FIFOTHRESHOLD_HF;
    hsai_template.Init.AudioFrequency  = SAI_AUDIO_FREQUENCY_48K;
    hsai_template.Init.SynchroExt      = SAI_SYNCEXT_DISABLE;
    hsai_template.Init.MonoStereoMode  = SAI_STEREOMODE;
    hsai_template.Init.CompandingMode  = SAI_NOCOMPANDING;
    hsai_template.Init.TriState        = SAI_OUTPUT_NOTRELEASED;
    hsai_template.Init.Mckdiv          = 0;
    hsai_template.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
    hsai_template.Init.MckOutput       = SAI_MCK_OUTPUT_ENABLE;

    /* I2S: HAL_SAI_InitProtocol fills in FrameInit/SlotInit from the
     * I2S Philips standard with the requested DataSize and slot count.
     * SPDIF: HAL_SAI_InitProtocol(SAI_SPDIF_PROTOCOL) is **not**
     * implemented — its switch returns HAL_ERROR for SPDIF. We set
     * the SPDIF-specific fields directly on the handle and call
     * HAL_SAI_Init, which is the level that actually programs CR1/
     * CR2/FRCR/SLOTR. Field values match RM0468 §41.6.5. */
    #define INIT_BLOCK_PROTO(handle, is_spdif)                                  \
        do {                                                                    \
            if (is_spdif) {                                                     \
                (handle).Init.Protocol     = SAI_SPDIF_PROTOCOL;                \
                (handle).Init.DataSize     = SAI_DATASIZE_24;                   \
                (handle).Init.FirstBit     = SAI_FIRSTBIT_MSB;                  \
                (handle).Init.ClockStrobing= SAI_CLOCKSTROBING_FALLINGEDGE;     \
                (handle).FrameInit.FrameLength       = 64U;                     \
                (handle).FrameInit.ActiveFrameLength = 32U;                     \
                (handle).FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION; \
                (handle).FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;       \
                (handle).FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;   \
                (handle).SlotInit.FirstBitOffset = 0;                           \
                (handle).SlotInit.SlotSize       = SAI_SLOTSIZE_32B;            \
                (handle).SlotInit.SlotNumber     = 2;                           \
                (handle).SlotInit.SlotActive     = SAI_SLOTACTIVE_ALL;          \
                if (HAL_SAI_Init(&(handle)) != HAL_OK) Error_Handler();         \
            } else {                                                            \
                if (HAL_SAI_InitProtocol(&(handle), SAI_I2S_STANDARD,           \
                                         SAI_PROTOCOL_DATASIZE_24BIT,           \
                                         2) != HAL_OK) Error_Handler();         \
            }                                                                   \
        } while (0)

    /* SAI1 Block A — slot 0. Always master TX. */
    hsai_BlockA1                       = hsai_template;
    hsai_BlockA1.Instance              = SAI1_Block_A;
    hsai_BlockA1.Init.SynchroExt       = a1_broadcast ? SAI_SYNCEXT_OUTBLOCKA_ENABLE
                                                      : SAI_SYNCEXT_DISABLE;
    hsai_BlockA1.Init.NoDivider        = s0_spdif ? SAI_MASTERDIVIDER_DISABLE
                                                  : SAI_MASTERDIVIDER_ENABLE;
    INIT_BLOCK_PROTO(hsai_BlockA1, s0_spdif);
    __HAL_LINKDMA(&hsai_BlockA1, hdmatx, hdma_sai1_a);

    /* SAI1 Block B — slot 1. I2S → sync-internal slave to SAI1_A
     * (zero phase offset to slot 0). SPDIF → own master. */
    hsai_BlockB1                       = hsai_template;
    hsai_BlockB1.Instance              = SAI1_Block_B;
    if (s1_sync_to_a1) {
        hsai_BlockB1.Init.AudioMode    = SAI_MODESLAVE_TX;
        hsai_BlockB1.Init.Synchro      = SAI_SYNCHRONOUS;
        hsai_BlockB1.Init.MckOutput    = SAI_MCK_OUTPUT_DISABLE;
    }
    hsai_BlockB1.Init.NoDivider        = s1_spdif ? SAI_MASTERDIVIDER_DISABLE
                                                  : SAI_MASTERDIVIDER_ENABLE;
    INIT_BLOCK_PROTO(hsai_BlockB1, s1_spdif);
    __HAL_LINKDMA(&hsai_BlockB1, hdmatx, hdma_sai1_b);

    /* SAI4 Block A — slot 2. I2S → sync-external slave to SAI1's
     * broadcast (cross-peripheral mesh). SPDIF → own master. */
    hsai_BlockA4                       = hsai_template;
    hsai_BlockA4.Instance              = SAI4_Block_A;
    if (s2_sync_to_s1) {
        hsai_BlockA4.Init.AudioMode    = SAI_MODESLAVE_TX;
        hsai_BlockA4.Init.Synchro      = SAI_SYNCHRONOUS_EXT_SAI1;
        hsai_BlockA4.Init.MckOutput    = SAI_MCK_OUTPUT_DISABLE;
    }
    hsai_BlockA4.Init.NoDivider        = s2_spdif ? SAI_MASTERDIVIDER_DISABLE
                                                  : SAI_MASTERDIVIDER_ENABLE;
    INIT_BLOCK_PROTO(hsai_BlockA4, s2_spdif);
    __HAL_LINKDMA(&hsai_BlockA4, hdmatx, hdma_sai4_a);

    /* SAI4 Block B — slot 3. I2S → sync-internal slave to SAI4_A.
     * SPDIF → own master. */
    hsai_BlockB4                       = hsai_template;
    hsai_BlockB4.Instance              = SAI4_Block_B;
    if (s3_sync_to_a4) {
        hsai_BlockB4.Init.AudioMode    = SAI_MODESLAVE_TX;
        hsai_BlockB4.Init.Synchro      = SAI_SYNCHRONOUS;
        hsai_BlockB4.Init.MckOutput    = SAI_MCK_OUTPUT_DISABLE;
    }
    hsai_BlockB4.Init.NoDivider        = s3_spdif ? SAI_MASTERDIVIDER_DISABLE
                                                  : SAI_MASTERDIVIDER_ENABLE;
    INIT_BLOCK_PROTO(hsai_BlockB4, s3_spdif);
    __HAL_LINKDMA(&hsai_BlockB4, hdmatx, hdma_sai4_b);

    #undef INIT_BLOCK_PROTO
}

/* During a hot-swap the SAI peripherals briefly disable their SD pin
 * outputs (when SAIEN goes 0 the pin returns to its AF default state,
 * effectively high-Z). Meanwhile SAI1_A's BCK/LRCLK on PE5/PE4 keep
 * ticking until that block itself is torn down — and any DAC wired
 * to PE3/PE6/PD11/PA0 happily samples whatever floating voltage or
 * EMI noise is on its SD pin during that window, which presents as
 * a sharp loud burst at re-enable.
 *
 * Defence: temporarily switch each SD pin from its alternate-function
 * role to plain GPIO output driving LOW for the duration of the swap.
 * The DAC samples a steady 0 (digital silence) instead of garbage.
 * After audio_configure_sais finishes, the pins go back to their AF
 * role (different AF per pin per board layout) and the SAI takes
 * over driving them again. */
static void mute_sd_pins_to_gpio_low(void) {
    GPIO_InitTypeDef g = {
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    /* Drive the pins low FIRST, then switch mode — order matters so
     * the pin doesn't glitch high-Z → high → low. */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);  /* slot 0 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);  /* slot 1 */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); /* slot 2 */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,  GPIO_PIN_RESET); /* slot 3 */
    g.Pin = GPIO_PIN_6;  HAL_GPIO_Init(GPIOE, &g);
    g.Pin = GPIO_PIN_3;  HAL_GPIO_Init(GPIOE, &g);
    g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOD, &g);
    g.Pin = GPIO_PIN_0;  HAL_GPIO_Init(GPIOA, &g);
}

static void unmute_sd_pins_to_af(void) {
    GPIO_InitTypeDef g = {
        .Mode  = GPIO_MODE_AF_PP,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };
    /* PE6 = SAI1_SD_A (AF6) — slot 0 */
    g.Pin = GPIO_PIN_6;  g.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &g);
    /* PE3 = SAI1_SD_B (AF6) — slot 1 */
    g.Pin = GPIO_PIN_3;  g.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &g);
    /* PD11 = SAI4_SD_A (AF10) — slot 2 */
    g.Pin = GPIO_PIN_11; g.Alternate = GPIO_AF10_SAI4;
    HAL_GPIO_Init(GPIOD, &g);
    /* PA0 = SAI4_SD_B (AF10) — slot 3 */
    g.Pin = GPIO_PIN_0;  g.Alternate = GPIO_AF10_SAI4;
    HAL_GPIO_Init(GPIOA, &g);
}

/* Flush the SAI FIFO on every sub-block. HAL_SAI_DeInit does NOT clear
 * the FIFO — it's hardware state outside HAL's tracking — and HAL_SAI_
 * Init doesn't flush it either. Without this, the first ~4 samples
 * shifted out after SAIEN goes back high are whatever was left in the
 * FIFO when teardown ran (typically loud since they predate the
 * pre-fill-with-zeros). FFLUSH is self-clearing per RM0468. */
static void flush_all_sai_fifos(void) {
    SAI1_Block_A->CR2 |= SAI_xCR2_FFLUSH;
    SAI1_Block_B->CR2 |= SAI_xCR2_FFLUSH;
    SAI4_Block_A->CR2 |= SAI_xCR2_FFLUSH;
    SAI4_Block_B->CR2 |= SAI_xCR2_FFLUSH;
}

/* Tear down all four SAI sub-blocks back to RESET state in preparation
 * for a hot-swap. After this returns, no SAI is clocking anything and
 * all four DMAs are halted.
 *
 * Why we bypass HAL_SAI_DMAStop: that function calls SAI_Disable which
 * polls SAI_xCR1.SAIEN for clear with SAI_LONG_TIMEOUT (1 second).
 * SAIEN clear is only effective at end-of-frame — for sync-slave
 * sub-blocks, that requires the master to keep clocking. After ANY
 * mixed-mode hot-swap, the sub-block dependency graph can change
 * such that a previously-quiet sub-block is now waiting on a clock
 * that's about to be torn down, and HAL_SAI_DMAStop hangs for the
 * full second. The next hot-swap then finds inconsistent HAL state
 * and HAL_SAI_Init returns HAL_ERROR → Error_Handler() infinite
 * loop, indistinguishable from a hardware crash.
 *
 * The reliable approach: clear SAIEN + DMAEN on every block in close
 * succession (so all blocks lose their clock requirement at roughly
 * the same time), wait one frame period (~21 µs at 48 kHz) so any
 * in-flight frame completes, then HAL_DMA_Abort the DMA streams and
 * HAL_SAI_DeInit the SAI handles. HAL_SAI_DeInit's internal call to
 * SAI_Disable then sees SAIEN already 0 and returns immediately. */
static void audio_teardown_sais(void) {
    /* Step 1: simultaneously clear SAIEN + DMAEN on every sub-block.
     * Direct register writes — no polling. The SAI hardware will
     * actually deassert at its next end-of-frame regardless of
     * whether HAL is watching. */
    SAI1_Block_A->CR1 &= ~(SAI_xCR1_SAIEN | SAI_xCR1_DMAEN);
    SAI1_Block_B->CR1 &= ~(SAI_xCR1_SAIEN | SAI_xCR1_DMAEN);
    SAI4_Block_A->CR1 &= ~(SAI_xCR1_SAIEN | SAI_xCR1_DMAEN);
    SAI4_Block_B->CR1 &= ~(SAI_xCR1_SAIEN | SAI_xCR1_DMAEN);
    __DSB();

    /* Step 2: wait a few frame periods so any in-flight frame ends
     * and the hardware actually deasserts SAIEN. One frame at 48 kHz
     * = 20.83 µs; 100 frames = 2 ms is conservative and unnoticed. */
    HAL_Delay(2);

    /* Step 3: abort each DMA stream/channel directly. HAL_DMA_Abort
     * disables the EN bit, polls for it to clear, and resets the
     * handle's State to READY — should be quick since we already
     * stopped the SAI from requesting transfers. */
    HAL_DMA_Abort(&hdma_sai1_a);
    HAL_DMA_Abort(&hdma_sai1_b);
    HAL_DMA_Abort(&hdma_sai4_a);
    HAL_DMA_Abort(&hdma_sai4_b);

    /* Step 4: HAL_SAI_DeInit each handle. Its internal SAI_Disable
     * sees SAIEN already cleared and returns without polling.
     * State is set to RESET so the next HAL_SAI_Init treats this as
     * a fresh init. */
    HAL_SAI_DeInit(&hsai_BlockB4);
    HAL_SAI_DeInit(&hsai_BlockA4);
    HAL_SAI_DeInit(&hsai_BlockB1);
    HAL_SAI_DeInit(&hsai_BlockA1);
}

/* ---------------------------------------------------------------------- */
/* Public init                                                            */
/* ---------------------------------------------------------------------- */
void Audio_Init(void) {
    /* RCC */
    __HAL_RCC_SAI1_CLK_ENABLE();
    __HAL_RCC_SAI4_CLK_ENABLE();      /* phase 2: slot 2/3 — H723 has SAI1+SAI4 only */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_BDMA_CLK_ENABLE();      /* SAI4 streams go through BDMA */
    /* DMAMUX1 / DMAMUX2 have no separate clock-enable on H7. */
    __HAL_RCC_GPIOA_CLK_ENABLE();     /* PA0 = SAI4_SD_B */
    __HAL_RCC_GPIOD_CLK_ENABLE();     /* PD11 = SAI4_SD_A */
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

    /* SAI4 data pins — clocks are SHARED with SAI1 via SyncExt so we
     * don't drive PD12/PD13/PE0 (would-be SAI4 FS/SCK/MCLK).
     *   PD11 = SAI4_SD_A  (AF10)  — slot 2 data
     *   PA0  = SAI4_SD_B  (AF10)  — slot 3 data
     * Per H723 datasheet (DS13313) Table 8: PA0/PD11's SAI4 alt is
     * AF10. The HAL header also defines AF1_SAI4 ("available on
     * STM32H72xxx/H73xxx") but that's for a different subset of pins
     * (e.g. PE0's SAI4_MCLK_A) — not these data lines. */
    GPIO_InitTypeDef g2 = {
        .Pin       = GPIO_PIN_11,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_NOPULL,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF10_SAI4,
    };
    HAL_GPIO_Init(GPIOD, &g2);

    g2.Pin       = GPIO_PIN_0;
    g2.Alternate = GPIO_AF10_SAI4;
    HAL_GPIO_Init(GPIOA, &g2);

    /* DMA1 Stream 0/1 → SAI1_A/B (D1 domain).
     * BDMA  Channel 0/1 → SAI4_A/B (D3 domain).
     * The LINKDMA calls happen later, AFTER the SAI handles are
     * configured (B's hsai struct is a post-init clone of A's, which
     * copies A's hdmatx pointer — must be re-linked to its own DMA
     * stream after the clone or both blocks end up pointing at A's
     * stream and B's data never moves). */
    dma_init_common(&hdma_sai1_a, DMA1_Stream0, DMA_REQUEST_SAI1_A);
    dma_init_common(&hdma_sai1_b, DMA1_Stream1, DMA_REQUEST_SAI1_B);
    bdma_init_common(&hdma_sai4_a, BDMA_Channel0, BDMA_REQUEST_SAI4_A);
    bdma_init_common(&hdma_sai4_b, BDMA_Channel1, BDMA_REQUEST_SAI4_B);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(BDMA_Channel0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
    HAL_NVIC_SetPriority(BDMA_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);

    /* M12 phase 3 / 4: per-slot output type drives protocol + sync model.
     * `output_types[slot]` (0=SPDIF, 1=I2S) tells each of the four sub-
     * blocks what to emit. The all-I2S → cross-peripheral sync chain
     * vs any-SPDIF → all-independent-masters rule lives entirely inside
     * audio_configure_sais; same helper is called at boot here AND from
     * Audio_HotSwap when a runtime type change comes in (Phase 4). */
    audio_configure_sais();

    /* Pre-fill ALL FOUR buffers with silence so the first BCK edges
     * after Audio_Start ship zero samples until USB starts delivering. */
    for (uint32_t i = 0; i < AUDIO_WORDS_TOTAL; ++i) {
        audio_buf_a[i] = 0;
        audio_buf_b[i] = 0;
        audio_buf_c[i] = 0;
        audio_buf_d[i] = 0;
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
    /* Phase 2 master/master sync strategy.
     *
     * SAI1 and SAI4 are independent masters off the same PLL2_P kernel
     * clock — frequency-locked. To make them PHASE-locked too, we need
     * both A-block SAIEN bits to assert within the same kernel-clock
     * cycle (PLL2_P period = 20 ns at 49.152 MHz). Without that, the
     * two clock generators latch their start on different kernel
     * edges and end up with a permanent BCK/FS phase offset (could be
     * several BCK cycles = sub-sample on a single sample, but worst
     * case ~0.5 sample of offset).
     *
     * HAL_SAI_Transmit_DMA does a lot — DMA arming, FIFO flush, then
     * SAIEN. Calling it sequentially for SAI1 and SAI4 takes µs, way
     * more than 20 ns. Approach: call HAL on all four blocks (which
     * sets SAIEN on each as part of its flow), then immediately CLEAR
     * SAIEN on both A-block masters, then re-assert both SAIEN bits
     * in back-to-back CPU stores inside an irq-disabled critical
     * section. Two adjacent stores at SYSCLK 550 MHz are ~2 ns apart,
     * comfortably inside one PLL2_P kernel cycle. The B sub-blocks
     * (sync-internal to their A) follow A automatically when A re-
     * enables — slave logic resumes from the next FS edge.
     *
     * Note SAI1's IRQ-driven half/full-cplt callbacks are the ONLY
     * source of fill_half() calls; if they're armed before SAIEN goes
     * high they sit idle waiting for DMA-half events. Disabling-then-
     * re-enabling SAIEN doesn't disturb the DMA arming state, so when
     * SAIEN comes back high the DMA controller resumes shipping the
     * already-buffered samples.
     */
    /* Start order: B sub-blocks first (slaves; sync-internal to their A),
     * then A masters last. Both A masters run from PLL2_P, so they're
     * frequency-locked = zero drift across all 8 outputs. The static
     * start-phase offset between SAI1_A and SAI4_A is bounded by the
     * inter-call execution time (~5–10 µs ≈ 0.3–0.5 sample at 48 kHz),
     * fixed at boot and reproducible. The earlier attempt at pulling
     * both SAIEN bits low and re-asserting them in the same kernel
     * cycle deadlocked SAI1 because RM0468 makes SAIEN-clear
     * effective only at end-of-frame (20.8 µs later) — a 180 ns wait
     * left both blocks in an undefined "disable pending" state. */
    if (HAL_SAI_Transmit_DMA(&hsai_BlockB4,
                             (uint8_t *)audio_buf_d,
                             AUDIO_WORDS_TOTAL) != HAL_OK) Error_Handler();
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA4,
                             (uint8_t *)audio_buf_c,
                             AUDIO_WORDS_TOTAL) != HAL_OK) Error_Handler();
    if (HAL_SAI_Transmit_DMA(&hsai_BlockB1,
                             (uint8_t *)audio_buf_b,
                             AUDIO_WORDS_TOTAL) != HAL_OK) Error_Handler();
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA1,
                             (uint8_t *)audio_buf_a,
                             AUDIO_WORDS_TOTAL) != HAL_OK) Error_Handler();
}

/* M12 phase 4: live runtime I2S↔SPDIF type swap.
 *
 * REQ_SET_OUTPUT_TYPE writes the new value into output_types[] from
 * the USB ISR and raises this flag; the main loop drains it and calls
 * Audio_HotSwap. The swap itself can't run in ISR context — HAL_SAI_*
 * functions take locks and HAL_SAI_DeInit calls back into MspDeInit
 * which touches RCC; both are unsafe to do under preemption.
 *
 * Volatile boolean rather than per-slot mask: the configure step
 * always reads ALL of output_types[] anyway (the all-I2S-vs-mixed
 * sync rule is global), so single-bit fan-out doesn't help. Lost
 * raise events between two close SETs aren't a problem either —
 * the second SET re-raises the flag, and the main loop reads
 * output_types fresh at swap time. */
volatile bool output_type_change_pending = false;

void Audio_HotSwap(void) {
    /* Defensive sanitize: cascade-coerce any I2S slot whose clock
     * parent is SPDIF down to SPDIF, so audio_configure_sais sees
     * only valid combinations. Coerced changes still get notified
     * to Console below — important when the user (or a bulk SET)
     * tries to set an invalid mix; Console UI updates to reflect
     * the actual applied state instead of silently disagreeing. */
    uint8_t coerced = sanitize_output_types();
    if (coerced) {
        extern uint8_t output_types[];
        for (int s = 0; s < NUM_SPDIF_INSTANCES; ++s) {
            if (coerced & (1u << s)) {
                uint8_t v = output_types[s];
                notify_param_write(
                    (uint16_t)(offsetof(WireBulkParams,
                                        i2s_config.output_types) + s),
                    1, &v);
            }
        }
    }

    /* fill_half writes into the four DMA buffers from SAI1's IRQ.
     * Disable that IRQ for the duration of teardown so we don't get
     * a DMA half-cplt firing into a half-deinitialised SAI handle. */
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_DisableIRQ(BDMA_Channel0_IRQn);
    HAL_NVIC_DisableIRQ(BDMA_Channel1_IRQn);

    /* Pin-level mute BEFORE the SAI tears down, so any DAC wired to
     * an SD line samples a steady 0 instead of a floating pin. */
    mute_sd_pins_to_gpio_low();

    audio_teardown_sais();

    /* Pre-fill all four DMA rings with silence so the first SAI BCK
     * after re-Start ships zero samples until fill_half catches up.
     * Without this we'd play whatever stale data was sitting in the
     * buffer when teardown halted DMA — typically a glitchy click. */
    for (uint32_t i = 0; i < AUDIO_WORDS_TOTAL; ++i) {
        audio_buf_a[i] = 0;
        audio_buf_b[i] = 0;
        audio_buf_c[i] = 0;
        audio_buf_d[i] = 0;
    }

    audio_configure_sais();

    /* Flush each SAI's FIFO of any stale words left over from before
     * the teardown — without this the next SAIEN=1 ships those out
     * first before DMA gets a chance to refill from the zeroed
     * buffers. */
    flush_all_sai_fifos();

    /* Hand the SD pins back to the SAI peripherals — the new sub-block
     * configuration takes over driving them. Re-AF'ing happens BEFORE
     * Audio_Start so the first sample SAI shifts out is on a properly
     * AF-muxed pin. */
    unmute_sd_pins_to_af();

    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(BDMA_Channel0_IRQn);
    HAL_NVIC_EnableIRQ(BDMA_Channel1_IRQn);

    Audio_Start();
}
