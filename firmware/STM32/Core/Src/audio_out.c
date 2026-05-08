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
#include <math.h>          /* M7e: fabsf for level meters */

/* ---- Single shared DMA ring in AXI SRAM (DMA1 cannot reach DTCM) ---- */
#define AUDIO_BUFFER_BASE  0x24000000UL

/* 192 stereo frames per ping-pong half × 2 halves × 2 ch = 768 int32 words.
 * 192 frames @ 48 kHz = 4 ms per half — comfortably above any HAL ISR
 * latency. */
#define AUDIO_FRAMES_HALF    192U
#define AUDIO_FRAMES_TOTAL   (AUDIO_FRAMES_HALF * 2)
#define AUDIO_WORDS_TOTAL    (AUDIO_FRAMES_TOTAL * 2)  /* L+R */

static int32_t * const audio_buf = (int32_t *)AUDIO_BUFFER_BASE;

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

/* Local DSP state (zeroed on init by Audio_Init). */
static CrossfeedState crossfeed_state;
static LevellerState  leveller_state;
static LevellerCoeffs leveller_coeffs;

/* Block-based per-stage scratch buffers — one half-buffer's worth.
 * Each stage operates on the full block in-place. AXI SRAM-resident
 * via the shared audio_buf placement isn't required here (these are
 * CPU-only) so they live as plain BSS in DTCM (faster, no DMA). */
static float buf_l[AUDIO_FRAMES_HALF];
static float buf_r[AUDIO_FRAMES_HALF];
static float buf_o0[AUDIO_FRAMES_HALF];
static float buf_o1[AUDIO_FRAMES_HALF];

/* Per-output EQ channel index: Out0 → channel 2, Out1 → channel 3. */
#define EQ_CH_OUT0    2
#define EQ_CH_OUT1    3

static void fill_half(int32_t *dst) {
    uint32_t got = usb_ring_pop_frames(pop_scratch, AUDIO_FRAMES_HALF);
    if (got < AUDIO_FRAMES_HALF) ++audio_underruns;

    /* Snapshot the matrix crosspoints once per buffer-half (≪ 1 µs each)
     * so the inner loop stays branch-light. Per-input gain folds in
     * phase invert. */
    MatrixCrosspoint *xp00 = &matrix_mixer.crosspoints[0][0];
    MatrixCrosspoint *xp10 = &matrix_mixer.crosspoints[1][0];
    MatrixCrosspoint *xp01 = &matrix_mixer.crosspoints[0][1];
    MatrixCrosspoint *xp11 = &matrix_mixer.crosspoints[1][1];

    float g_l_to_o0 = xp00->enabled
        ? (xp00->phase_invert ? -xp00->gain_linear : xp00->gain_linear) : 0.0f;
    float g_r_to_o0 = xp10->enabled
        ? (xp10->phase_invert ? -xp10->gain_linear : xp10->gain_linear) : 0.0f;
    float g_l_to_o1 = xp01->enabled
        ? (xp01->phase_invert ? -xp01->gain_linear : xp01->gain_linear) : 0.0f;
    float g_r_to_o1 = xp11->enabled
        ? (xp11->phase_invert ? -xp11->gain_linear : xp11->gain_linear) : 0.0f;

    OutputChannel *out0 = &matrix_mixer.outputs[0];
    OutputChannel *out1 = &matrix_mixer.outputs[1];
    float out0_post_gain = (out0->enabled && !out0->mute) ? out0->gain_linear : 0.0f;
    float out1_post_gain = (out1->enabled && !out1->mute) ? out1->gain_linear : 0.0f;

    /* M7d: per-input preamp + master volume + bypass snapshots —
     * once per buffer-half so the per-sample loops stay branch-light. */
    float preamp_l = global_preamp_linear[0];
    float preamp_r = global_preamp_linear[1];
    float master   = master_volume_linear;
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

    /* === Stage 5: matrix mixer → per-output buffers. */
    for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
        buf_o0[k] = buf_l[k] * g_l_to_o0 + buf_r[k] * g_r_to_o0;
        buf_o1[k] = buf_l[k] * g_l_to_o1 + buf_r[k] * g_r_to_o1;
    }

    /* === Stage 6: per-output EQ (block-based, channels 2/3). */
    if (!eq_bypass) {
        dsp_process_channel_block(filters[EQ_CH_OUT0], buf_o0,
                                  AUDIO_FRAMES_HALF, EQ_CH_OUT0);
        dsp_process_channel_block(filters[EQ_CH_OUT1], buf_o1,
                                  AUDIO_FRAMES_HALF, EQ_CH_OUT1);
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
        const int32_t dly0 = channel_delay_samples[0];
        const int32_t dly1 = channel_delay_samples[1];
        if (dly0 > 0) {
            float *dline = delay_lines[0];
            uint32_t widx = delay_write_idx;
            for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
                dline[widx] = buf_o0[k];
                buf_o0[k]   = dline[(widx - dly0) & MAX_DELAY_MASK];
                widx = (widx + 1) & MAX_DELAY_MASK;
            }
        }
        if (dly1 > 0) {
            float *dline = delay_lines[1];
            uint32_t widx = delay_write_idx;
            for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
                dline[widx] = buf_o1[k];
                buf_o1[k]   = dline[(widx - dly1) & MAX_DELAY_MASK];
                widx = (widx + 1) & MAX_DELAY_MASK;
            }
        }
        delay_write_idx = (delay_write_idx + AUDIO_FRAMES_HALF) & MAX_DELAY_MASK;
    }

    /* === Stage 7: per-output gain + master volume + clamp + 24-bit
     *               quantisation into the SAI ping-pong slot.  Peaks
     *               folded into the same loop so we don't re-traverse. */
    float pk_in_l = 0.0f, pk_in_r = 0.0f;
    float pk_o0   = 0.0f, pk_o1   = 0.0f;
    bool  clip0   = false, clip1  = false;

    for (uint32_t k = 0; k < AUDIO_FRAMES_HALF; ++k) {
        /* Pre-gain "master/input" peaks (track buf_l/buf_r, post-EQ but
         * before output-routing — closest analogue to the RP master peak
         * so Console's "USB L/R" meter behaves the same). */
        float al = fabsf(buf_l[k]); if (al > pk_in_l) pk_in_l = al;
        float ar = fabsf(buf_r[k]); if (ar > pk_in_r) pk_in_r = ar;

        float o0 = buf_o0[k] * out0_post_gain * master;
        float o1 = buf_o1[k] * out1_post_gain * master;

        /* Clip detection BEFORE clamp (RP convention — flag if any sample
         * tried to exceed full-scale before the limiter hides it). */
        float ao0 = fabsf(o0); if (ao0 > pk_o0) pk_o0 = ao0;
        float ao1 = fabsf(o1); if (ao1 > pk_o1) pk_o1 = ao1;
        if (ao0 > CLIP_THRESH_F) clip0 = true;
        if (ao1 > CLIP_THRESH_F) clip1 = true;

        if (o0 >  1.0f) o0 =  1.0f; else if (o0 < -1.0f) o0 = -1.0f;
        if (o1 >  1.0f) o1 =  1.0f; else if (o1 < -1.0f) o1 = -1.0f;
        dst[2*k + 0] = (int32_t)(o0 * FLOAT_TO_24);
        dst[2*k + 1] = (int32_t)(o1 * FLOAT_TO_24);
    }

    /* Publish to global_status — converted u16 [0..32767]. clip_flags is a
     * sticky bitmask cleared by REQ_CLEAR_CLIPS so brief overshoots stay
     * visible until the user explicitly resets. */
    if (pk_in_l > 1.0f) pk_in_l = 1.0f;
    if (pk_in_r > 1.0f) pk_in_r = 1.0f;
    if (pk_o0   > 1.0f) pk_o0   = 1.0f;
    if (pk_o1   > 1.0f) pk_o1   = 1.0f;
    global_status.peaks[CH_MASTER_LEFT]  = (uint16_t)(pk_in_l * 32767.0f);
    global_status.peaks[CH_MASTER_RIGHT] = (uint16_t)(pk_in_r * 32767.0f);
    global_status.peaks[CH_OUT_1]        = (uint16_t)(pk_o0   * 32767.0f);
    global_status.peaks[CH_OUT_2]        = (uint16_t)(pk_o1   * 32767.0f);
    if (clip0) global_status.clip_flags |= (1u << CH_OUT_1);
    if (clip1) global_status.clip_flags |= (1u << CH_OUT_2);
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    /* Only A drives the fill — B reads the same buffer from its own DMA.
     * A and B's half-cplt callbacks fire essentially simultaneously
     * (they share the BCK/FS clocks); a stray B-side callback would
     * just refill the same data redundantly, so we ignore it. */
    if (hsai == &hsai_BlockA1) {
        fill_half(&audio_buf[0]);
    }
    ++audio_dma_callbacks;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai == &hsai_BlockA1) {
        fill_half(&audio_buf[AUDIO_FRAMES_HALF * 2]);
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

    /* Pre-fill the shared buffer with silence so the first BCK edges
     * after Audio_Start ship zero samples until USB starts delivering. */
    for (uint32_t i = 0; i < AUDIO_WORDS_TOTAL; ++i) {
        audio_buf[i] = 0;
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
    /* Both A and B stream from the SAME ring (audio_buf). Two DMA
     * streams reading from the same AXI SRAM range is fine — the AXI
     * arbiter handles concurrent reads, and we only WRITE during the
     * half-cplt callback (when DMA is reading the OTHER half).
     *
     * Start the synchronous slave (B) FIRST so its DMA + SAIEN are
     * armed and waiting on A's BCK/FS edges. Then start A — the moment
     * A's SAIEN goes high, BCK/FS begin toggling and B shifts out its
     * first sample on the same edge as A's, giving sample-aligned
     * output. */
    if (HAL_SAI_Transmit_DMA(&hsai_BlockB1,
                             (uint8_t *)audio_buf,
                             AUDIO_WORDS_TOTAL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA1,
                             (uint8_t *)audio_buf,
                             AUDIO_WORDS_TOTAL) != HAL_OK) {
        Error_Handler();
    }
}
