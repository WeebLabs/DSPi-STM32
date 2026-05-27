/*
 * spdif_input.c — STM32H723 SPDIFRX peripheral, Stage 1
 *
 * Stage 1 brings the SPDIFRX peripheral up to the point where Console
 * can query lock state and measured sample rate via REQ_GET_SPDIF_RX_
 * STATUS (0xE2). It does NOT route audio into fill_half yet — that's
 * Stage 2 which adds circular DMA + the resampler glue.
 *
 * Hardware:
 *   PD8 / D8               — SPDIFRX1_IN2, AF9 (WeAct header P1 pin 40).
 *                             RM0468/HAL select this as INSEL=1; the
 *                             datasheet pin label is one-based.
 *   SPDIFRX peripheral     — D2 domain, APB1L bus, base 0x40004000
 *   Kernel clock           — PLL2_R @ 98.304 MHz (selected via
 *                             RCC_D2CCIP1R.SPDIFSEL = 01); same PLL2
 *                             already powering SAI1/SAI4 audio
 *   DMA1 Stream 2 + 3      — request lines 93 (data) and 94 (control)
 *                             via DMAMUX1; both placed in AXI SRAM
 *   No D-cache concerns    — D-cache is OFF through M6a on this build
 *
 * Sample rate detection method (Stage 1):
 *   SPDIFRX_SR.WIDTH5[14:0] reports the duration of 5 S/PDIF symbols
 *   in spdif_clk periods. Fs ≈ 5 × kclk / (WIDTH5 × 64). At 48 kHz
 *   with a 98.304 MHz kernel clock, WIDTH5 ≈ 160. Updated every IEC
 *   block.
 *
 * Lock state machine:
 *   INACTIVE  → start()      → ACQUIRING
 *   ACQUIRING → SYNCD set    → LOCKED
 *   LOCKED    → SERR/FERR/TERR → RELOCKING
 *   RELOCKING → SYNCD set    → LOCKED
 *   any       → stop()       → INACTIVE
 */

#include "main.h"
#include "spdif_input.h"
#include "audio_input.h"
#include "config.h"
#include "resampler.h"
#include <string.h>
#include <math.h>

#if defined(STM32H723xx)

/* Peripheral kernel-clock source (must match what main.c programs in
 * RCC_D2CCIP1R). Used by spdif_input_poll() to convert WIDTH5 into Hz.
 *
 * Sourced from PLL3_R = 120 MHz. PLL3 is never servo-tuned, so this
 * value is exact (within HSE crystal tolerance) and never moves —
 * unlike the SAI clock on PLL2 which slews under FRACN control. */
#define SPDIFRX_KERNEL_CLOCK_HZ  120000000U

/* HAL handles — peripheral + the two DMAs (data + control). */
static SPDIFRX_HandleTypeDef hspdif;
static DMA_HandleTypeDef     hdma_spdifrx_dt;   /* audio data, request 93  */
static DMA_HandleTypeDef     hdma_spdifrx_cs;   /* channel/user, request 94 */

/* DMA ring buffers — placed via fixed pointers in AXI SRAM (D2-reachable,
 * non-cacheable on this build because D-cache is off through M6a).
 * Each SPDIFRX subframe = 1 32-bit DR word; in mode 0 the audio sits in
 * bits [23:0] with metadata in [29:24] and PT[1:0] in [29:28].
 *
 * Memory map (AXI SRAM at 0x24000000):
 *   0x24000000  audio_buf_a  (3 KB)  — SAI1_A DMA, owned by audio_out.c
 *   0x24001000  audio_buf_b  (3 KB)  — SAI1_B DMA, owned by audio_out.c
 *   0x24002000  spdifrx_dt   (4 KB)  — 1024 × u32 SPDIF data words (NEW)
 *   0x24003000  spdifrx_cs   (1 KB)  — 256  × u32 channel-status words
 *   0x24004000  flash_mirror (48 KB) — preset region (M11)
 * The 3 KB scratch hole at 0x24002000-0x24004000 is currently
 * unallocated; we slot the SPDIF rings in here.
 *
 * Buffer sizes: at 192 kHz (worst case) the data DMA produces 384k
 * stereo frames per second = 192k subframes per second per channel.
 * 1024-word ring = ~5.3 ms at 192 kHz, ~10.6 ms at 48 kHz — comfortable
 * vs the main-loop polling cadence. CSR delivers ~1 word per
 * subframe pair; 256 words = ~5 IEC blocks at 48 kHz, plenty to
 * accumulate one full 24-byte channel-status block. */
#define SPDIFRX_DT_BUF_BASE    0x24002000UL
#define SPDIFRX_CS_BUF_BASE    0x24003000UL
#define SPDIFRX_DT_BUF_WORDS   1024U
#define SPDIFRX_CS_BUF_WORDS   256U

static uint32_t * const spdifrx_dt_buf = (uint32_t *)SPDIFRX_DT_BUF_BASE;
static uint32_t * const spdifrx_cs_buf = (uint32_t *)SPDIFRX_CS_BUF_BASE;

/* DEBUG (M9) — source-switch silent-failure tracking. */
volatile uint32_t spdif_start_call_count       = 0;
volatile uint32_t spdif_start_dt_rc            = 0xFFFFFFFF;
volatile uint32_t spdif_start_cs_rc            = 0xFFFFFFFF;
volatile uint32_t spdif_start_pre_hspdif_state = 0;
volatile uint32_t spdif_start_dt_dma_state     = 0;
volatile uint32_t spdif_start_pre_cr           = 0;

/* Run-time state. */
static volatile SpdifInputState spdif_state    = SPDIF_INPUT_INACTIVE;
static volatile uint8_t         lock_count     = 0;
static volatile uint8_t         loss_count     = 0;
static volatile uint32_t        parity_errors  = 0;
static volatile uint32_t        sample_rate_hz = 0;
static bool                     hw_initialised = false;
static uint32_t                 last_sync_retry_ms = 0;

/* Stage 2: 16-bit stereo demux ring fed by the DMA half/full callbacks
 * and drained by fill_half. Power-of-two size lets us mask instead of
 * branching for wraps. 1024 stereo frames = ~21 ms of headroom at
 * 48 kHz — comfortable buffer for any plausible jitter between the
 * SPDIF input rate and the SAI output rate while the servo settles.
 *
 * Placed in AXI SRAM (D1, 320 KB) via a fixed pointer at 0x24012000 —
 * past the DSP scratch buffers which run up to ~0x24011E00 — so it
 * doesn't pile onto the DTCM BSS budget where it'd overflow. The
 * pattern mirrors audio_buf_a/b and flash_mirror; AXI SRAM is DMA-
 * coherent on this build (D-cache off through M6c). */
#define SPDIF_RING_BASE    0x24012000UL
#define SPDIF_RING_FRAMES  1024U
#define SPDIF_RING_MASK    (SPDIF_RING_FRAMES - 1U)
/* int32 ring carrying the full 24-bit SPDIF samples (sign-extended). At
 * 1024 frames × 2 ch × 4 B = 8 KB it exactly fills the 0x24012000-0x24014000
 * hole below the resampler's filter table. We deliberately do NOT truncate to
 * int16 here — the float DSP/resampler downstream consume the full depth. */
static int32_t * const spdif_ring = (int32_t *)SPDIF_RING_BASE;
static volatile uint32_t spdif_ring_widx = 0;       /* DMA-callback writes */
static volatile uint32_t spdif_ring_ridx = 0;       /* fill_half reads     */
/* Last sample captured per channel (24-bit, sign-extended) — held over when
 * the source goes mute / parity-error / not-valid, so a brief bad subframe
 * doesn't inject a click. */
static int32_t  last_sample_l = 0;
static int32_t  last_sample_r = 0;

/* Channel-status accumulator — 192 bits = 24 bytes per IEC block.
 * CSR words deliver one CS bit at a time; we shift them in and snapshot
 * the completed block when SOB fires for the next block. */
static uint8_t cs_block_current[24];
static uint8_t cs_block_complete[24];
static bool    cs_block_valid = false;

/* Forward decls — FRACN servo state + helpers live below the public
 * API for layout reasons (the reader meets spdif_input_init first),
 * but spdif_input_stop and spdif_input_poll touch them. */
#define FRACN_NOMINAL          2490U
#define SPDIF_RING_TARGET_FILL_FWD 512   /* mirrors SPDIF_RING_TARGET_FILL below */
static int32_t  servo_int_acc;        /* defined below */
static int32_t  servo_last_fracn;     /* defined below */
static int32_t  servo_fill_lpf;       /* defined below */
static double   servo_filtered_diff;  /* defined below */
static void spdifrx_restart_sync(void);
static void servo_tick(void);
static void pll2_fracn_write(uint32_t new_fracn);

/* HAL_SPDIFRX_MspInit override — pulls in GPIO + RCC + DMA setup so
 * HAL_SPDIFRX_Init can do its register programming with everything
 * around it already running. Called by HAL_SPDIFRX_Init when the
 * handle's State is RESET. */
void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef *hsai) {
    if (hsai->Instance != SPDIFRX) return;

    /* RCC: peripheral clock + DMA1 (DMA1 already enabled by Audio_Init,
     * but enable is idempotent). */
    __HAL_RCC_SPDIFRX_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* M9: PD8 → AF9 SPDIFRX1_IN2 (WeAct P1 pin 40 on the schematic).
     * The pin table calls this SPDIFRX1_IN2, but RM0468's INSEL field
     * and HAL constants are zero-based here: INSEL=1 selects this pad.
     * PD8 doesn't conflict with the onboard W25Q SPI flash (which
     * uses PD7 as MOSI), so the flash keeps working. Internal pull-
     * up enabled to help the open-collector TOSLINK receiver — the
     * SPDIFRX discriminator's internal bias network loads the input,
     * so without a pull-up the open-collector receiver can't pull
     * the line back to logic-high. External pull-up to 3V3 is the
     * stronger version of this fix if the 40 kΩ internal isn't
     * enough. */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {
        .Pin       = GPIO_PIN_8,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_PULLUP,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF9_SPDIF,
    };
    HAL_GPIO_Init(GPIOD, &g);

    /* Data DMA: DMA1 Stream 2, request 93 (DMAMUX1 SPDIFRX_DT).
     * Circular peripheral-to-memory, 32-bit words, high priority. */
    hdma_spdifrx_dt.Instance                 = DMA1_Stream2;
    hdma_spdifrx_dt.Init.Request             = DMA_REQUEST_SPDIF_RX_DT;
    hdma_spdifrx_dt.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_spdifrx_dt.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_spdifrx_dt.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_spdifrx_dt.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_spdifrx_dt.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_spdifrx_dt.Init.Mode                = DMA_CIRCULAR;
    hdma_spdifrx_dt.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_spdifrx_dt.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_spdifrx_dt.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_spdifrx_dt.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_spdifrx_dt.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_spdifrx_dt) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(hsai, hdmaDrRx, hdma_spdifrx_dt);

    /* Control DMA: DMA1 Stream 3, request 94 (DMAMUX1 SPDIFRX_CS).
     * Same shape as data, just a different stream + request. */
    hdma_spdifrx_cs                          = hdma_spdifrx_dt; /* clone */
    hdma_spdifrx_cs.Instance                 = DMA1_Stream3;
    hdma_spdifrx_cs.Init.Request             = DMA_REQUEST_SPDIF_RX_CS;
    if (HAL_DMA_Init(&hdma_spdifrx_cs) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(hsai, hdmaCsRx, hdma_spdifrx_cs);

    /* DMA NVIC — same priority class as the SAI DMAs. */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    /* Peripheral IRQ for SYNCD / lock-event reporting. */
    HAL_NVIC_SetPriority(SPDIF_RX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPDIF_RX_IRQn);
}

/* DMA stream IRQ handlers — forward to HAL. fill_half-equivalent
 * audio handling will be added in Stage 2. */
void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spdifrx_dt); }
void DMA1_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spdifrx_cs); }
void SPDIF_RX_IRQHandler(void)     { HAL_SPDIFRX_IRQHandler(&hspdif); }

/* Stage 2: demux one DMA half of SPDIFRX_DR words into the L/R ring.
 * Each 32-bit word in mode 0 (DRFMT=00) is laid out per RM0468 §38.5.4:
 *
 *   [31:30] reserved          [29:28] PT (preamble type)
 *   [27]    C (channel status) [26]    U (user)
 *   [25]    V (validity)       [24]    PE (parity error)
 *   [23:0]  24-bit audio sample, MSB at bit 23
 *
 * PT field tells us channel: 10 = B/Z (start of block / left), 00 = M
 * (left subframe in non-block frames), 01 = W (right). The ST training
 * material and the EEVBlog/Linux references all converge on:
 *   PT == 0b10 → left, PT == 0b01 → right
 * We treat anything non-right as left, so a corrupted preamble lands
 * on left and gets sample-replaced if invalid.
 *
 * Per word we:
 *   1. Sign-extend the 24-bit audio to int32 (`(w << 8) >> 8`) and store the
 *      full 24-bit value in the ring — no truncation. The resampler and the
 *      DSP graph are float, so they consume the full input depth; SPDIF is no
 *      longer needlessly degraded to the 16-bit depth USB happens to deliver.
 *   3. If V or PE bit set, hold last good sample (mute the bad
 *      subframe rather than emit garbage).
 *   4. When we have both an L and an R, write the stereo frame to
 *      spdif_ring at widx and advance widx by 1. Until both halves
 *      are present we cache the orphan in last_sample_*.
 *
 * This runs from DMA1 Stream 2 ISR — keep it tight. 256 words at
 * 48 kHz = 5.3 ms callback-to-callback, easily within budget. */
static void spdif_demux_words(const uint32_t *words, uint16_t count) {
    bool have_l = false, have_r = false;
    int32_t pending_l = last_sample_l;
    int32_t pending_r = last_sample_r;
    uint32_t widx = spdif_ring_widx;

    for (uint16_t i = 0; i < count; ++i) {
        uint32_t w = words[i];
        bool err  = (w & ((1u << 24) | (1u << 25))) != 0; /* PE or !V */
        if (err) {
            if (w & (1u << 24)) parity_errors++;          /* count PE only */
        }
        /* sign-extend 24-bit → int32 and keep full depth (no truncation) */
        int32_t s32 = err ? 0 : ((int32_t)(w << 8) >> 8);

        /* STM32H7 SPDIFRX PT encoding (empirically verified via raw DR
         * snapshot — vendor cmd 0xF8):
         *   PT = 0b01 → B preamble (left, block-start, infrequent)
         *   PT = 0b10 → M preamble (left, regular)
         *   PT = 0b11 → W preamble (right)
         *   PT = 0b00 → no preamble decoded (shouldn't appear in
         *               stable RCV mode)
         * The HAL doesn't expose these constants, and the RM0468
         * description of DR mode 0 doesn't enumerate the PT values.
         * What it boils down to: pt == 0b11 means RIGHT, anything
         * else means LEFT. */
        uint8_t pt = (uint8_t)((w >> 28) & 0x3);
        if (pt == 0x3) {            /* W preamble = right channel */
            pending_r = s32;
            have_r = true;
        } else {                    /* B/M = left channel */
            pending_l = s32;
            have_l = true;
        }
        if (have_l && have_r) {
            uint32_t pos = (widx & SPDIF_RING_MASK) * 2;
            spdif_ring[pos + 0] = pending_l;
            spdif_ring[pos + 1] = pending_r;
            widx++;
            have_l = have_r = false;
        }
    }

    spdif_ring_widx  = widx;
    last_sample_l    = pending_l;
    last_sample_r    = pending_r;
}

void HAL_SPDIFRX_RxHalfCpltCallback(SPDIFRX_HandleTypeDef *h) {
    (void)h;
    spdif_demux_words(&spdifrx_dt_buf[0], SPDIFRX_DT_BUF_WORDS / 2);
}

void HAL_SPDIFRX_RxCpltCallback(SPDIFRX_HandleTypeDef *h) {
    (void)h;
    spdif_demux_words(&spdifrx_dt_buf[SPDIFRX_DT_BUF_WORDS / 2],
                      SPDIFRX_DT_BUF_WORDS / 2);
}

/* Channel-status accumulation: each CSR word carries 1 bit of channel
 * status (DR[26]) + 16 user bits + SOB. Per RM, the SOB bit signals
 * the start of an IEC 60958 block; we accumulate 192 status bits into
 * cs_block_current, then snapshot to cs_block_complete when the next
 * SOB arrives. Stage 2 is not yet decoding sample rate from byte 3
 * (we use WIDTH5 + DMA-fill servo for that); cs_block_complete is
 * exposed verbatim via REQ_GET_SPDIF_RX_CH_STATUS for Console to
 * decode if the user wants pro/consumer mode flags etc. */
static uint32_t cs_bit_index = 0;     /* 0..191                         */
static uint16_t cs_words_seen = 0;    /* sanity guard against runaway   */

static void spdif_demux_cs(const uint32_t *words, uint16_t count) {
    for (uint16_t i = 0; i < count; ++i) {
        uint32_t w = words[i];
        if (w & (1u << 24)) {           /* SOB — start of a new block */
            if (cs_bit_index >= 191) {
                memcpy(cs_block_complete, cs_block_current, 24);
                cs_block_valid = true;
            }
            memset(cs_block_current, 0, 24);
            cs_bit_index = 0;
        }
        if (cs_bit_index < 192) {
            uint32_t byte = cs_bit_index >> 3;
            uint32_t bit  = cs_bit_index & 0x7;
            /* CSR word's [23:16] is the assembled CS byte already, but
             * we want bit-level granularity so we use [16] = LSB of
             * that byte = the C bit for this subframe. */
            if (w & (1u << 16)) cs_block_current[byte] |= (1u << bit);
            cs_bit_index++;
        }
        cs_words_seen++;
    }
}

void HAL_SPDIFRX_CxHalfCpltCallback(SPDIFRX_HandleTypeDef *h) {
    (void)h;
    spdif_demux_cs(&spdifrx_cs_buf[0], SPDIFRX_CS_BUF_WORDS / 2);
}

void HAL_SPDIFRX_CxCpltCallback(SPDIFRX_HandleTypeDef *h) {
    (void)h;
    spdif_demux_cs(&spdifrx_cs_buf[SPDIFRX_CS_BUF_WORDS / 2],
                   SPDIFRX_CS_BUF_WORDS / 2);
}

void HAL_SPDIFRX_ErrorCallback(SPDIFRX_HandleTypeDef *h) {
    (void)h;
    /* Lock loss event. Tally it; the main-loop poll will see the state
     * transition next tick. */
    if (spdif_state == SPDIF_INPUT_LOCKED) {
        spdif_state = SPDIF_INPUT_RELOCKING;
        if (loss_count < 0xFF) loss_count++;
    }
}

static void spdifrx_restart_sync(void) {
    /* The HAL's initial DMA receive call can time out while leaving the
     * peripheral armed in SYNC mode. If the hardware then raises SERR/FERR/
     * TERR during acquisition, nudge the SPDIFEN state machine back through
     * IDLE->SYNC so hot-plug/noisy-start attempts keep retrying. DMA request
     * bits and INSEL stay programmed in CR. */
    SPDIFRX->CR &= ~SPDIFRX_CR_SPDIFEN;
    __DSB();
    SPDIFRX->IFCR = SPDIFRX_IFCR_PERRCF
                  | SPDIFRX_IFCR_OVRCF
                  | SPDIFRX_IFCR_SBDCF
                  | SPDIFRX_IFCR_SYNCDCF;
    SPDIFRX->CR = (SPDIFRX->CR & ~SPDIFRX_CR_SPDIFEN) | SPDIFRX_STATE_SYNC;
}

/* ----- Public API ----- */

void spdif_input_init(void) {
    /* Idempotent — bail if already initialised. */
    if (hw_initialised) return;

    hspdif.Instance                       = SPDIFRX;
    hspdif.Init.InputSelection            = SPDIFRX_INPUT_IN1;       /* PD8 / D8 */
    hspdif.Init.Retries                   = SPDIFRX_MAXRETRIES_63;   /* most tolerant */
    /* WFA=ON per Linux stm32_spdifrx driver: "avoids issuing sync
     * errors when SPDIF signal is missing on input." Without WFA the
     * sync state machine starts the retry counter immediately and
     * burns through NBTR retries before any signal can be measured. */
    hspdif.Init.WaitForActivity           = SPDIFRX_WAITFORACTIVITY_ON;
    hspdif.Init.ChannelSelection          = SPDIFRX_CHANNEL_A;
    hspdif.Init.DataFormat                = SPDIFRX_DATAFORMAT_LSB;  /* mode 0 */
    hspdif.Init.StereoMode                = SPDIFRX_STEREOMODE_ENABLE;
    hspdif.Init.PreambleTypeMask          = SPDIFRX_PREAMBLETYPEMASK_OFF;
    hspdif.Init.ChannelStatusMask         = SPDIFRX_CHANNELSTATUS_OFF;
    hspdif.Init.ValidityBitMask           = SPDIFRX_VALIDITYMASK_OFF;
    hspdif.Init.ParityErrorMask           = SPDIFRX_PARITYERRORMASK_OFF;
    hspdif.Init.SymbolClockGen            = DISABLE;
    hspdif.Init.BackupSymbolClockGen      = DISABLE;
    if (HAL_SPDIFRX_Init(&hspdif) != HAL_OK) Error_Handler();

    /* Software polyphase resampler (Walch / Smith). PLL2 stays nailed
     * at NOMINAL; rate matching is done by interpolation, driven by
     * a PI controller on ring-fill error in seconds. */
    resampler_init();

    hw_initialised = true;
    spdif_state    = SPDIF_INPUT_INACTIVE;
}

void spdif_input_start(void) {
    if (!hw_initialised) spdif_input_init();
    if (spdif_state != SPDIF_INPUT_INACTIVE) return;  /* already running */

    /* Reset lock-cycle counters so the new acquisition session starts
     * clean. parity_errors is cumulative across sessions on purpose. */
    lock_count   = 0;
    loss_count   = 0;
    last_sync_retry_ms = 0;
    cs_block_valid = false;
    memset(cs_block_current,  0, sizeof(cs_block_current));
    memset(cs_block_complete, 0, sizeof(cs_block_complete));

    spdif_state = SPDIF_INPUT_ACQUIRING;

    /* DEBUG (M9): capture the HAL result + DMA EN bit + hspdif state
     * for the source-switch silent-failure bug. Volatile so the probe
     * (vendor 0xED) can read while audio path is hot. */
    extern volatile uint32_t spdif_start_call_count;
    extern volatile uint32_t spdif_start_dt_rc;
    extern volatile uint32_t spdif_start_cs_rc;
    extern volatile uint32_t spdif_start_pre_hspdif_state;
    extern volatile uint32_t spdif_start_dt_dma_state;
    extern volatile uint32_t spdif_start_pre_cr;
    spdif_start_call_count++;
    spdif_start_pre_hspdif_state = hspdif.State;
    spdif_start_dt_dma_state = hdma_spdifrx_dt.State;
    spdif_start_pre_cr = (uint32_t)(DMA1_Stream2->CR & 0xFFFF);

    /* HAL_SPDIFRX_ReceiveDataFlow_DMA does the DR DMA arming + sets
     * SPDIFEN=01 (SYNC mode) + polls SR.SYNCD for ~3 ms via a count-
     * down loop. With no signal connected the poll times out and HAL
     * returns HAL_TIMEOUT — but the peripheral is left in SYNC mode
     * and will catch a signal whenever it arrives. We treat HAL_OK
     * and HAL_TIMEOUT identically here: keep state at ACQUIRING and
     * let spdif_input_poll() finalise the transition to LOCKED when
     * it sees SR.SYNCD go high. Anything else is a real failure. */
    HAL_StatusTypeDef rc = HAL_SPDIFRX_ReceiveDataFlow_DMA(&hspdif,
                                                          spdifrx_dt_buf,
                                                          SPDIFRX_DT_BUF_WORDS);
    spdif_start_dt_rc = (uint32_t)rc;
    if (rc != HAL_OK && rc != HAL_TIMEOUT) {
        spdif_state = SPDIF_INPUT_INACTIVE;
        return;
    }

    /* Arm the CSR DMA the same way. HAL_SPDIFRX_ReceiveCtrlFlow_DMA
     * also polls for SYNCD if the peripheral isn't already in RCV
     * state — same timeout semantics, same forgiveness. */
    rc = HAL_SPDIFRX_ReceiveCtrlFlow_DMA(&hspdif, spdifrx_cs_buf,
                                         SPDIFRX_CS_BUF_WORDS);
    spdif_start_cs_rc = (uint32_t)rc;
    if (rc != HAL_OK && rc != HAL_TIMEOUT) {
        /* CSR failure isn't fatal — we can still receive audio,
         * we just won't track channel status. Leave state at
         * ACQUIRING, the data path may still succeed. */
    }
}

void spdif_input_stop(void) {
    if (spdif_state == SPDIF_INPUT_INACTIVE) return;
    /* HAL_SPDIFRX_DMAStop writes EN=0 and returns — it does NOT wait
     * for the DMA streams to actually disable. On a fast restart
     * (USB→SPDIF→USB→SPDIF) the next HAL_DMA_Start_IT can see EN still
     * set and return HAL_BUSY, which our start() interprets as a hard
     * failure (silently re-INACTIVE → source switch ignored).
     *
     * HAL_DMA_Abort polls EN with a 5 ms timeout. The SPDIFRX peripheral
     * sometimes holds the DMA's request line long enough that the abort
     * times out and leaves hdma->State = HAL_DMA_STATE_TIMEOUT, even
     * though EN actually does clear shortly after. State=TIMEOUT then
     * makes the next HAL_DMA_Start_IT reject.
     *
     * Belt-and-braces: call Abort (best-effort), then DMAStop, then
     * explicitly force the DMA handle state back to READY so the next
     * start succeeds regardless of how the abort wait went. The
     * hardware EN bit is already cleared by both abort and stop. */
    HAL_DMA_Abort(&hdma_spdifrx_dt);
    HAL_DMA_Abort(&hdma_spdifrx_cs);
    HAL_SPDIFRX_DMAStop(&hspdif);
    hdma_spdifrx_dt.State = HAL_DMA_STATE_READY;
    hdma_spdifrx_dt.ErrorCode = HAL_DMA_ERROR_NONE;
    hdma_spdifrx_cs.State = HAL_DMA_STATE_READY;
    hdma_spdifrx_cs.ErrorCode = HAL_DMA_ERROR_NONE;
    hspdif.State = HAL_SPDIFRX_STATE_READY;
    hspdif.ErrorCode = HAL_SPDIFRX_ERROR_NONE;
    spdif_state = SPDIF_INPUT_INACTIVE;
    sample_rate_hz = 0;
    /* Reset resampler state on stop so the next lock starts clean. */
    servo_int_acc = 0;
    servo_last_fracn = (int32_t)FRACN_NOMINAL;
    servo_fill_lpf = SPDIF_RING_TARGET_FILL_FWD;
    servo_filtered_diff = 0.0;
    resampler_reset();
    /* Reset the ring state too — any partially-demuxed frames left
     * over are stale by the time we re-arm. */
    spdif_ring_widx = 0;
    spdif_ring_ridx = 0;
    last_sample_l = last_sample_r = 0;
}

uint32_t spdif_input_poll(void) {
    if (spdif_state == SPDIF_INPUT_INACTIVE) return 0;

    /* Read SPDIFRX_SR. The peripheral exposes:
     *   bit 5  SYNCD  — synchronisation done (locked to preambles)
     *   [30:16] WIDTH5 — duration of 5 BMC cells in spdif_clk periods
     * SYNCD goes high once we lock, stays high while locked. */
    uint32_t sr = SPDIFRX->SR;

    const uint32_t sync_error_flags = SPDIFRX_SR_FERR
                                    | SPDIFRX_SR_SERR
                                    | SPDIFRX_SR_TERR;
    if (sr & sync_error_flags) {
        if (spdif_state == SPDIF_INPUT_LOCKED) {
            spdif_state = SPDIF_INPUT_RELOCKING;
            if (loss_count < 0xFF) loss_count++;
        }
        if (spdif_state == SPDIF_INPUT_ACQUIRING ||
            spdif_state == SPDIF_INPUT_RELOCKING) {
            uint32_t now = HAL_GetTick();
            if (last_sync_retry_ms == 0 ||
                now - last_sync_retry_ms >= 100U) {
                last_sync_retry_ms = now;
                spdifrx_restart_sync();
            }
        }
    } else if (sr & SPDIFRX_SR_SYNCD) {
        if (spdif_state != SPDIF_INPUT_LOCKED) {
            /* Transition: if the peripheral is still in SYNC mode
             * (which it will be after the HAL_TIMEOUT path through
             * spdif_input_start, since HAL stopped polling but left
             * SPDIFEN=01), bump it to RCV mode so data actually
             * flows out of DR. Idempotent — if it's already RCV
             * the OR is a no-op. */
            if ((SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) != SPDIFRX_STATE_RCV) {
                __HAL_SPDIFRX_RCV(&hspdif);
            }
            spdif_state = SPDIF_INPUT_LOCKED;
            if (lock_count < 0xFF) lock_count++;
            /* Re-configure the resampler at the newly-detected source
             * rate. PLL2_P is fixed at NOMINAL → SAI output is 48 kHz
             * (or whatever the configured nominal is). The resampler
             * computes the ratio at this point and builds the Kaiser-
             * windowed sinc prototype for it. */
            /* Reset deferred to first lock-poll where we have a valid
             * snapped sample_rate_hz — see below. */
        }
        /* Sample-rate from WIDTH5: Fs ≈ kclk / (WIDTH5 × 64). The
         * formula is per RM0468 §38.4.7. WIDTH5 is unsigned 15-bit. */
        uint32_t width5 = (sr >> 16) & 0x7FFF;
        if (width5 != 0) {
            uint32_t fs = (5U * SPDIFRX_KERNEL_CLOCK_HZ) / (width5 * 64U);
            /* Snap to nearest standard rate within ±2% tolerance —
             * WIDTH5 has a few percent of jitter from biphase clock
             * recovery. Helps Console show "48000" instead of "48137". */
            static const uint32_t std_rates[] = {
                32000, 44100, 48000, 88200, 96000, 176400, 192000
            };
            for (size_t i = 0; i < sizeof(std_rates)/sizeof(std_rates[0]); ++i) {
                uint32_t r  = std_rates[i];
                uint32_t lo = r - r / 50;  /* −2% */
                uint32_t hi = r + r / 50;  /* +2% */
                if (fs >= lo && fs <= hi) { fs = r; break; }
            }
            sample_rate_hz = fs;

            /* Configure the resampler when we first see a valid snapped
             * source rate — happens once per lock acquisition. */
            if (!resampler_is_initialised()) {
                resampler_configure((double)fs, 48000.0);
            }
        }
    }

    /* Stage 2: PLL2 FRACN servo runs from main-loop poll cadence. The
     * fill-level error is the difference between the SPDIF input ring
     * write head (advanced by DMA callbacks at the source rate) and
     * the read head (advanced by fill_half at the SAI output rate).
     * If the source is faster than the sink, gap grows → speed up
     * SAI by increasing PLL2_P. If slower, shrink PLL2_P.
     *
     * Pure integral controller: K_I × fill_error / 100 ms. With the
     * fill-error deadband at ±1 frame and slew-limited to ±2 FRACN
     * steps per call (~2.5 ppm / 100 ms = 25 ppm/sec — well below the
     * 100 ppm/sec safe slew limit), the loop is unconditionally
     * stable for any source within ±1000 ppm of nominal. Lock time
     * is ~10 s for a 50 ppm offset, which matches typical consumer
     * SPDIF source variability. */
    if (spdif_state == SPDIF_INPUT_LOCKED) {
        servo_tick();
    }

    return 0;
}

/* Scratch buffers for the resampler — placed in AXI SRAM so they
 * don't pile onto DTCM BSS. Sized to cover one fill_half call of
 * 192 output frames at up to 4× input ratio (192k → 48k). */
#define POP_INPUT_MAX_FRAMES   512
#define POP_OUTPUT_MAX_FRAMES  256

#define POP_BUF_INPUT_L_ADDR   0x24020000UL
#define POP_BUF_INPUT_R_ADDR   0x24020800UL
#define POP_BUF_OUTPUT_L_ADDR  0x24021000UL
#define POP_BUF_OUTPUT_R_ADDR  0x24021400UL

static float * const pop_in_l  = (float *)POP_BUF_INPUT_L_ADDR;
static float * const pop_in_r  = (float *)POP_BUF_INPUT_R_ADDR;
static float * const pop_out_l = (float *)POP_BUF_OUTPUT_L_ADDR;
static float * const pop_out_r = (float *)POP_BUF_OUTPUT_R_ADDR;

/* 24-bit full-scale reciprocal: ring samples are sign-extended 24-bit, so
 * [-2^23, 2^23-1] maps to ~[-1, 1). */
#define SPDIF_INT24_RECIP   (1.0f / 8388608.0f)

/* Pop `want_frames` raw input frames from the demux ring into the
 * pop_in_l / pop_in_r float buffers (24-bit int → float [-1, 1]).
 * Returns the number actually popped. */
static uint32_t pop_raw_to_float(uint32_t want_frames) {
    uint32_t widx = spdif_ring_widx;
    uint32_t ridx = spdif_ring_ridx;
    uint32_t avail = widx - ridx;
    if (avail > SPDIF_RING_FRAMES) {
        ridx = widx - SPDIF_RING_FRAMES;
        avail = SPDIF_RING_FRAMES;
    }
    uint32_t got = (avail < want_frames) ? avail : want_frames;
    if (got > POP_INPUT_MAX_FRAMES) got = POP_INPUT_MAX_FRAMES;
    for (uint32_t i = 0; i < got; ++i) {
        uint32_t pos = ((ridx + i) & SPDIF_RING_MASK) * 2;
        pop_in_l[i] = (float)spdif_ring[pos + 0] * SPDIF_INT24_RECIP;
        pop_in_r[i] = (float)spdif_ring[pos + 1] * SPDIF_INT24_RECIP;
    }
    spdif_ring_ridx = ridx + got;
    return got;
}

/* Consumer API used by audio_out.c::fill_half when SPDIF is the active
 * input. Reads the full-depth 24-bit frames from the demux ring, converts
 * to float [-1, 1], passes through the polyphase resampler (which
 * interpolates to the SAI rate based on its current `step` ratio), and
 * writes the resampler's native float output straight into `dst` — no
 * intermediate integer requantization, so the only depth loss in the whole
 * SPDIF path is the resampler itself. The PI servo (servo_tick) drives
 * `step` so the ring fill stays at target. `dst` is interleaved L/R float. */
uint32_t spdif_input_pop_frames(float *dst, uint32_t want_frames) {
    if (want_frames == 0) return 0;
    if (want_frames > POP_OUTPUT_MAX_FRAMES) want_frames = POP_OUTPUT_MAX_FRAMES;

    /* If resampler isn't yet configured (we're acquiring), fall back
     * to direct ring read (no resampling — typically zeros until lock). */
    if (!resampler_is_initialised()) {
        uint32_t widx = spdif_ring_widx;
        uint32_t ridx = spdif_ring_ridx;
        uint32_t avail = widx - ridx;
        if (avail > SPDIF_RING_FRAMES) {
            ridx = widx - SPDIF_RING_FRAMES;
            avail = SPDIF_RING_FRAMES;
        }
        uint32_t got = (avail < want_frames) ? avail : want_frames;
        for (uint32_t i = 0; i < got; ++i) {
            uint32_t pos = ((ridx + i) & SPDIF_RING_MASK) * 2;
            dst[i * 2 + 0] = (float)spdif_ring[pos + 0] * SPDIF_INT24_RECIP;
            dst[i * 2 + 1] = (float)spdif_ring[pos + 1] * SPDIF_INT24_RECIP;
        }
        spdif_ring_ridx = ridx + got;
        for (uint32_t i = got; i < want_frames; ++i) {
            dst[i * 2 + 0] = 0.0f;
            dst[i * 2 + 1] = 0.0f;
        }
        return got;
    }

    /* Estimate input needed = want × step + halfFilterLength margin. */
    double step = resampler_get_step();
    int32_t halfLen = resampler_get_half_filter_length();
    uint32_t need = (uint32_t)((double)want_frames * step) + halfLen + 4;
    if (need > POP_INPUT_MAX_FRAMES) need = POP_INPUT_MAX_FRAMES;
    uint32_t got = pop_raw_to_float(need);

    int32_t processed = 0, out_count = 0;
    resampler_resample(pop_in_l, pop_in_r, (int32_t)got, &processed,
                       pop_out_l, pop_out_r, (int32_t)want_frames, &out_count);

    /* If the resampler consumed less than we provided, push the
     * un-consumed tail back into the ring by rewinding ridx. */
    if (processed < (int32_t)got) {
        spdif_ring_ridx -= (uint32_t)((int32_t)got - processed);
    }

    /* Hand the resampler's float output straight to the DSP graph. */
    for (int32_t i = 0; i < out_count; ++i) {
        dst[i * 2 + 0] = pop_out_l[i];
        dst[i * 2 + 1] = pop_out_r[i];
    }
    /* Silence-pad shortfall. */
    for (int32_t i = out_count; i < (int32_t)want_frames; ++i) {
        dst[i * 2 + 0] = 0.0f;
        dst[i * 2 + 1] = 0.0f;
    }
    return (uint32_t)out_count;
}

/* ---- PLL2 FRACN servo ----
 * Fill-level integral control. Target a steady fill of ~half the
 * ring (512 frames at 1024-frame ring) to give symmetric headroom
 * either side of nominal lock.
 *
 * FRACN nominal for our PLL2 config (DIVM=5, DIVN=98, FRACN=2490,
 * DIVP=10): live in main.c's SystemClock_Config. We snapshot it on
 * first servo entry and slew around it. Range ±5000 ppm (full FRACN
 * range) but we clamp to ±200 ppm in practice via the integral
 * accumulator clamp — that's > 4× the worst-case consumer SPDIF
 * tolerance of 50 ppm. */
#define FRACN_MIN              0U
#define FRACN_MAX              8191U
#define SPDIF_RING_TARGET_FILL 512    /* ring at half-full = balanced */
/* Deadband: the instantaneous ring fill swings by ±256 frames just
 * from producer (256-frame DMA bursts) vs consumer (192-frame SAI
 * bursts) being at different periods. That oscillation is pure
 * measurement noise — not real drift. We compensate by LPF'ing the
 * fill and using a generous deadband; both together keep the servo
 * from chasing burst-induced phantom drift. */
#define SERVO_DEADBAND_FRAMES  32     /* ±32 frames (filtered) before reacting */
#define SERVO_SLEW_PER_TICK    1      /* max ±1 FRACN step / 100 ms    */
#define SERVO_INT_CLAMP_STEPS  400    /* integral accumulator clamp.
                                       * 400 × 1.24 ppm = ±496 ppm pull
                                       * range. Empirically, with the
                                       * dither engine now handling the
                                       * actual PLL writes, the servo
                                       * needs to transiently overshoot
                                       * past steady-state to push the
                                       * ring fill from its acquisition
                                       * floor (~256 frames) up to target
                                       * (512). Wider clamp gives that
                                       * room. */
#define SERVO_LOCK_TIMEOUT_MS  100    /* min interval between updates  */
/* Minimum FRACN delta before issuing a pll2_fracn_write. Each write
 * does a FRACEN disable→reconfig→enable cycle that briefly perturbs
 * PLL2_P (the SAI kernel clock) AND PLL2_R (the SPDIFRX kernel
 * clock). The latter can cause the biphase decoder to glitch and
 * raise FERR → spurious lock loss. So we want writes to be rare:
 * accumulator continues tracking sub-threshold drift, but the actual
 * register write waits until the change is meaningful. At
 * ~1.24 ppm per FRACN step, ±8 = ~10 ppm — invisible to DACs, well
 * below SPDIFRX biphase tolerance. */
#define SERVO_FRACN_WRITE_THRESHOLD 8
/* Fill LPF: first-order IIR with alpha = 1/32 (5-tick time constant
 * ≈ 0.5 s at 100 ms ticks). Smooths the ±256-frame burstiness so
 * the servo sees the slow-drift component the source/sink rates
 * actually exhibit. */
#define SERVO_FILL_LPF_SHIFT   5

static int32_t  servo_int_acc      = 0;     /* declared above; defined here */
static uint32_t servo_last_tick_ms = 0;

static void pll2_fracn_write(uint32_t new_fracn) {
    /* DEPRECATED: kept for legacy spdif_input_stop() reset path, but
     * the servo no longer calls this directly. CPU-driven FRACN writes
     * produce sigma-delta slew transients with audible-band spectral
     * content, even with the documented disable-write-enable sequence.
     * The audio servo now uses the BDMA-pumped dither engine
     * (pll_dither.c) which lives entirely in hardware and modulates
     * FRACN at >50 kHz where transients are inaudible.
     *
     * This function still exists because we want a single deterministic
     * write to push FRACN back to NOMINAL when SPDIF input is stopped —
     * a one-shot inaudible glitch we accept on source switch. */
    __HAL_RCC_PLL2FRACN_DISABLE();
    __HAL_RCC_PLL2FRACN_CONFIG(new_fracn);
    __HAL_RCC_PLL2FRACN_ENABLE();
}

/* Last FRACN value we wrote — used to skip redundant pll2_fracn_write
 * calls. Each write briefly disables and re-enables the FRACN modulator,
 * which glitches PLL2_P (and therefore SAI), so we only want to write
 * when the target actually changes. */
static int32_t servo_last_fracn = (int32_t)FRACN_NOMINAL;

/* Diagnostic counter for vendor cmd 0xF7 — how many times the servo
 * has actually moved FRACN. If this stays 0 while LOCKED, the servo
 * is in deadband (good) or starved (bad). */
static uint32_t servo_fracn_writes = 0;

/* Diagnostic: when non-zero, the servo skips its pll2_fracn_write call
 * entirely. Accumulator/LPF still update so we can see what the servo
 * *would* have done; the PLL just isn't touched. Used to test whether
 * the audible dropouts originate from the FRACN-write disturbance vs
 * something elsewhere in the path. Set via vendor cmd 0xF9. */
static volatile uint8_t servo_frozen = 0;
void spdif_input_set_servo_frozen(uint8_t frozen) { servo_frozen = frozen; }
uint8_t spdif_input_get_servo_frozen(void) { return servo_frozen; }

/* Low-pass filtered ring fill — fed by every servo_tick() entry,
 * read by both the servo logic and the debug snapshot. Initialised
 * to TARGET so the servo doesn't see a huge spike on first tick.
 * Declared static (defined here, forward-decl'd above for the
 * spdif_input_stop reset). */
static int32_t servo_fill_lpf = SPDIF_RING_TARGET_FILL_FWD;

/* Bang-bang servo state. NORM = currently-applied "center" FRACN
 * (initialised to nominal, may be re-centered by the open-loop rate
 * tracker as the source drifts further than ±1 LSB from nominal). */
typedef enum {
    DITHER_FAST,    /* center+1: PLL slightly faster, SAI consumes faster */
    DITHER_NORM,    /* center:   nominal rate                             */
    DITHER_SLOW,    /* center−1: PLL slightly slower, SAI consumes slower */
} dither_mode_t;
static dither_mode_t servo_last_mode  = DITHER_NORM;
static int32_t       servo_center_fracn = (int32_t)FRACN_NOMINAL;

/* widx-rate tracker for the open-loop center adjustment. */
static uint32_t servo_last_widx_snap     = 0;
static uint32_t servo_last_widx_snap_ms  = 0;

#define SAI_NOMINAL_FS  48000U
#define DITHER_DEADBAND_FRAMES  64    /* ±64 frames around target = NORM */

/* SAI Fs to PLL2_P ratio. With M=5, N=98, P=10, MCKDIV=8, and 32-bit
 * BCK frame slots × 2 channels, total divider from PLL2_P to SAI_Fs is
 * 2 × MCKDIV × 64 = 1024. So PLL2_P = SAI_Fs × 1024.
 *
 * To get target FRACN from target SAI_Fs:
 *   PLL2_P = (HSE/M) × (N + FRACN/8192) / P
 *          = 500000 × (98 + FRACN/8192)
 *   SAI_Fs × 1024 = 500000 × (98 + FRACN/8192)
 *   FRACN = ((SAI_Fs × 1024 / 500000) - 98) × 8192
 *
 * Working in integer math (target_fs in fps):
 *   FRACN = ((target_fs × 1024 - 49000000) × 8192) / 500000
 *
 * The widx rate measurement gives target_fs directly:
 *   target_fs = delta_widx × 1000 / delta_ms
 *
 * Substituting:
 *   FRACN = ((delta_widx × 1024000 / delta_ms - 49000000) × 8192) / 500000
 *         = (delta_widx × 1024000 × 8192 / delta_ms - 49000000 × 8192) / 500000
 *
 * Operating bounds at 48 kHz, 1 s window:
 *   delta_widx ≈ 48000
 *   delta_widx × 8388608000 ≈ 4 × 10^14 — fits in 64-bit
 */
/* PI controller driving the resampler's `step` ratio. Runs at the
 * resampler's `periodeLength` cadence (default 128 output samples
 * ≈ 2.7 ms at 48 kHz). The error term is the deviation of the SPDIF
 * input ring's fill from a target latency, expressed in SECONDS.
 *
 * We low-pass the fill measurement before feeding the PI loop so
 * that the producer/consumer burstiness (±256 frames swing in our
 * ring) doesn't propagate into the resampler ratio as wow/flutter. */
#define SERVO_TARGET_LATENCY_S  0.010    /* 10 ms target = 480 frames @48k */
#define SERVO_PI_TICK_MS        10       /* call updateIncrement at 100 Hz */

static double  servo_filtered_diff = 0.0;
static uint32_t servo_pi_last_ms   = 0;

static void servo_tick(void) {
    if (spdif_state != SPDIF_INPUT_LOCKED) return;
    if (!resampler_is_initialised())       return;

    uint32_t now = HAL_GetTick();
    if (now - servo_pi_last_ms < SERVO_PI_TICK_MS) return;
    servo_pi_last_ms = now;

    /* Snapshot ring fill. Source-rate is what the SPDIFRX peripheral
     * detected — use that to convert "frames in ring" into seconds. */
    int32_t fill = (int32_t)(spdif_ring_widx - spdif_ring_ridx);
    if (fill < 0 || fill > (int32_t)SPDIF_RING_FRAMES) return;
    if (sample_rate_hz == 0) return;
    double fill_s   = (double)fill / (double)sample_rate_hz;
    double raw_diff = fill_s - SERVO_TARGET_LATENCY_S;

    /* First-order IIR LPF on the diff. Aggressive smoothing — α=0.01
     * gives a ~1 s time constant at 100 Hz tick. The PI loop is slow,
     * so a slow LPF prevents burst-driven hunting. */
    servo_filtered_diff += (raw_diff - servo_filtered_diff) * 0.01;

    bool settled = resampler_update_increment(servo_filtered_diff);
    (void)settled;

    /* Diagnostic telemetry — convert step→ppm for the probe. */
    double step_ratio = resampler_get_step();
    double cfg_step   = resampler_get_configured_step();
    if (cfg_step > 0.0) {
        double ppm = (step_ratio / cfg_step - 1.0) * 1e6;
        servo_int_acc = (int32_t)ppm;
    }
    servo_fracn_writes++;
}

void spdif_input_get_status(SpdifRxStatusPacket *out) {
    if (!out) return;
    out->state           = (uint8_t)spdif_state;
    out->input_source    = active_input_source;
    out->lock_count      = lock_count;
    out->loss_count      = loss_count;
    out->sample_rate     = (spdif_state == SPDIF_INPUT_LOCKED) ? sample_rate_hz : 0;
    out->parity_errors   = parity_errors;
    out->fifo_fill_pct   = 0;          /* Stage 2 will compute from DMA NDTR */
    out->reserved        = 0;
}

void spdif_input_get_channel_status(uint8_t *out_24_bytes) {
    if (!out_24_bytes) return;
    if (cs_block_valid) memcpy(out_24_bytes, cs_block_complete, 24);
    else                memset(out_24_bytes, 0, 24);
}

extern volatile uint32_t audio_underruns;
extern volatile uint32_t audio_fill_peak_cycles;
extern const    uint32_t audio_fill_budget_cycles;

void spdif_input_get_servo_debug(SpdifServoDebugPacket *out) {
    if (!out) return;
    uint32_t widx = spdif_ring_widx;
    uint32_t ridx = spdif_ring_ridx;
    int32_t  fill = (int32_t)(widx - ridx);
    out->fill           = fill;
    out->err            = fill - (int32_t)SPDIF_RING_TARGET_FILL;
    out->int_acc        = servo_int_acc;
    out->current_fracn  = servo_last_fracn;
    out->fracn_writes   = servo_fracn_writes;
    out->widx           = widx;
    out->ridx           = ridx;
    out->underruns      = audio_underruns;
    /* Snapshot-and-reset so each probe gets a fresh peak observation. */
    out->peak_cycles    = audio_fill_peak_cycles;
    audio_fill_peak_cycles = 0;
    out->budget_cycles  = audio_fill_budget_cycles;
}

#else  /* not STM32H723xx */

void spdif_input_init(void)               { }
void spdif_input_start(void)              { }
void spdif_input_stop(void)               { }
uint32_t spdif_input_poll(void)           { return 0; }
uint32_t spdif_input_pop_frames(float *dst, uint32_t want_frames) {
    if (dst) for (uint32_t i = 0; i < want_frames * 2; ++i) dst[i] = 0.0f;
    return 0;
}
void spdif_input_get_status(SpdifRxStatusPacket *out) {
    if (out) memset(out, 0, sizeof(*out));
}
void spdif_input_get_channel_status(uint8_t *out_24_bytes) {
    if (out_24_bytes) memset(out_24_bytes, 0, 24);
}

#endif  /* STM32H723xx */
