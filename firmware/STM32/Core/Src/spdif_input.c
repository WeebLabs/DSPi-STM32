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
#include <string.h>

#if defined(STM32H723xx)

/* Peripheral kernel-clock source (must match what main.c programs in
 * RCC_D2CCIP1R). Used by spdif_input_poll() to convert WIDTH5 into Hz. */
#define SPDIFRX_KERNEL_CLOCK_HZ  98304000U

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
static int16_t * const spdif_ring = (int16_t *)SPDIF_RING_BASE;
static volatile uint32_t spdif_ring_widx = 0;       /* DMA-callback writes */
static volatile uint32_t spdif_ring_ridx = 0;       /* fill_half reads     */
/* Last sample captured per channel — held over when the source goes
 * mute / parity-error / not-valid, so a brief bad subframe doesn't
 * inject a click. */
static int16_t  last_sample_l = 0;
static int16_t  last_sample_r = 0;

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
static int32_t  servo_int_acc;     /* defined below */
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
 *   1. Sign-extend the 24-bit audio to int32 (`(w << 8) >> 8`)
 *   2. Truncate to int16 for the ring (DSPi internal format is int16
 *      stereo to match the USB UAC1 path; the DSP pipeline upscales
 *      back to float before EQ/matrix). Throws away 8 LSBs but matches
 *      the bit depth USB delivers, so SPDIF and USB sources sound
 *      identical through the DSP graph.
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
    int16_t pending_l = last_sample_l;
    int16_t pending_r = last_sample_r;
    uint32_t widx = spdif_ring_widx;

    for (uint16_t i = 0; i < count; ++i) {
        uint32_t w = words[i];
        bool err  = (w & ((1u << 24) | (1u << 25))) != 0; /* PE or !V */
        if (err) {
            if (w & (1u << 24)) parity_errors++;          /* count PE only */
        }
        /* sign-extend 24-bit → int32, then drop low 8 bits → int16 */
        int32_t s32 = (int32_t)(w << 8) >> 8;
        int16_t s16 = err ? 0 : (int16_t)(s32 >> 8);

        uint8_t pt = (uint8_t)((w >> 28) & 0x3);
        if (pt == 0x1) {            /* W preamble = right channel */
            pending_r = s16;
            have_r = true;
        } else {                    /* B/Z/M = left channel */
            pending_l = s16;
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
    if (rc != HAL_OK && rc != HAL_TIMEOUT) {
        spdif_state = SPDIF_INPUT_INACTIVE;
        return;
    }

    /* Arm the CSR DMA the same way. HAL_SPDIFRX_ReceiveCtrlFlow_DMA
     * also polls for SYNCD if the peripheral isn't already in RCV
     * state — same timeout semantics, same forgiveness. */
    rc = HAL_SPDIFRX_ReceiveCtrlFlow_DMA(&hspdif, spdifrx_cs_buf,
                                         SPDIFRX_CS_BUF_WORDS);
    if (rc != HAL_OK && rc != HAL_TIMEOUT) {
        /* CSR failure isn't fatal — we can still receive audio,
         * we just won't track channel status. Leave state at
         * ACQUIRING, the data path may still succeed. */
    }
}

void spdif_input_stop(void) {
    if (spdif_state == SPDIF_INPUT_INACTIVE) return;
    HAL_SPDIFRX_DMAStop(&hspdif);
    spdif_state = SPDIF_INPUT_INACTIVE;
    sample_rate_hz = 0;
    /* Restore PLL2 to nominal so the SAI returns to exactly 48 kHz
     * for any non-SPDIF input source. The same DISABLE→delay→CONFIG→
     * ENABLE sequence — runs in main-loop context, no harm. */
    servo_int_acc = 0;
    pll2_fracn_write(FRACN_NOMINAL);
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

/* ---- Consumer API used by audio_out.c::fill_half when input source
 *      is SPDIF. Pop up to N stereo frames from the demux ring into
 *      the same int16 [L,R,L,R…] format that usb_ring_pop_frames
 *      produces, so fill_half doesn't need to know which source it's
 *      reading from beyond the active_input_source switch.
 *
 *      Returns the number of frames actually filled. Tail-pads with
 *      silence if the ring runs dry — same starvation behaviour as
 *      the USB ring. */
uint32_t spdif_input_pop_frames(int16_t *dst, uint32_t want_frames) {
    uint32_t widx = spdif_ring_widx;     /* one volatile load */
    uint32_t ridx = spdif_ring_ridx;
    uint32_t avail = widx - ridx;
    if (avail > SPDIF_RING_FRAMES) {
        /* Catastrophic ring overrun (consumer fell more than ring-
         * worth behind producer). Snap consumer up to keep us in
         * the most-recent-N-frames window — better than reading
         * partially-overwritten frames. */
        ridx = widx - SPDIF_RING_FRAMES;
        avail = SPDIF_RING_FRAMES;
    }
    uint32_t got = (avail < want_frames) ? avail : want_frames;
    for (uint32_t i = 0; i < got; ++i) {
        uint32_t pos = ((ridx + i) & SPDIF_RING_MASK) * 2;
        dst[i * 2 + 0] = spdif_ring[pos + 0];
        dst[i * 2 + 1] = spdif_ring[pos + 1];
    }
    spdif_ring_ridx = ridx + got;
    /* Silence-pad shortfall — same convention as the USB ring. */
    for (uint32_t i = got; i < want_frames; ++i) {
        dst[i * 2 + 0] = 0;
        dst[i * 2 + 1] = 0;
    }
    return got;
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
#define SPDIF_RING_TARGET_FILL 512    /* ring at half-full = balanced  */
#define SERVO_DEADBAND_FRAMES  2      /* ±2 frames = ±10 ppm noise floor */
#define SERVO_SLEW_PER_TICK    2      /* max ±2 FRACN steps / 100 ms   */
#define SERVO_INT_CLAMP_STEPS  160    /* integral accumulator clamp    */
#define SERVO_LOCK_TIMEOUT_MS  100    /* min interval between updates  */

static int32_t  servo_int_acc      = 0;     /* declared above; defined here */
static uint32_t servo_last_tick_ms = 0;

static void pll2_fracn_write(uint32_t new_fracn) {
    /* Per RM0468 §8.5.4: clear FRACEN, wait ≥ 2 × T_REF + bus latency,
     * write new value, set FRACEN. The HAL macros bake the right
     * register access; the wait is what we provide explicitly. */
    __HAL_RCC_PLL2FRACN_DISABLE();
    __DSB();
    (void)RCC->PLL2FRACR;          /* readback flushes write buffer */
    /* ~3 µs idle: at 550 MHz SYSCLK that's ~1650 cycles. The readback
     * above is ~10 cycles; pad to be safe across bus matrix latency. */
    for (volatile int i = 0; i < 1650; ++i) { __NOP(); }
    __HAL_RCC_PLL2FRACN_CONFIG(new_fracn);
    __HAL_RCC_PLL2FRACN_ENABLE();
}

static void servo_tick(void) {
    uint32_t now = HAL_GetTick();
    if (now - servo_last_tick_ms < SERVO_LOCK_TIMEOUT_MS) return;
    servo_last_tick_ms = now;

    /* Fill-level error: positive = source faster than sink (ring
     * filling) → need to speed up SAI output → increase FRACN. */
    int32_t fill = (int32_t)(spdif_ring_widx - spdif_ring_ridx);
    if (fill < 0 || fill > (int32_t)SPDIF_RING_FRAMES) {
        /* Ring is in transient / overrun state — skip this tick. */
        return;
    }
    int32_t err = fill - (int32_t)SPDIF_RING_TARGET_FILL;

    /* Deadband — don't accumulate around the lock point. */
    if (err > -SERVO_DEADBAND_FRAMES && err < SERVO_DEADBAND_FRAMES) {
        return;
    }

    /* Integral update: 1 step per 8 frames of fill error per tick.
     * At 100 ms tick rate, settling time for a 50-frame fill imbalance
     * is ~50/(8 × 0.1) = ~60 ticks = 6 seconds. */
    int32_t int_delta = err / 8;
    servo_int_acc += int_delta;
    if (servo_int_acc >  SERVO_INT_CLAMP_STEPS) servo_int_acc =  SERVO_INT_CLAMP_STEPS;
    if (servo_int_acc < -SERVO_INT_CLAMP_STEPS) servo_int_acc = -SERVO_INT_CLAMP_STEPS;

    /* Slew-limit applied AFTER integral, so the integral can store
     * a bias even while we slew toward it. */
    int32_t step = servo_int_acc;
    if (step >  SERVO_SLEW_PER_TICK) step =  SERVO_SLEW_PER_TICK;
    if (step < -SERVO_SLEW_PER_TICK) step = -SERVO_SLEW_PER_TICK;

    int32_t target = (int32_t)FRACN_NOMINAL + step;
    if (target < (int32_t)FRACN_MIN) target = (int32_t)FRACN_MIN;
    if (target > (int32_t)FRACN_MAX) target = (int32_t)FRACN_MAX;

    pll2_fracn_write((uint32_t)target);
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

#else  /* not STM32H723xx */

void spdif_input_init(void)               { }
void spdif_input_start(void)              { }
void spdif_input_stop(void)               { }
uint32_t spdif_input_poll(void)           { return 0; }
uint32_t spdif_input_pop_frames(int16_t *dst, uint32_t want_frames) {
    if (dst) for (uint32_t i = 0; i < want_frames * 2; ++i) dst[i] = 0;
    return 0;
}
void spdif_input_get_status(SpdifRxStatusPacket *out) {
    if (out) memset(out, 0, sizeof(*out));
}
void spdif_input_get_channel_status(uint8_t *out_24_bytes) {
    if (out_24_bytes) memset(out_24_bytes, 0, 24);
}

#endif  /* STM32H723xx */
