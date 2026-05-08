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
static void fill_half(int32_t *dst) {
    /* Pop up to one half's worth of stereo frames. The ring may be
     * starving, in which case we fill the rest with silence (0). */
    uint32_t got = usb_ring_pop_frames(pop_scratch, AUDIO_FRAMES_HALF);
    if (got < AUDIO_FRAMES_HALF) ++audio_underruns;

    /* Convert int16 → int32 with 8-bit headroom shift so the host's
     * 16-bit samples become 24-bit-equivalent values right-aligned in
     * the int32 word. The SAI peripheral reads bits [23:0] for 24-bit
     * datasize (RM0468 §34.4); putting the source value into bits [23:8]
     * preserves audio level (16-bit −0 dBFS → 24-bit −0 dBFS) while
     * keeping the data in the LSB-aligned half of the int32 the SAI
     * expects. */
    uint32_t i;
    for (i = 0; i < got; ++i) {
        int32_t L = (int32_t)pop_scratch[2*i + 0] << 8;
        int32_t R = (int32_t)pop_scratch[2*i + 1] << 8;
        dst[2*i + 0] = L;
        dst[2*i + 1] = R;
    }
    /* Pad shortfall with silence. */
    for (; i < AUDIO_FRAMES_HALF; ++i) {
        dst[2*i + 0] = 0;
        dst[2*i + 1] = 0;
    }
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
