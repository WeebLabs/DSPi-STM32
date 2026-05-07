/**
 * audio_out.c — DSPi STM32H723, M4 SAI1_A I2S TX (1 kHz tone)
 *
 * Brings up SAI1 sub-block A as an asynchronous I2S master TX in Philips
 * standard, 24-bit data MSB-aligned inside 32-bit slots, 48 kHz frame
 * rate, MCLK = 256×Fs = 12.288 MHz on PE2.
 *
 * DMA buffer placement: lives at a fixed AXI SRAM address (0x24000000)
 * because DMA1 cannot reach DTCM, and stm32-cmake's auto-generated
 * linker script only declares the DTCM region. D-cache is off through
 * M4, so no MPU surgery needed yet — that arrives in M5 along with a
 * proper hand-written linker script.
 *
 * The 48-entry sine table loops once per 1 ms = exactly 1 kHz at 48 kHz
 * Fs. Both channels carry the same waveform, so the scope on either
 * SD output edge tells you frame alignment is correct without an L/R
 * channel swap test. Eventual UAC1-fed audio (M5) replaces this fill.
 */

#include "main.h"
#include <math.h>

/* Audio DMA ring (AXI SRAM, fixed address — see file header) */
#define AUDIO_BUFFER_BASE       0x24000000UL

/* 192 stereo frames per ping-pong half × 2 halves × 2 ch = 768 int32 words.
 * Each int32 carries one sample (24-bit MSB-aligned in the upper bits).
 * 192 frames @ 48 kHz = 4 ms per half, comfortably above any HAL ISR
 * latency we'll ever hit. */
#define AUDIO_FRAMES_HALF       192U
#define AUDIO_FRAMES_TOTAL      (AUDIO_FRAMES_HALF * 2)
#define AUDIO_WORDS_TOTAL       (AUDIO_FRAMES_TOTAL * 2)  /* L+R */

static int32_t * const audio_buf = (int32_t *)AUDIO_BUFFER_BASE;

/* 1 kHz sine, 48 samples per cycle = 1.000 kHz exact at 48 kHz Fs.
 * 24-bit signed PCM stored RIGHT-aligned in the int32_t (bits [23:0]).
 * STM32 SAI's SAI_xDR register is right-aligned per RM0468 §34.4
 * regardless of slot size — i.e. for DataSize=24 / SlotSize=32, the
 * SAI hardware takes bits [23:0] of each 32-bit DMA word and serialises
 * them MSB-first into the slot, padding the remaining 8 bits with
 * zero. Conservative -12 dBFS amplitude so even a misbehaving DAC
 * doesn't clip on the very first listen.
 *
 * (The earlier draft used `s << 8` to "MSB-align," matching what some
 * other DSPs do, but on STM32 SAI that put the sample's lower 16 bits
 * into [23:8] and produced an audibly clipped sound on the wire.) */
#define SINE_LEN  48U
static int32_t sine_table[SINE_LEN];
static volatile uint32_t sine_phase = 0;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

volatile uint32_t audio_dma_callbacks = 0;

/* ---------------------------------------------------------------------- */
/* Sine table generation — call once at init                              */
/* ---------------------------------------------------------------------- */
static void sine_table_init(void) {
    for (uint32_t i = 0; i < SINE_LEN; ++i) {
        float ang = (2.0f * 3.14159265358979323846f * (float)i) / (float)SINE_LEN;
        float v   = sinf(ang) * 0.25f;          /* −12 dBFS */
        int32_t s = (int32_t)(v * 8388607.0f);  /* 24-bit signed: −2097151..+2097151 */
        sine_table[i] = s;                      /* right-aligned in int32_t — SAI reads [23:0] */
    }
}

/* ---------------------------------------------------------------------- */
/* DMA half-buffer fill — both halves share the same generator state     */
/* ---------------------------------------------------------------------- */
static void fill_half(int32_t *dst) {
    for (uint32_t i = 0; i < AUDIO_FRAMES_HALF; ++i) {
        int32_t s = sine_table[sine_phase];
        dst[i * 2 + 0] = s;   /* L */
        dst[i * 2 + 1] = s;   /* R */
        if (++sine_phase >= SINE_LEN) sine_phase = 0;
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai != &hsai_BlockA1) return;
    fill_half(&audio_buf[0]);
    ++audio_dma_callbacks;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (hsai != &hsai_BlockA1) return;
    fill_half(&audio_buf[AUDIO_FRAMES_HALF * 2]);
    ++audio_dma_callbacks;
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    /* Track but don't crash — useful when scope-debugging a DAC that
     * holds BCK/LRCLK low and stalls the SAI. */
    (void)hsai;
}

/* ---------------------------------------------------------------------- */
/* DMA1_Stream0 IRQ → HAL                                                 */
/* ---------------------------------------------------------------------- */
void DMA1_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_sai1_a);
}

/* ---------------------------------------------------------------------- */
/* Public init                                                            */
/* ---------------------------------------------------------------------- */
void Audio_Init(void) {
    /* RCC */
    __HAL_RCC_SAI1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* DMAMUX1 has no separate clock-enable on H7 — it's part of the DMA1
     * peripheral domain and powers up automatically. */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* GPIO PE2/4/5/6 → AF6 (SAI1)
     *   PE2 = SAI1_MCLK_A
     *   PE4 = SAI1_FS_A
     *   PE5 = SAI1_SCK_A
     *   PE6 = SAI1_SD_A
     */
    GPIO_InitTypeDef g = {
        .Pin       = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_NOPULL,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF6_SAI1,
    };
    HAL_GPIO_Init(GPIOE, &g);

    /* DMA1 Stream 0 → SAI1_A.
     *
     * 32-bit transfers because samples are MSB-aligned in 32-bit slots.
     * Circular mode means HAL_SAI_Transmit_DMA's buffer is treated as a
     * ring; HAL fires the half- and full-complete callbacks at the
     * right boundaries automatically. */
    hdma_sai1_a.Instance                 = DMA1_Stream0;
    hdma_sai1_a.Init.Request             = DMA_REQUEST_SAI1_A;
    hdma_sai1_a.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai1_a.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai1_a.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sai1_a.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_sai1_a.Init.Mode                = DMA_CIRCULAR;
    hdma_sai1_a.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai1_a.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_sai1_a.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai1_a.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai1_a.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&hsai_BlockA1, hdmatx, hdma_sai1_a);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    /* SAI1 Block A — I2S Philips master TX, 24-bit data in 32-bit slot */
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
    hsai_BlockA1.Init.Mckdiv           = 0;   /* HAL will compute from AudioFrequency */
    hsai_BlockA1.Init.MckOverSampling  = SAI_MCK_OVERSAMPLING_DISABLE;
    hsai_BlockA1.Init.MckOutput        = SAI_MCK_OUTPUT_ENABLE;
    if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD,
                             SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK) {
        Error_Handler();
    }

    /* Fill the buffer once with sine before starting DMA so the very
     * first BCK edge has a valid sample to ship. */
    sine_table_init();
    fill_half(&audio_buf[0]);
    fill_half(&audio_buf[AUDIO_FRAMES_HALF * 2]);
}

void Audio_Start(void) {
    if (HAL_SAI_Transmit_DMA(&hsai_BlockA1,
                             (uint8_t *)audio_buf,
                             AUDIO_WORDS_TOTAL) != HAL_OK) {
        Error_Handler();
    }
}
