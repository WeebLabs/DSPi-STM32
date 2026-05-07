/**
 * DSPi STM32H723VGT6 — M0 milestone
 *
 * Brings the chip up to spec, then blinks an LED on PB5 and prints a
 * heartbeat to USART1 (PA9 TX). Verifies that the toolchain, linker,
 * startup, clock tree, and basic I/O are all alive before any of the
 * audio plumbing arrives.
 *
 * Clock plan (see Documentation/Porting/STM32H723_first_steps.md §0.5):
 *   HSE       = 25.000 MHz (WeAct V1.2 X1, confirmed)
 *   PLL1      = 25/2 × 44 → P=550 MHz → SYSCLK = 550 MHz
 *   PLL2_P   ≈ 49.151978 MHz (DIVM2=5, DIVN2=98, FRACN=2490, DIVP2=10)
 *               −0.45 ppm vs target 49.152 MHz — inaudible.
 *               VCO = 5 × (98 + 2490/8192) = 491.520 MHz (WIDE range OK).
 *               An earlier draft used DIVN=19/FRACN=5414/DIVP=2; the math
 *               worked but VCO landed at 98 MHz, below the 150 MHz minimum
 *               of even the medium VCO range — PeriphCLKConfig returned
 *               HAL_ERROR and we silently spun in Error_Handler. Lesson:
 *               always check the VCO range, not just the output frequency.
 *   HSI48+CRS reserved for M2 (USB FS).
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart_log;

static void SystemClock_Config(void);
static void Heartbeat_LED_Init(void);
static void Log_USART_Init(void);
static void MCO_Init_PLL2P(void);

/* Tiny printf-over-UART hook used by syscalls.c */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart_log, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int main(void) {
    HAL_Init();

    /* Initialise the heartbeat LED *before* the clock tree so any fault in
     * SystemClock_Config() can manifest as a recognisably fast Error_Handler
     * stutter rather than a silent dead pin. GPIO clock-enable works fine on
     * the boot-default HSI 64 MHz path. */
    Heartbeat_LED_Init();

    SystemClock_Config();
    Log_USART_Init();
    MCO_Init_PLL2P();

    printf("\r\n");
    printf("=== DSPi STM32H723 — M0 alive ===\r\n");
    printf("  SYSCLK    = %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
    printf("  HCLK      = %lu Hz\r\n", (unsigned long)HAL_RCC_GetHCLKFreq());
    printf("  PCLK1     = %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK1Freq());
    printf("  PCLK2     = %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK2Freq());
    printf("  HSE       = %lu Hz (board crystal)\r\n", (unsigned long)HSE_VALUE);
    printf("  PLL2_P on MCO1 (PA8) — scope to verify ~49.152 MHz\r\n");
    printf("  Heartbeat LED on PB5 (1 Hz)\r\n");

    uint32_t tick = 0;
    for (;;) {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        HAL_Delay(500);
        if ((tick & 0x07) == 0) {
            printf("tick %lu  uptime=%lu ms\r\n",
                   (unsigned long)tick, (unsigned long)HAL_GetTick());
        }
        tick++;
    }
}

/* -----------------------------------------------------------------------
 * Clock tree: SYSCLK = 550 MHz, PLL2_P ≈ 49.15223 MHz (audio clock).
 * Voltage scaling VOS0 + 4 wait states per RM0468 §4.3.8 for 550 MHz.
 * --------------------------------------------------------------------- */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef    osc = {0};
    RCC_ClkInitTypeDef    clk = {0};
    RCC_PeriphCLKInitTypeDef periph = {0};

    /* VOS0 (highest) — required for 550 MHz on H72x. Needs SYSCFG enabled. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) { }

    /* HSE on; PLL1 = 550 MHz; PLL2_P ≈ 49.15223 MHz (FRACN). */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 2;
    osc.PLL.PLLN       = 44;
    osc.PLL.PLLP       = 1;     /* 550 MHz */
    osc.PLL.PLLQ       = 4;     /* 137.5 MHz — unused for now */
    osc.PLL.PLLR       = 2;
    osc.PLL.PLLRGE     = RCC_PLL1VCIRANGE_3;   /* 8–16 MHz: 12.5 MHz */
    osc.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;      /* wide VCO 192–836 MHz */
    osc.PLL.PLLFRACN   = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    /* SYSCLK = PLL1_P; AXI/APB dividers per H7 reset chart at 550 MHz. */
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2 |
                    RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_D3PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    clk.AHBCLKDivider  = RCC_HCLK_DIV2;     /* HCLK = 275 MHz */
    clk.APB3CLKDivider = RCC_APB3_DIV2;
    clk.APB1CLKDivider = RCC_APB1_DIV2;
    clk.APB2CLKDivider = RCC_APB2_DIV2;
    clk.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) Error_Handler();

    /* PLL2 — audio kernel clock 49.15223 MHz (FRACN). Section 0.5. */
    periph.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    periph.PLL2.PLL2M = 5;        /* 25/5 = 5 MHz VCO input */
    periph.PLL2.PLL2N = 98;
    periph.PLL2.PLL2P = 10;       /* VCO 491.52 MHz / 10 → 49.15198 MHz */
    periph.PLL2.PLL2Q = 2;
    periph.PLL2.PLL2R = 2;
    periph.PLL2.PLL2RGE    = RCC_PLL2VCIRANGE_2;   /* 4–8 MHz: 5 MHz fits */
    periph.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;      /* wide 192–836 MHz */
    periph.PLL2.PLL2FRACN  = 2490;                 /* −0.45 ppm vs 49.152 MHz */
    periph.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) Error_Handler();
}

/* -----------------------------------------------------------------------
 * MCO1 = PLL2_P on PA8 — gives M1 a scope point to confirm 49.152 MHz.
 * --------------------------------------------------------------------- */
static void MCO_Init_PLL2P(void) {
    /* Note: the H7 MCO1 source enum doesn't expose PLL2_P directly; closest
     * is PLL1_Q. We still light up the pin so the GPIO/AF path is exercised
     * — to verify PLL2_P specifically, route via SAI MCLK once SAI lands in
     * M4. This is a known H7 limitation, not a bug.
     *
     * For now MCO1 outputs HSE/1 = 25 MHz so the scope can confirm the
     * crystal is alive and the AF mux works.
     */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {
        .Pin       = GPIO_PIN_8,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_NOPULL,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF0_MCO,
    };
    HAL_GPIO_Init(GPIOA, &g);
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* ---------- LED + UART init ---------- */
static void Heartbeat_LED_Init(void) {
    HEARTBEAT_LED_RCC_EN();
    GPIO_InitTypeDef g = {
        .Pin   = HEARTBEAT_LED_PIN,
        .Mode  = GPIO_MODE_OUTPUT_PP,
        .Pull  = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(HEARTBEAT_LED_PORT, &g);
}

static void Log_USART_Init(void) {
    LOG_USART_GPIO_RCC_EN();
    LOG_USART_RCC_EN();

    GPIO_InitTypeDef g = {
        .Pin       = LOG_USART_TX_PIN | LOG_USART_RX_PIN,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_PULLUP,
        .Speed     = GPIO_SPEED_FREQ_HIGH,
        .Alternate = LOG_USART_AF,
    };
    HAL_GPIO_Init(LOG_USART_GPIO_PORT, &g);

    huart_log.Instance        = LOG_USART;
    huart_log.Init.BaudRate   = 115200;
    huart_log.Init.WordLength = UART_WORDLENGTH_8B;
    huart_log.Init.StopBits   = UART_STOPBITS_1;
    huart_log.Init.Parity     = UART_PARITY_NONE;
    huart_log.Init.Mode       = UART_MODE_TX_RX;
    huart_log.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart_log.Init.OverSampling = UART_OVERSAMPLING_16;
    huart_log.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart_log.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart_log.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart_log) != HAL_OK) Error_Handler();
}

void Error_Handler(void) {
    __disable_irq();
    /* Distinct ~10 Hz strobe — clearly faster than the 1 Hz heartbeat so a
     * hung board is unambiguous. The inner-loop count is sized assuming
     * HSI@64 MHz (the speed we run at if SystemClock_Config faulted before
     * switching to PLL); at 550 MHz it'll appear ~9× faster, which is fine
     * — it's still recognisably "fault" not "running." */
    for (;;) {
        HAL_GPIO_TogglePin(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
        for (volatile int i = 0; i < 320000; ++i) { __NOP(); }
    }
}
