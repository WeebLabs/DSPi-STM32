/**
 * stm32h7xx_hal_conf.h — DSPi STM32H723 HAL config
 *
 * Slimmed from STM32CubeH7's stm32h7xx_hal_conf_template.h to only enable
 * modules that the M0 milestone (clocks + LED + UART) actually uses. Audio
 * peripherals (SAI, SPDIFRX, DMA, USB OTG, I2C, SPI) get re-enabled in their
 * respective milestones.
 */

#ifndef STM32H7xx_HAL_CONF_H
#define STM32H7xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------- Module Selection (M0 minimum) ------------- */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED          /* M7g: internal temp sensor on ADC3_IN18 */
#define HAL_SPI_MODULE_ENABLED          /* M11: SPI3 -> W25Q64 preset flash */
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
/* HAL_FLASH stays enabled (HAL_RCC_ClockConfig needs FLASH_LATENCY_4 +
 * __HAL_FLASH_GET_LATENCY). flash_clkdiv.h #undefs FLASH_SECTOR_SIZE /
 * FLASH_PAGE_SIZE before redefining them — the HAL macros for H7
 * internal flash are 128 KB / 32 byte, the wrong sizes for the W25Q64
 * SPI NOR. The undef-then-redefine is contained in flash_clkdiv.h so
 * runtime code that touches both internal and external flash isn't
 * silently using the wrong constants. */
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_HSEM_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SAI_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* The following modules will be re-enabled when their milestone arrives.
 * Left commented as a checklist:
 *
 * #define HAL_DCACHE_MODULE_ENABLED        // M5 (cache + MPU)
 * #define HAL_MDMA_MODULE_ENABLED
 * #define HAL_BDMA_MODULE_ENABLED
 * #define HAL_DMAMUX_MODULE_ENABLED        // implicit via DMA on H7
 * #define HAL_PCD_MODULE_ENABLED           // M2 (USB)
 * #define HAL_SAI_MODULE_ENABLED           // M4
 * #define HAL_SPDIFRX_MODULE_ENABLED       // M7
 * #define HAL_SPI_MODULE_ENABLED           // M10 (PDM) + M11 (W25Q)
 * #define HAL_I2C_MODULE_ENABLED           // M12
 * #define HAL_TIM_MODULE_ENABLED
 * #define HAL_CRS_MODULE_ENABLED           // USB FS clock trim
 */

/* ------------- Oscillator values (board-specific) ------------- */
#if !defined(HSE_VALUE)
#define HSE_VALUE       25000000U   /* WeAct V1.2: confirmed 25 MHz crystal X1 */
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT     100U
#endif

#define CSI_VALUE       4000000U
#define HSI_VALUE       64000000U
#define LSI_VALUE       32000U
#define LSE_VALUE       32768U      /* WeAct populates X2 32.768 kHz LSE */
#define LSE_STARTUP_TIMEOUT     5000U
#define EXTERNAL_CLOCK_VALUE    12288000U

/* ------------- System config ------------- */
#define VDD_VALUE                   3300U
#define TICK_INT_PRIORITY           0x0FU
#define USE_RTOS                    0
#define USE_SD_TRANSCEIVER          0U
#define USE_SPI_CRC                 0U

/* ------------- Ethernet (unused) ------------- */
/* No ETH on this board. */

/* ------------- Assert ------------- */
/* #define USE_FULL_ASSERT     1U */

/* ------------- Includes (only for enabled modules) ------------- */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32h7xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32h7xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32h7xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32h7xx_hal_cortex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32h7xx_hal_flash.h"
#endif
#ifdef HAL_HSEM_MODULE_ENABLED
  #include "stm32h7xx_hal_hsem.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32h7xx_hal_pwr.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32h7xx_hal_uart.h"
#endif
#ifdef HAL_SAI_MODULE_ENABLED
  #include "stm32h7xx_hal_sai.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32h7xx_hal_adc.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
  #include "stm32h7xx_hal_spi.h"
#endif

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32H7xx_HAL_CONF_H */
