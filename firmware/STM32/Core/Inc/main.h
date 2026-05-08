/* M0 globals */
#ifndef DSPi_STM32_MAIN_H
#define DSPi_STM32_MAIN_H

#include "stm32h7xx_hal.h"

/* Pin choices — see Documentation/Porting/STM32H723_first_steps.md §0.4
 *
 * PB5  — heartbeat LED (free header pin, externally wired with a series
 *        resistor; ACTIVE HIGH — LED lights when PB5 is driven high).
 *        Moved here from PE3 in M5 because PE3 became SAI1_SD_B (the
 *        only LQFP100 pin available for that signal). The on-board
 *        BLUE_LED no longer functions as a heartbeat indicator.
 *
 * PE3  — SAI1_SD_B (used by audio_out.c since M5).
 * PE2  — SAI1_MCK_A
 * PE4  — SAI1_FS_A
 * PE5  — SAI1_SCK_A
 * PE6  — SAI1_SD_A
 *
 * PA9  — USART1 TX (header pin, also documented in the WeAct README).
 * PA10 — USART1 RX (header pin).
 */
#define HEARTBEAT_LED_PORT       GPIOB
#define HEARTBEAT_LED_PIN        GPIO_PIN_5
#define HEARTBEAT_LED_RCC_EN()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define HEARTBEAT_LED_ACTIVE_LOW 0

#define LOG_USART               USART1
#define LOG_USART_RCC_EN()      __HAL_RCC_USART1_CLK_ENABLE()
#define LOG_USART_GPIO_PORT     GPIOA
#define LOG_USART_TX_PIN        GPIO_PIN_9
#define LOG_USART_RX_PIN        GPIO_PIN_10
#define LOG_USART_AF            GPIO_AF7_USART1
#define LOG_USART_GPIO_RCC_EN() __HAL_RCC_GPIOA_CLK_ENABLE()

extern UART_HandleTypeDef huart_log;

void Error_Handler(void);

/* M2 USB — see usb_glue.c */
void USB_HW_Init(void);    /* clocks, GPIO, voltage detector, NVIC */
void USB_App_Init(void);   /* tud_init(0) */
void USB_Task(void);       /* tud_task() — call from main loop */

/* M4 audio output (SAI1_A) — see audio_out.c
 *
 * Pin map per Documentation/Porting/STM32H723_first_steps.md §0.4:
 *   SAI1_MCLK_A: PE2  (12.288 MHz, 256×Fs)
 *   SAI1_FS_A  : PE4  (LRCLK, 48 kHz)
 *   SAI1_SCK_A : PE5  (BCK, 3.072 MHz)
 *   SAI1_SD_A  : PE6  (DATA, MSB-aligned 24-bit in 32-bit slots)
 *
 * All four pins use AF6 (SAI1) on Port E. None of them are usable on this
 * board until camera + OSPI flash are unpopulated/uninitialised — both
 * are off in our build.
 */
void Audio_Init(void);
void Audio_Start(void);
extern volatile uint32_t audio_dma_callbacks;   /* increments on each
                                                   half/cplt — proves the
                                                   DMA pipeline is alive */

#endif
