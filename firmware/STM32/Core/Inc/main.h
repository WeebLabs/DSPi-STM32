/* M0 globals */
#ifndef DSPi_STM32_MAIN_H
#define DSPi_STM32_MAIN_H

#include "stm32h7xx_hal.h"

/* Pin choices for M0 — see Documentation/Porting/STM32H723_first_steps.md §0.4
 *
 * PB5  — heartbeat LED (free header pin; the on-board PE3 BLUE_LED is
 *        permanently sacrificed to SAI1_SD_B in the audio plan).
 * PA9  — USART1 TX (header pin, also documented in the WeAct README).
 * PA10 — USART1 RX (header pin).
 */
#define HEARTBEAT_LED_PORT      GPIOB
#define HEARTBEAT_LED_PIN       GPIO_PIN_5
#define HEARTBEAT_LED_RCC_EN()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define LOG_USART               USART1
#define LOG_USART_RCC_EN()      __HAL_RCC_USART1_CLK_ENABLE()
#define LOG_USART_GPIO_PORT     GPIOA
#define LOG_USART_TX_PIN        GPIO_PIN_9
#define LOG_USART_RX_PIN        GPIO_PIN_10
#define LOG_USART_AF            GPIO_AF7_USART1
#define LOG_USART_GPIO_RCC_EN() __HAL_RCC_GPIOA_CLK_ENABLE()

extern UART_HandleTypeDef huart_log;

void Error_Handler(void);

#endif
