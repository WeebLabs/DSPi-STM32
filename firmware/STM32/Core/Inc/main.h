/* M0 globals */
#ifndef DSPi_STM32_MAIN_H
#define DSPi_STM32_MAIN_H

#include "stm32h7xx_hal.h"

/* Pin choices for M0 — see Documentation/Porting/STM32H723_first_steps.md §0.4
 *
 * PE3  — on-board BLUE_LED via PNP transistor (PDTC114ET). ACTIVE LOW:
 *        drive PE3 low to light the LED. After GPIO init the ODR is 0 by
 *        default, so the LED comes up ON immediately — first useful sign
 *        of life. Toggle then alternates ON/OFF at 1 Hz.
 *
 *        TODO(M5): PE3 is the only LQFP100 pin available for SAI1_SD_B
 *        per plan §0.4. When 4-channel audio comes online this heartbeat
 *        must move to a free header pin (e.g. PB5, PA15, PC6) or be
 *        retired entirely.
 *
 * PA9  — USART1 TX (header pin, also documented in the WeAct README).
 * PA10 — USART1 RX (header pin).
 */
#define HEARTBEAT_LED_PORT      GPIOE
#define HEARTBEAT_LED_PIN       GPIO_PIN_3
#define HEARTBEAT_LED_RCC_EN()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define HEARTBEAT_LED_ACTIVE_LOW 1

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
