/* HAL MSP — peripheral-side init that HAL calls back into.
 * For M0 the GPIO/RCC for USART1 is already done in Log_USART_Init() in
 * main.c, so HAL_UART_MspInit() is a no-op here. Stays as a stub so the
 * weak HAL default (which does nothing useful for our pinout) doesn't get
 * pulled in surprisingly later.
 */

#include "main.h"

void HAL_MspInit(void) {
    /* HAL_Init() calls this. We do PWR/SYSCFG enable in SystemClock_Config()
     * to make the dependency obvious; nothing extra needed here. */
}
