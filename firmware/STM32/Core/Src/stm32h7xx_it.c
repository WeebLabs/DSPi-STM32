/* Cortex-M7 + peripheral IRQ handlers. */
#include "main.h"
#include "tusb.h"

void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void MemManage_Handler(void)  { while (1) {} }
void BusFault_Handler(void)   { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void)        { }
void DebugMon_Handler(void)   { }
void PendSV_Handler(void)     { }

void SysTick_Handler(void) { HAL_IncTick(); }

/* H72x has only one USB peripheral and its IRQ is named OTG_HS_IRQHandler
 * regardless of the FS-only PHY. dwc2_stm32.h aliases OTG_FS_IRQn to
 * OTG_HS_IRQn so TinyUSB's generic code routes correctly. rhport=0 is the
 * single port. */
void OTG_HS_IRQHandler(void) {
    tud_int_handler(0);
}
