/**
 * usb_glue.c — DSPi STM32H723 USB plumbing for TinyUSB on dwc2
 *
 * H72x quirk: only one USB peripheral, internally named USB1_OTG_HS even
 * though it bonds to FS pads only (no internal HS PHY, no external ULPI
 * required for FS speeds). TinyUSB's family.c is written assuming
 * USB_OTG_FS / USB2_OTG_FS macros that don't exist on H72x; the
 * stm32h723nucleo BSP works around it with the same alias trick we use
 * here. dwc2_stm32.h handles OTG_FS_IRQn → OTG_HS_IRQn internally.
 *
 * Vendor class is configured for echo: bytes received on OUT come right
 * back on IN. This proves the descriptor + endpoint pipeline end-to-end
 * before we layer UAC1 on top in M3.
 */

#include "main.h"
#include "tusb.h"
#include "usb_descriptors.h"

/* H72x USB aliases — see commentary above. Mirror the BSP. */
#ifndef USB_OTG_FS
#define USB_OTG_FS                       USB_OTG_HS
#endif
#ifndef GPIO_AF10_OTG_FS
#define GPIO_AF10_OTG_FS                 GPIO_AF10_OTG1_HS
#endif

/* ----------------------------------------------------------------------
 * USB peripheral hardware init: GPIO + clock + voltage detector + IRQ.
 *
 * Called once after SystemClock_Config. PLL3Q must already be sourcing
 * 48 MHz to RCC_USBCLKSOURCE_PLL3 (done in SystemClock_Config).
 * -------------------------------------------------------------------- */
void USB_HW_Init(void) {
    /* USB DM/DP on PA11/PA12, AF10. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {
        .Pin       = GPIO_PIN_11 | GPIO_PIN_12,
        .Mode      = GPIO_MODE_AF_PP,
        .Pull      = GPIO_NOPULL,
        .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF10_OTG1_HS,
    };
    HAL_GPIO_Init(GPIOA, &g);

    /* Required to power up the USB FS transceiver on H7. */
    HAL_PWREx_EnableUSBVoltageDetector();

    /* Enable the (single) USB peripheral clock. The HAL macro is named
     * after the peripheral's silicon name, USB1_OTG_HS, even though we
     * use it in FS mode. */
    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

    /* Force *device* mode on the controller.
     *
     * H72x has a single dual-role USB controller and decides device-vs-host
     * by sampling the OTG ID pin (PA10). PA10 is consumed by USART1 RX in
     * our pinout, so the ID-pin path can't decide for us — we must force
     * device mode explicitly via GUSBCFG.FDMOD. The dwc2 reference manual
     * requires waiting ≥25 ms after a mode change before further config;
     * 50 ms is the round-up everyone uses. The TinyUSB H7 BSP does this
     * only for the HS port, but on H72x where FS rides on the HS
     * controller we need it here too. */
    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
    USB_OTG_FS->GUSBCFG |=  USB_OTG_GUSBCFG_FDMOD;
    HAL_Delay(50);

    /* No physical VBUS sense pin available on this carrier — force the
     * controller to believe VBUS is valid so it'll respond to bus
     * activity even on hosts that gate enumeration on session-valid. */
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;

    /* Second voltage-detector enable matching the TinyUSB H7 BSP. The first
     * call before the clock enable powers the supply; this one re-asserts
     * after the controller is clocked. Cheap insurance. */
    HAL_PWREx_EnableUSBVoltageDetector();

    /* IRQ priority: keep below SysTick so HAL_Delay still fires.  The
     * audio ISRs in later milestones will live higher than this. */
    HAL_NVIC_SetPriority(OTG_HS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

void USB_App_Init(void) {
    tud_init(BOARD_TUD_RHPORT);
}

void USB_Task(void) {
    tud_task();
}

/* M3 USB callbacks — minimal. Class-specific work lives in usb_audio.c.
 * These are the device-state bookkeeping hooks TinyUSB exposes to apps
 * regardless of which class drivers are loaded. */
void tud_mount_cb(void)   { /* host issued SetConfiguration */ }
void tud_umount_cb(void)  { /* host disconnected or reset */ }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }
void tud_resume_cb(void)  { }
