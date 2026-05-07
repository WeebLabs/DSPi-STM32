/**
 * tusb_config.h — DSPi STM32H723 (M2: vendor-class echo)
 *
 * Mirrors the conservative side of the existing RP DSPi config: TinyUSB's
 * built-in audio class stays disabled (CFG_TUD_AUDIO=0); UAC1 lands later
 * via a custom class driver registered through usbd_app_driver_get_cb()
 * — same pattern used in firmware/DSPi/usb_audio.c. For now we only need
 * a vendor-class echo to prove enumeration + endpoint plumbing.
 */

#ifndef DSPI_STM32_TUSB_CONFIG_H
#define DSPI_STM32_TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TUSB_MCU                OPT_MCU_STM32H7
#define CFG_TUSB_OS                 OPT_OS_NONE
#define CFG_TUSB_DEBUG              0

#define BOARD_TUD_RHPORT            0     /* H72x has one USB port,
                                             logically rhport 0 (FS) */
#define BOARD_TUD_MAX_SPEED         OPT_MODE_FULL_SPEED

/* Be explicit: the BOARD_TUD_MAX_SPEED → TUD_RHPORT_MODE → CFG_TUD_MAX_SPEED
 * chain only fires inside TinyUSB's BSP framework, which we don't use. Set
 * the binding directly so dwc2_core_is_highspeed() returns false and the
 * driver picks phy_fs_init() / DCFG_DSPD_FS without ambiguity. */
#define CFG_TUD_MAX_SPEED           OPT_MODE_FULL_SPEED

/* Enable device stack only — host stack stays off until/unless we ever
 * need it (we don't; this is an audio device). */
#define CFG_TUD_ENABLED             1

/* Endpoint-zero size on FS = 64 (USB 2.0 spec: 8/16/32/64 allowed; 64 is
 * the standard for any non-trivial control transfer). */
#define CFG_TUD_ENDPOINT0_SIZE      64

/* Class enables — vendor only for M2.  Audio + bulk-notify EP arrive in
 * later milestones via usbd_app_driver_get_cb().  Keeping the slate
 * empty here matches the RP project's current convention. */
#define CFG_TUD_AUDIO               0
#define CFG_TUD_CDC                 0
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              1
#define CFG_TUD_DFU_RUNTIME         0
#define CFG_TUD_ECM_RNDIS           0
#define CFG_TUD_NCM                 0
#define CFG_TUD_BTH                 0

/* Vendor class FIFOs — one direction at a time, 256 B each is plenty for
 * an echo loop and leaves a wide margin in the dwc2 4 KB FIFO budget. */
#define CFG_TUD_VENDOR_RX_BUFSIZE   256
#define CFG_TUD_VENDOR_TX_BUFSIZE   256
#define CFG_TUD_VENDOR_EPSIZE       64

/* Memory placement / alignment.  TinyUSB on H7 dwc2 reads/writes the FIFO
 * via the AHB; descriptor and packet buffers can sit in regular SRAM and
 * dwc2 owns its dedicated 4 KB FIFO RAM inside the peripheral.  No cache
 * surgery is needed at FS speeds for this milestone — the M5/M6 audio
 * milestones will revisit AXI-SRAM placement + MPU. */
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#define CFG_TUSB_MEM_SECTION

#ifdef __cplusplus
}
#endif

#endif
