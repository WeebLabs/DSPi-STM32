/**
 * vendor_commands.h — DSPi STM32H723 vendor USB control request dispatch
 *
 * Mirrors the contract used by firmware/DSPi/vendor_commands.c. The
 * actual table of command handlers is small in M7a (only REQ_GET_PLATFORM)
 * and grows with each subsequent milestone as the DSP pipeline lands.
 *
 * Overrides TinyUSB's weak tud_vendor_control_xfer_cb. Per usbd.c, all
 * VENDOR-type control transfers go directly to this hook regardless of
 * recipient (interface vs endpoint vs device), so we don't need a class
 * driver for the vendor interface beyond claiming it in usb_audio.c.
 */

#ifndef DSPI_STM32_VENDOR_COMMANDS_H
#define DSPI_STM32_VENDOR_COMMANDS_H

#include "tusb.h"

bool tud_vendor_control_xfer_cb(uint8_t rhport,
                                uint8_t stage,
                                tusb_control_request_t const *req);

#endif
