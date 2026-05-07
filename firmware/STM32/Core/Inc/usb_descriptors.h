#ifndef DSPI_STM32_USB_DESCRIPTORS_H
#define DSPI_STM32_USB_DESCRIPTORS_H

#include <stdint.h>

/* TinyUSB descriptor callbacks (implemented in usb_descriptors.c). */
uint8_t  const *tud_descriptor_device_cb(void);
uint8_t  const *tud_descriptor_configuration_cb(uint8_t index);
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid);

/* String table indices — must stay in sync with usb_descriptors.c */
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_AC_INTERFACE,
    STRID_AS_INTERFACE,
    STRID_INPUT_TERMINAL,
    STRID_OUTPUT_TERMINAL,
};

#endif
