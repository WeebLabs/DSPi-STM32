#ifndef DSPI_STM32_USB_DESCRIPTORS_H
#define DSPI_STM32_USB_DESCRIPTORS_H

#include <stdint.h>

/* TinyUSB calls these by name when the host issues GET_DESCRIPTOR.
 * Implementation is in usb_descriptors.c. */
uint8_t const *tud_descriptor_device_cb(void);
uint8_t const *tud_descriptor_configuration_cb(uint8_t index);
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid);

/* M2 endpoint addresses (vendor class echo). EP numbers are arbitrary;
 * 0x01 OUT and 0x81 IN are the conventional lowest non-control endpoints. */
#define VENDOR_EP_OUT   0x01
#define VENDOR_EP_IN    0x81

#endif
