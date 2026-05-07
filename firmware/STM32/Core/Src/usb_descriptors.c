/**
 * usb_descriptors.c — DSPi STM32H723, M2 vendor-class echo
 *
 * Bare-minimum descriptor set: device + one configuration + one vendor
 * interface with a paired bulk OUT/IN endpoint.  No string-based product
 * differentiation yet beyond manufacturer / product / serial; the audio /
 * notification / DFU descriptor work lands in later milestones.
 *
 * VID/PID: 0xCAFE / 0x4001 — TinyUSB's "test VID + arbitrary PID" range.
 * Replace with the project's real VID/PID before any external release.
 */

#include "tusb.h"
#include "usb_descriptors.h"

/* ------------------------ Device Descriptor --------------------------- */

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,           /* USB 2.0 (FS) */

    /* Class-per-interface — host walks the config to find the vendor IF. */
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCAFE,
    .idProduct          = 0x4001,
    .bcdDevice          = 0x0001,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

/* --------------------- Configuration Descriptor ----------------------- */

enum {
    ITF_NUM_VENDOR = 0,
    ITF_NUM_TOTAL,
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

uint8_t const desc_configuration[] = {
    /* Config: 1 interface, bus-powered, 100 mA */
    TUD_CONFIG_DESCRIPTOR(/*config*/1, ITF_NUM_TOTAL, /*str_idx*/0,
                          CONFIG_TOTAL_LEN, 0x00, 100),

    /* Vendor interface: one OUT + one IN bulk endpoint, 64 B each (FS) */
    TUD_VENDOR_DESCRIPTOR(ITF_NUM_VENDOR, /*str_idx*/4,
                          VENDOR_EP_OUT, VENDOR_EP_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

/* ----------------------- String Descriptors --------------------------- */

static char const *string_desc_arr[] = {
    /* [0] LANGID — filled by string callback at runtime */
    [1] = "Weeb Labs",
    [2] = "DSPi STM32H723",
    [3] = "0001",                /* TODO: pull from chip UID for uniqueness */
    [4] = "DSPi Vendor",
};

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    size_t chr_count;

    if (index == 0) {
        _desc_str[1] = 0x0409;   /* English (US) */
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;
        const char *str = string_desc_arr[index];
        if (!str) return NULL;

        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;

        for (size_t i = 0; i < chr_count; ++i) {
            _desc_str[1 + i] = (uint16_t)str[i];
        }
    }

    /* Length byte + descriptor type byte packed into the first u16 */
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
