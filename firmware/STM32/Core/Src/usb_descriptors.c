/**
 * usb_descriptors.c — DSPi STM32H723, M3 UAC1 OUT
 *
 * Hand-rolled UAC1 config descriptor as a packed byte array. TinyUSB's
 * TUD_AUDIO_DESC_* macros emit UAC2-shaped descriptors and can't be
 * reused. The byte map mirrors the descriptor in firmware/DSPi/
 * usb_descriptors.c but stripped to a single 16-bit / 48 kHz alt setting,
 * no vendor interface, no notification endpoint — those return in later
 * milestones.
 *
 * Layout (offsets from start of usb_config_descriptor[]):
 *
 *    0  Config descriptor                              (9 bytes)
 *    9  IAD covers AC + AS                             (8 bytes)
 *   17  AC std interface (itf 0)                       (9 bytes)
 *   26  AC CS header                                   (9 bytes)
 *   35  AC CS input terminal (USB streaming)          (12 bytes)
 *   47  AC CS feature unit (mute + master volume)     (10 bytes)
 *   57  AC CS output terminal (speaker)                (9 bytes)
 *   66  AS std interface alt 0 (zero-bw)               (9 bytes)
 *   75  AS std interface alt 1 (16-bit / 48 kHz)       (9 bytes)
 *   84  AS CS general                                  (7 bytes)
 *   91  AS CS format type I (PCM, 2 ch, 16-bit, 48k)  (11 bytes)
 *  102  Std iso EP OUT (async)                         (9 bytes)
 *  111  CS iso data EP                                 (7 bytes)
 *  118  Std iso feedback EP IN                         (9 bytes)
 *  127  total
 */

#include <string.h>

#include "tusb.h"
#include "class/audio/audio.h"

#include "usb_audio.h"
#include "usb_descriptors.h"

/* ---------------------------------------------------------------------- */
/* DEVICE DESCRIPTOR                                                      */
/* ---------------------------------------------------------------------- */

static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    /* IAD signaling triplet — required at device level whenever the
     * configuration uses an IAD. Without it, Windows treats interface 0
     * (AudioControl) as the whole device and skips composite handling. */
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    /* Match the original RP DSPi project (firmware/DSPi/usb_descriptors.h)
     * so host-side tooling that already keys off these IDs works against
     * the STM32 build unchanged. */
    .idVendor           = 0x2E8B,
    .idProduct          = 0xFEAA,
    .bcdDevice          = 0x0201,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

/* ---------------------------------------------------------------------- */
/* CONFIGURATION DESCRIPTOR                                               */
/* ---------------------------------------------------------------------- */

#define U16_LE(x)   ((x) & 0xFF), (((x) >> 8) & 0xFF)
#define U24_LE(x)   ((x) & 0xFF), (((x) >> 8) & 0xFF), (((x) >> 16) & 0xFF)

/* M7a: vendor interface (itf 2, 9 B) + std bulk IN notify EP (7 B) appended.
 * +3 B vs the original 143 for the second discrete sample rate (44.1 kHz) in
 * the Type-I format descriptor. */
#define CONFIG_TOTAL_LEN 146

static uint8_t const desc_configuration[CONFIG_TOTAL_LEN] = {
    /* ---- 0: Configuration ---- */
    9, TUSB_DESC_CONFIGURATION,
    U16_LE(CONFIG_TOTAL_LEN),
    ITF_NUM_TOTAL,            /* bNumInterfaces (AC + AS + Vendor) */
    1,                        /* bConfigurationValue */
    0,                        /* iConfiguration */
    0x80,                     /* bmAttributes — bus-powered, no remote wakeup */
    50,                       /* bMaxPower (×2 mA = 100 mA) */

    /* ---- 9: IAD (binds AC+AS to one function so both route to our driver) ---- */
    8, TUSB_DESC_INTERFACE_ASSOCIATION,
    ITF_NUM_AC,               /* bFirstInterface */
    2,                        /* bInterfaceCount */
    TUSB_CLASS_AUDIO,         /* bFunctionClass = 0x01 (audio) */
    0x00,                     /* bFunctionSubClass = 0 (matches RP build) */
    AUDIO_FUNC_PROTOCOL_CODE_UNDEF,
    STRID_AC_INTERFACE,

    /* ---- 17: AC std interface (itf 0, 0 EPs, UAC1) ---- */
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AC, 0, 0,         /* itf number, alt, num EPs */
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_CONTROL,
    AUDIO_FUNC_PROTOCOL_CODE_UNDEF,   /* UAC1 protocol = 0x00 */
    STRID_AC_INTERFACE,

    /* ---- 26: AC CS header (UAC1.0, total CS length, references AS itf) ---- */
    9, TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_HEADER,
    U16_LE(0x0100),           /* bcdADC = 1.00 */
    U16_LE(40),               /* wTotalLength of AC CS descriptors (header+IT+FU+OT = 9+12+10+9) */
    1,                        /* bInCollection */
    ITF_NUM_AS,               /* baInterfaceNr(1) */

    /* ---- 35: AC CS input terminal (USB streaming, ID 1, 2 ch L+R) ---- */
    12, TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL,
    UAC1_INPUT_TERMINAL_ID,
    U16_LE(AUDIO_TERM_TYPE_USB_STREAMING),  /* 0x0101 */
    0,                        /* bAssocTerminal */
    AUDIO_CHANNELS,           /* bNrChannels */
    U16_LE(AUDIO_CHANNEL_CONFIG_FRONT_LEFT | AUDIO_CHANNEL_CONFIG_FRONT_RIGHT),
    0,                        /* iChannelNames */
    STRID_INPUT_TERMINAL,

    /* ---- 47: AC CS feature unit (ID 2, source IT 1, master mute+volume) ---- */
    10, TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_FEATURE_UNIT,
    UAC1_FEATURE_UNIT_ID,
    UAC1_INPUT_TERMINAL_ID,
    1,                        /* bControlSize = 1 byte per logical channel */
    0x03,                     /* master ch: bit0=MUTE, bit1=VOLUME */
    0,                        /* L channel: no per-channel controls */
    0,                        /* R channel: no per-channel controls */
    0,                        /* iFeature */

    /* ---- 57: AC CS output terminal (Speaker, ID 3, source FU 2) ---- */
    9, TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL,
    UAC1_OUTPUT_TERMINAL_ID,
    U16_LE(AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER),  /* 0x0301 */
    0,                        /* bAssocTerminal */
    UAC1_FEATURE_UNIT_ID,     /* bSourceID */
    STRID_OUTPUT_TERMINAL,

    /* ---- 66: AS std interface alt 0 (zero bandwidth) ---- */
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AS, 0, 0,
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    AUDIO_FUNC_PROTOCOL_CODE_UNDEF,
    STRID_AS_INTERFACE,

    /* ---- 75: AS std interface alt 1 (active, 2 EPs: data OUT + feedback IN) ---- */
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AS, 1, 2,
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_STREAMING,
    AUDIO_FUNC_PROTOCOL_CODE_UNDEF,
    STRID_AS_INTERFACE,

    /* ---- 84: AS CS general (links to input terminal, PCM format) ---- */
    7, TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    UAC1_INPUT_TERMINAL_ID,   /* bTerminalLink */
    1,                        /* bDelay (frames) */
    U16_LE(AUDIO_DATA_FORMAT_TYPE_I_PCM),

    /* ---- 91: AS CS format type I (PCM, 2 ch, 2 byte/sample, 16-bit, 2 freqs) ---- */
    14, TUSB_DESC_CS_INTERFACE,
    AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,
    AUDIO_FORMAT_TYPE_I,
    AUDIO_CHANNELS,
    AUDIO_BYTES_PER_SAMPLE,   /* bSubframeSize */
    AUDIO_BIT_DEPTH,          /* bBitResolution */
    2,                        /* bSamFreqType (two discrete sample rates) */
    U24_LE(AUDIO_SAMPLE_RATE),       /* 48000 (default) */
    U24_LE(AUDIO_SAMPLE_RATE_ALT),   /* 44100 */

    /* ---- 105: Std iso EP OUT (async, max 200 B, 1 ms interval) ---- */
    9, TUSB_DESC_ENDPOINT,
    AUDIO_OUT_ENDPOINT,
    0x05,                     /* bmAttributes: ISO (0x01) + async (0x04) + data (0x00) */
    U16_LE(AUDIO_EP_MAX_PKT),
    1,                        /* bInterval (1 ms on FS) */
    0,                        /* bRefresh — required field, ignored for data EP */
    AUDIO_FB_ENDPOINT,        /* bSynchAddress — points to feedback EP */

    /* ---- 114: CS iso data EP — declares sampling-frequency control ----
     * macOS treats this byte as the discovery hint for the device's
     * sample-rate query path. With bmAttributes=0 ("no controls"),
     * Core Audio binds the format briefly on SET_INTERFACE alt 1 then
     * gives up a few seconds in. bit 0 = sampling-frequency control;
     * the actual GET_CUR/SAM_FREQ handler is in usb_audio.c. */
    7, TUSB_DESC_CS_ENDPOINT,
    AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x01,                     /* bmAttributes: sampling-frequency control */
    0,                        /* bLockDelayUnits */
    U16_LE(0),                /* wLockDelay */

    /* ---- 121: Std iso feedback EP IN (3 bytes, 1 ms) ----
     * UAC1 spec mandates 10.14 fixed-point in exactly 3 bytes for FS.
     * macOS enumerates either way but rejects the stream on alt 1
     * unless wMaxPacketSize == 3. The dwc2 ISO FIFO is still
     * allocated in 4-byte words (one 32-bit slot) — see the
     * usbd_edpt_iso_alloc(rhport, AUDIO_FB_ENDPOINT, 4) in
     * usb_audio.c::uac1_open. */
    9, TUSB_DESC_ENDPOINT,
    AUDIO_FB_ENDPOINT,
    0x11,                     /* bmAttributes: ISO (0x01) + no-sync (0x00) + explicit-FB (0x10) */
    U16_LE(3),                /* wMaxPacketSize: 3 bytes per UAC1 FS feedback */
    1,                        /* bInterval (1 ms) */
    2,                        /* bRefresh — host polls every 2^2 ms */
    0,                        /* bSynchAddress */

    /* ---- 130: Vendor std interface (itf 2, 1 EP, class 0xFF) ---- */
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_VENDOR, 0, 1,     /* itf number, alt, num EPs (notify) */
    0xFF,                     /* bInterfaceClass: vendor specific */
    0x00,                     /* bInterfaceSubClass */
    0x00,                     /* bInterfaceProtocol */
    0x00,                     /* iInterface */

    /* ---- 139: Std bulk IN notification EP (0x83, 64 B FS bulk) ---- */
    7, TUSB_DESC_ENDPOINT,
    NOTIFY_IN_ENDPOINT,
    TUSB_XFER_BULK,           /* bmAttributes (0x02 — bulk) */
    U16_LE(NOTIFY_EP_MAX_PKT),
    NOTIFY_EP_INTERVAL_MS,
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

/* ---------------------------------------------------------------------- */
/* STRINGS                                                                */
/* ---------------------------------------------------------------------- */

static char const *const string_table[] = {
    [STRID_LANGID]           = NULL,
    [STRID_MANUFACTURER]     = "GitHub.com/WeebLabs",
    [STRID_PRODUCT]          = "Weeb Labs DSPi for STM32",
    [STRID_SERIAL]           = "0001",
    /* macOS Core Audio surfaces the AS-interface string as the device
     * name in Audio MIDI Setup, NOT the device-level iProduct. Match
     * iProduct so the Sound Output list shows the device by its real
     * product name. */
    [STRID_AC_INTERFACE]     = "Weeb Labs DSPi for STM32",
    [STRID_AS_INTERFACE]     = "Weeb Labs DSPi for STM32",
    [STRID_INPUT_TERMINAL]   = "USB Stream",
    [STRID_OUTPUT_TERMINAL]  = "Speaker",
};

static uint16_t _desc_str[32 + 1];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    size_t chr_count;

    if (index == STRID_LANGID) {
        _desc_str[1] = 0x0409;       /* English (US) */
        chr_count = 1;
    } else {
        if (index >= sizeof(string_table) / sizeof(string_table[0])) return NULL;
        const char *str = string_table[index];
        if (!str) return NULL;

        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (size_t i = 0; i < chr_count; ++i) _desc_str[1 + i] = (uint16_t)str[i];
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}
