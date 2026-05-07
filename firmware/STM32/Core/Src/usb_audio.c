/**
 * usb_audio.c — DSPi STM32H723 minimal UAC1 OUT class driver (M3)
 *
 * Registered with TinyUSB via usbd_app_driver_get_cb() because the bundled
 * audio class driver is UAC2-only. Mirrors the contract used by
 * firmware/DSPi/usb_audio.c so future milestones (M4-onwards) can layer the
 * real DSP pipeline behind the same interface without changing the
 * descriptor surface or class-request handlers.
 *
 * M3 scope: receive ISO OUT bytes, throw them away, return a fixed-rate
 * feedback packet every SOF. Counters are exposed for the heartbeat
 * printf so we can confirm the host actually streams.
 */

#include <string.h>

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "class/audio/audio.h"

#include "usb_audio.h"
#include "usb_descriptors.h"

/* ---------------- Endpoint buffers ----------------
 * AUDIO_EP_MAX_PKT covers nominal + jitter at the highest format we'll
 * eventually accept (24-bit 96 kHz). 200 B is plenty for M3 (16-bit/48k =
 * 192 B). DMA-capable, 4-byte aligned. */
static uint8_t __attribute__((aligned(4))) audio_out_buf[AUDIO_EP_MAX_PKT];

/* Feedback EP packet — 10.14 fixed-point in 3 bytes, but the DCD/dwc2
 * iso allocator reserves 4 bytes minimum. Pre-baked nominal value for
 * 48 kHz at FS: 48 << 14 = 0x000C0000 → little-endian {0x00, 0x00, 0x0C}.
 * In M4-onwards, this gets replaced by a SOF-driven PID controller that
 * trims based on the device's actual audio clock. */
static uint8_t __attribute__((aligned(4))) audio_fb_buf[4] = {
    0x00, 0x00, 0x0C, 0x00,
};

/* ---------------- Public counters (heartbeat reads these) ---------------- */
volatile uint32_t audio_bytes_received   = 0;
volatile uint32_t audio_packets_received = 0;
volatile bool     audio_streaming        = false;

/* ---------------- Internal driver state ---------------- */
static struct {
    uint8_t cur_alt;       /* current AS alt setting (0 = idle, 1 = 16-bit/48k) */
    bool    ep_data_open;
    bool    ep_fb_open;
    uint8_t pending_cs;    /* class-request SETUP scratch */
    uint8_t pending_recipient;
    uint8_t pending_len;
} uac1 = { .cur_alt = 0 };

static uint8_t  uac1_ctrl_buf[8];

/* ====================================================================== */
/* Class driver vtable                                                    */
/* ====================================================================== */

static void uac1_init(void)   {
    memset(&uac1, 0, sizeof(uac1));
    audio_streaming = false;
    audio_bytes_received = 0;
    audio_packets_received = 0;
}

static bool uac1_deinit(void) { return true; }

static void uac1_reset(uint8_t rhport) {
    (void)rhport;
    uac1.cur_alt = 0;
    uac1.ep_data_open = false;
    uac1.ep_fb_open = false;
    audio_streaming = false;
}

/* TinyUSB walks the configuration descriptor at SetConfiguration time and
 * calls open() once per interface group it can't bind itself. For our
 * IAD-grouped (AC + AS) audio function, open() must:
 *   1. Claim both AC and all AS alt settings.
 *   2. Pre-allocate the dwc2 hardware FIFOs for the isochronous endpoints
 *      via usbd_edpt_iso_alloc() — without this, usbd_edpt_open() at
 *      SET_INTERFACE alt-1 time silently fails on dwc2 (the
 *      TUP_DCD_EDPT_ISO_ALLOC contract).
 *   3. Return the total byte count consumed so the parser skips past us. */
static uint16_t uac1_open(uint8_t rhport,
                          tusb_desc_interface_t const *itf_desc,
                          uint16_t max_len) {
    if (itf_desc->bInterfaceClass != TUSB_CLASS_AUDIO) return 0;
    /* The first interface in our function is the AC interface (alt 0). */
    if (itf_desc->bInterfaceSubClass != AUDIO_SUBCLASS_CONTROL) return 0;
    if (itf_desc->bAlternateSetting   != 0) return 0;

    /* Reserve worst-case FIFO for the two ISO EPs. Done once at config
     * time; the actual open/close happens on SET_INTERFACE. */
#ifdef TUP_DCD_EDPT_ISO_ALLOC
    usbd_edpt_iso_alloc(rhport, AUDIO_OUT_ENDPOINT, AUDIO_EP_MAX_PKT);
    usbd_edpt_iso_alloc(rhport, AUDIO_FB_ENDPOINT, 4);
#endif

    /* Walk forward through CS-interface, AS-interface alts, and their
     * CS+EP descriptors until we hit either a non-audio std interface or
     * the end of the supplied buffer. */
    uint8_t const *p   = (uint8_t const *)itf_desc;
    uint8_t const *end = p + max_len;
    p += p[0];
    while (p < end) {
        if (p[1] == TUSB_DESC_INTERFACE) {
            tusb_desc_interface_t const *next = (tusb_desc_interface_t const *)p;
            if (next->bInterfaceClass != TUSB_CLASS_AUDIO) break;
        }
        p += p[0];
    }
    return (uint16_t)(p - (uint8_t const *)itf_desc);
}

/* ---------------- Class control requests ---------------- */
/*
 * UAC1 GET/SET_CUR/MIN/MAX/RES on the feature unit + sample-frequency
 * control on the EP. macOS probes a small number of these on enumeration
 * and again whenever the user touches the volume slider; we need to
 * answer plausibly even though there's no real DSP behind any of it yet.
 */

/* UAC1 bRequest values (raw, since TinyUSB's AUDIO_CS_REQ_* enum is UAC2-
 * shaped and only defines a few of these). */
#define UAC1_REQ_SET_CUR  0x01
#define UAC1_REQ_GET_CUR  0x81
#define UAC1_REQ_GET_MIN  0x82
#define UAC1_REQ_GET_MAX  0x83
#define UAC1_REQ_GET_RES  0x84

static bool handle_get_request(uint8_t stage, tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP) return true;

    uint8_t const cs        = tu_u16_high(req->wValue);   /* control selector */
    uint8_t const cn        = tu_u16_low (req->wValue);   /* channel number   */
    uint8_t const recipient = tu_u16_low (req->wIndex);   /* unit ID or EP    */

    /* Feature-unit master controls */
    if (recipient == UAC1_FEATURE_UNIT_ID && cn == 0) {
        if (cs == AUDIO_FU_CTRL_MUTE) {
            static uint8_t v = 0;
            return tud_control_xfer(0, (tusb_control_request_t *)req, &v, 1);
        }
        if (cs == AUDIO_FU_CTRL_VOLUME) {
            static int16_t cur = 0, min = -90 * 256, max = 0, res = 256;
            switch (req->bRequest) {
                case UAC1_REQ_GET_CUR: return tud_control_xfer(0, (tusb_control_request_t *)req, &cur, 2);
                case UAC1_REQ_GET_MIN: return tud_control_xfer(0, (tusb_control_request_t *)req, &min, 2);
                case UAC1_REQ_GET_MAX: return tud_control_xfer(0, (tusb_control_request_t *)req, &max, 2);
                case UAC1_REQ_GET_RES: return tud_control_xfer(0, (tusb_control_request_t *)req, &res, 2);
            }
        }
    }

    /* Sample-frequency control on the EP. Only one rate (48 kHz). */
    if (cs == AUDIO_CS_CTRL_SAM_FREQ && req->bRequest == UAC1_REQ_GET_CUR) {
        static uint8_t freq[3] = {
            (AUDIO_SAMPLE_RATE)       & 0xFF,
            (AUDIO_SAMPLE_RATE >>  8) & 0xFF,
            (AUDIO_SAMPLE_RATE >> 16) & 0xFF,
        };
        return tud_control_xfer(0, (tusb_control_request_t *)req, freq, 3);
    }

    return false;
}

static bool handle_set_request(uint8_t stage, tusb_control_request_t const *req) {
    /* SETUP — record what to expect and accept the data stage */
    if (stage == CONTROL_STAGE_SETUP) {
        uac1.pending_cs        = tu_u16_high(req->wValue);
        uac1.pending_recipient = tu_u16_low (req->wIndex);
        uac1.pending_len       = (uint8_t)req->wLength;
        if (uac1.pending_len > sizeof(uac1_ctrl_buf)) return false;
        return tud_control_xfer(0, (tusb_control_request_t *)req,
                                uac1_ctrl_buf, uac1.pending_len);
    }

    /* DATA / ACK — silently accept. M3 doesn't yet wire mute/volume to
     * anything physical; later milestones will pull these out of
     * uac1_ctrl_buf and update the DSP pipeline. */
    return true;
}

static bool uac1_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *req) {
    (void)rhport;

    /* Standard SET_INTERFACE on the AS interface — open/close EPs. */
    if (req->bmRequestType == 0x01 /* dir=H2D, type=std, recip=interface */
        && req->bRequest    == TUSB_REQ_SET_INTERFACE
        && (req->wIndex & 0xFF) == ITF_NUM_AS) {
        if (stage != CONTROL_STAGE_SETUP) return true;

        uint8_t const alt = (uint8_t)req->wValue;
        uac1.cur_alt = alt;

        if (alt == 0) {
            /* Zero-bandwidth — close any open EPs */
            if (uac1.ep_data_open) usbd_edpt_close(0, AUDIO_OUT_ENDPOINT);
            if (uac1.ep_fb_open)   usbd_edpt_close(0, AUDIO_FB_ENDPOINT);
            uac1.ep_data_open = false;
            uac1.ep_fb_open   = false;
            audio_streaming = false;
        } else if (alt == 1) {
            /* Open EPs from descriptor records — find them by walking
             * the configuration descriptor we already gave the host.
             * For simplicity we hardcode the EP attributes since we
             * own them. */
            /* Build ad-hoc endpoint descriptors for usbd_edpt_open. The
             * bitfield layout in tusb_desc_endpoint_t.bmAttributes is
             * .xfer:2 / .sync:2 / .usage:2 / reserved:2 — see
             * tusb_types.h. We mirror the wire-format bytes from the
             * config descriptor at offsets 102 and 118: 0x05 and 0x11. */
            tusb_desc_endpoint_t ep_out = {
                .bLength          = sizeof(tusb_desc_endpoint_t),
                .bDescriptorType  = TUSB_DESC_ENDPOINT,
                .bEndpointAddress = AUDIO_OUT_ENDPOINT,
                .bmAttributes     = {.xfer = TUSB_XFER_ISOCHRONOUS,
                                     .sync = 1, /* asynchronous */
                                     .usage = 0 /* data endpoint */},
                .wMaxPacketSize   = AUDIO_EP_MAX_PKT,
                .bInterval        = 1,
            };
            tusb_desc_endpoint_t ep_fb = {
                .bLength          = sizeof(tusb_desc_endpoint_t),
                .bDescriptorType  = TUSB_DESC_ENDPOINT,
                .bEndpointAddress = AUDIO_FB_ENDPOINT,
                .bmAttributes     = {.xfer = TUSB_XFER_ISOCHRONOUS,
                                     .sync = 0, /* no sync */
                                     .usage = 1 /* feedback */},
                .wMaxPacketSize   = 4,
                .bInterval        = 1,
            };
            uac1.ep_data_open = usbd_edpt_open(0, &ep_out);
            uac1.ep_fb_open   = usbd_edpt_open(0, &ep_fb);

            if (uac1.ep_data_open) {
                usbd_edpt_xfer(0, AUDIO_OUT_ENDPOINT, audio_out_buf, AUDIO_EP_MAX_PKT);
            }
            if (uac1.ep_fb_open) {
                usbd_edpt_xfer(0, AUDIO_FB_ENDPOINT, audio_fb_buf, 4);
            }
            audio_streaming = true;
        }
        tud_control_status(rhport, req);
        return true;
    }

    /* Class requests — feature-unit and EP controls */
    if (TUSB_REQ_TYPE_CLASS == req->bmRequestType_bit.type) {
        bool const is_get = (req->bmRequestType & 0x80) != 0;
        if (is_get) return handle_get_request(stage, req);
        return handle_set_request(stage, req);
    }

    return false;
}

/* ---------------- Endpoint transfer completions ---------------- */

static bool uac1_xfer_cb(uint8_t rhport, uint8_t ep_addr,
                         xfer_result_t result, uint32_t xferred_bytes) {
    (void)rhport; (void)result;

    if (ep_addr == AUDIO_OUT_ENDPOINT) {
        /* M3: discard. Just count. */
        audio_bytes_received   += xferred_bytes;
        audio_packets_received += 1;
        /* Re-arm for the next 1 ms frame. */
        usbd_edpt_xfer(0, AUDIO_OUT_ENDPOINT, audio_out_buf, AUDIO_EP_MAX_PKT);
        return true;
    }

    if (ep_addr == AUDIO_FB_ENDPOINT) {
        /* Re-arm with the same fixed nominal value. M4+ updates
         * audio_fb_buf from a SOF-driven PID. */
        usbd_edpt_xfer(0, AUDIO_FB_ENDPOINT, audio_fb_buf, 4);
        return true;
    }

    return false;
}

/* SOF — empty for M3. M4+ will stamp the host's frame counter into the
 * feedback PID and call the audio pipeline. */
static void uac1_sof(uint8_t rhport, uint32_t frame_count) {
    (void)rhport; (void)frame_count;
}

/* ====================================================================== */
/* Class driver registration                                              */
/* ====================================================================== */

static const usbd_class_driver_t uac1_driver = {
    .name            = "DSPi_UAC1",
    .init            = uac1_init,
    .deinit          = uac1_deinit,
    .reset           = uac1_reset,
    .open            = uac1_open,
    .control_xfer_cb = uac1_control_xfer_cb,
    .xfer_cb         = uac1_xfer_cb,
    .sof             = uac1_sof,
};

/* TinyUSB looks up this weak symbol during tud_init() — returning our
 * driver makes the stack route AC + AS interface and EP traffic through
 * our callbacks. Same hook the RP DSPi build uses. */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &uac1_driver;
}
