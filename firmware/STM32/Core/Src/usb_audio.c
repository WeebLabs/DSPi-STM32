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

    /* Enable SOF event delivery for the audio class. Required: the dwc2
     * ISO IN feedback transfer logic uses the SOF interrupt to schedule
     * the next packet's data into the FIFO before the host's IN token
     * arrives. Without this enabled, subsequent feedback transfers may
     * silently produce zero-length packets after the first few frames,
     * which the host eventually treats as a dead device and USB-suspends
     * — matching the symptom of "device disappears after ~15-50 s." */
    usbd_sof_enable(rhport, SOF_CONSUMER_AUDIO, true);

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

/* UAC1 class-request wIndex layout (USB Audio 1.0 spec §5.2.1.1):
 *
 *   bmRequestType.recipient = 0x01 (Interface):
 *       wIndex.HI = entity ID (input terminal / feature unit / output term)
 *       wIndex.LO = interface number
 *
 *   bmRequestType.recipient = 0x02 (Endpoint):
 *       wIndex.HI = 0
 *       wIndex.LO = endpoint address
 *
 * The earlier draft used tu_u16_low() unconditionally, which made every
 * feature-unit query fall through to the sample-rate clause (cs=0x01 is
 * also AUDIO_FU_CTRL_MUTE) and return 3 bytes for a 1-byte mute query.
 * macOS detects the wLength mismatch on enumeration tolerates it, but
 * tears down the device a few seconds after SET_INTERFACE alt 1 once it
 * notices the feature-unit interrogations are also broken. */

#define UAC1_RECIPIENT_INTERFACE  0x01
#define UAC1_RECIPIENT_ENDPOINT   0x02

static bool handle_get_request(uint8_t stage, tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP) return true;

    uint8_t const cs    = tu_u16_high(req->wValue);   /* control selector */
    uint8_t const cn    = tu_u16_low (req->wValue);   /* channel number   */
    uint8_t const recip = req->bmRequestType & 0x1F;

    if (recip == UAC1_RECIPIENT_INTERFACE) {
        uint8_t const entity_id = tu_u16_high(req->wIndex);

        if (entity_id == UAC1_FEATURE_UNIT_ID && cn == 0) {
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
    } else if (recip == UAC1_RECIPIENT_ENDPOINT) {
        /* Sample-frequency control on the EP. Only one rate (48 kHz). */
        if (cs == AUDIO_CS_CTRL_SAM_FREQ && req->bRequest == UAC1_REQ_GET_CUR) {
            static uint8_t freq[3] = {
                (AUDIO_SAMPLE_RATE)       & 0xFF,
                (AUDIO_SAMPLE_RATE >>  8) & 0xFF,
                (AUDIO_SAMPLE_RATE >> 16) & 0xFF,
            };
            return tud_control_xfer(0, (tusb_control_request_t *)req, freq, 3);
        }
    }

    return false;
}

static bool handle_set_request(uint8_t stage, tusb_control_request_t const *req) {
    /* SETUP — record what to expect and accept the data stage */
    if (stage == CONTROL_STAGE_SETUP) {
        uint8_t const recip = req->bmRequestType & 0x1F;
        uac1.pending_cs        = tu_u16_high(req->wValue);
        uac1.pending_recipient = (recip == UAC1_RECIPIENT_INTERFACE)
                                 ? tu_u16_high(req->wIndex)   /* entity ID */
                                 : tu_u16_low (req->wIndex);  /* EP addr   */
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

/* Apply an AS-interface alt change.
 *
 * dwc2-specific protocol (see tinyusb/src/class/audio/audio_device.c):
 * the FIFO allocation is done ONCE in driver_open() via
 * usbd_edpt_iso_alloc(); subsequent alt changes use
 * usbd_edpt_iso_activate() to bring the EP up, and on alt 0 we simply
 * stop arming transfers — we do NOT call usbd_edpt_close(), because
 * close() releases controller-level state (including the ISO frame-
 * parity counter) and a re-open lands on the wrong parity, triggering
 * IISOIXFR retries until iso_retry exhausts and the EP is silently
 * disabled. That manifests as "device works on auto-select but
 * disappears after switching away and back."
 *
 * Idempotent SET_INTERFACE(alt = current_alt) is common from host
 * driver probes; bail early so we don't tear down a healthy stream. */
static bool uac1_apply_alt(uint8_t rhport, uint8_t alt) {
    if (alt == uac1.cur_alt) return true;
    uac1.cur_alt = alt;

    if (alt == 0) {
        /* No close. Just stop the data flow; the next alt-1 reactivates
         * the same dwc2 EP without losing frame-parity sync. */
        uac1.ep_data_open = false;
        uac1.ep_fb_open   = false;
        audio_streaming = false;
        return true;
    }

    if (alt == 1) {
        /* The bmAttributes bitfield layout is .xfer:2 / .sync:2 / .usage:2
         * — values mirror the wire-format bytes 0x05 (data) and 0x11 (FB)
         * from the config descriptor. */
        tusb_desc_endpoint_t ep_out = {
            .bLength          = sizeof(tusb_desc_endpoint_t),
            .bDescriptorType  = TUSB_DESC_ENDPOINT,
            .bEndpointAddress = AUDIO_OUT_ENDPOINT,
            .bmAttributes     = {.xfer = TUSB_XFER_ISOCHRONOUS,
                                 .sync = 1, .usage = 0},
            .wMaxPacketSize   = AUDIO_EP_MAX_PKT,
            .bInterval        = 1,
        };
        tusb_desc_endpoint_t ep_fb = {
            .bLength          = sizeof(tusb_desc_endpoint_t),
            .bDescriptorType  = TUSB_DESC_ENDPOINT,
            .bEndpointAddress = AUDIO_FB_ENDPOINT,
            .bmAttributes     = {.xfer = TUSB_XFER_ISOCHRONOUS,
                                 .sync = 0, .usage = 1},
            .wMaxPacketSize   = 3,
            .bInterval        = 1,
        };
        uac1.ep_data_open = usbd_edpt_iso_activate(rhport, &ep_out);
        uac1.ep_fb_open   = usbd_edpt_iso_activate(rhport, &ep_fb);

        /* Clear any stale halt state from a previous activation cycle —
         * the audio_device.c TODO note in upstream TinyUSB calls this a
         * workaround for an ep_close() omission, but we need it for the
         * activate() path too on dwc2. */
        usbd_edpt_clear_stall(rhport, AUDIO_OUT_ENDPOINT);
        usbd_edpt_clear_stall(rhport, AUDIO_FB_ENDPOINT);

        if (uac1.ep_data_open) {
            usbd_edpt_xfer(rhport, AUDIO_OUT_ENDPOINT, audio_out_buf, AUDIO_EP_MAX_PKT);
        }
        /* Feedback EP is armed on the very next SOF (uac1_sof). Arming
         * it here would land on a frame the dwc2 controller hasn't
         * scheduled an IN token for yet, defeating the SOF timing fix. */
        audio_streaming = true;
        return true;
    }

    return false;
}

static bool uac1_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *req) {
    if (stage == CONTROL_STAGE_SETUP) {
        /* ---- Standard requests on our interfaces ----
         *
         * Custom class drivers must answer GET_INTERFACE and SET_INTERFACE
         * for every interface they own. macOS aggressively probes both
         * after the user picks the device as output — STALLing either
         * causes Core Audio to un-route audio (while leaving the USB
         * session alive, so tud_mounted() stays true and the LED keeps
         * blinking). Earlier draft only handled SET_INTERFACE on AS and
         * silently STALLed everything else; macOS dropped the device
         * from the active output list a few seconds in.
         */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            uint8_t const itf = (uint8_t)tu_u16_low(req->wIndex);

            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t const alt = (uint8_t)req->wValue;
                if (itf == ITF_NUM_AC) {
                    /* AC has only alt 0 */
                    if (alt != 0) return false;
                    return tud_control_status(rhport, (tusb_control_request_t *)req);
                }
                if (itf == ITF_NUM_AS) {
                    if (!uac1_apply_alt(rhport, alt)) return false;
                    return tud_control_status(rhport, (tusb_control_request_t *)req);
                }
                return false;
            }

            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                static uint8_t alt_resp;
                if      (itf == ITF_NUM_AC) alt_resp = 0;
                else if (itf == ITF_NUM_AS) alt_resp = uac1.cur_alt;
                else                        return false;
                return tud_control_xfer(rhport, (tusb_control_request_t *)req,
                                        &alt_resp, 1);
            }

            return false;
        }

        /* ---- Class requests on our interfaces / endpoints ---- */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
            bool const is_get = (req->bmRequestType & 0x80) != 0;
            if (is_get) return handle_get_request(stage, req);
            return handle_set_request(stage, req);
        }

        return false;
    }

    /* Pass DATA / ACK stages through to the same per-direction handlers
     * for SET requests that need to consume their data payload. */
    if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS) {
        bool const is_get = (req->bmRequestType & 0x80) != 0;
        if (!is_get) return handle_set_request(stage, req);
    }
    return true;
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
        /* Do NOT re-arm here. The dwc2 ISO IN feedback path needs the
         * next packet armed via the SOF interrupt path (which sets the
         * SEVNFRM/SODDFRM frame-parity bits in DIEPCTL correctly).
         * Re-arming from xfer_cb lands on the wrong parity, triggers
         * IISOIXFR retries, and after iso_retry exhaustion the dwc2
         * driver disables the endpoint silently — at which point macOS
         * stops seeing feedback responses, waits ~30 s, then suspends
         * the device. That was the M3+M4 "disappears after 15-50 s"
         * symptom. The arming happens in uac1_sof() below. */
        return true;
    }

    return false;
}

/* SOF callback — fires once per USB frame (1 ms on FS). dwc2 sets up the
 * next ISO IN packet from this context with correct frame-parity bits.
 * `usbd_edpt_busy` guards against double-arming when a previous transfer
 * is still in flight. M5+ will replace the static feedback value with a
 * SOF-driven PID controller that trims the device's audio clock to the
 * host's. */
static void uac1_sof(uint8_t rhport, uint32_t frame_count) {
    (void)frame_count;
    if (uac1.cur_alt != 1) return;
    if (!uac1.ep_fb_open)  return;
    if (usbd_edpt_busy(rhport, AUDIO_FB_ENDPOINT)) return;

    usbd_edpt_xfer(rhport, AUDIO_FB_ENDPOINT, audio_fb_buf, 3);
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
