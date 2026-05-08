/**
 * vendor_commands.c — DSPi STM32H723 vendor USB control request dispatch
 *
 * M7a: GET_PLATFORM stub.
 * M7b: + GET_ALL_PARAMS / SET_ALL_PARAMS bulk transfers, GET_STATUS.
 *      DSPi Console reads the entire DSP state in one shot via
 *      REQ_GET_ALL_PARAMS and populates its UI from that. SET_ALL_PARAMS
 *      writes back; the DSP-pipeline globals get the new values, but
 *      audio processing isn't wired up yet (M7c).
 *
 * Subsequent milestones (M7c–M7d) port the rest of the ~30 vendor
 * commands from firmware/DSPi/vendor_commands.c.
 */

#include <string.h>

#include "tusb.h"
#include "config.h"
#include "vendor_commands.h"
#include "bulk_params.h"

/* Bulk SET payload buffer — sized for one WireBulkParams transfer.
 * tud_control_xfer chunks the actual EP0 transfers; the application
 * just provides one contiguous buffer of wLength bytes. */
static uint8_t __attribute__((aligned(4))) bulk_param_buf[sizeof(WireBulkParams)];

/* Set on the SET-side ACK stage; the main loop checks and calls
 * bulk_params_apply() when true (M7c will hook the main loop into
 * this — for M7b we just record it and let the DSP-pipeline-less
 * code path effectively no-op the application). */
volatile bool bulk_params_pending = false;

/* Captured at SETUP so the DATA-stage handler knows what to do. */
static uint8_t  vendor_last_request = 0;
static uint16_t vendor_last_wLength = 0;

bool tud_vendor_control_xfer_cb(uint8_t rhport,
                                uint8_t stage,
                                tusb_control_request_t const *req) {
    /* ---- SETUP stage ---- */
    if (stage == CONTROL_STAGE_SETUP) {
        bool const is_get = (req->bmRequestType & 0x80) != 0;

        if (is_get) {
            switch (req->bRequest) {
                case REQ_GET_PLATFORM: {
                    /* DSPi Console expects 4 bytes:
                     *   [0] platform ID
                     *   [1] FW major (BCD high byte)
                     *   [2] FW minor (high nibble) + patch (low nibble)
                     *   [3] NUM_OUTPUT_CHANNELS (informational)
                     * Returning fewer = short-read STALL = Console dies. */
                    static uint8_t resp[4] = {
                        (uint8_t)DSPI_PLATFORM,
                        (uint8_t)(FW_VERSION_BCD >> 8),
                        (uint8_t)(FW_VERSION_BCD & 0xFF),
                        (uint8_t)NUM_OUTPUT_CHANNELS,
                    };
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            resp, sizeof(resp));
                }
                case REQ_GET_ALL_PARAMS: {
                    bulk_params_collect((WireBulkParams *)bulk_param_buf);
                    uint16_t len = sizeof(WireBulkParams);
                    if (req->wLength < len) len = req->wLength;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            bulk_param_buf, len);
                }
                default:
                    return false;
            }
        }

        /* SET path — record context, set up DATA-stage receive. */
        vendor_last_request = req->bRequest;
        vendor_last_wLength = req->wLength;

        if (req->bRequest == REQ_SET_ALL_PARAMS
            && req->wLength == sizeof(WireBulkParams)) {
            return tud_control_xfer(rhport,
                                    (tusb_control_request_t *)req,
                                    bulk_param_buf, req->wLength);
        }

        if (req->wLength == 0) {
            /* Zero-length SET — ack immediately. */
            return tud_control_status(rhport, (tusb_control_request_t *)req);
        }

        /* All other SETs not yet implemented — STALL. M7c+ adds. */
        return false;
    }

    /* ---- DATA stage (SET payload arrived) ---- */
    if (stage == CONTROL_STAGE_DATA) {
        /* M7b only handles bulk SET; per-field SETs come in M7c+. */
        return true;
    }

    /* ---- ACK stage ---- */
    if (stage == CONTROL_STAGE_ACK) {
        if ((req->bmRequestType & 0x80) == 0
            && req->bRequest == REQ_SET_ALL_PARAMS) {
            bulk_params_pending = true;
        }
        return true;
    }

    return true;
}
