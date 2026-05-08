/**
 * vendor_commands.c — DSPi STM32H723 vendor USB control request dispatch
 *
 * M7a scaffold: just enough to handshake DSPi Console.
 *   REQ_GET_PLATFORM (0x7F) → returns PLATFORM_STM32H723 (=2)
 *
 * Subsequent milestones (M7b–M7d) bring over the rest of the ~30 vendor
 * commands from firmware/DSPi/vendor_commands.c, adapting platform-
 * specific bits (vreg/clocks/PIO references) as each handler arrives.
 */

#include <string.h>

#include "tusb.h"
#include "config.h"
#include "vendor_commands.h"

bool tud_vendor_control_xfer_cb(uint8_t rhport,
                                uint8_t stage,
                                tusb_control_request_t const *req) {
    if (stage != CONTROL_STAGE_SETUP) return true;

    /* GET requests (host wants data back) */
    if ((req->bmRequestType & 0x80) != 0) {
        switch (req->bRequest) {
            case REQ_GET_PLATFORM: {
                static uint8_t platform = (uint8_t)DSPI_PLATFORM;
                return tud_control_xfer(rhport, (tusb_control_request_t *)req,
                                        &platform, 1);
            }
            default:
                /* Unknown — STALL via false return so the host's command
                 * dispatch can detect "not supported" without us having
                 * to faux-respond. M7b populates the rest of the table. */
                return false;
        }
    }

    /* SET requests — none implemented yet. STALL until M7b. */
    return false;
}
