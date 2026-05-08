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
#include <math.h>

#include "tusb.h"
#include "config.h"
#include "vendor_commands.h"
#include "bulk_params.h"
#include "dsp_pipeline.h"

#include "usb_audio.h"   /* update_master_volume, AudioState, channel_names */
#include "audio_input.h" /* INPUT_SOURCE_USB */

extern uint8_t channel_band_counts[NUM_CHANNELS];
extern MatrixMixer matrix_mixer;
extern volatile float    global_preamp_db    [NUM_INPUT_CHANNELS];
extern volatile float    global_preamp_linear[NUM_INPUT_CHANNELS];
extern volatile int32_t  global_preamp_mul   [NUM_INPUT_CHANNELS];
extern volatile float    master_volume_db;
extern volatile SystemStatusPacket global_status;

static inline float db_to_linear(float db) {
    return powf(10.0f, db * 0.05f);
}

/* M7d helper: keep all three preamp views (db / linear / Q28 mul) in
 * sync. The DSP fill_half currently uses `_linear`; the others are
 * preserved so the bulk_params snapshot matches the RP build's wire
 * format. */
static void update_preamp_ch(uint8_t ch, float db) {
    if (ch >= NUM_INPUT_CHANNELS) return;
    if (db < -90.0f) db = -90.0f;
    if (db >  20.0f) db =  20.0f;
    float lin = db_to_linear(db);
    global_preamp_db    [ch] = db;
    global_preamp_linear[ch] = lin;
    global_preamp_mul   [ch] = (int32_t)(lin * (1 << 28));
}

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
static uint16_t vendor_last_wValue  = 0;
static uint16_t vendor_last_wLength = 0;
static uint8_t  vendor_rx_buf[64];   /* Generic SET payload buffer (≤ EP0 size) */

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
                case REQ_GET_STATUS: {
                    /* Console polls wValue=9 for combined status:
                     *   NUM_CHANNELS×u16 peaks + cpu0 + cpu1 + u16 clip_flags
                     * = 11×2 + 1 + 1 + 2 = 26 bytes on STM32H723 (RP2350-shape).
                     * wValue=15 returns the current sample rate as u32. */
                    static uint8_t status_buf[NUM_CHANNELS * 2 + 4];
                    if (req->wValue == 9) {
                        for (int i = 0; i < NUM_CHANNELS; ++i) {
                            status_buf[i*2]     = global_status.peaks[i] & 0xFF;
                            status_buf[i*2 + 1] = global_status.peaks[i] >> 8;
                        }
                        status_buf[NUM_CHANNELS * 2]     = global_status.cpu0_load;
                        status_buf[NUM_CHANNELS * 2 + 1] = global_status.cpu1_load;
                        status_buf[NUM_CHANNELS * 2 + 2] = global_status.clip_flags & 0xFF;
                        status_buf[NUM_CHANNELS * 2 + 3] = global_status.clip_flags >> 8;
                        return tud_control_xfer(rhport,
                                                (tusb_control_request_t *)req,
                                                status_buf, sizeof(status_buf));
                    }
                    if (req->wValue == 15) {
                        static uint32_t rate;
                        rate = audio_state.freq;
                        return tud_control_xfer(rhport,
                                                (tusb_control_request_t *)req,
                                                &rate, 4);
                    }
                    return false;
                }
                case REQ_GET_PREAMP: {
                    /* Legacy single-value GET — returns channel 0's preamp. */
                    static float v;
                    v = global_preamp_db[0];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_PREAMP_CH: {
                    uint8_t ch = req->wValue & 0xFF;
                    if (ch >= NUM_INPUT_CHANNELS) return false;
                    static float v;
                    v = global_preamp_db[ch];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_MASTER_VOLUME: {
                    static float v;
                    v = master_volume_db;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_INPUT_SOURCE: {
                    /* USB-only on STM32 until M8 brings SPDIFRX. */
                    static uint8_t src = INPUT_SOURCE_USB;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &src, 1);
                }
                case REQ_GET_EQ_PARAM: {
                    /* wValue encodes (channel<<8) | (band<<4) | param.
                     *   param: 0=type, 1=freq(f32), 2=Q(f32), 3=gain_db(f32), 4=bypass */
                    uint8_t channel = (req->wValue >> 8) & 0xFF;
                    uint8_t band    = (req->wValue >> 4) & 0x0F;
                    uint8_t param   =  req->wValue       & 0x0F;
                    if (channel >= NUM_CHANNELS
                        || band >= channel_band_counts[channel]) return false;
                    static uint32_t resp;
                    EqParamPacket *p = &filter_recipes[channel][band];
                    switch (param) {
                        case 0: resp = (uint32_t)p->type; break;
                        case 1: memcpy(&resp, &p->freq,    4); break;
                        case 2: memcpy(&resp, &p->Q,       4); break;
                        case 3: memcpy(&resp, &p->gain_db, 4); break;
                        case 4: resp = (p->bypass == 1) ? 1u : 0u; break;
                        default: return false;
                    }
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &resp, 4);
                }
                default:
                    return false;
            }
        }

        /* SET path — record context, set up DATA-stage receive. */
        vendor_last_request = req->bRequest;
        vendor_last_wValue  = req->wValue;
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

        /* All other small SETs land in vendor_rx_buf for the DATA-stage
         * dispatcher to consume. */
        if (req->wLength <= sizeof(vendor_rx_buf)) {
            return tud_control_xfer(rhport,
                                    (tusb_control_request_t *)req,
                                    vendor_rx_buf, req->wLength);
        }

        return false;
    }

    /* ---- DATA stage (SET payload arrived) ---- */
    if (stage == CONTROL_STAGE_DATA) {
        if ((req->bmRequestType & 0x80) != 0) return true;   /* GET — ignore */

        switch (vendor_last_request) {
            case REQ_SET_EQ_PARAM: {
                if (vendor_last_wLength < sizeof(EqParamPacket)) break;
                EqParamPacket pkt;
                memcpy(&pkt, vendor_rx_buf, sizeof(pkt));
                pkt.bypass = (pkt.bypass == 1) ? 1 : 0;
                if (pkt.channel < NUM_CHANNELS
                    && pkt.band < channel_band_counts[pkt.channel]) {
                    filter_recipes[pkt.channel][pkt.band] = pkt;
                    dsp_compute_coefficients(&filter_recipes[pkt.channel][pkt.band],
                                             &filters[pkt.channel][pkt.band],
                                             48000.0f);
                }
                break;
            }

            case REQ_SET_MATRIX_ROUTE: {
                if (vendor_last_wLength < sizeof(MatrixRoutePacket)) break;
                MatrixRoutePacket pkt;
                memcpy(&pkt, vendor_rx_buf, sizeof(pkt));
                if (pkt.input < NUM_INPUT_CHANNELS
                    && pkt.output < NUM_OUTPUT_CHANNELS) {
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[pkt.input][pkt.output];
                    xp->enabled      = pkt.enabled;
                    xp->phase_invert = pkt.phase_invert;
                    xp->gain_db      = pkt.gain_db;
                    xp->gain_linear  = db_to_linear(pkt.gain_db);
                }
                break;
            }

            case REQ_SET_OUTPUT_ENABLE: {
                uint8_t out = vendor_last_wValue & 0xFF;
                if (out < NUM_OUTPUT_CHANNELS && vendor_last_wLength >= 1) {
                    matrix_mixer.outputs[out].enabled = (vendor_rx_buf[0] != 0);
                }
                break;
            }

            case REQ_SET_OUTPUT_GAIN: {
                uint8_t out = vendor_last_wValue & 0xFF;
                if (out < NUM_OUTPUT_CHANNELS && vendor_last_wLength >= 4) {
                    float db;
                    memcpy(&db, vendor_rx_buf, 4);
                    matrix_mixer.outputs[out].gain_db     = db;
                    matrix_mixer.outputs[out].gain_linear = db_to_linear(db);
                }
                break;
            }

            case REQ_SET_OUTPUT_MUTE: {
                uint8_t out = vendor_last_wValue & 0xFF;
                if (out < NUM_OUTPUT_CHANNELS && vendor_last_wLength >= 1) {
                    matrix_mixer.outputs[out].mute = vendor_rx_buf[0];
                }
                break;
            }

            case REQ_SET_OUTPUT_DELAY: {
                uint8_t out = vendor_last_wValue & 0xFF;
                if (out < NUM_OUTPUT_CHANNELS && vendor_last_wLength >= 4) {
                    float ms;
                    memcpy(&ms, vendor_rx_buf, 4);
                    matrix_mixer.outputs[out].delay_ms = ms;
                }
                break;
            }

            case REQ_SET_PREAMP: {
                /* Legacy: single-value SET applies to all input channels. */
                if (vendor_last_wLength < 4) break;
                float db; memcpy(&db, vendor_rx_buf, 4);
                for (uint8_t ch = 0; ch < NUM_INPUT_CHANNELS; ++ch) {
                    update_preamp_ch(ch, db);
                }
                break;
            }
            case REQ_SET_PREAMP_CH: {
                if (vendor_last_wLength < 4) break;
                uint8_t ch = vendor_last_wValue & 0xFF;
                float db; memcpy(&db, vendor_rx_buf, 4);
                update_preamp_ch(ch, db);
                break;
            }

            case REQ_SET_MASTER_VOLUME: {
                if (vendor_last_wLength < 4) break;
                float db; memcpy(&db, vendor_rx_buf, 4);
                update_master_volume(db);
                break;
            }

            default:
                /* Other SETs land here once their handlers arrive. */
                break;
        }
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
