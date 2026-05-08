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

extern volatile bool   bypass_master_eq;
extern volatile bool   loudness_enabled;
extern volatile float  loudness_ref_spl;
extern volatile float  loudness_intensity_pct;
extern volatile bool   loudness_recompute_pending;
#include "crossfeed.h"
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool            crossfeed_update_pending;
#include "leveller.h"
extern volatile LevellerConfig leveller_config;
extern volatile bool           leveller_update_pending;
extern volatile bool           leveller_reset_pending;
extern volatile float channel_gain_db    [3];
extern volatile float channel_gain_linear[3];
extern volatile int32_t channel_gain_mul [3];
extern volatile bool  channel_mute       [3];
extern float channel_delays_ms[NUM_CHANNELS];
extern int32_t channel_delay_samples[NUM_DELAY_CHANNELS];
extern bool    any_delay_active;
extern uint8_t output_types[NUM_SPDIF_INSTANCES];
extern uint8_t output_pins[NUM_PIN_OUTPUTS];
extern uint8_t  i2s_bck_pin;
extern uint8_t  i2s_mck_pin;
extern bool     i2s_mck_enabled;
extern uint16_t i2s_mck_multiplier;

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
/* bulk_param_buf is non-static — main.c drains it from the main loop after
 * REQ_SET_ALL_PARAMS deposits a full state via tud_control_xfer. */
uint8_t __attribute__((aligned(4))) bulk_param_buf[sizeof(WireBulkParams)];

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

                case REQ_GET_BYPASS: {
                    static uint8_t v;
                    v = bypass_master_eq ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_DELAY: {
                    uint8_t ch = req->wValue & 0xFF;
                    if (ch >= NUM_CHANNELS) return false;
                    static float v;
                    v = channel_delays_ms[ch];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_CHANNEL_GAIN: {
                    uint8_t ch = req->wValue & 0xFF;
                    if (ch >= 3) return false;
                    static float v;
                    v = channel_gain_db[ch];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_CHANNEL_MUTE: {
                    uint8_t ch = req->wValue & 0xFF;
                    if (ch >= 3) return false;
                    static uint8_t v;
                    v = channel_mute[ch] ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }

                case REQ_GET_LOUDNESS: {
                    static uint8_t v; v = loudness_enabled ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_LOUDNESS_REF: {
                    static float v; v = loudness_ref_spl;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_LOUDNESS_INTENSITY: {
                    static float v; v = loudness_intensity_pct;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }

                case REQ_GET_CROSSFEED: {
                    static uint8_t v; v = crossfeed_config.enabled ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_CROSSFEED_PRESET: {
                    static uint8_t v; v = crossfeed_config.preset;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_CROSSFEED_FREQ: {
                    static float v; v = crossfeed_config.custom_fc;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_CROSSFEED_FEED: {
                    static float v; v = crossfeed_config.custom_feed_db;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_CROSSFEED_ITD: {
                    static uint8_t v; v = crossfeed_config.itd_enabled ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }

                case REQ_GET_LEVELLER_ENABLE: {
                    static uint8_t v; v = leveller_config.enabled ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_LEVELLER_AMOUNT: {
                    static float v; v = leveller_config.amount;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_LEVELLER_SPEED: {
                    static uint8_t v; v = leveller_config.speed;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_LEVELLER_MAX_GAIN: {
                    static float v; v = leveller_config.max_gain_db;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_LEVELLER_LOOKAHEAD: {
                    static uint8_t v; v = leveller_config.lookahead ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_LEVELLER_GATE: {
                    static float v; v = leveller_config.gate_threshold_db;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case 0xFE: {  /* DEBUG: dump delay-stage internal state */
                    static struct __attribute__((packed)) {
                        int32_t  ds0;       /* channel_delay_samples[0]   */
                        int32_t  ds1;       /* channel_delay_samples[1]   */
                        uint8_t  active;    /* any_delay_active           */
                        uint8_t  pad[3];
                        uint32_t freq;      /* audio_state.freq           */
                        float    ms_out0;   /* channel_delays_ms[CH_OUT_1]*/
                        float    ms_out1;   /* channel_delays_ms[CH_OUT_2]*/
                    } d;
                    d.ds0     = channel_delay_samples[0];
                    d.ds1     = channel_delay_samples[1];
                    d.active  = any_delay_active ? 1 : 0;
                    d.freq    = audio_state.freq;
                    d.ms_out0 = channel_delays_ms[CH_OUT_1];
                    d.ms_out1 = channel_delays_ms[CH_OUT_2];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &d, sizeof(d));
                }

                case REQ_GET_MATRIX_ROUTE: {
                    uint8_t in_  = (req->wValue >> 8) & 0xFF;
                    uint8_t out  =  req->wValue       & 0xFF;
                    if (in_ >= NUM_INPUT_CHANNELS || out >= NUM_OUTPUT_CHANNELS)
                        return false;
                    static MatrixRoutePacket pkt;
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[in_][out];
                    pkt.input = in_; pkt.output = out;
                    pkt.enabled = xp->enabled;
                    pkt.phase_invert = xp->phase_invert;
                    pkt.gain_db = xp->gain_db;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &pkt, sizeof(pkt));
                }
                case REQ_GET_OUTPUT_ENABLE: {
                    uint8_t out = (uint8_t)req->wValue;
                    if (out >= NUM_OUTPUT_CHANNELS) return false;
                    static uint8_t v;
                    v = matrix_mixer.outputs[out].enabled;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_OUTPUT_GAIN: {
                    uint8_t out = (uint8_t)req->wValue;
                    if (out >= NUM_OUTPUT_CHANNELS) return false;
                    static float v;
                    v = matrix_mixer.outputs[out].gain_db;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_GET_OUTPUT_MUTE: {
                    uint8_t out = (uint8_t)req->wValue;
                    if (out >= NUM_OUTPUT_CHANNELS) return false;
                    static uint8_t v;
                    v = matrix_mixer.outputs[out].mute;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_OUTPUT_DELAY: {
                    uint8_t out = (uint8_t)req->wValue;
                    if (out >= NUM_OUTPUT_CHANNELS) return false;
                    static float v;
                    v = matrix_mixer.outputs[out].delay_ms;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }

                case REQ_GET_CHANNEL_NAME: {
                    uint8_t ch = req->wValue & 0xFF;
                    if (ch >= NUM_CHANNELS) return false;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            channel_names[ch], PRESET_NAME_LEN);
                }
                case REQ_GET_OUTPUT_TYPE: {
                    uint8_t slot = (uint8_t)req->wValue;
                    if (slot >= NUM_SPDIF_INSTANCES) return false;
                    static uint8_t v;
                    v = output_types[slot];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }

                /* ---- I2S clock / pin config (Console probes these) ---- */
                case REQ_GET_I2S_BCK_PIN: {
                    static uint8_t v; v = i2s_bck_pin;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_MCK_ENABLE: {
                    static uint8_t v; v = i2s_mck_enabled ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_MCK_PIN: {
                    static uint8_t v; v = i2s_mck_pin;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_MCK_MULTIPLIER: {
                    static uint16_t v; v = i2s_mck_multiplier;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 2);
                }

                /* ---- Per-output pin (SPDIF/PDM GPIO assignments) ---- */
                case REQ_GET_OUTPUT_PIN: {
                    uint8_t out = (uint8_t)req->wValue;
                    if (out >= NUM_PIN_OUTPUTS) return false;
                    static uint8_t v; v = output_pins[out];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }

                /* ---- Identification ---- */
                case REQ_GET_SERIAL: {
                    /* 16-byte ASCII serial. Use the same string Console
                     * shows in System Info; padded with zeros. */
                    static uint8_t serial[16] = "0001";
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            serial, sizeof(serial));
                }

                /* ---- Core 1 mode (no Core 1 on H723) ---- */
                case REQ_GET_CORE1_MODE: {
                    static uint8_t v = 0; /* CORE1_MODE_IDLE — no second core */
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_CORE1_CONFLICT: {
                    /* No conflicts — single core handles everything. */
                    static uint8_t v = 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }

                /* ---- Master volume mode (independent / per-preset) ---- */
                case REQ_GET_MASTER_VOLUME_MODE: {
                    static uint8_t v = 0; /* MASTER_VOLUME_MODE_INDEPENDENT */
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }

                /* ---- SPDIF RX pin (no SPDIF RX in M7d) ---- */
                case REQ_GET_SPDIF_RX_PIN: {
                    static uint8_t v = 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }

                /* ---- Preset directory: empty (no preset storage in M7d) ----
                 *
                 *   [0-1] occupied bitmask u16 = 0  (no slots filled)
                 *   [2]   startup_mode = 0         (no auto-load)
                 *   [3]   default_slot = 0
                 *   [4]   last_active = 0
                 *   [5]   include_pins = 0
                 *   [6]   master_volume_mode = 0
                 */
                case REQ_PRESET_GET_DIR: {
                    static uint8_t dir[7] = { 0 };
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            dir, sizeof(dir));
                }
                case REQ_PRESET_GET_ACTIVE: {
                    static uint8_t v = 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_PRESET_GET_STARTUP: {
                    static uint8_t v[3] = { 0, 0, 0 };
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            v, sizeof(v));
                }
                case REQ_PRESET_GET_INCLUDE_PINS: {
                    static uint8_t v = 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_PRESET_GET_NAME: {
                    /* All slots empty — return zero-filled name. */
                    static char nm[32] = { 0 };
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            nm, sizeof(nm));
                }

                /* ---- Clear clips ---- */
                case REQ_CLEAR_CLIPS: {
                    static uint16_t f;
                    f = global_status.clip_flags;
                    global_status.clip_flags = 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &f, 2);
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
                    if (ms < 0) ms = 0;
                    /* Two storages, must stay in sync (matches bulk_params_apply):
                     *   - matrix_mixer.outputs[out].delay_ms — surfaced via
                     *     REQ_GET_OUTPUT_DELAY and the bulk wire format
                     *   - channel_delays_ms[CH_OUT_1 + out] — what
                     *     dsp_update_delay_samples actually reads to compute
                     *     the per-output sample counts the audio path uses
                     * The earlier impl wrote only the first field, so Console's
                     * output-delay slider was a silent no-op even though the
                     * GET round-tripped fine. */
                    matrix_mixer.outputs[out].delay_ms = ms;
                    channel_delays_ms[CH_OUT_1 + out]  = ms;
                    dsp_update_delay_samples((float)audio_state.freq);
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

            case REQ_SET_BYPASS:
                if (vendor_last_wLength >= 1)
                    bypass_master_eq = (vendor_rx_buf[0] != 0);
                break;

            case REQ_SET_DELAY: {
                uint8_t ch = vendor_last_wValue & 0xFF;
                if (ch < NUM_CHANNELS && vendor_last_wLength >= 4) {
                    float ms; memcpy(&ms, vendor_rx_buf, 4);
                    if (ms < 0) ms = 0;
                    channel_delays_ms[ch] = ms;
                    /* Recompute delay-sample counts + any_delay_active bypass
                     * flag for fill_half's Stage 6.5 delay-line stage. */
                    dsp_update_delay_samples((float)audio_state.freq);
                }
                break;
            }

            case REQ_SET_CHANNEL_GAIN: {
                uint8_t ch = vendor_last_wValue & 0xFF;
                if (ch < 3 && vendor_last_wLength >= 4) {
                    float db; memcpy(&db, vendor_rx_buf, 4);
                    float lin = db_to_linear(db);
                    channel_gain_db    [ch] = db;
                    channel_gain_linear[ch] = lin;
                    channel_gain_mul   [ch] = (int32_t)(lin * 32768.0f);
                }
                break;
            }
            case REQ_SET_CHANNEL_MUTE: {
                uint8_t ch = vendor_last_wValue & 0xFF;
                if (ch < 3 && vendor_last_wLength >= 1)
                    channel_mute[ch] = (vendor_rx_buf[0] != 0);
                break;
            }

            case REQ_SET_LOUDNESS:
                if (vendor_last_wLength >= 1)
                    loudness_enabled = (vendor_rx_buf[0] != 0);
                break;
            case REQ_SET_LOUDNESS_REF:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v <  40.0f) v =  40.0f;
                    if (v > 100.0f) v = 100.0f;
                    loudness_ref_spl = v;
                    loudness_recompute_pending = true;
                }
                break;
            case REQ_SET_LOUDNESS_INTENSITY:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v <   0.0f) v =   0.0f;
                    if (v > 200.0f) v = 200.0f;
                    loudness_intensity_pct = v;
                    loudness_recompute_pending = true;
                }
                break;

            case REQ_SET_CROSSFEED:
                if (vendor_last_wLength >= 1) {
                    crossfeed_config.enabled = (vendor_rx_buf[0] != 0);
                    crossfeed_update_pending = true;
                }
                break;
            case REQ_SET_CROSSFEED_PRESET:
                if (vendor_last_wLength >= 1) {
                    uint8_t p = vendor_rx_buf[0];
                    if (p <= CROSSFEED_PRESET_CUSTOM) {
                        crossfeed_config.preset = p;
                        crossfeed_update_pending = true;
                    }
                }
                break;
            case REQ_SET_CROSSFEED_FREQ:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v < CROSSFEED_FREQ_MIN) v = CROSSFEED_FREQ_MIN;
                    if (v > CROSSFEED_FREQ_MAX) v = CROSSFEED_FREQ_MAX;
                    crossfeed_config.custom_fc = v;
                    if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM)
                        crossfeed_update_pending = true;
                }
                break;
            case REQ_SET_CROSSFEED_FEED:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v < CROSSFEED_FEED_MIN) v = CROSSFEED_FEED_MIN;
                    if (v > CROSSFEED_FEED_MAX) v = CROSSFEED_FEED_MAX;
                    crossfeed_config.custom_feed_db = v;
                    if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM)
                        crossfeed_update_pending = true;
                }
                break;
            case REQ_SET_CROSSFEED_ITD:
                if (vendor_last_wLength >= 1) {
                    crossfeed_config.itd_enabled = (vendor_rx_buf[0] != 0);
                    crossfeed_update_pending = true;
                }
                break;

            case REQ_SET_LEVELLER_ENABLE:
                if (vendor_last_wLength >= 1) {
                    leveller_config.enabled = (vendor_rx_buf[0] != 0);
                    leveller_update_pending = true;
                    leveller_reset_pending  = true;
                }
                break;
            case REQ_SET_LEVELLER_AMOUNT:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v < LEVELLER_AMOUNT_MIN) v = LEVELLER_AMOUNT_MIN;
                    if (v > LEVELLER_AMOUNT_MAX) v = LEVELLER_AMOUNT_MAX;
                    leveller_config.amount = v;
                    leveller_update_pending = true;
                }
                break;
            case REQ_SET_LEVELLER_SPEED:
                if (vendor_last_wLength >= 1) {
                    uint8_t s = vendor_rx_buf[0];
                    if (s < LEVELLER_SPEED_COUNT) {
                        leveller_config.speed = s;
                        leveller_update_pending = true;
                    }
                }
                break;
            case REQ_SET_LEVELLER_MAX_GAIN:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v < LEVELLER_MAX_GAIN_MIN) v = LEVELLER_MAX_GAIN_MIN;
                    if (v > LEVELLER_MAX_GAIN_MAX) v = LEVELLER_MAX_GAIN_MAX;
                    leveller_config.max_gain_db = v;
                    leveller_update_pending = true;
                }
                break;
            case REQ_SET_LEVELLER_LOOKAHEAD:
                if (vendor_last_wLength >= 1) {
                    leveller_config.lookahead = (vendor_rx_buf[0] != 0);
                    leveller_update_pending = true;
                    leveller_reset_pending  = true;
                }
                break;
            case REQ_SET_LEVELLER_GATE:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v < LEVELLER_GATE_MIN) v = LEVELLER_GATE_MIN;
                    if (v > LEVELLER_GATE_MAX) v = LEVELLER_GATE_MAX;
                    leveller_config.gate_threshold_db = v;
                    leveller_update_pending = true;
                }
                break;

            case REQ_SET_CHANNEL_NAME: {
                uint8_t ch = vendor_last_wValue & 0xFF;
                if (ch < NUM_CHANNELS && vendor_last_wLength > 0) {
                    memset(channel_names[ch], 0, PRESET_NAME_LEN);
                    size_t copy_len = vendor_last_wLength < (PRESET_NAME_LEN - 1)
                                    ? vendor_last_wLength : (PRESET_NAME_LEN - 1);
                    memcpy(channel_names[ch], vendor_rx_buf, copy_len);
                }
                break;
            }
            case REQ_SET_OUTPUT_TYPE: {
                uint8_t slot     =  vendor_last_wValue       & 0xFF;
                uint8_t new_type = (vendor_last_wValue >> 8) & 0xFF;
                if (slot < NUM_SPDIF_INSTANCES && new_type <= 1) {
                    output_types[slot] = new_type;
                }
                break;
            }

            /* ---- I2S clock + pin config ---- */
            case REQ_SET_I2S_BCK_PIN:
                if (vendor_last_wLength >= 1) i2s_bck_pin = vendor_rx_buf[0];
                break;
            case REQ_SET_MCK_ENABLE:
                if (vendor_last_wLength >= 1) i2s_mck_enabled = (vendor_rx_buf[0] != 0);
                break;
            case REQ_SET_MCK_PIN:
                if (vendor_last_wLength >= 1) i2s_mck_pin = vendor_rx_buf[0];
                break;
            case REQ_SET_MCK_MULTIPLIER:
                if (vendor_last_wLength >= 2)
                    memcpy((void *)&i2s_mck_multiplier, vendor_rx_buf, 2);
                break;

            case REQ_SET_OUTPUT_PIN: {
                uint8_t out = vendor_last_wValue & 0xFF;
                if (out < NUM_PIN_OUTPUTS && vendor_last_wLength >= 1)
                    output_pins[out] = vendor_rx_buf[0];
                break;
            }

            /* ---- Preset SETs are silent no-ops in M7d (no flash storage)
             * — Console UI will think the save succeeded but nothing
             * persists. M11 wires real preset storage on the W25Q64. */
            case REQ_PRESET_SAVE:
            case REQ_PRESET_LOAD:
            case REQ_PRESET_DELETE:
            case REQ_PRESET_SET_NAME:
            case REQ_PRESET_SET_STARTUP:
            case REQ_PRESET_SET_INCLUDE_PINS:
                break;

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
