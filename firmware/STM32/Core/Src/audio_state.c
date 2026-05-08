/**
 * audio_state.c — DSP-pipeline globals for the STM32 build (M7b)
 *
 * The DSP source files imported from firmware/DSPi/ (bulk_params.c,
 * notify.c, crossfeed.c, leveller.c, loudness.c) reference a long list
 * of `extern volatile` globals that the original RP project defines in
 * usb_audio.c. Our STM32 usb_audio.c is much smaller and doesn't carry
 * those state fields yet, so we define them here in one place — all
 * zero-initialised. Subsequent milestones (M7c+) wire the real DSP code
 * to these symbols.
 *
 * Until then, REQ_GET_ALL_PARAMS reads them out as zeros / defaults
 * and DSPi Console populates its UI with a "freshly powered" device.
 */

#include "config.h"
#include "usb_audio.h"
#include "crossfeed.h"
#include "leveller.h"
#include "loudness.h"
#include "audio_input.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>   /* snprintf for default channel names */

/* -------- Per-input-channel preamp -------- */
volatile float    global_preamp_db    [NUM_INPUT_CHANNELS] = { 0 };
volatile int32_t  global_preamp_mul   [NUM_INPUT_CHANNELS] = { 0 };
volatile float    global_preamp_linear[NUM_INPUT_CHANNELS] = { 1.0f, 1.0f };

/* -------- Master volume (post-output gain ceiling) -------- */
volatile float    master_volume_db     = 0.0f;
volatile float    master_volume_linear = 1.0f;
volatile int32_t  master_volume_q15    = (1 << 15);

/* -------- Per-output channel-group gain / mute (3 = L/R/sub or similar
 *          group on RP; preserved verbatim for wire compatibility) -------- */
volatile float    channel_gain_db    [3] = { 0 };
volatile int32_t  channel_gain_mul   [3] = { 0 };
volatile float    channel_gain_linear[3] = { 1.0f, 1.0f, 1.0f };
volatile bool     channel_mute       [3] = { false };

/* -------- Loudness compensation -------- */
volatile bool   loudness_enabled            = false;
volatile float  loudness_ref_spl            = 83.0f;
volatile float  loudness_intensity_pct      = 100.0f;
volatile bool   loudness_recompute_pending  = false;

/* -------- Crossfeed -------- */
volatile CrossfeedConfig crossfeed_config       = { 0 };
volatile bool            crossfeed_update_pending = false;

/* -------- Leveller --------
 * Defaults: enabled=false (must be turned on by Console), but
 * amount/max_gain/gate seeded so that flipping enable produces an
 * audible effect immediately without further tweaking. */
volatile LevellerConfig leveller_config = {
    .enabled           = false,
    .amount            = 100.0f,    /* full upward compression ratio (20:1) */
    .speed             = 1,         /* medium attack/release */
    .max_gain_db       = 15.0f,     /* +15 dB ceiling on upward gain */
    .lookahead         = false,
    .gate_threshold_db = -96.0f,    /* effectively no gate */
};
volatile bool           leveller_update_pending = false;
volatile bool           leveller_reset_pending  = false;

/* -------- Matrix mixer + per-output state -------- */
MatrixMixer matrix_mixer;
uint8_t output_pins[NUM_PIN_OUTPUTS];

/* M7c: stereo pass-through defaults (mirrors RP usb_audio.c::
 * matrix_init_defaults). Without this Console sees zero enabled
 * outputs and refuses to render any channel rows in its UI. */
void matrix_init_defaults(void) {
    memset(&matrix_mixer, 0, sizeof(matrix_mixer));

    /* L → Out1, R → Out2 — first SPDIF stereo pair (Out 0-1).
     * NOTE: on this STM32 build the only active physical output is
     * SAI1 (PE6/PE3); the SPDIF/PDM channels are placeholders for
     * wire-format compatibility with DSPi Console, not real outputs
     * yet. The crosspoint config still drives the matrix-mixer DSP
     * stage so Console reads back the expected default routing. */
    matrix_mixer.crosspoints[0][0].enabled     = 1;
    matrix_mixer.crosspoints[0][0].gain_db     = 0.0f;
    matrix_mixer.crosspoints[0][0].gain_linear = 1.0f;
    matrix_mixer.crosspoints[1][1].enabled     = 1;
    matrix_mixer.crosspoints[1][1].gain_db     = 0.0f;
    matrix_mixer.crosspoints[1][1].gain_linear = 1.0f;

    matrix_mixer.outputs[0].enabled     = 1;
    matrix_mixer.outputs[0].gain_linear = 1.0f;
    matrix_mixer.outputs[1].enabled     = 1;
    matrix_mixer.outputs[1].gain_linear = 1.0f;
    for (int o = 2; o < NUM_OUTPUT_CHANNELS; ++o) {
        matrix_mixer.outputs[o].enabled     = 0;
        matrix_mixer.outputs[o].gain_linear = 1.0f;
    }
}

/* Forward decl — defined further down. Needed here because
 * init_default_channel_names() consults it for SPDIF/I2S labels. */
extern uint8_t output_types[NUM_SPDIF_INSTANCES];

/* -------- Channel names (per-preset, user-editable) --------
 * Zero-init at boot; init_default_channel_names() populates with the
 * canonical "USB L/R" / "SPDIF n L/R" / "PDM" labels Console uses when
 * no preset overrides them. Mirrors the RP get_default_channel_name
 * scheme exactly so a preset .json snapshot from one platform reads
 * naturally on the other. */
char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];

void get_default_channel_name(int ch, uint8_t input_source,
                              const uint8_t *out_types, char *buf) {
    memset(buf, 0, PRESET_NAME_LEN);
    if (ch < 0 || ch >= NUM_CHANNELS) return;

    if (ch < NUM_INPUT_CHANNELS) {
        const char *prefix;
        switch (input_source) {
            case INPUT_SOURCE_SPDIF: prefix = "SPDIF"; break;
            case INPUT_SOURCE_USB:
            default:                 prefix = "USB";   break;
        }
        snprintf(buf, PRESET_NAME_LEN, "%s %c", prefix, (ch == 0) ? 'L' : 'R');
        return;
    }

    if (ch == NUM_CHANNELS - 1) {
        strncpy(buf, "PDM", PRESET_NAME_LEN - 1);
        return;
    }

    int slot_idx = (ch - NUM_INPUT_CHANNELS) / 2;
    int side     = (ch - NUM_INPUT_CHANNELS) % 2;
    uint8_t type = (out_types && slot_idx < NUM_SPDIF_INSTANCES)
                       ? out_types[slot_idx]
                       : OUTPUT_TYPE_SPDIF;
    const char *prefix = (type == OUTPUT_TYPE_I2S) ? "I2S" : "SPDIF";
    snprintf(buf, PRESET_NAME_LEN, "%s %d %c",
             prefix, slot_idx + 1, (side == 0) ? 'L' : 'R');
}

void init_default_channel_names(void) {
    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        get_default_channel_name(ch,
                                 INPUT_SOURCE_USB,
                                 output_types,
                                 channel_names[ch]);
    }
}

/* (filter_recipes[][], channel_delays_ms[], filters[][], delay_lines[][],
 *  channel_bypassed[], delay_write_idx, channel_delay_samples[],
 *  any_delay_active are all defined in dsp_pipeline.c — imported in M7c.) */

/* -------- Output type config (SPDIF / I2S / PDM per output) --------
 * NUM_SPDIF_INSTANCES on RP2350 = 4. Output[0] = SPDIF by default;
 * any output can be flipped to I2S via vendor command. PDM is the
 * fixed extra slot. */
uint8_t output_types[NUM_SPDIF_INSTANCES] = { 0 };

/* -------- I2S output pin / clock config --------
 * Stub for wire compatibility; not active in M7b — the SAI1 audio
 * path uses fixed pins (PE2/4/5/6, see audio_out.c). */
uint8_t  i2s_bck_pin        = 0;
uint8_t  i2s_mck_pin        = 0;
bool     i2s_mck_enabled    = false;
uint16_t i2s_mck_multiplier = 128;

/* -------- Audio state (top-level UI mirror) -------- */
volatile AudioState audio_state = { .freq = 48000 };

/* -------- System status packet (REQ_GET_STATUS combined response) --------
 * Console polls this for peak meters, CPU load, clip flags. M7d ships
 * zeros — real metering hooks in M8+ once we have the buffer-watermark
 * + clip-detect plumbing. */
volatile SystemStatusPacket global_status = { 0 };

/* -------- Other globals the DSP files reach for -------- */
volatile bool bypass_master_eq = false;

/* SPDIF RX pin-change pending — referenced by imported bulk_params.c when
 * Console SETs the SPDIF input pin via REQ_SET_ALL_PARAMS. STM32 has no
 * SPDIF RX yet (deferred per project plan); the flag is harmless and
 * read by nothing in this build. Will become live in M9 SPDIF RX. */
volatile bool spdif_rx_pin_change_pending = false;

/* -------- Stub: master volume update --------
 * Wired to the real SAI output-gain stage in M7c. For M7b we just clamp,
 * store, and accept the value so bulk_params SETs don't fail. The
 * `linear` / `q15` derivatives are computed for wire-format consumers
 * but the audio path doesn't apply them yet. */
#include <math.h>
void update_master_volume(float db) {
    /* DSPi convention: 0 dB = unity, -127 dB ≈ silent, -128 dB = mute */
    if (db < -128.0f) db = -128.0f;
    if (db >    0.0f) db =    0.0f;
    master_volume_db = db;
    if (db <= -128.0f) {
        master_volume_linear = 0.0f;
        master_volume_q15    = 0;
    } else {
        float lin = powf(10.0f, db * 0.05f);
        master_volume_linear = lin;
        master_volume_q15    = (int32_t)(lin * 32768.0f + 0.5f);
    }
}
