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

#include "stm32h7xx_hal.h"   /* HAL_RCC_GetSysClockFreq for REQ_GET_STATUS w=13 */
#include "tusb.h"
#include "config.h"
#include "vendor_commands.h"
#include "bulk_params.h"
#include "dsp_pipeline.h"

#include "usb_audio.h"   /* update_master_volume, AudioState, channel_names */
#include "audio_input.h" /* INPUT_SOURCE_USB / SPDIF + change-pending flag */
#include "spdif_input.h" /* M9: SpdifRxStatusPacket + status accessor */
#include "notify.h"      /* notify_param_write — M7j */
#include "flash_storage.h"  /* preset_save / load / delete / etc. — M11 */
#include "w25q.h"           /* debug GETs probe W25Q directly */
#include <stddef.h>      /* offsetof */

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
                    /* M7g — system stats Console polls per-second:
                     *   13: SYSCLK (Hz) — 550 MHz on STM32H723 at VOS0
                     *   14: Vdda  (mV) — back-computed from VREFINT
                     *   16: Tj    (centi-°C) — internal temp sensor
                     */
                    if (req->wValue == 13) {
                        static uint32_t hz;
                        hz = HAL_RCC_GetSysClockFreq();
                        return tud_control_xfer(rhport,
                                                (tusb_control_request_t *)req,
                                                &hz, 4);
                    }
                    /* wValue=14 (Vdda mV) intentionally falls through to
                     * STALL — Console treats a failed request as "feature
                     * not supported" and hides the field. STM32H7's Vdda is
                     * pin-bonded to 3.3 V on this board, so a back-computed
                     * value adds noise without information. */
                    if (req->wValue == 16) {
                        extern int16_t read_temperature_cdeg(void);
                        static int32_t cdeg;
                        cdeg = (int32_t)read_temperature_cdeg();
                        return tud_control_xfer(rhport,
                                                (tusb_control_request_t *)req,
                                                &cdeg, 4);
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
                    static uint8_t src;
                    src = active_input_source;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &src, 1);
                }
                /* REQ_SET_INPUT_SOURCE — NOT handled here. Despite the
                 * name and our REQ_SET_OUTPUT_TYPE precedent (which IS
                 * a side-effecting vendor IN), this one is a plain
                 * vendor OUT with a 1-byte DATA payload per the RP
                 * convention. See the SET/DATA-stage handler below. */

                case REQ_GET_SPDIF_RX_STATUS: {
                    static SpdifRxStatusPacket pkt;
                    spdif_input_get_status(&pkt);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &pkt, sizeof(pkt));
                }
                case REQ_GET_SPDIF_RX_CH_STATUS: {
                    static uint8_t cs[24];
                    spdif_input_get_channel_status(cs);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            cs, sizeof(cs));
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
                case 0xFA: {  /* DEBUG (M12 P3): per-slot SAI CR1.PRTCFG.
                               * Returns 4 bytes: byte N = PRTCFG bits[3:2]
                               * for slot N's SAI sub-block. 0=I2S/free,
                               * 1=SPDIF. Verify reboot-after-SET actually
                               * landed in SPDIF mode without a scope. */
                    static uint8_t buf[4];
                    buf[0] = (uint8_t)((SAI1_Block_A->CR1 >> 2) & 0x3);
                    buf[1] = (uint8_t)((SAI1_Block_B->CR1 >> 2) & 0x3);
                    buf[2] = (uint8_t)((SAI4_Block_A->CR1 >> 2) & 0x3);
                    buf[3] = (uint8_t)((SAI4_Block_B->CR1 >> 2) & 0x3);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xF2: {  /* DEBUG (M9): runtime switch SPDIFRX INSEL
                               * field. wValue lo byte = new INSEL value
                               * (0..3). Restarts the SYNC state machine
                               * on the new input. Returns the new CR
                               * value (4 bytes) so caller can confirm. */
                    uint32_t insel = req->wValue & 0x3;
                    /* Disable peripheral, change INSEL, re-enable SYNC. */
                    SPDIFRX->CR &= ~SPDIFRX_CR_SPDIFEN;
                    while (SPDIFRX->CR & SPDIFRX_CR_SPDIFEN) ;
                    SPDIFRX->CR = (SPDIFRX->CR & ~SPDIFRX_CR_INSEL)
                                | (insel << 16);
                    SPDIFRX->CR |= 1;   /* SPDIFEN[1:0] = 01 (SYNC) */
                    static uint32_t resp;
                    resp = SPDIFRX->CR;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &resp, 4);
                }
                case 0xF4: {  /* DEBUG (M9): SPDIFRX register snapshot.
                               * 32-byte payload, u32 LE: */
                    static uint32_t buf[8];
                    buf[0] = SPDIFRX->CR;
                    buf[1] = SPDIFRX->SR;
                    buf[2] = SPDIFRX->IMR;
                    buf[3] = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPDIFRX);
                    buf[4] = GPIOD->AFR[1];       /* PD8 AF nibble bits[3:0] of AFR[1] */
                    buf[5] = GPIOD->MODER;        /* PD8 mode bits[17:16] */
                    buf[6] = GPIOD->PUPDR;        /* PD8 pull bits[17:16] */
                    buf[7] = (uint32_t)HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)
                           | ((uint32_t)(GPIOD->IDR & GPIO_PIN_8) << 8);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xF9: {  /* DEBUG (M9): freeze/thaw the FRACN servo.
                               * wValue=1 → freeze (servo stops writing
                               * to PLL2FRACR, accumulator still tracks);
                               * wValue=0 → thaw. Returns current state.
                               * Used to A/B test whether FRACN write
                               * glitches are causing audible DAC dropouts. */
                    extern void spdif_input_set_servo_frozen(uint8_t f);
                    extern uint8_t spdif_input_get_servo_frozen(void);
                    spdif_input_set_servo_frozen(req->wValue ? 1 : 0);
                    static uint8_t st;
                    st = spdif_input_get_servo_frozen();
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &st, 1);
                }
                case 0xED: {  /* DEBUG (M9): SPDIF source-switch start state.
                               * Snapshot of conditions just before the
                               * last HAL_SPDIFRX_ReceiveDataFlow_DMA call. */
                    extern volatile uint32_t spdif_start_call_count;
                    extern volatile uint32_t spdif_start_dt_rc;
                    extern volatile uint32_t spdif_start_cs_rc;
                    extern volatile uint32_t spdif_start_pre_hspdif_state;
                    extern volatile uint32_t spdif_start_dt_dma_state;
                    extern volatile uint32_t spdif_start_pre_cr;
                    static uint32_t buf[6];
                    buf[0] = spdif_start_call_count;
                    buf[1] = spdif_start_dt_rc;
                    buf[2] = spdif_start_cs_rc;
                    buf[3] = spdif_start_pre_hspdif_state;
                    buf[4] = spdif_start_dt_dma_state;
                    buf[5] = spdif_start_pre_cr;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xEC: {  /* DEBUG (M9): per-stage cycle counters from
                               * fill_half. Returns 9 u32: count of
                               * fill_half calls since last read, then 8
                               * stage cycle accumulators. Snapshot-and-
                               * reset semantics so each probe shows the
                               * window since the previous probe. */
                    extern volatile uint32_t audio_stage_cycles[8];
                    extern volatile uint32_t audio_stage_calls;
                    static uint32_t buf[9];
                    buf[0] = audio_stage_calls;
                    audio_stage_calls = 0;
                    for (int i = 0; i < 8; i++) {
                        buf[1 + i] = audio_stage_cycles[i];
                        audio_stage_cycles[i] = 0;
                    }
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xEB: {  /* DEBUG (M9): read FPSCR. Bit 24 = FZ (Flush-to-Zero).
                               * If FZ is 0, denormals are slow-path emulated. */
                    uint32_t fpscr = __get_FPSCR();
                    static uint32_t buf[2];
                    buf[0] = fpscr;
                    buf[1] = (fpscr >> 24) & 1;  /* FZ bit isolated */
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xEA: {  /* DEBUG (M9): dump dither buffers + TIM7 +
                               * DMA1 Stream 4/5 status + DMA error
                               * flags to see why the CFGR DMA isn't
                               * toggling FRACEN. */
                    static uint32_t buf[16];
                    volatile uint32_t *cfgr_b  = (uint32_t*)0x24016100UL;
                    buf[0] = cfgr_b[0];      /* should have FRACEN=0  */
                    buf[1] = cfgr_b[1];      /* should have FRACEN=1  */
                    buf[2] = DMA1_Stream4->CR;
                    buf[3] = DMA1_Stream4->NDTR;
                    buf[4] = DMA1_Stream5->CR;
                    buf[5] = DMA1_Stream5->NDTR;
                    buf[6] = DMA1->LISR;          /* err flags streams 0-3 */
                    buf[7] = DMA1->HISR;          /* err flags streams 4-7 */
                    /* DMAMUX1 channel CCR for stream 4 + 5. Channel index
                     * = stream index since DMA1 is the first 8 channels. */
                    buf[8] = DMAMUX1_Channel4->CCR;
                    buf[9] = DMAMUX1_Channel5->CCR;
                    buf[10] = (uint32_t)DMA1_Stream5->M0AR;
                    buf[11] = (uint32_t)DMA1_Stream5->PAR;
                    buf[12] = RCC->PLLCFGR;
                    buf[13] = TIM7->CNT;
                    buf[14] = TIM7->DIER;
                    buf[15] = 0xdeadbeef;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xF8: {  /* DEBUG (M9): snapshot 16 consecutive raw
                               * SPDIFRX_DR words from the DMA buffer so
                               * the host can decode PT / V / PE / audio
                               * bit positions and confirm the demux is
                               * extracting the right fields. Reads
                               * directly from the AXI-SRAM DMA ring at
                               * 0x24002000 — that's where the SPDIFRX
                               * DMA dumps subframe words. */
                    static uint32_t snap[16];
                    volatile uint32_t *src = (volatile uint32_t *)0x24002000UL;
                    /* Grab the 16 words around the middle of the DMA
                     * buffer — should be settled past any half-cplt
                     * boundary effects. Volatile copy so the compiler
                     * doesn't reorder vs the DMA writes. */
                    for (int i = 0; i < 16; i++) snap[i] = src[i + 256];
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            snap, sizeof(snap));
                }
                case 0xF7: {  /* DEBUG (M9): servo state snapshot — fill,
                               * err, integral accumulator, current
                               * FRACN, and pll2_fracn_write call count.
                               * Use to verify the FRACN servo is
                               * actually moving the PLL during LOCKED. */
                    static SpdifServoDebugPacket pkt;
                    spdif_input_get_servo_debug(&pkt);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &pkt, sizeof(pkt));
                }
                case 0xF6: {  /* DEBUG (M9): PD8 as plain INPUT + internal
                               * pull-up, AF routing disconnected. Used
                               * during M9 bring-up to isolate whether
                               * the active AF (originally AF8, since
                               * fixed to AF9) was routing PD8 to a
                               * conflicting peripheral that drove the
                               * line low. With AF disconnected the pin
                               * goes high (pull-up wins), proving the
                               * AF mux was the culprit, not the
                               * SPDIFRX block itself.
                               *
                               * Also disables SPDIFEN to take the
                               * SPDIFRX peripheral cleanly out of the
                               * picture for the test. */
                    __HAL_RCC_GPIOD_CLK_ENABLE();
                    /* Force SPDIFRX peripheral OFF — SPDIFEN field to 00 */
                    SPDIFRX->CR &= ~(0x3u << 0);
                    /* MODER bits[17:16] = 00 (INPUT) */
                    GPIOD->MODER &= ~(0x3u << 16);
                    /* OTYPER bit 8: don't care in input mode */
                    /* PUPDR bits[17:16] = 01 (pull-up) */
                    GPIOD->PUPDR  &= ~(0x3u << 16);
                    GPIOD->PUPDR  |=  (0x1u << 16);
                    /* AFRH nibble[3:0] = 0 */
                    GPIOD->AFR[1] &= ~0xFu;

                    static uint32_t buf[5];
                    buf[0] = GPIOD->MODER;
                    buf[1] = GPIOD->PUPDR;
                    buf[2] = GPIOD->AFR[1];
                    buf[3] = GPIOD->IDR;
                    buf[4] = SPDIFRX->CR;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xF5: {  /* DEBUG (M9): drive PD8 as a plain GPIO
                               * push-pull output HIGH, bypassing the AF
                               * routing (clears AFR nibble). Used during
                               * M9 bring-up to prove the pad itself was
                               * healthy when AF routing was suspect.
                               * Returns the resulting MODER, PUPDR, ODR
                               * and live IDR so the host can confirm the
                               * config actually landed. If the pin still
                               * reads 0 V externally with this command
                               * issued, the silicon is damaged. */
                    __HAL_RCC_GPIOD_CLK_ENABLE();
                    /* MODER bits[17:16] for PD8 — clear then set to 01 (output) */
                    GPIOD->MODER &= ~(0x3u << 16);
                    GPIOD->MODER |=  (0x1u << 16);
                    /* OTYPER bit 8 = 0 (push-pull) */
                    GPIOD->OTYPER &= ~(1u << 8);
                    /* OSPEEDR bits[17:16] = 11 (very high) */
                    GPIOD->OSPEEDR |= (0x3u << 16);
                    /* PUPDR bits[17:16] = 00 (no pull — output drives it) */
                    GPIOD->PUPDR  &= ~(0x3u << 16);
                    /* AFRH nibble [3:0] = 0 — irrelevant in OUTPUT mode but
                     * tidy. */
                    GPIOD->AFR[1] &= ~0xFu;
                    /* Drive HIGH via BSRR */
                    GPIOD->BSRR = (1u << 8);

                    static uint32_t buf[4];
                    buf[0] = GPIOD->MODER;
                    buf[1] = GPIOD->PUPDR;
                    buf[2] = GPIOD->ODR;
                    buf[3] = GPIOD->IDR;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xF3: {  /* DEBUG (M9): RCC + PLL2 + SPDIFRX clock-tree
                               * snapshot — figure out where the kernel
                               * clock got lost. Extended for M9-dither:
                               * also captures live TIM7 state and the
                               * DMA1 Stream 4/5 status so we can verify
                               * the dither engine is alive. 64-byte
                               * payload, u32 LE. */
                    static uint32_t buf[16];
                    buf[0]  = RCC->CR;        /* PLL2RDY bit 27          */
                    buf[1]  = RCC->PLLCFGR;
                    buf[2]  = RCC->PLL2DIVR;
                    buf[3]  = RCC->PLL2FRACR; /* live FRACN snapshot     */
                    buf[4]  = RCC->D2CCIP1R;
                    buf[5]  = RCC->APB1LENR;
                    buf[6]  = RCC->CFGR;
                    buf[7]  = TIM7->CNT;      /* live counter            */
                    buf[8]  = TIM7->ARR;      /* period (= kclk/100k - 1)*/
                    buf[9]  = TIM7->CR1;      /* CEN bit 0 must be set   */
                    buf[10] = TIM7->DIER;     /* UDE bit 8 must be set   */
                    buf[11] = DMA1_Stream4->CR;   /* EN bit 0 must be set */
                    buf[12] = DMA1_Stream4->NDTR; /* remaining transfers */
                    buf[13] = DMA1_Stream5->CR;
                    buf[14] = DMA1_Stream5->NDTR;
                    buf[15] = RCC->AHB1ENR;       /* TIM7EN visible here */
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xFD: {  /* DEBUG (M11): W25Q64 JEDEC ID + presence flag */
                    extern uint8_t w25q_jedec_id[3];
                    extern bool    w25q_present;
                    static uint8_t buf[4];
                    buf[0] = w25q_jedec_id[0];
                    buf[1] = w25q_jedec_id[1];
                    buf[2] = w25q_jedec_id[2];
                    buf[3] = w25q_present ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xFB: {  /* DEBUG (M11): last preset_* return code +
                               * also dump first 16 bytes of W25Q slot 0 so
                               * we can see whether the on-chip data was
                               * actually written. */
                    extern volatile uint8_t last_preset_result;
                    static uint8_t buf[20];
                    buf[0] = last_preset_result;
                    buf[1] = 0; buf[2] = 0; buf[3] = 0;
                    /* Read W25Q at offset 4096 (SLOT 0) directly — bypass
                     * the mirror so we can see what physically landed. */
                    w25q_read(4096, &buf[4], 16);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            buf, sizeof(buf));
                }
                case 0xFC: {  /* DEBUG (M11): W25Q sector RW round-trip test.
                               * Erases sector at byte offset 0, programs an
                               * 8-byte signature, reads back, returns the 8
                               * bytes. A successful round-trip returns:
                               *   D5 9P 1D 5C 0D ED B0 0B
                               * Anything else = SPI link broken.            */
                    #include "w25q.h"
                    static const uint8_t sig[8] = {
                        0xD5,0x9F,0x1D,0x5C,0x0D,0xED,0xB0,0x0B
                    };
                    static uint8_t out[8];
                    w25q_sector_erase_4k(0);
                    w25q_page_program(0, sig, sizeof(sig));
                    w25q_read(0, out, sizeof(out));
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            out, sizeof(out));
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
                case REQ_SET_OUTPUT_TYPE: {
                    /* Console sends this as a vendor IN with side effects:
                     *   wValue = (new_type << 8) | slot_index
                     *   wLength = 1 (status response)
                     * Mirror the RP convention so Console's
                     * "set_output_type → poll-for-status" flow works.
                     * Status returned is 0 (success) on valid input,
                     * 0xFF (generic fail) for any rejection.
                     *
                     * Phase 4: when the type actually changes, raise a
                     * flag that the main loop drains via Audio_HotSwap.
                     * The actual SAI tear-down + re-init can't run from
                     * the USB ISR (HAL_SAI_DeInit touches RCC under
                     * locks), so the swap happens in main-loop context. */
                    uint8_t slot     =  req->wValue       & 0xFF;
                    uint8_t new_type = (req->wValue >> 8) & 0xFF;
                    static uint8_t status;
                    if (slot < NUM_SPDIF_INSTANCES && new_type <= 1) {
                        if (output_types[slot] != new_type) {
                            output_types[slot] = new_type;
                            extern volatile bool output_type_change_pending;
                            output_type_change_pending = true;
                            notify_param_write(
                                (uint16_t)(offsetof(WireBulkParams,
                                                    i2s_config.output_types)
                                           + slot),
                                1, &new_type);
                        }
                        status = 0;
                    } else {
                        status = 0xFF;
                    }
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &status, 1);
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
                    /* M7g — 16-byte ASCII hex serial derived from the H7's
                     * 96-bit unique device ID at 0x1FF1E800. We encode the
                     * low 64 bits (8 bytes) as 16 hex chars — uniqueness is
                     * already guaranteed by the factory programming. The
                     * full 96-bit ID would need 24 chars, more than the
                     * Console's 16-byte buffer; the upper 32 bits are
                     * mostly wafer/lot info and rarely change between
                     * adjacent dies, so dropping them costs ~no entropy in
                     * practice. */
                    static uint8_t serial[16];
                    static int     serial_built = 0;
                    if (!serial_built) {
                        const uint32_t *uid = (const uint32_t *)0x1FF1E800UL;
                        uint64_t lo64 = ((uint64_t)uid[1] << 32) | uid[0];
                        static const char hex[] = "0123456789ABCDEF";
                        for (int i = 0; i < 16; ++i) {
                            int shift = (15 - i) * 4;
                            serial[i] = hex[(lo64 >> shift) & 0xF];
                        }
                        serial_built = 1;
                    }
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
                    static uint8_t v;
                    uint16_t occ; uint8_t m, d, la, inc_pins;
                    preset_get_directory(&occ, &m, &d, &la, &inc_pins, &v);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_GET_SAVED_MASTER_VOLUME: {
                    /* Directory's independent master volume (mode-0 boot value). */
                    static float v;
                    v = preset_get_saved_master_volume();
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 4);
                }
                case REQ_SAVE_MASTER_VOLUME: {
                    /* Action command (mirrors REQ_PRESET_SAVE): persist the live
                     * master volume into the directory's independent field and
                     * ship the 1-byte status in this transfer's data stage.
                     * Inline flash write — same convention as REQ_PRESET_SAVE. */
                    static uint8_t status;
                    status = preset_save_master_volume();
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &status, 1);
                }

                /* ---- SPDIF RX pin (fixed PD8 / WeAct P1 pin 40 on STM32) ---- */
                case REQ_GET_SPDIF_RX_PIN: {
                    static uint8_t v;
                    v = spdif_rx_pin;
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
                    /* M11: pull live directory from flash_storage.c.
                     * Layout (7 bytes):
                     *   [0..1] slot_occupied (u16, bit n = slot n filled)
                     *   [2]    startup_mode  (0 = specified, 1 = last_active)
                     *   [3]    default_slot  (0..9, applies when mode=0)
                     *   [4]    last_active   (last loaded/saved slot)
                     *   [5]    include_pins  (0/1)
                     *   [6]    master_volume_mode (0/1) */
                    static uint8_t dir[7];
                    uint16_t occ; uint8_t m, d, la, inc_pins, mv_mode;
                    preset_get_directory(&occ, &m, &d, &la, &inc_pins, &mv_mode);
                    dir[0] = (uint8_t)(occ & 0xFF);
                    dir[1] = (uint8_t)(occ >> 8);
                    dir[2] = m;
                    dir[3] = d;
                    dir[4] = la;
                    dir[5] = inc_pins;
                    dir[6] = mv_mode;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            dir, sizeof(dir));
                }
                case REQ_PRESET_GET_ACTIVE: {
                    static uint8_t v;
                    v = preset_get_active();
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_PRESET_GET_STARTUP: {
                    /* 3 bytes: startup_mode, default_slot, include_pins. */
                    static uint8_t v[3];
                    uint16_t occ; uint8_t la, mv_mode;
                    preset_get_directory(&occ, &v[0], &v[1], &la, &v[2], &mv_mode);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            v, sizeof(v));
                }
                case REQ_PRESET_GET_INCLUDE_PINS: {
                    static uint8_t v;
                    uint16_t occ; uint8_t m, d, la, mv_mode;
                    preset_get_directory(&occ, &m, &d, &la, &v, &mv_mode);
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req, &v, 1);
                }
                case REQ_PRESET_GET_NAME: {
                    /* wValue = slot index (0..9). Returns 32-byte name. */
                    static char nm[PRESET_NAME_LEN];
                    uint8_t slot = req->wValue & 0xFF;
                    if (preset_get_name(slot, nm) != 0) {
                        memset(nm, 0, sizeof(nm));
                    }
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            nm, sizeof(nm));
                }

                /* Console wraps SAVE / LOAD / DELETE as GETs that return a
                 * 1-byte status (PRESET_OK or PRESET_ERR_*). The device
                 * performs the action synchronously and ships the result
                 * back in the data stage of the same control transfer. */
                case REQ_PRESET_SAVE: {
                    extern volatile uint8_t last_preset_result;
                    static uint8_t status;
                    status = preset_save(req->wValue & 0xFF);
                    last_preset_result = status;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &status, 1);
                }
                case REQ_PRESET_LOAD: {
                    extern volatile uint8_t last_preset_result;
                    static uint8_t status;
                    status = preset_load(req->wValue & 0xFF);
                    last_preset_result = status;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &status, 1);
                }
                case REQ_PRESET_DELETE: {
                    extern volatile uint8_t last_preset_result;
                    static uint8_t status;
                    status = preset_delete(req->wValue & 0xFF);
                    last_preset_result = status;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &status, 1);
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
                case REQ_GET_BAND_BYPASS: {
                    /* wValue = (channel<<8) | band — payload 1 byte. */
                    uint8_t ch   = (req->wValue >> 8) & 0xFF;
                    uint8_t band =  req->wValue       & 0xFF;
                    if (ch >= NUM_CHANNELS
                        || band >= channel_band_counts[ch]) return false;
                    static uint8_t v;
                    v = (filter_recipes[ch][band].bypass == 1) ? 1 : 0;
                    return tud_control_xfer(rhport,
                                            (tusb_control_request_t *)req,
                                            &v, 1);
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
            /* Zero-length SET — slot/index lives in wValue, no payload.
             * Dispatch action here (no DATA stage will fire), then ack. */
            extern volatile uint8_t last_preset_result;
            uint8_t slot = req->wValue & 0xFF;
            switch (req->bRequest) {
                case REQ_PRESET_SAVE:
                    last_preset_result = preset_save(slot);
                    break;
                case REQ_PRESET_LOAD:
                    last_preset_result = preset_load(slot);
                    break;
                case REQ_PRESET_DELETE:
                    last_preset_result = preset_delete(slot);
                    break;
                case REQ_PRESET_SET_STARTUP: {
                    uint8_t mode = (req->wValue >> 8) & 0xFF;
                    uint8_t dflt =  req->wValue       & 0xFF;
                    last_preset_result = preset_set_startup(mode, dflt);
                    break;
                }
                default: break;
            }
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
                    /* Push the entire WireBandParams (16 bytes) so Console
                     * sees the type/freq/Q/gain change in one event. */
                    WireBandParams wp = {
                        .type    = pkt.type,
                        .bypass  = pkt.bypass,
                        .freq    = pkt.freq,
                        .q       = pkt.Q,
                        .gain_db = pkt.gain_db,
                    };
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, eq)
                                   + (pkt.channel * WIRE_MAX_BANDS + pkt.band)
                                       * sizeof(WireBandParams)),
                        sizeof(WireBandParams), &wp);
                }
                break;
            }

            case REQ_SET_BAND_BYPASS: {
                /* Console toggles the per-band bypass via this dedicated
                 * opcode (NOT REQ_SET_EQ_PARAM). wValue = (ch<<8)|band,
                 * payload = 1 byte (1 = bypass, else active). Recompute
                 * coefficients so dsp_pipeline picks up the new bypass
                 * flag — without that the filters[][] entry's compiled
                 * state still applies the old (un-bypassed) curve. */
                if (vendor_last_wLength < 1) break;
                uint8_t ch   = (vendor_last_wValue >> 8) & 0xFF;
                uint8_t band =  vendor_last_wValue       & 0xFF;
                if (ch < NUM_CHANNELS && band < channel_band_counts[ch]) {
                    uint8_t v = (vendor_rx_buf[0] == 1) ? 1 : 0;
                    filter_recipes[ch][band].bypass = v;
                    dsp_compute_coefficients(&filter_recipes[ch][band],
                                             &filters[ch][band],
                                             (float)audio_state.freq);
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, eq)
                                   + (ch * WIRE_MAX_BANDS + band) * sizeof(WireBandParams)
                                   + offsetof(WireBandParams, bypass)),
                        1, &v);
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
                    WireCrosspoint wxp = {
                        .enabled      = pkt.enabled,
                        .phase_invert = pkt.phase_invert,
                        .gain_db      = pkt.gain_db,
                    };
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, crosspoints)
                                   + (pkt.input * WIRE_MAX_OUTPUT_CHANNELS + pkt.output)
                                       * sizeof(WireCrosspoint)),
                        sizeof(WireCrosspoint), &wxp);
                }
                break;
            }

            case REQ_SET_OUTPUT_ENABLE: {
                uint8_t out = vendor_last_wValue & 0xFF;
                if (out < NUM_OUTPUT_CHANNELS && vendor_last_wLength >= 1) {
                    uint8_t v = (vendor_rx_buf[0] != 0) ? 1 : 0;
                    matrix_mixer.outputs[out].enabled = v;
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, outputs)
                                   + out * sizeof(WireOutputChannel)
                                   + offsetof(WireOutputChannel, enabled)),
                        1, &v);
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
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, outputs)
                                   + out * sizeof(WireOutputChannel)
                                   + offsetof(WireOutputChannel, gain_db)),
                        sizeof(float), &db);
                }
                break;
            }

            case REQ_SET_OUTPUT_MUTE: {
                uint8_t out = vendor_last_wValue & 0xFF;
                if (out < NUM_OUTPUT_CHANNELS && vendor_last_wLength >= 1) {
                    uint8_t v = vendor_rx_buf[0] ? 1 : 0;
                    matrix_mixer.outputs[out].mute = v;
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, outputs)
                                   + out * sizeof(WireOutputChannel)
                                   + offsetof(WireOutputChannel, mute)),
                        1, &v);
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
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, outputs)
                                   + out * sizeof(WireOutputChannel)
                                   + offsetof(WireOutputChannel, delay_ms)),
                        sizeof(float), &ms);
                }
                break;
            }

            case REQ_SET_PREAMP: {
                /* Legacy: single-value SET applies to all input channels. */
                if (vendor_last_wLength < 4) break;
                float db; memcpy(&db, vendor_rx_buf, 4);
                for (uint8_t ch = 0; ch < NUM_INPUT_CHANNELS; ++ch) {
                    update_preamp_ch(ch, db);
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, preamp.preamp_db)
                                   + ch * sizeof(float)),
                        sizeof(float), &db);
                }
                break;
            }
            case REQ_SET_PREAMP_CH: {
                if (vendor_last_wLength < 4) break;
                uint8_t ch = vendor_last_wValue & 0xFF;
                if (ch >= NUM_INPUT_CHANNELS) break;
                float db; memcpy(&db, vendor_rx_buf, 4);
                update_preamp_ch(ch, db);
                notify_param_write(
                    (uint16_t)(offsetof(WireBulkParams, preamp.preamp_db)
                               + ch * sizeof(float)),
                    sizeof(float), &db);
                break;
            }

            case REQ_SET_MASTER_VOLUME: {
                if (vendor_last_wLength < 4) break;
                float db; memcpy(&db, vendor_rx_buf, 4);
                update_master_volume(db);
                notify_param_write(offsetof(WireBulkParams, master_volume.master_volume_db),
                                   sizeof(float), &db);
                break;
            }

            case REQ_SET_BYPASS:
                if (vendor_last_wLength >= 1) {
                    bypass_master_eq = (vendor_rx_buf[0] != 0);
                    uint8_t v = bypass_master_eq ? 1 : 0;
                    notify_param_write(offsetof(WireBulkParams, global.bypass), 1, &v);
                }
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
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, delays.delay_ms)
                                   + ch * sizeof(float)),
                        sizeof(float), &ms);
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
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, legacy.gain_db)
                                   + ch * sizeof(float)),
                        sizeof(float), &db);
                }
                break;
            }
            case REQ_SET_CHANNEL_MUTE: {
                uint8_t ch = vendor_last_wValue & 0xFF;
                if (ch < 3 && vendor_last_wLength >= 1) {
                    uint8_t v = (vendor_rx_buf[0] != 0) ? 1 : 0;
                    channel_mute[ch] = v;
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, legacy.mute) + ch),
                        1, &v);
                }
                break;
            }

            case REQ_SET_LOUDNESS:
                if (vendor_last_wLength >= 1) {
                    loudness_enabled = (vendor_rx_buf[0] != 0);
                    uint8_t v = loudness_enabled ? 1 : 0;
                    notify_param_write(offsetof(WireBulkParams, global.loudness_enabled),
                                       1, &v);
                }
                break;
            case REQ_SET_LOUDNESS_REF:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v <  40.0f) v =  40.0f;
                    if (v > 100.0f) v = 100.0f;
                    loudness_ref_spl = v;
                    loudness_recompute_pending = true;
                    notify_param_write(offsetof(WireBulkParams, global.loudness_ref_spl),
                                       sizeof(float), &v);
                }
                break;
            case REQ_SET_LOUDNESS_INTENSITY:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v <   0.0f) v =   0.0f;
                    if (v > 200.0f) v = 200.0f;
                    loudness_intensity_pct = v;
                    loudness_recompute_pending = true;
                    notify_param_write(offsetof(WireBulkParams, global.loudness_intensity_pct),
                                       sizeof(float), &v);
                }
                break;

            case REQ_SET_CROSSFEED:
                if (vendor_last_wLength >= 1) {
                    crossfeed_config.enabled = (vendor_rx_buf[0] != 0);
                    crossfeed_update_pending = true;
                    uint8_t v = crossfeed_config.enabled ? 1 : 0;
                    notify_param_write(offsetof(WireBulkParams, crossfeed.enabled),
                                       1, &v);
                }
                break;
            case REQ_SET_CROSSFEED_PRESET:
                if (vendor_last_wLength >= 1) {
                    uint8_t p = vendor_rx_buf[0];
                    if (p <= CROSSFEED_PRESET_CUSTOM) {
                        crossfeed_config.preset = p;
                        crossfeed_update_pending = true;
                        notify_param_write(offsetof(WireBulkParams, crossfeed.preset),
                                           1, &p);
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
                    notify_param_write(offsetof(WireBulkParams, crossfeed.custom_fc),
                                       sizeof(float), &v);
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
                    notify_param_write(offsetof(WireBulkParams, crossfeed.custom_feed_db),
                                       sizeof(float), &v);
                }
                break;
            case REQ_SET_CROSSFEED_ITD:
                if (vendor_last_wLength >= 1) {
                    crossfeed_config.itd_enabled = (vendor_rx_buf[0] != 0);
                    crossfeed_update_pending = true;
                    uint8_t v = crossfeed_config.itd_enabled ? 1 : 0;
                    notify_param_write(offsetof(WireBulkParams, crossfeed.itd_enabled),
                                       1, &v);
                }
                break;

            case REQ_SET_LEVELLER_ENABLE:
                if (vendor_last_wLength >= 1) {
                    leveller_config.enabled = (vendor_rx_buf[0] != 0);
                    leveller_update_pending = true;
                    leveller_reset_pending  = true;
                    uint8_t v = leveller_config.enabled ? 1 : 0;
                    notify_param_write(offsetof(WireBulkParams, leveller.enabled),
                                       1, &v);
                }
                break;
            case REQ_SET_LEVELLER_AMOUNT:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v < LEVELLER_AMOUNT_MIN) v = LEVELLER_AMOUNT_MIN;
                    if (v > LEVELLER_AMOUNT_MAX) v = LEVELLER_AMOUNT_MAX;
                    leveller_config.amount = v;
                    leveller_update_pending = true;
                    notify_param_write(offsetof(WireBulkParams, leveller.amount),
                                       sizeof(float), &v);
                }
                break;
            case REQ_SET_LEVELLER_SPEED:
                if (vendor_last_wLength >= 1) {
                    uint8_t s = vendor_rx_buf[0];
                    if (s < LEVELLER_SPEED_COUNT) {
                        leveller_config.speed = s;
                        leveller_update_pending = true;
                        notify_param_write(offsetof(WireBulkParams, leveller.speed),
                                           1, &s);
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
                    notify_param_write(offsetof(WireBulkParams, leveller.max_gain_db),
                                       sizeof(float), &v);
                }
                break;
            case REQ_SET_LEVELLER_LOOKAHEAD:
                if (vendor_last_wLength >= 1) {
                    leveller_config.lookahead = (vendor_rx_buf[0] != 0);
                    leveller_update_pending = true;
                    leveller_reset_pending  = true;
                    uint8_t v = leveller_config.lookahead ? 1 : 0;
                    notify_param_write(offsetof(WireBulkParams, leveller.lookahead),
                                       1, &v);
                }
                break;
            case REQ_SET_LEVELLER_GATE:
                if (vendor_last_wLength >= 4) {
                    float v; memcpy(&v, vendor_rx_buf, 4);
                    if (v < LEVELLER_GATE_MIN) v = LEVELLER_GATE_MIN;
                    if (v > LEVELLER_GATE_MAX) v = LEVELLER_GATE_MAX;
                    leveller_config.gate_threshold_db = v;
                    leveller_update_pending = true;
                    notify_param_write(offsetof(WireBulkParams, leveller.gate_threshold_db),
                                       sizeof(float), &v);
                }
                break;

            case REQ_SET_CHANNEL_NAME: {
                uint8_t ch = vendor_last_wValue & 0xFF;
                if (ch < NUM_CHANNELS && vendor_last_wLength > 0) {
                    memset(channel_names[ch], 0, PRESET_NAME_LEN);
                    size_t copy_len = vendor_last_wLength < (PRESET_NAME_LEN - 1)
                                    ? vendor_last_wLength : (PRESET_NAME_LEN - 1);
                    memcpy(channel_names[ch], vendor_rx_buf, copy_len);
                    /* Push the FULL 32-byte name slot — easier than carving
                     * a partial diff for variable-length string updates. */
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, channel_names)
                                   + ch * WIRE_NAME_LEN),
                        WIRE_NAME_LEN, channel_names[ch]);
                }
                break;
            }
            case REQ_SET_OUTPUT_TYPE: {
                uint8_t slot     =  vendor_last_wValue       & 0xFF;
                uint8_t new_type = (vendor_last_wValue >> 8) & 0xFF;
                if (slot < NUM_SPDIF_INSTANCES && new_type <= 1
                    && output_types[slot] != new_type) {
                    output_types[slot] = new_type;
                    extern volatile bool output_type_change_pending;
                    output_type_change_pending = true;
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, i2s_config.output_types)
                                   + slot),
                        1, &new_type);
                }
                break;
            }

            case REQ_SET_INPUT_SOURCE: {
                /* Console sends this as a plain vendor OUT with a
                 * 1-byte payload: vendor_rx_buf[0] = InputSource enum
                 * value. Mirror the RP convention exactly — the apply
                 * runs in main-loop context to keep HAL_SPDIFRX_* and
                 * PLL2 manipulation out of USB ISR. The notify is
                 * fired at apply time in main.c, not here, so the
                 * Console shadow tracks the active state rather than
                 * the requested-but-not-yet-applied state. */
                if (vendor_last_wLength >= 1) {
                    uint8_t src = vendor_rx_buf[0];
                    if (input_source_valid(src)
                        && src != active_input_source) {
                        pending_input_source = src;
                        __DMB();
                        input_source_change_pending = true;
                    }
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

            /* ---- Preset SETs (M11) — backed by W25Q64 + flash_storage.c.
             * The slot index is wValue (0..9). preset_save / load take
             * zero-byte payloads; the rest carry their data in vendor_rx_buf. */
            case REQ_PRESET_SAVE: {
                uint8_t slot = vendor_last_wValue & 0xFF;
                extern volatile uint8_t last_preset_result;
                last_preset_result = preset_save(slot);
                break;
            }
            case REQ_PRESET_LOAD: {
                uint8_t slot = vendor_last_wValue & 0xFF;
                preset_load(slot);
                /* preset_load runs filter recompute + delay update inside
                 * itself; matches the bulk-SET drain in the main loop. */
                break;
            }
            case REQ_PRESET_DELETE: {
                uint8_t slot = vendor_last_wValue & 0xFF;
                preset_delete(slot);
                break;
            }
            case REQ_PRESET_SET_NAME: {
                uint8_t slot = vendor_last_wValue & 0xFF;
                if (vendor_last_wLength > 0 && vendor_last_wLength < 64) {
                    char name[PRESET_NAME_LEN];
                    size_t n = vendor_last_wLength < (PRESET_NAME_LEN - 1)
                             ? vendor_last_wLength : (PRESET_NAME_LEN - 1);
                    memset(name, 0, sizeof(name));
                    memcpy(name, vendor_rx_buf, n);
                    preset_set_name(slot, name);
                }
                break;
            }
            case REQ_PRESET_SET_STARTUP: {
                /* wValue HI = mode (0=specified, 1=last_active),
                 * wValue LO = default slot index (used only when mode=0). */
                uint8_t mode = (vendor_last_wValue >> 8) & 0xFF;
                uint8_t dflt =  vendor_last_wValue       & 0xFF;
                preset_set_startup(mode, dflt);
                break;
            }
            case REQ_PRESET_SET_INCLUDE_PINS: {
                if (vendor_last_wLength >= 1) {
                    preset_set_include_pins(vendor_rx_buf[0] != 0);
                }
                break;
            }
            case REQ_SET_MASTER_VOLUME_MODE: {
                /* Payload: 1 byte mode (0 = independent, 1 = with-preset).
                 * preset_set_master_volume_mode() clamps + flushes the dir. */
                if (vendor_last_wLength >= 1) {
                    preset_set_master_volume_mode(vendor_rx_buf[0]);
                }
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
