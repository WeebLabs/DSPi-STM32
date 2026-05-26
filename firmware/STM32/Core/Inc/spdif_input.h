/*
 * spdif_input.h — STM32 SPDIFRX peripheral integration
 *
 * Wire-format parity with the RP build's spdif_input.h so DSPi Console
 * sees identical SpdifRxStatusPacket / SpdifInputState semantics. The
 * STM32H723 path uses the on-chip SPDIFRX peripheral on PD8 (AF9, INSEL=1) with
 * DMA1 streams 2 (data) + 3 (control), running off PLL2_R as kernel
 * clock — same PLL2 already programmed for SAI1/SAI4 audio.
 *
 * Stage 1 (current): peripheral up, lock detection, sample-rate
 *                    measurement via SPDIFRX_SR.WIDTH5, status query.
 *                    No audio integration with fill_half yet.
 * Stage 2 (next):    DMA data flow, demux L/R via PT field, hook into
 *                    audio_input switching and the resampler.
 */

#ifndef SPDIF_INPUT_H
#define SPDIF_INPUT_H

#include <stdint.h>
#include <stdbool.h>

/* SPDIF RX input state machine — matches the RP enum value-for-value. */
typedef enum {
    SPDIF_INPUT_INACTIVE   = 0,   /* RX hardware stopped (not selected) */
    SPDIF_INPUT_ACQUIRING  = 1,   /* waiting for initial signal lock     */
    SPDIF_INPUT_LOCKED     = 2,   /* receiving and processing audio      */
    SPDIF_INPUT_RELOCKING  = 3,   /* signal lost, waiting for re-lock    */
} SpdifInputState;

/* 16-byte status packet returned by REQ_GET_SPDIF_RX_STATUS (0xE2).
 * Layout MUST match the RP build verbatim — Console treats both
 * platforms with one binary parser. */
typedef struct __attribute__((packed)) {
    uint8_t  state;          /* SpdifInputState                         */
    uint8_t  input_source;   /* current active InputSource enum         */
    uint8_t  lock_count;     /* successful locks since activation       */
    uint8_t  loss_count;     /* lock losses since activation            */
    uint32_t sample_rate;    /* detected sample rate in Hz (0 = none)   */
    uint32_t parity_errors;  /* cumulative parity error count           */
    uint16_t fifo_fill_pct;  /* RX FIFO fill percentage (0-100)         */
    uint16_t reserved;
} SpdifRxStatusPacket;

/* Initialise the SPDIFRX peripheral (RCC, GPIO, DMA handles, NVIC).
 * Does NOT start receiving — call spdif_input_start() to arm DMA. */
void spdif_input_init(void);

/* Arm the DMAs and put the peripheral into SYNC mode. Hardware will
 * auto-progress to RCV state once it sees a valid biphase signal on
 * PD8. Idempotent: calling twice is a no-op. */
void spdif_input_start(void);

/* Stop receiving and idle the peripheral. Safe to call when already
 * stopped. */
void spdif_input_stop(void);

/* Main-loop poll: refresh state machine, read WIDTH5 for sample-rate
 * measurement, accumulate channel-status block, and (Stage 2) drive
 * the PLL2 FRACN servo from the input-ring fill level. Returns 0 —
 * audio data is consumed via spdif_input_pop_frames() from the audio
 * IRQ, not from the polling caller. */
uint32_t spdif_input_poll(void);

/* Stage 2: drain up to want_frames stereo frames from the SPDIFRX
 * input ring into dst[2 × want_frames] (interleaved L/R int16, same
 * format as usb_ring_pop_frames). Returns frames actually written;
 * shortfall is silence-padded so the caller never has to handle a
 * partial fill. Called from the SAI1 DMA half-cplt context — must
 * be lock-free vs the SPDIFRX DMA callbacks that fill the ring. */
uint32_t spdif_input_pop_frames(int16_t *dst, uint32_t want_frames);

/* Populate the 16-byte status packet for vendor cmd 0xE2. */
void spdif_input_get_status(SpdifRxStatusPacket *out);

/* Copy the 24-byte IEC 60958 channel status block into out_24_bytes
 * for vendor cmd 0xE3. Returns zeros if no block has been received. */
void spdif_input_get_channel_status(uint8_t *out_24_bytes);

/* Debug: snapshot of current servo state. fill = ring write-head minus
 * read-head; err = fill - target; int_acc = integral accumulator
 * (== current FRACN offset from nominal); fracn = last FRACN value
 * actually written. All values are point-in-time and may race the
 * servo by one tick — that's fine for a diagnostic. */
typedef struct __attribute__((packed)) {
    int32_t  fill;            /* current ring fill in stereo frames     */
    int32_t  err;             /* fill - SPDIF_RING_TARGET_FILL          */
    int32_t  int_acc;         /* servo_int_acc                          */
    int32_t  current_fracn;   /* servo_last_fracn (signed for clarity)  */
    uint32_t fracn_writes;    /* count of pll2_fracn_write calls        */
    uint32_t widx;            /* spdif_ring_widx (raw monotonic)        */
    uint32_t ridx;            /* spdif_ring_ridx (raw monotonic)        */
    uint32_t underruns;       /* cumulative fill_half short-pop count   */
    uint32_t peak_cycles;     /* worst fill_half cycle count (rolling)  */
    uint32_t budget_cycles;   /* fill_half budget = 4 ms × SYSCLK       */
} SpdifServoDebugPacket;

void spdif_input_get_servo_debug(SpdifServoDebugPacket *out);

#endif /* SPDIF_INPUT_H */
