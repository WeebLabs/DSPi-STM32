/**
 * usb_audio.h — DSPi STM32H723 UAC1 OUT (M3 silence consumer)
 *
 * Constants shared between usb_audio.c (custom UAC1 class driver) and
 * usb_descriptors.c. Mirrors the conventions used by firmware/DSPi/
 * usb_audio.c so future milestones can rebase without renaming things.
 */

#ifndef DSPI_STM32_USB_AUDIO_H
#define DSPI_STM32_USB_AUDIO_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"     /* NUM_CHANNELS, PRESET_NAME_LEN, MatrixMixer, etc. */

/* ---- Endpoint addresses ---- */
#define AUDIO_OUT_ENDPOINT      0x01    /* ISO OUT, host -> device */
#define AUDIO_FB_ENDPOINT       0x82    /* ISO IN feedback, async sync */

/* ---- Sample format ---- */
#define AUDIO_SAMPLE_RATE       48000U
#define AUDIO_CHANNELS          2
#define AUDIO_BIT_DEPTH         16U
#define AUDIO_BYTES_PER_SAMPLE  (AUDIO_BIT_DEPTH / 8)
#define AUDIO_BYTES_PER_FRAME   (AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE)

/* Max payload per 1 ms USB FS frame, including +1 jitter sample.
 * 48 nominal + 1 jitter = 49 frames * 4 B = 196 bytes. Round up. */
#define AUDIO_EP_MAX_PKT        200U

/* UAC1 entity IDs (arbitrary, distinct, non-zero) */
#define UAC1_INPUT_TERMINAL_ID  0x01
#define UAC1_FEATURE_UNIT_ID    0x02
#define UAC1_OUTPUT_TERMINAL_ID 0x03

/* Interface numbering — order is what the descriptor declares them */
#define ITF_NUM_AC              0   /* AudioControl */
#define ITF_NUM_AS              1   /* AudioStreaming */
#define ITF_NUM_VENDOR          2   /* Vendor-class control + notify EP (M7a) */
#define ITF_NUM_TOTAL           3

/* Notification (device → host) bulk IN endpoint on the vendor interface.
 * 64-byte FS bulk packets carry asynchronous status events (peak meters,
 * audio source change, lock state, etc.) that the host polls when ready.
 * bInterval is ignored for bulk on FS — set to 0 to match the RP build's
 * convention. */
#define NOTIFY_IN_ENDPOINT      0x83U
#define NOTIFY_EP_MAX_PKT       64U
#define NOTIFY_EP_INTERVAL_MS   0U

/* M3 stats — exposed for the heartbeat printf. */
extern volatile uint32_t audio_bytes_received;
extern volatile uint32_t audio_packets_received;
extern volatile bool     audio_streaming;

/* AudioState — top-level UI mirror. Imported from
 * firmware/DSPi/usb_audio.h so the bulk_params wire format matches.
 * Fields are written by USB control requests (volume/mute) and the
 * input source (sample rate) and read by the DSP/host. */
typedef struct {
    uint32_t freq;
    int16_t  volume;
    int16_t  vol_mul;
    bool     mute;
} AudioState;
extern volatile AudioState audio_state;
extern volatile bool       bypass_master_eq;

/* DSP-pipeline globals — defined in audio_state.c. The bulk_params /
 * notify / vendor command code expects these names verbatim from the RP
 * project so wire-format compatibility holds. */
extern char channel_names[NUM_CHANNELS][PRESET_NAME_LEN];

/* Called when the host changes master volume through any path (UAC1
 * volume control, vendor command, bulk param SET). Currently a stub
 * (M7b) that just clamps and stores the value; M7c+ wires it to the
 * SAI output gain stage. */
void update_master_volume(float db);

/* USB→SAI ring (M6a). The UAC1 ISO OUT EP writes 16-bit stereo PCM
 * frames here; audio_out.c reads them out into the SAI ping-pong
 * buffer. One "frame" = one stereo pair = two int16 samples = 4 bytes.
 *
 * usb_ring_pop_frames returns how many full stereo frames it
 * actually delivered (≤ requested). Caller fills any shortfall with
 * silence. The ring is power-of-two sized so the modulo is a mask. */
#define USB_RING_FRAMES   1024U     /* 21 ms at 48 kHz, ~4 KB */

uint32_t usb_ring_pop_frames(int16_t *dst, uint32_t want_frames);
uint32_t usb_ring_level_frames(void);   /* current fill, for diagnostics */

#endif
