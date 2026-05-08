/**
 * config.h — DSPi STM32H723, project-portable constants
 *
 * Subset of firmware/DSPi/config.h relevant to the STM32 port. Just the
 * symbols vendor_commands.c, usb_descriptors.c, and usb_audio.c reach for.
 * Full DSP-pipeline configuration (channel counts, EQ band counts, delay
 * sizes, etc.) lands when M7c brings the actual pipeline over.
 */

#ifndef DSPI_STM32_CONFIG_H
#define DSPI_STM32_CONFIG_H

#include <stdint.h>

/* Platform identifiers — vendor command REQ_GET_PLATFORM returns one
 * of these so the host (DSPi Console) can pick the right command set. */
#define PLATFORM_RP2040       0
#define PLATFORM_RP2350       1
#define PLATFORM_STM32H723    2     /* M7a addition */

#define DSPI_PLATFORM         PLATFORM_STM32H723

/* Vendor command IDs — wire-protocol-stable. Subset for M7a; the full
 * table is at firmware/DSPi/config.h:125+ and gets ported across as
 * each handler is implemented. */
#define REQ_GET_PLATFORM            0x7F

/* Microsoft OS string descriptor vendor code (unused on macOS but kept
 * for protocol parity with the RP build). */
#define MS_VENDOR_CODE              0x01

#endif
