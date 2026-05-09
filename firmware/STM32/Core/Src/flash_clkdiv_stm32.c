/**
 * flash_clkdiv_stm32.c — STM32 implementation of the flash_storage.c
 * platform shim. See flash_clkdiv.h for the contract.
 *
 * Strategy: keep a 48 KB RAM mirror of the preset region. Reads from
 * flash_storage.c go through XIP_BASE which we point at the mirror —
 * zero-cost on the read path. Writes go to BOTH the W25Q64 (via the
 * SPI driver) AND the mirror, so subsequent reads see the updated
 * value without any cache-invalidation choreography.
 *
 * Erase costs ~45 ms per 4 KB sector on a typical W25Q64 — same order
 * of magnitude as the RP path's flash_range_erase. The audio path
 * already mutes via preset_loading during preset save/load, so the
 * blocking SPI work is invisible.
 */

#include "flash_clkdiv.h"
#include "w25q.h"
#include <string.h>
#include <stdbool.h>

/* The mirror lives in AXI SRAM (320 KB at 0x24000000), placed past
 * audio_buf which sits at the start. audio_buf size = 768 stereo frames
 * * 4 bytes = 3 KB; we reserve 16 KB just to leave headroom and align
 * the mirror to a clean boundary. The mirror runs from 0x24004000 to
 * 0x24010000 (48 KB), leaving 256 KB of AXI SRAM free for future use
 * (delay-line growth, log buffers, etc.).
 *
 * No section attribute / linker script edit needed — placement is via
 * a fixed pointer constant, same trick audio_out.c uses for audio_buf. */
#define FLASH_MIRROR_BASE   0x24004000UL
uint8_t * const flash_mirror = (uint8_t *)FLASH_MIRROR_BASE;

/* MIRROR_BYTES: total preset-region size in the mirror. Cannot use
 * sizeof(flash_mirror) because flash_mirror is declared `uint8_t * const`
 * (a const POINTER, not an array) — sizeof would give 4 bytes (the M7
 * pointer width), making every offset > 4 fail bounds-check and the
 * write silently vanish. PICO_FLASH_SIZE_BYTES is the canonical size. */
#define MIRROR_BYTES   (PICO_FLASH_SIZE_BYTES)

bool dspi_flash_init(void) {
    /* Snapshot the preset region from W25Q byte 0..47KB into the mirror.
     * Subsequent reads go through XIP_BASE = &flash_mirror[0]. */
    w25q_read(0, flash_mirror, MIRROR_BYTES);
    return true;
}

void dspi_flash_range_erase(uint32_t flash_offs, size_t count) {
    /* W25Q sector erase, then reflect the post-erase 0xFF state in the
     * mirror. Bounds check so a stray erase request can't trash memory
     * past the mirror. */
    if (flash_offs >= MIRROR_BYTES) return;
    if (flash_offs + count > MIRROR_BYTES) {
        count = MIRROR_BYTES - flash_offs;
    }
    w25q_range_erase(flash_offs, (uint32_t)count);
    memset(&flash_mirror[flash_offs], 0xFF, count);
}

void dspi_flash_range_program(uint32_t flash_offs,
                              const uint8_t *data, size_t count) {
    if (flash_offs >= MIRROR_BYTES) return;
    if (flash_offs + count > MIRROR_BYTES) {
        count = MIRROR_BYTES - flash_offs;
    }
    w25q_range_program(flash_offs, data, (uint32_t)count);
    memcpy(&flash_mirror[flash_offs], data, count);
}
