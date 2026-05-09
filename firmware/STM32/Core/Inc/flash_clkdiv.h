/**
 * flash_clkdiv.h — STM32 platform shim for the imported flash_storage.c.
 *
 * The imported flash_storage.c (originally RP) uses three abstractions
 * from the underlying platform:
 *
 *   1. XIP_BASE              — base of the memory-mapped flash window
 *   2. PICO_FLASH_SIZE_BYTES — total mirrored flash size
 *   3. dspi_flash_range_erase / _program — write-side primitives
 *
 * On RP the values come from pico-sdk + a custom XIP-cache-respecting
 * write path. On STM32 we don't have memory-mapped flash; the W25Q64 is
 * SPI-attached. We mirror the entire preset region (48 KB) in RAM at
 * boot, point XIP_BASE at that mirror, and route the write-side calls
 * through the w25q driver while keeping the mirror in sync.
 *
 * The mirror lives in BSS (uncached AXI / SRAM4 are options if cache
 * coherency becomes an issue later — the M7 D-cache is currently off).
 */

#ifndef FLASH_CLKDIV_H
#define FLASH_CLKDIV_H

#include <stdint.h>
#include <stddef.h>
#include "config.h"

/* Imported flash_storage.c expects these names verbatim — but the H7
 * HAL header defines FLASH_SECTOR_SIZE = 128 KB (the H7 internal flash
 * sector size). Force-redefine to the W25Q64 SPI NOR geometry. The
 * collision is harmless because we never call HAL flash erase/program
 * on the internal flash from runtime — only HAL_RCC_ClockConfig uses
 * the latency macros, which don't depend on these sizes. */
#undef  FLASH_SECTOR_SIZE
#undef  FLASH_PAGE_SIZE
#define FLASH_SECTOR_SIZE        4096U
#define FLASH_PAGE_SIZE          256U

/* Total preset region: 12 sectors. flash_storage.c computes
 * PRESET_BASE_OFFSET = PICO_FLASH_SIZE_BYTES - PRESET_TOTAL_SECTORS * 4096
 * so we declare PICO_FLASH_SIZE_BYTES exactly equal to the preset region
 * — that makes PRESET_BASE_OFFSET = 0 and everything maps directly into
 * our 48 KB RAM mirror without any address translation. */
#define PICO_FLASH_SIZE_BYTES    (12U * FLASH_SECTOR_SIZE)

/* The RAM mirror lives in AXI SRAM (320 KB starting at 0x24000000) so it
 * doesn't compete with the 128 KB DTCM budget. Placed at a fixed offset
 * past audio_buf — see flash_clkdiv_stm32.c for the address constant.
 * Declared `uint8_t * const` so flash_mirror[X] still indexes naturally. */
extern uint8_t * const flash_mirror;
#define XIP_BASE                 ((uintptr_t)flash_mirror)

/* Initialize: probe W25Q (already done by w25q_init()), then read the
 * entire preset region into flash_mirror. Call once at boot before
 * preset_boot_load(). Returns true on success. */
bool dspi_flash_init(void);

/* Erase a contiguous range — wraps w25q_range_erase + mirror memset.
 * `flash_offs` must be sector-aligned; `count` rounded up to sector. */
void dspi_flash_range_erase(uint32_t flash_offs, size_t count);

/* Program a range — wraps w25q_range_program + mirror memcpy.
 * `flash_offs` should be page-aligned for best W25Q efficiency. */
void dspi_flash_range_program(uint32_t flash_offs,
                              const uint8_t *data, size_t count);

#endif /* FLASH_CLKDIV_H */
