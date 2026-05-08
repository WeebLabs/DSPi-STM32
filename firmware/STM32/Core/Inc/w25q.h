/**
 * w25q.h — Winbond W25Q64 (8 MB) SPI NOR flash driver, used as the
 * preset-storage backing store for the STM32 build.
 *
 * Geometry (matches the existing RP preset layout 1:1, no remap):
 *   - 4 KB sectors  (smallest erase unit)
 *   - 256 B pages   (program unit)
 *   - 8 MB total — first 48 KB are reserved for the preset region; the
 *     rest is unused but available for future log/scratch storage.
 *
 * Wiring on WeAct MiniSTM32H723 V1.2 (per WeAct example 06-SPIFlash_Test
 * MspInit — the porting doc said "SPI3" but the chip is actually on SPI1):
 *   PB3  -> W25Q CLK     (SPI1_SCK,  AF5)   — overrides JTDO default
 *   PB4  -> W25Q DO      (SPI1_MISO, AF5)   — overrides NJTRST default
 *   PD7  -> W25Q DI      (SPI1_MOSI, AF5)
 *   PD6  -> W25Q /CS     (GPIO out, software-controlled)
 *
 * Endurance: 100 k erase cycles per sector.  No wear-levelling required
 * for the 10 user-writable preset slots — sufficient for decades of use.
 */

#ifndef W25Q_H
#define W25Q_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define W25Q_SECTOR_SIZE   4096U
#define W25Q_PAGE_SIZE     256U
#define W25Q_TOTAL_BYTES   (8U * 1024U * 1024U)   /* 8 MB */

/* JEDEC manufacturer IDs that may show up on a WeAct V1.2 — board ships
 * with either Winbond (0xEF) or PuYa (0x85) silicon depending on stock.
 * Both speak the same standard SPI NOR command set; the driver only
 * uses CMD_READ_DATA / CMD_PAGE_PROGRAM / CMD_SECTOR_ERASE_4K which are
 * identical across vendors at this density (0x40 0x17 / 0x20 0x17 = 64
 * Mbit / 8 MB). */
#define W25Q_JEDEC_MFR_WINBOND    0xEF
#define W25Q_JEDEC_MFR_PUYA       0x85
#define W25Q_JEDEC_DEV1_8MB       0x17

/* Bring up SPI3 + GPIO, run a JEDEC-ID probe.  Returns true if the
 * detected manufacturer ID is W25Q_JEDEC_EXPECTED_MFR; logs id_out (3
 * bytes) for diagnostics regardless. */
bool w25q_init(uint8_t id_out[3]);

/* Read `len` bytes starting at flash byte offset `addr` into `dst`.
 * Crosses page/sector boundaries freely; one SPI transaction per call. */
void w25q_read(uint32_t addr, uint8_t *dst, uint32_t len);

/* Erase a single 4 KB sector.  `addr` must be sector-aligned (low 12
 * bits zero); silently rounds down if not.  Blocks until WIP clears. */
void w25q_sector_erase_4k(uint32_t addr);

/* Program up to W25Q_PAGE_SIZE bytes starting at `addr`.  The caller is
 * responsible for not crossing a 256-byte page boundary — the W25Q
 * silently wraps within the page if you do.  Blocks until WIP clears. */
void w25q_page_program(uint32_t addr, const uint8_t *src, uint16_t len);

/* Erase a contiguous range, sector-aligned.  Issues N sector erases. */
void w25q_range_erase(uint32_t addr, uint32_t len);

/* Program a range, page-by-page.  Splits arbitrary-length writes into
 * 256-byte chunks aligned to page boundaries.  Blocks per page. */
void w25q_range_program(uint32_t addr, const uint8_t *src, uint32_t len);

#endif /* W25Q_H */
