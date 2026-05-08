/**
 * w25q.c — Winbond W25Q64 SPI NOR driver via SPI3 on STM32H7.
 *
 * Sequence-by-sequence implementation: each public API call drives one
 * SPI transaction (or a chain of them for paged writes).  No DMA, no
 * interrupts — flash work happens in the main loop while audio is muted
 * via preset_loading, identical to the RP path's audio-blackout window.
 *
 * The W25Q64 is on SPI3 with a 1.5 V level-shift tolerated by the H7's
 * 3.3 V GPIOs at the DO/DI rails (W25Q is 2.7-3.6 V tolerant). All four
 * pins are 3.3 V on this board.
 */

#include "main.h"
#include "stm32h7xx_hal.h"
#include "w25q.h"
#include <string.h>

/* ---------- Pin / port macros (board-specific) -----------------------
 * WeAct MiniSTM32H723 V1.2 wires the W25Q64 to SPI1 (not SPI3 as the
 * porting doc originally claimed). All four data lines use AF5. Note
 * that PB3 / PB4 are JTDO / NJTRST after reset — once HAL re-muxes them
 * to AF5 SPI1, JTAG access on those pins is gone (SWD via PA13/PA14
 * keeps working). */
#define W25Q_SPI                SPI1
#define W25Q_SPI_CLK_ENABLE()   __HAL_RCC_SPI1_CLK_ENABLE()
#define W25Q_GPIOB_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define W25Q_GPIOD_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

#define W25Q_SCK_PORT      GPIOB
#define W25Q_SCK_PIN       GPIO_PIN_3
#define W25Q_SCK_AF        GPIO_AF5_SPI1

#define W25Q_MISO_PORT     GPIOB
#define W25Q_MISO_PIN      GPIO_PIN_4
#define W25Q_MISO_AF       GPIO_AF5_SPI1

#define W25Q_MOSI_PORT     GPIOD
#define W25Q_MOSI_PIN      GPIO_PIN_7
#define W25Q_MOSI_AF       GPIO_AF5_SPI1

#define W25Q_CS_PORT       GPIOD
#define W25Q_CS_PIN        GPIO_PIN_6

/* ---------- W25Q command opcodes ------------------------------------ */
#define CMD_WRITE_ENABLE        0x06
#define CMD_WRITE_DISABLE       0x04
#define CMD_READ_STATUS_REG1    0x05
#define CMD_READ_DATA           0x03
#define CMD_PAGE_PROGRAM        0x02
#define CMD_SECTOR_ERASE_4K     0x20
#define CMD_CHIP_ERASE          0xC7
#define CMD_RDID                0x9F   /* JEDEC manufacturer + device IDs */
#define CMD_RELEASE_PWRDN       0xAB

#define SR1_WIP_MASK            0x01   /* Write-in-progress bit */

/* ---------- Driver state ------------------------------------------- */
static SPI_HandleTypeDef hspi3;

/* ---------- Low-level helpers --------------------------------------- */

static inline void cs_low(void)  { HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET);   }

/* Single-direction transfer.  HAL_SPI_TransmitReceive simplifies the
 * full-duplex case (e.g. RDID) without a separate state machine. */
static void spi_xfer(const uint8_t *tx, uint8_t *rx, uint16_t n) {
    static const uint8_t dummy[1] = { 0xFF };
    if (tx && rx) {
        HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)tx, rx, n, HAL_MAX_DELAY);
    } else if (tx) {
        HAL_SPI_Transmit(&hspi3, (uint8_t *)tx, n, HAL_MAX_DELAY);
    } else {
        /* Pure read → clock out 0xFF dummy bytes per byte to receive. */
        for (uint16_t i = 0; i < n; ++i) {
            HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)dummy, &rx[i],
                                    1, HAL_MAX_DELAY);
        }
    }
}

static void cmd_only(uint8_t op) {
    cs_low();
    spi_xfer(&op, NULL, 1);
    cs_high();
}

static uint8_t read_status1(void) {
    uint8_t cmd = CMD_READ_STATUS_REG1;
    uint8_t sr  = 0;
    cs_low();
    spi_xfer(&cmd, NULL, 1);
    spi_xfer(NULL, &sr, 1);
    cs_high();
    return sr;
}

/* Block until the on-chip write/erase completes.  Worst-case wait is a
 * full 64 KB block erase (~150 ms typical, 400 ms max) — we never issue
 * those, so the typical 4 KB sector erase (~45 ms) bounds us. */
static void wait_busy(void) {
    while (read_status1() & SR1_WIP_MASK) {
        /* tight poll — nothing useful to do until WIP clears */
    }
}

static void write_enable(void) { cmd_only(CMD_WRITE_ENABLE); }

/* ---------- Public API --------------------------------------------- */

bool w25q_init(uint8_t id_out[3]) {
    /* GPIO clocks */
    W25Q_GPIOB_CLK_ENABLE();
    W25Q_GPIOD_CLK_ENABLE();

    /* CS first — drive high before peripheral comes alive so the W25Q
     * doesn't see a glitch on power-up that could latch it into an
     * unexpected state. */
    GPIO_InitTypeDef g = {0};
    g.Pin   = W25Q_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(W25Q_CS_PORT, &g);
    cs_high();

    /* SCK / MISO / MOSI in alt-function mode */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

    g.Pin       = W25Q_SCK_PIN;
    g.Alternate = W25Q_SCK_AF;
    HAL_GPIO_Init(W25Q_SCK_PORT, &g);

    g.Pin       = W25Q_MISO_PIN;
    g.Alternate = W25Q_MISO_AF;
    HAL_GPIO_Init(W25Q_MISO_PORT, &g);

    g.Pin       = W25Q_MOSI_PIN;
    g.Alternate = W25Q_MOSI_AF;
    HAL_GPIO_Init(W25Q_MOSI_PORT, &g);

    /* SPI3 peripheral. PCLK1 = 137.5 MHz on H723 at SYSCLK=550, APB1
     * = HCLK/4 = 137.5. /16 prescaler -> 8.6 MHz SCK — well under
     * W25Q64 max (104 MHz read, 80 MHz program), conservative for first
     * bring-up. Can crank to /4 (34 MHz) once we trust the board. */
    W25Q_SPI_CLK_ENABLE();
    hspi3.Instance               = W25Q_SPI;
    hspi3.Init.Mode              = SPI_MODE_MASTER;
    hspi3.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;     /* CPOL=0 */
    hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;      /* CPHA=0 */
    hspi3.Init.NSS               = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial     = 7;
    hspi3.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    hspi3.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    hspi3.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    hspi3.Init.MasterSSIdleness  = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi3.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi3.Init.IOSwap            = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) return false;

    /* Wake from possible deep power-down, then probe JEDEC ID. */
    cmd_only(CMD_RELEASE_PWRDN);
    HAL_Delay(1);

    uint8_t cmd = CMD_RDID;
    uint8_t id[3] = { 0, 0, 0 };
    cs_low();
    spi_xfer(&cmd, NULL, 1);
    spi_xfer(NULL, id, 3);
    cs_high();

    if (id_out) {
        id_out[0] = id[0];
        id_out[1] = id[1];
        id_out[2] = id[2];
    }
    /* Accept either Winbond or PuYa silicon (both ship on WeAct V1.2),
     * size byte 0x17 = 64 Mbit. The vendor's "memory family" byte differs
     * (0x40 vs 0x20) and is informational only at this layer. */
    bool mfr_ok  = (id[0] == W25Q_JEDEC_MFR_WINBOND) ||
                   (id[0] == W25Q_JEDEC_MFR_PUYA);
    bool size_ok = (id[2] == W25Q_JEDEC_DEV1_8MB);
    return mfr_ok && size_ok;
}

void w25q_read(uint32_t addr, uint8_t *dst, uint32_t len) {
    if (len == 0) return;
    uint8_t hdr[4] = {
        CMD_READ_DATA,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >>  8),
        (uint8_t)(addr      ),
    };
    cs_low();
    spi_xfer(hdr, NULL, 4);
    spi_xfer(NULL, dst, (uint16_t)len);  /* HAL caps at uint16 — split if needed */
    cs_high();
}

void w25q_sector_erase_4k(uint32_t addr) {
    addr &= ~(W25Q_SECTOR_SIZE - 1);
    write_enable();
    uint8_t hdr[4] = {
        CMD_SECTOR_ERASE_4K,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >>  8),
        (uint8_t)(addr      ),
    };
    cs_low();
    spi_xfer(hdr, NULL, 4);
    cs_high();
    wait_busy();
}

void w25q_page_program(uint32_t addr, const uint8_t *src, uint16_t len) {
    if (len == 0) return;
    /* W25Q wraps within a page; clamp to the page boundary if caller
     * over-provided. */
    uint32_t page_end = (addr & ~(W25Q_PAGE_SIZE - 1)) + W25Q_PAGE_SIZE;
    if (addr + len > page_end) len = (uint16_t)(page_end - addr);

    write_enable();
    uint8_t hdr[4] = {
        CMD_PAGE_PROGRAM,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >>  8),
        (uint8_t)(addr      ),
    };
    cs_low();
    spi_xfer(hdr, NULL, 4);
    spi_xfer(src, NULL, len);
    cs_high();
    wait_busy();
}

void w25q_range_erase(uint32_t addr, uint32_t len) {
    /* Round to sector boundaries. */
    uint32_t end  = addr + len;
    uint32_t base = addr & ~(W25Q_SECTOR_SIZE - 1);
    while (base < end) {
        w25q_sector_erase_4k(base);
        base += W25Q_SECTOR_SIZE;
    }
}

void w25q_range_program(uint32_t addr, const uint8_t *src, uint32_t len) {
    while (len) {
        uint32_t page_off  = addr & (W25Q_PAGE_SIZE - 1);
        uint32_t page_room = W25Q_PAGE_SIZE - page_off;
        uint32_t chunk     = (len < page_room) ? len : page_room;
        w25q_page_program(addr, src, (uint16_t)chunk);
        addr += chunk;
        src  += chunk;
        len  -= chunk;
    }
}
