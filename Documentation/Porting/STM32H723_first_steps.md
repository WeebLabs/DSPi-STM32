
# STM32H723VGT6 Porting Plan for DSPi Firmware

## Executive Summary

The DSPi firmware is a well-structured, platform-layered real-time audio engine. Approximately 60% of the code (DSP pipeline, EQ, leveller, crossfeed, matrix mixer, loudness, ASRC, presets, USB class driver logic) is fully portable C with no RP2040/RP2350 dependencies. The remaining 40% splits into: TinyUSB device stack integration (portable if using the same stack), and the RP-specific I/O layer (PIO programs, pico-extras audio libraries, pico-sdk HAL calls). The H723 porting work is almost entirely a rewrite of that I/O layer.

The single-core Cortex-M7 at 550 MHz is not a performance bottleneck for this workload. The harder problems are: (1) SPDIF transmit has no native peripheral—you will drive it via the SAI SPDIF-protocol hardware encoder, which does BMC automatically, negating the need to port the PIO BMC lookup-table encoder; (2) the 128 KB flash sector granularity destroys the existing 4 KB preset sector layout and needs a redesign; (3) SAI4's BDMA limitation restricts it to 64 KB SRAM4, making it unsuitable as a primary audio output DMA path; (4) D-cache coherency is the single most common STM32H7 audio debugging trap.

Verdict: go. The project is well-suited to this port. Proceed with the monorepo approach described in Section 13.

---

## 0. Board-Specific Confirmations — WeAct MiniSTM32H723 V1.2

*Added 2026-05-07 after inspecting the schematic, BOM, and example projects in [WeActStudio.MiniSTM32H723](https://github.com/WeActStudio/WeActStudio.MiniSTM32H723). These confirmations override the generic guidance in Sections 2, 3, and 14 where they conflict.*

### 0.1 Board: confirmed WeAct MiniSTM32H723 V1.2

LQFP100, USB-C, **on-board SWD header (P3): 3V3 / SWDIO=PA13 / SWCLK=PA14 / GND**, 2×22 pin headers (P1, P2), no on-board ST-LINK. Additional onboard parts: 8 MB SPI flash (W25Q64 on SPI3), 8 MB OSPI flash (W25Q64), MicroSD socket (SDMMC1), removable ST7735 TFT, removable camera FPC, blue user LED on PE3 via PNP, BOOT0/NRST/K1 buttons.

### 0.2 HSE crystal: confirmed **25 MHz**

Verified three ways: schematic X1 silkscreen "25Mhz", WeAct example `.ioc` declares `VCOInput1Freq_Value=12500000` (= 25 MHz / DIVM=2), and example `main.c` SystemClock_Config uses `PLLM=2, PLLN=44, PLLP=1` (25/2 × 44 = 550 MHz SYSCLK). Crystal load caps are 10 pF (C1, C3). LSE 32.768 kHz also populated on PC14/PC15 with 7 pF caps — RTC available.

### 0.3 USB DFU: confirmed available

USB-C J1 → PA11/PA12 → STM32H7 internal USB FS PHY → System Memory bootloader at `0x1FF09800`. Entry: hold BOOT0, press+release NRST, release BOOT0 ~0.5 s later. macOS flashing options: `dfu-util` (open source, `brew install dfu-util`) or STM32CubeProgrammer. **DFU is sufficient for flashing — no debug probe required for M0.** SWD probe still recommended for runtime debugging (RTT, breakpoints) — a $10 ST-LINK V2 clone or Raspberry Pi Debug Probe via SWDIO/SWCLK header is the minimum cost-effective option.

### 0.4 Pin assignment for DSPi audio (LQFP100 + remove unneeded onboard peripherals)

The full DSPi feature set fits if camera/TFT are not installed and the OSPI flash is not initialized. The required sacrifices:

| Sacrifice | Cost | Justification |
|---|---|---|
| Don't populate camera FPC | None (not installed by default) | Frees DCMI pins consumed by SAI1_A bus, SAI2_MCLK_A, SPDIFRX alternate |
| Don't populate TFT | None (not installed by default) | Frees PE10–PE14 (SAI2_B alternate, SPI2 alternate) |
| Don't init OSPI flash | Lose 8 MB OSPI; keep 8 MB SPI flash for presets (the plan already preferred SPI flash + LittleFS — Section 10) | Frees SAI1_MCLK_A (PE2) and SAI2_A bus (PD11/12/13) |
| Repurpose PE3 (BLUE_LED) for SAI1_SD_B | Lose status LED — wire one to any free header pin | PE3 is the **only** LQFP100 pin for SAI1_SD_B; no alternative |
| Don't use SDMMC1 | Lose SD card | Optional — keep if FW dump/load via SD is desired |

**Confirmed pin assignment for the DSPi port:**

| Function | Pin | Notes |
|---|---|---|
| USB-C D+/D− | PA11 / PA12 | Hard-wired |
| SWD | PA13 / PA14 | Header P3 |
| SAI1_MCLK_A | PE2 | Was OSPI_BK1_IO2 |
| SAI1_SCK_A | PE5 | Was DCMI_D6 |
| SAI1_FS_A | PE4 | Was DCMI_D4 |
| SAI1_SD_A | PE6 | Was DCMI_D7 — **first audio output (master clocks here)** |
| SAI1_SD_B | PE3 | Was BLUE_LED — **second audio output, internal slave to SAI1_A** |
| SAI2_MCLK_A | PE0 | Was DCMI_D2 (SAI2 clock generator; or sync to SAI1 via GCR) |
| SAI2_SCK_A | PD13 | Was OSPI_BK1_IO3 |
| SAI2_FS_A | PD12 | Was OSPI_BK1_IO1 |
| SAI2_SD_A | PD11 | Was OSPI_BK1_IO0 — **third audio output** |
| SAI2_SD_B | PA0 | Free header pin — **fourth audio output, internal slave to SAI2_A** |
| SPDIFRX_IN3 | PD8 | Free header pin (alt: PB7 if camera not installed) |
| SPI2_SCK (PDM CLK) | PB13 | Free |
| SPI2_MOSI (PDM data) | PB15 | Free |
| I2C target SCL/SDA | PB10 / PB11 | I2C2 — both free header pins |
| W25Q64 SPI flash (preset store) | PB3 / PB4 / PD7 / PD6 | SPI3, on-board |
| Status LED | choose any free pin from header | Wire externally |
| K1 user button | PC13 | On-board |

Result: **4 sample-aligned stereo audio outputs + SPDIFRX in + PDM out + USB + I2C target + 8 MB external flash**. Same channel count as RP2350.

### 0.5 PLL plan refined for 25 MHz HSE

The Section 3 plan offered a 55 ppm fractional-N solution. Better numbers using all 13 bits of FRACN:

```
PLL1 (SYSCLK = 550 MHz):
  DIVM1 = 2, DIVN1 = 44, DIVP1 = 1, FRACN = 0
  VCO = 25/2 × 44 = 550 MHz   →  P = 550 MHz   (matches WeAct examples)

PLL2 (SAI kernel = 49.152 MHz):
  DIVM2 = 5, DIVN2 = 19, FRACN = 5414, DIVP2 = 2
  VCO = 25/5 × (19 + 5414/8192) = 5 × 19.66089 = 98.30445 MHz
  P   = VCO / 2 = 49.15223 MHz   →  +4.7 ppm error  (vs target 49.152 MHz)

USB FS = 48 MHz:
  HSI48 + CRS locked to USB SOF — better than ±0.25%, no PLL needed
```

PLL3 left free for future use (e.g., dedicated 44.1 kHz family clock if you decide to add that path later instead of relying on ASRC).

4.7 ppm is within IEC 60958 class I (±100 ppm) and trivially within any DAC's PLL pull range. The Section 3 conclusion (use 24.576 MHz HSE) is no longer relevant — we have the board, the crystal is 25 MHz, and FRACN gets us close enough to be inaudible.

### 0.6 M0 milestone updated for this board

Skip the J-Link purchase initially. Bring-up sequence:
1. Flash via DFU: hold BOOT0, tap NRST, release BOOT0 → `dfu-util -a 0 -s 0x08000000 -D firmware.bin`
2. UART stdio over USART1 (PA9 TX / PA10 RX) — header pins, USB-serial cable required
3. K1 (PC13) for user input, drive any free header pin for status LED
4. Add SWD probe (ST-LINK V2 clone $10) when you need breakpoints + RTT — wire to header P3

---

## 1. Toolchain and Build

### Recommendation: hand-written CMake + ObKo/stm32-cmake + direct STM32CubeH7 submodule

The three real options are:

| Approach | Pros | Cons |
|---|---|---|
| **ObKo/stm32-cmake** (recommended) | Pure CMake, no CubeMX involvement, fine-grained `HAL::STM32::H7::LL_<driver>` targets, active maintenance, STM32H7 well-supported since [Issue #90](https://github.com/ObKo/stm32-cmake/issues/90) | Requires `STM32_CUBE_H7_PATH` pointing at a CubeH7 checkout; first-time setup takes an hour |
| CubeMX "Export as CMake" (6.x) | Lets CubeMX write linker scripts, startup files | Generates opinionated boilerplate; `ioc` file becomes a source-of-truth conflict with hand-managed CMake; CubeMX has a habit of regenerating and clobbering `main.c`; avoid for a project this size |
| Hand-rolled from scratch | Maximum control | Maintaining a startup file + linker script + HAL source list by hand for H7 is tedious and error-prone; ObKo already solved it |

**Concrete setup:**

```cmake
# In your root CMakeLists.txt
set(STM32_CUBE_H7_PATH ${CMAKE_SOURCE_DIR}/vendor/STM32CubeH7)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/vendor/stm32-cmake/cmake)

find_package(CMSIS COMPONENTS STM32H7 REQUIRED)
find_package(HAL   COMPONENTS STM32H7 REQUIRED)

target_link_libraries(DSPi_STM32
    CMSIS::STM32::H723ZG        # CMSIS device headers + startup
    HAL::STM32::H7::RCC         # always needed
    HAL::STM32::H7::LL_SAI      # preferred: LL for audio
    HAL::STM32::H7::LL_DMA      # DMAMUX configuration
    HAL::STM32::H7::USB_OTG_FS  # HAL required for dwc2 TinyUSB port
    ...
)
```

Add `STM32CubeH7` and `stm32-cmake` as git submodules under `vendor/`. The CubeH7 repo ([github.com/STMicroelectronics/STM32CubeH7](https://github.com/STMicroelectronics/STM32CubeH7)) contains HAL, LL, CMSIS, and middleware without requiring CubeMX.

### LL-only vs HAL

Use **LL for everything audio-critical** (SAI, DMAMUX, RCC, GPIO). LL avoids HAL's state-machine overhead in IRQ context and keeps latency predictable.

Use **HAL for USB only.** TinyUSB's STM32 dwc2 port calls `HAL_PCDEx_SetRxFiFo`, `HAL_PCDEx_SetTxFiFo`, and the PCD (Peripheral Control Driver) HAL layer to configure FIFO sizes and enable clocks. You cannot bypass this without rewriting TinyUSB's STM32 backend. Link `HAL::STM32::H7::USB_OTG_FS` and `HAL::STM32::H7::PCD` specifically.

**Use HAL also for I2C** if you prefer the `HAL_I2C_EnableListen_IT` / `HAL_I2C_Slave_Seq_Receive_DMA` API (see Section 12). The LL I2C slave mode is doable but requires manually clearing the ADDR flag in the address-match ISR, which is error-prone.

Flash erase can use either; use LL (`LL_FLASH_Unlock`, `LL_FLASH_EraseSector`) for fine control. The audio ISR must never be active during flash erase (see Section 10).

---

## 2. Dev Board and Bring-up Hardware

### NUCLEO-H723ZG vs STM32H723VGT6 target

The NUCLEO-H723ZG uses the **STM32H723ZGT6** in LQFP144, which is the same silicon die and revision as the LQFP100 target (same DS13313 datasheet, same RM0468 reference manual, same errata sheet). The peripheral set is identical; only the pin-out differs. Key practical differences:

| Item | NUCLEO-H723ZG (LQFP144) | Your target (LQFP100) |
|---|---|---|
| SAI pin richness | All SAI1/2/4 sub-block pins exposed | SAI2 fully present; SAI4 data pins present but clock/sync pins constrained (see below) |
| SPDIFRX | PA2/PD8/PD9 available | Same pins present in LQFP100 |
| USB FS | CN13 (Micro-B via ST-LINK); CN1 separate USB-C | Depends on carrier board |
| Onboard ST-LINK V3 | Yes, built-in | Separate probe needed |
| SAI4_MCLK_B / SAI4_SCK_B in LQFP100 | N/A | **Not bonded out in LQFP100**—SAI4 sub-block B clock pins (PA2 and PB9 alternatives) are absent. Use SAI4 only in synchronous-slave mode (slaved to SAI1 or SAI2) or restrict to sub-block A. CubeMX community confirmed: "SAI1-A and SAI4-A don't have synchronous slave option in LQFP100" was a CubeMX bug, not a hardware limitation, since SYNCEN=11 uses the GCR internal path, not external pins. |

The NUCLEO is the correct first bring-up board. Prototype the full audio path there before spinning a custom LQFP100 PCB.

### WeAct MiniSTM32H723

The [WeActStudio/WeActStudio.MiniSTM32H723](https://github.com/WeActStudio/WeActStudio.MiniSTM32H723) is an LQFP100 board with USB-C, 8 MB SPI Flash, 8 MB OctoSPI Flash, SD slot, TFT connector, and 2×22 2.54 mm pin headers. It is breadboard-hostile (too wide) but suits a custom carrier PCB. There is no onboard crystal frequency documented in the GitHub README—**verify the HSE crystal before ordering**; the most common choice is 25 MHz but WeAct has shipped some boards with 8 MHz or no crystal. For audio work you want 25 MHz HSE (see Section 3).

No dedicated audio I/O is on the WeAct board; you will add your own S/PDIF coax transformers, I2S headers, etc. on a carrier.

### Debug Probe

Recommendation: **SEGGER J-Link EDU Mini** for personal/open-source use, **J-Link BASE** for commercial. Reasons:

- TinyUSB development on H7 benefits from J-Link's RTT logging (`SEGGER_RTT_printf`), which is zero-overhead even inside ISR context and survives cache-enabled operation without locking the bus.
- OpenOCD supports J-Link via `interface jlink` on macOS (Homebrew `openocd`). J-Link GDB Server is faster but proprietary.
- ST-LINK V3 (onboard on NUCLEO) works fine for the early milestones. Switch to J-Link when debugging USB + audio ISR timing.
- CMSIS-DAP (DAPLink, Raspberry Pi Debug Probe): fully open, cheap, adequate—but RTT requires the Cortex-Debug VS Code extension and is less robust.

**RTT viability on H7:** Fully viable. DTCM is not cached (it's tightly coupled), so RTT ring buffers placed in DTCM (`.ram_d1` section at 0x20000000) are coherent without any SCB_CleanDCache calls. Place the `_SEGGER_RTT` struct in DTCM with `__attribute__((section(".dtcm_data")))`.

---

## 3. Clock Tree

### The problem with 25 MHz HSE and audio frequencies

IEC 60958 demands that the SPDIF clock be derived from an integer multiple of Fs. At 48 kHz, the SAI SPDIF clock must be 128×Fs = 6.144 MHz. At 96 kHz, it's 12.288 MHz. These frequencies are derived from a 49.152 MHz (or 98.304 MHz) root clock. USB FS requires exactly 48 MHz. The two clock families (48k audio and USB) are incompatible from a single integer PLL—you need separate PLLs.

STM32H723 has three PLLs: PLL1 (system clock), PLL2, PLL3. All can be independently fed from HSE.

### Concrete PLL plan (HSE = 25 MHz)

**PLL1 — CPU at 550 MHz (system clock)**

```
DIVM1 = 5     → 25/5   = 5 MHz PLL1 ref (VCO input)
DIVN1 = 110   → 5×110  = 550 MHz VCO
DIVP1 = 1     → 550/1  = 550 MHz (SYSCLK)
DIVQ1 = 4     → 550/4  = 137.5 MHz (not used for audio)
```
VCO = 550 MHz, within the 192–836 MHz range for PLL1.

**PLL2 — Audio clock: 49.152 MHz for SAI kernel clock**

The challenge: 49.152 / 25 = 1.96608, which has no exact integer decomposition. The classical trick is:

```
Target: 49.152 MHz
25 MHz / DIVM2 × DIVN2 / DIVP2 = 49.152 MHz
Try DIVM2=25, DIVN2=1228, DIVP2=1: 25/25 × 1228/1 = 1228 MHz (VCO out of range, max 836 MHz for PLL2P)
Try DIVM2=25, DIVN2=614, DIVP2=1: 25/25 × 614 = 614 MHz VCO; 614/1 = 614 (wrong)
Try DIVM2=5, DIVN2=49.152*N/5: not integer...

Best exact solution from community practice:
DIVM2=25, DIVN2=384, DIVP2=1 → VCO=384 MHz; 384/1=384 (wrong)
Alternative: use 147/160 relationship:
DIVM2=25, DIVN2=294, DIVP2=3 → VCO=294 MHz; 294/3=98 MHz, then /2 = 49 MHz (not exact)

Exact approach: accept a fractional VCO. PLL2 supports FRACN (fractional mode):
DIVM2=5, DIVN2=196, FRACN=2458, DIVP2=1
VCO = (25/5) × (196 + 2458/8192) = 5 × 196.30 = 981.5 MHz → too high

Practical best-integer solution:
DIVM2=25, DIVN2=294, DIVP2=3 → VCO=294 MHz, P=98 MHz
SAI kernel = 98.304 MHz is achievable only if DIVN2=245.76 — not integer.
```

**Verdict:** Pure integer PLLs from 25 MHz HSE cannot produce exact 49.152 MHz. Use the FRACN fractional divider:

```
PLL2 with FRACN enabled:
DIVM2 = 5
DIVN2 = 98
FRACN = 2048    (fractional part = 2048/8192 = 0.25)
VCO = (25/5) × (98.25) = 5 × 98.25 = 491.25 MHz → DIVP2=10 → 49.125 MHz (−0.055% = −55 ppm error)
```

That error is 55 ppm, well within IEC 60958 class II (±1000 ppm) and even class I (±100 ppm). For the best result, use a **24.576 MHz HSE crystal** instead of 25 MHz:

```
With HSE=24.576 MHz:
DIVM2=2, DIVN2=8, DIVP2=1, FRACN=0
VCO = (24.576/2) × 8 = 98.304 MHz; DIVP2=2 → 49.152 MHz  EXACT
DIVM3=1, DIVN3=25, DIVP3=13, FRACN=0 
→ PLL3 for USB: 24.576 × 25 / 13 ≈ 47.26 MHz (not exact; use HSI48 + CRS for USB instead)
```

**Best board recommendation: use 24.576 MHz HSE.** This is the standard audio crystal (available from Abracon, TXC, NDK). It gives exact SAI clocks with integer PLL. Use HSI48 + CRS trimmed by USB SOF for the 48 MHz USB FS clock. CRS achieves ±0.25% accuracy required by USB spec.

**PLL3 — 44.1 kHz family (optional)**

```
With HSE=24.576 MHz:
Target: 45.1584 MHz (= 44100 × 1024)
DIVM3=24, DIVN3=882, DIVP3=20: VCO=(24.576/24)×882=1024.07 (out of range)
Use fractional: DIVM3=1, DIVN3=22, FRACN=6722, DIVP3=12
→ VCO ≈ 539.5 MHz, P ≈ 44.96 MHz  (~2000 ppm error from nominal)
```

44.1 kHz exact clock from a 24.576 MHz crystal is impossible with integer PLL (the 48k and 44.1k families are incommensurable). Options: (a) accept ~2000 ppm error for 44.1 kHz and let the ASRC handle rate adaptation from SPDIF RX; (b) add an optional Si5351 I2C programmable clock for 44.1 kHz applications; (c) add a second 22.5792 MHz (= 44100 × 512) crystal on a GPIO-selectable path. For the initial port, accept option (a)—the ASRC already exists.

| PLL | Configuration | Output | Purpose |
|---|---|---|---|
| PLL1 | DIVM=2, DIVN=110, DIVP=1 (24.576 MHz HSE) | 550 MHz | SYSCLK |
| PLL2 | DIVM=2, DIVN=8, DIVP=2 | 49.152 MHz | SAI1/2/4 kernel clock (SAICLKSEL=PLL2_P) |
| PLL2R | DIVR=4 | 24.576 MHz | Optional MCLK out (256×Fs) |
| HSI48+CRS | — | 48 MHz ±0.25% | USB FS (USBSEL=HSI48) |

MCO pin: route PLL2_P to MCO1 to verify 49.152 MHz on a scope at M1.

---

## 4. USB Stack and UAC1

### TinyUSB dwc2 on H7 in 2025–2026

TinyUSB v0.17+ (and the current `master` as of November 2025 / v0.20.0) has stable dwc2 support for STM32H7. Confirmed improvements include:

- ISO IN transfer with `bInterval > 1` fixed ([#1249](https://github.com/hathach/tinyusb/issues/1249) was FSDEV; dwc2 ISO fixes came in 0.15.x).
- `edpt_xfer_fifo()` API added, allowing zero-copy ISO transfer from a pre-shared FIFO buffer—useful for audio.
- RP2040-style `__no_inline_not_in_flash_func` can be replaced with `__attribute__((noinline))` on M7; no functional difference.

**Known risk:** the dwc2 HS port (for external ULPI PHY) has had more regressions than the FS port. Since you are using USB FS only (internal PHY), the risk is lower. There is a Nikitarc fork ([github.com/Nikitarc/tinyusb_H7](https://github.com/Nikitarc/tinyusb_H7)) specifically for STM32H7 with some additional fixes; monitor it but prefer upstream.

### `usbd_app_driver_get_cb` compatibility

This mechanism is entirely within the TinyUSB device-stack layer (`usbd.c`), not in the hardware driver. It works identically on dwc2 as on RP2040's native USB. The RP2040 Pico SDK itself uses exactly this mechanism for its reset interface. Your custom UAC1 class driver requires zero changes to port. Confirmed in [TinyUSB Issue #467](https://github.com/hathach/tinyusb/issues/467).

### Bandwidth math: USB FS for UAC1

| Configuration | Bytes/frame | % of 1023-byte FS max |
|---|---|---|
| Stereo 16-bit 48 kHz | 192 | 18.8% |
| Stereo 24-bit 48 kHz | 288 | 28.2% |
| Stereo 24-bit 96 kHz | 576 | 56.3% |
| Stereo 24-bit 96 kHz + 4-byte feedback EP | 580 | 56.7% |
| 4-ch 24-bit 96 kHz (hypothetical) | 1152 | **over FS limit** |

Stereo 24-bit/96 kHz consumes 576 bytes per 1 ms SOF frame, with a 3-byte feedback endpoint adding 4 bytes (USB FS isochronous feedback is 3 bytes). **FS is fine for the current 2-channel use case at any rate up to 96 kHz.** The 1023-byte/frame ceiling means you cannot do more than stereo 24/96 over FS. For multi-channel UAC2, you would need external ULPI HS PHY.

### dwc2 FIFO Sizing — the footgun

The H723 USB FS OTG has a 1.25 KB shared FIFO pool (320 32-bit words). Allocation is split between the RX FIFO, one non-periodic TX FIFO, and per-endpoint periodic TX FIFOs. The canonical rule from ST community and TinyUSB docs:

```
RX FIFO   ≥  (2 × max_ISO_OUT_packet_size/4) + 1 + 1
             = (2 × 576/4) + 2 = 290 words  for 48/96 kHz stereo 24-bit
Non-periodic TX ≥ 16 words  (control EP0, max 64 bytes)
Periodic TX (ISO IN feedback) ≥ 4 words  (3-byte feedback + overhead)
Total used: 290 + 16 + 4 = 310 words of 320 available — barely fits.
```

In `tusb_config.h` or the board file, set:

```c
#define CFG_TUD_DWC2_RXFIFO_DEPTH    290   // words
#define CFG_TUD_DWC2_TXFIFO_DEPTH_0  16    // EP0
#define CFG_TUD_DWC2_TXFIFO_DEPTH_1  4     // ISO IN feedback EP
```

If you reduce the ISO OUT packet size to 48 kHz stereo 24-bit (288 bytes), the RX FIFO drops to 146 words and you have headroom for a vendor bulk EP.

**Critical:** do NOT place the USB ISO OUT packet buffer in DTCM. DMA1 and DMA2 (which the dwc2 uses internally) cannot access DTCM (0x20000000). Place USB packet buffers in AXI SRAM (0x24000000) or SRAM1 (0x30000000), marked non-cacheable via MPU.

### Known-good UAC1 reference on STM32 + TinyUSB

[dragonman225/stm32f469-usbaudio](https://github.com/dragonman225/stm32f469-usbaudio) — UAC1, 24-bit, 96 kHz, asynchronous feedback, STM32F469 (dwc2 FS). The SOF-based feedback PID controller pattern is directly comparable to the project's existing `usb_feedback_controller.c`. This is the closest available reference; note it uses STM32 USB HAL, not TinyUSB, but the descriptor structure and feedback math are reusable.

---

## 5. I2S Out — Multiple Instances

### SAI availability on STM32H723

The STM32H723 has three SAI instances: SAI1, SAI2, and SAI4. Each has two sub-blocks (A and B) that can be independently configured. Mapping:

| SAI sub-block | DMA controller | Memory constraint | Notes |
|---|---|---|---|
| SAI1_A | DMA1/DMA2 via DMAMUX1 | AXI SRAM, SRAM1/2/3 (not DTCM) | Primary audio output: use this |
| SAI1_B | DMA1/DMA2 via DMAMUX1 | Same | Slave to SAI1_A or independent |
| SAI2_A | DMA1/DMA2 via DMAMUX1 | Same | Additional stereo channel |
| SAI2_B | DMA1/DMA2 via DMAMUX1 | Same | Slave to SAI2_A or independent |
| SAI4_A | **BDMA only** (DMAMUX2) | **SRAM4 only** (64 KB, 0x38000000) | Major constraint—limit use |
| SAI4_B | **BDMA only** | SRAM4 only | Avoid for large audio buffers |

**SAI4 constraint is serious.** BDMA can only access SRAM4 (64 KB total for D3 domain). All audio DMA buffers for SAI4 must live there. With 64 KB total, you can fit approximately 16 KB of ping-pong buffers for one SAI4 sub-block (2 × 192 × 4 bytes × 2 channels = 3 KB per side, easily fits). The constraint becomes binding when you also need SRAM4 for other D3 peripherals. **Recommendation: use SAI1 and SAI2 for all primary audio outputs. Use SAI4 only if you have exhausted SAI1/2 sub-blocks or for a low-bandwidth output.**

This gives a maximum of 4 independently DMAed stereo streams from SAI1 + SAI2, which matches the RP2350's 4 SPDIF instances exactly.

### Master/Slave synchronization for sample alignment

The SAI GCR (Global Configuration Register, RM0468 §34.7.1) controls inter-instance sync:

```
GCR.SYNCOUT[1:0]: which sub-block provides the sync signal to other instances
  00 = no output
  01 = SAI1_A provides sync
  10 = SAI1_B provides sync

GCR.SYNCIN[1:0]: which external SAI feeds this instance
  (SAI1 can receive from SAI4, SAI2 can receive from SAI1 or SAI4, etc.)
```

Within a single SAI instance, `SAI_xCR1.SYNCEN[1:0]`:
- `00` = asynchronous (master)
- `01` = synchronous with other sub-block of same SAI (internal slave)
- `10` = synchronous with sub-block from another SAI via GCR (external slave)
- `11` = synchronous with SAI4 via SYNCIN (cross-domain, H7 specific)

**Recommended topology for 4 stereo outputs (8 channels):**

```
SAI1_A: master (generates MCLK, BCK, LRCLK)
  SYNCEN = 00 (asynchronous master)
  MCK output pin → drives codec or transformer MCLK
  
SAI1_B: internal slave to SAI1_A
  SYNCEN = 01

SAI2_A: external slave to SAI1_A
  SAI1 GCR.SYNCOUT = 01 (SAI1_A provides sync)
  SAI2 GCR.SYNCIN  = selects SAI1 as sync provider
  SAI2_A SYNCEN    = 10 (external sync)

SAI2_B: internal slave to SAI2_A
  SYNCEN = 01
```

This gives 4 sub-blocks all phase-locked to one SAI1_A master clock. DMA start sequencing: arm all 4 DMA channels, then enable SAI1_A last (its enable releases the BCK/LRCLK edges that ungate the slaves simultaneously). The first DMA half-complete interrupt from SAI1_A acts as the timing master for all output buffer refills.

**For SPDIF TX via SAI:** SAI in SPDIF protocol mode (PRTCFG=10) requires the sub-block to be asynchronous (SYNCEN=00). This means a SPDIF-mode SAI sub-block cannot share BCK/LRCLK with an I2S slave. You need a dedicated sub-block per SPDIF stream. SAI1_A and SAI2_A in SPDIF mode, SAI1_B and SAI2_B in I2S master/slave mode is one viable partition.

### DMA buffer cache coherency

Place all SAI DMA buffers in a dedicated MPU region marked non-cacheable. The preferred allocation:

```c
// In linker script or startup:
// Declare a 32-byte aligned non-cacheable section in AXI SRAM:
uint32_t sai_dma_buf[4][2][192*2]  // 4 sub-blocks, ping-pong, 192 stereo frames
  __attribute__((section(".noncacheable"), aligned(32)));
```

MPU configuration (call in `SystemInit` before `SCB_EnableDCache()`):

```c
MPU_Region_InitTypeDef mpu = {
    .Enable           = MPU_REGION_ENABLE,
    .Number           = MPU_REGION_NUMBER0,
    .BaseAddress      = 0x24000000,   // AXI SRAM
    .Size             = MPU_REGION_SIZE_512KB,
    .SubRegionDisable = 0x00,
    .TypeExtField     = MPU_TEX_LEVEL1,
    .AccessPermission = MPU_REGION_FULL_ACCESS,
    .DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE,
    .IsShareable      = MPU_ACCESS_NOT_SHAREABLE,
    .IsCacheable      = MPU_ACCESS_NOT_CACHEABLE,
    .IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE,
};
HAL_MPU_ConfigRegion(&mpu);
```

Alternatively, mark only the DMA buffer sub-region non-cacheable and keep the rest of AXI SRAM write-back cached. This is described in [ST AN4839](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf). The non-cacheable approach is simpler and has zero `SCB_CleanDCache` overhead in the audio callback—recommended for audio firmware.

Place DSP computation buffers (`buf_out`, delay lines, EQ state) in AXI SRAM with the default write-back cached policy (they are CPU-only). Only the final DMA ping-pong buffers need to be non-cacheable.

---

## 6. SPDIF Transmit

### Approach comparison

| Approach | Effort | Parallel streams | CPU cost | Notes |
|---|---|---|---|---|
| **(A) SAI SPDIF protocol (PRTCFG=10)** | Low-medium | Up to 4 (one per sub-block) | Zero (BMC in hardware) | Recommended |
| (B) TIM + DMA bit-bang | High | Limited by TIM count | Zero (DMA) | 6.144 MHz DMA rate is DMA-intensive |
| (C) External IC (WM8804, CS8406) | Low | One chip per stream | Zero | Cost + BOM; justified only if GPIO count is tight |

### Approach A: SAI SPDIF hardware encoder

When `SAI_xCR1.PRTCFG = 0b10`, the SAI block switches to IEC 60958-compatible SPDIF protocol. The SAI hardware:

1. Accepts 32-bit words from the DMA FIFO (lower 24 bits = audio sample, upper 8 bits = V+U+C+P status).
2. Internally generates the preamble (B/M/W), appends V/U/C/P bits, and performs BMC encoding.
3. Outputs the biphase-mark-coded stream at 64× Fs (= 3.072 MHz at 48 kHz).

**This completely replaces the RP2040 PIO BMC lookup-table encoder.** The pre-encoded subframe buffers, `spdif_update_subframe()`, and `spdif_lookup[256]` table are not needed. The DMA word format for SAI SPDIF is:

```
Bits [23:0]  = audio sample (24-bit, left-justified, LSB = bit 0)
Bit  [24]    = validity flag (0 = valid audio)
Bit  [25]    = user data bit (set from 192-bit channel status frame if desired)
Bit  [26]    = channel status bit (set from IEC 60958-3 byte array)
Bit  [27]    = parity (set to 0; hardware computes actual parity)
Bits [31:28] = ignored in SPDIF mode
```

The SAI in SPDIF mode cannot synchronize with another sub-block (SYNCEN must be 00). Each SPDIF TX stream requires an independent SAI sub-block with its own clock. However, because all SAI sub-blocks share the same kernel clock (PLL2_P = 49.152 MHz) and each has the same integer divider (49.152 / 3.072 = 16, so `MCKDIV=4` for internal clock doubling), their DMA interrupt cadences are phase-locked in steady state even without explicit synchronization.

**Clock for SPDIF TX:** SAI SPDIF bit clock = 64 × Fs = 3.072 MHz at 48 kHz. SAI kernel clock = 49.152 MHz. Divider = 49.152 / (2 × 3.072) = 8. Set `MCKDIV = 4` (the internal divider gives SAI_CLK = kernel/2/MCKDIV = 49.152/2/4 = 6.144 MHz bit clock, which after BMC halving gives 3.072 MHz symbol rate). Double-check with RM0468 §34.4.4 (SAI clock generator).

**4 parallel SPDIF streams on H723:** SAI1_A, SAI1_B, SAI2_A, SAI2_B each in SPDIF PRTCFG=10 asynchronous mode, all fed from PLL2_P kernel clock, all with the same divider. DMA interrupt from SAI1_A fires first (by software-start order) and acts as the timing master to fill all four DMA ping-pong buffers.

**CPU cost:** Near zero. The DSP pipeline produces `int32_t` samples in `buf_out[]`. The STM32 port of `spdif_update_subframe` becomes a simple copy:

```c
// STM32 version — no BMC table needed
static inline void spdif_update_subframe_h7(uint32_t *dst, int32_t sample,
                                             uint8_t ch_status_bit) {
    uint32_t w = (uint32_t)(sample >> 8) & 0x00FFFFFF;  // 24-bit audio
    w |= ch_status_bit << 26;  // C bit
    // V=0 (valid), U=0, P=0 (hardware computes)
    *dst = w;
}
```

This is a register-pressure-free 3-instruction sequence per sample, negligible vs the DSP workload.

**Has anyone done this on H7?** The ST community thread "STM32F767 SAI SPDIF Output" confirmed successful SPDIF TX at 192 kHz via SAI SPDIF mode with DMA. The Linux kernel ASoC STM32 SAI driver has had IEC 60958 SPDIF TX support since 2017 (RFC PATCH "[ASoC: stm32: sai: Add support of S/PDIF playback](https://www.mail-archive.com/linux-kernel@vger.kernel.org/msg1613697.html)"). This is well-validated at the silicon level.

### Sample alignment across SPDIF + I2S

Because SPDIF sub-blocks must be asynchronous masters, they cannot slave to the I2S master in hardware. The alignment strategy is:

1. Pre-load all DMA buffers with silence before starting any peripheral.
2. Start peripherals in a fixed sequence within one critical section: SAI1_A (I2S master) → SAI1_B → SAI2_A → SAI2_B in rapid succession (< 1 µs apart).
3. The single clock source (PLL2_P) means all SAI clock generators step at the same rate. Their relative phase jitter is bounded by the start-sequence propagation time (~10 ns), which is < 1 sample period at 48 kHz (20.8 µs). This is the same alignment guarantee the RP2040 `audio_spdif_enable_sync()` provides.
4. In the audio callback, always fill all output DMA buffers before re-enabling DMA for any of them. This is the `complete_pipeline_reset()` guarantee the CLAUDE.md constraint requires.

---

## 7. SPDIF Input

### SPDIFRX peripheral

The STM32H723 has a dedicated SPDIFRX peripheral (RM0468, Chapter 35). This is a significant upgrade over the PIO-based receiver.

**Hardware capability:**
- Decodes IEC 60958 / IEC 61937 SPDIF input at up to 192 kHz
- Automatically recovers the symbol clock via a 4× oversampling FSM; no external comparator or Schmitt trigger needed beyond a 75Ω termination and level shifter (see AN5073)
- Outputs decoded data, extracted clock (spdifrx_symb_ck), channel status bits, and user bits via separate DMA streams

**Output word format (SPDIFRX_DR register, DMA_SPDIFRX_DT stream):**

```
Bits [23:0]  = 24-bit audio sample (decoded, LSB-first)
Bit  [24]    = parity error flag
Bit  [25]    = V bit (validity)
Bit  [26]    = U bit (user data)
Bit  [27]    = C bit (channel status)
Bits [30:28] = PT[2:0] (preamble type: 0=M, 1=W, 2=B)
Bit  [31]    = 0
```

To extract the 24-bit sample: `sample = (int32_t)((DR << 8) >> 8)` (sign-extend bit 23). This is directly compatible with the project's 24-bit sample representation after sign extension.

**Channel status DMA (DMA_SPDIFRX_CS):** delivers the 192-bit IEC 60958 channel status word. This replaces `spdif_rx_get_channel_status()` from the elehobica library. All the existing channel-status decode logic (sample rate detection, byte 3/4 fields) is preserved.

**Recovered sample rate:** `spdifrx_symb_ck` is exposed as a clock output. You cannot read a frequency counter from the MCU side directly, but SPDIFRX_SR.FERR and SPDIFRX_SR.SYNCD indicate lock/unlock status, and the rate can be inferred from the channel status byte 3. The project's existing `audio_state.freq` detection from channel status works unchanged.

### ASRC integration

**Recommendation: keep the existing ASRC (windowed-sinc + cubic Hermite) running on top of SPDIFRX output, not in hardware-synchronous mode.** Reasons:

1. The SPDIFRX recovered clock cannot drive the SAI TX clocks—there is no internal routing between `spdifrx_symb_ck` and PLL fractional dividers on H723. Hardware synchronous operation would require an external VCXO or I2S reclocking chip.
2. The ASRC already works. Its 31.32 phase accumulator handles all rate families. The only change needed is the input: instead of reading from the elehobica ring buffer, read from the SPDIFRX DMA circular buffer.
3. SPDIFRX output has lower jitter than the PIO-based receiver (hardware oversampling FSM vs software PIO interrupt latency). The ASRC's ratio is updated from channel status byte 3 + a fill-level trim, unchanged.

The PIO clock-servo mode (`spdif_use_resampler=0`) has no direct equivalent on H7 (you cannot tune PLL2_P at 1 frac-unit resolution in real-time while feeding audio). Drop the servo mode for the H7 port; ASRC-only is the correct design.

---

## 8. PDM Output

**SAI does not support PDM output (transmit).** SAI PDM mode is receive-only (input from PDM microphones). This is confirmed in the STM32H7 SAI training slides: "PDM microphone interface supported" for input; no PDM TX is mentioned.

Options for PDM output on H7:

| Approach | Notes |
|---|---|
| **SPI in TX-only mode** (recommended) | Configure SPI as transmitter only, MOSI = PDM data, SCLK = PDM clock (PDM_OVERSAMPLE × Fs). DMA continuous circular from a sigma-delta modulated buffer. Identical bit-stream to the RP PIO approach. Most STM32H7 SPI instances support 550 MHz / 256 / 48000 = 44.8 divider → use SPI baud rate prescaler to get 12.288 MHz PDM clock. |
| TIM + DMA bit-bang | More complex setup, no advantage over SPI |
| External PDM DAC (MAX98357, UDA1334) | Removes MCU PDM generation; adds BOM but simplifies firmware |

For the SPI approach, use SPI1 or SPI4 (both have DMA1/2 access). Configure for 8-bit or 16-bit frames; 16-bit is more efficient (DMA transfers twice as many PDM bits per burst). The sigma-delta modulator output (`pdm_generator.c`) is platform-agnostic—it produces a `uint32_t` bitstream. Wrap it in a DMA-refill callback identical to the existing `pdm_dma_handler`.

---

## 9. DMA Architecture

### Overview table (H723, RM0468 §17)

| DMA | Domain | Bus master access | FIFO | DMAMUX | Peripherals served |
|---|---|---|---|---|---|
| DMA1 | D2 | AXI SRAM, SRAM1/2/3, Flash; **NOT** DTCM/ITCM | Yes (4-word) | DMAMUX1 ch 0-7 | SAI1, SAI2, SPI1/2/3, I2C1/2/3, USART, TIM, etc. |
| DMA2 | D2 | Same as DMA1 | Yes | DMAMUX1 ch 8-15 | Overlapping set; some requests shared with DMA1 |
| BDMA | D3 | **SRAM4 only** (+ backup SRAM) | No | DMAMUX2 ch 0-7 | SAI4, LPUART, I2C4, SPI6, ADC3 |
| MDMA | D1 | All memories incl. DTCM/ITCM, external RAM | Yes | N/A (direct) | Primarily memory-to-memory, QUADSPI, FMC |

**DMAMUX1 request IDs for audio peripherals (H723, RM0468 Table 121):**

| Request | ID |
|---|---|
| SAI1_A | 87 |
| SAI1_B | 88 |
| SAI2_A | 89 |
| SAI2_B | 90 |
| SPDIFRX_DT | 93 |
| SPDIFRX_CS | 94 |
| SPI1_TX | 38 |

**Recommended DMA channel assignment:**

| Channel | DMA | Stream | Peripheral | Buffer location |
|---|---|---|---|---|
| SAI1_A TX (SPDIF out 1) | DMA1 | S0 | SAI1_A | AXI SRAM, non-cacheable |
| SAI1_B TX (SPDIF out 2) | DMA1 | S1 | SAI1_B | AXI SRAM, non-cacheable |
| SAI2_A TX (SPDIF out 3) | DMA1 | S2 | SAI2_A | AXI SRAM, non-cacheable |
| SAI2_B TX (SPDIF out 4) | DMA1 | S3 | SAI2_B | AXI SRAM, non-cacheable |
| SPDIFRX DT RX | DMA2 | S0 | SPDIFRX | AXI SRAM, non-cacheable |
| SPDIFRX CS RX | DMA2 | S1 | SPDIFRX | AXI SRAM, non-cacheable |
| SPI1 TX (PDM) | DMA2 | S2 | SPI1 | AXI SRAM, non-cacheable |

With this layout, DMA1 handles all output audio and DMA2 handles all input. Interrupt priorities: SAI1_A DMA half-complete = highest audio IRQ priority; USB = lower (it fills the source buffer that SAI1_A drains).

### Cache coherency rules summary

| Memory | Cached? | Required action for DMA TX | Required action for DMA RX |
|---|---|---|---|
| DTCM (0x20000000) | No (TCM) | None (but DMA1/2 cannot access it) | N/A |
| AXI SRAM with non-cacheable MPU | No | None | None |
| AXI SRAM with write-back MPU | Yes | `SCB_CleanDCache_by_Addr` before DMA start | `SCB_InvalidateDCache_by_Addr` after DMA completes |
| SRAM1/2/3 non-cacheable MPU | No | None | None |
| SRAM4 (BDMA) | No (no cache path to D3) | None | None |

**Recommendation: allocate all audio DMA ping-pong buffers in a dedicated non-cacheable MPU region in AXI SRAM.** This eliminates all cache maintenance in the audio callback—exactly what you want at 48 kHz interrupt rate. Keep the DSP computation buffers (delay lines, `buf_out`, EQ state arrays) in a separate cached write-back region in AXI SRAM for best CPU performance.

---

## 10. Storage / Preset System

### The 128 KB sector problem

The STM32H723 has 8 × 128 KB flash sectors (1 MB total, single bank). The existing preset system uses 12 × 4 KB sectors (48 KB). There is no sub-sector erase—you cannot erase 4 KB; the minimum is 128 KB. One 128 KB sector erase takes approximately **800 ms** based on community reports ([Zephyr discussion #59714](https://github.com/zephyrproject-rtos/zephyr/discussions/59714)). During erase, code execution from flash must be stalled unless code runs from ITCM or SRAM.

**Flash sector erase kills the audio path.** On the RP2040, the DSPi already handles this by executing the flash code from RAM and muting audio for ~45 ms. On H7 the erase is ~18× longer (800 ms vs 45 ms).

### Decision (2026-05-07): straight port to on-board W25Q64, no filesystem

The WeAct board has an **8 MB W25Q64 SPI flash on SPI3** (PB3/PB4/PD7/PD6). Its erase granularity is **4 KB** — *identical* to the existing RP layout, so no redesign is needed. We do a 1:1 port:

- Keep the existing 12-sector × 4 KB preset layout (`PresetSlot`, `dir_cache`, `slot_buf`, `write_buf`).
- Map the original "flash offset" addresses straight to W25Q64 byte offsets — same arithmetic, different physical store.
- Replace the two pico-sdk call sites in `flash_storage.c` with a thin W25Q SPI driver: `w25q_sector_erase_4k(addr)` and `w25q_page_program(addr, buf, n)`. Page = 256 bytes (W25Q standard), sector = 4 KB (matches existing assumptions).
- 4 KB sector erase ≈ 45–100 ms typical; same audio-mute window the RP build already implements via `preset_loading`. No timing redesign needed.
- W25Q64 endurance = 100,000 erase cycles per sector. With 10 user-writable preset slots and a typical user save rate, this lasts decades. No wear leveling needed.

**No LittleFS, no internal-flash workaround, no on-board OSPI flash usage.** The OSPI flash pins are reclaimed for SAI2 (Section 0.4). The internal STM32 flash holds only firmware code — preset data lives entirely on external SPI flash.

The `flash_storage.c` portability surface is clean: it uses `flash_range_erase()` and `flash_range_program()` from pico-sdk. Those two call sites get a W25Q-driven HAL/LL SPI replacement. The directory cache, slot buffer, version-migration code, and all preset parse/serialize logic is platform-agnostic and ports unchanged.

---

## 11. Single-Core Performance Budget

### Raw IPC comparison

| Core | Clock | DMIPS/MHz | Total DMIPS |
|---|---|---|---|
| Cortex-M7 (H723) | 550 MHz | ~2.14 | ~1177 |
| Cortex-M33 (RP2350) | 307 MHz per core × 2 | ~1.5 | ~922 |
| Cortex-M0+ (RP2040) | 133 MHz × 2 | ~0.95 | ~253 |

The M7@550 MHz provides approximately **1.28× the combined throughput of dual M33@307 MHz** and roughly **4.7× dual M0+@133 MHz** in raw DMIPS. The M7's dual-issue pipeline and 32 KB I/D cache further widen the gap for cache-resident DSP loops.

### DSP workload estimate

The audio engine at 48 kHz / 192-sample blocks / 11 channels:

- **Biquad EQ:** 10 bands × 11 channels = 110 biquads per block. At ~6 cycles/biquad on M7 (CMSIS-DSP `arm_biquad_cascade_df2T_f32` benchmark: ~5.4 cycles/sample for TDF2 float on M7), 110 × 192 × 6 = ~127,000 cycles = 0.23 ms at 550 MHz.
- **Leveller, loudness, crossfeed, matrix mixer:** estimate 50,000 cycles combined = 0.09 ms.
- **ASRC (192 samples, 13-tap):** 192 × 13 × 4 = 9,984 MAC operations; at ~3 cycles/MAC on M7 = ~30,000 cycles = 0.054 ms.
- **SPDIF subframe packing (4 streams × 384 samples):** 1536 iterations × ~5 cycles = 7680 cycles = 0.014 ms.
- **Total DSP per 192-sample block:** ~215,000 cycles ≈ 0.39 ms.
- **Available budget per 192-sample block at 48 kHz:** 192/48000 × 1000 = 4 ms.
- **CPU utilization:** ~10% at 48 kHz.

At 96 kHz (192-sample blocks, 2× call rate): ~20% CPU utilization. You have substantial headroom.

**No escape hatch is needed.** However, for future 9-channel 96 kHz or additional SVF passes, these optimizations are available in priority order:

1. **CMSIS-DSP biquad cascade** (`arm_biquad_cascade_df2T_f32`): uses VLDR/VSTD + VMLA intrinsics, ~20% faster than naive C float on M7.
2. **ITCM placement** (`__attribute__((section(".itcm_text")))`): reduces I-cache misses for tight DSP loops; the most impactful optimization for M7 with large code.
3. **DSP intrinsics** (`__SMLALD`, `__SMULL`): for fixed-point paths only; not relevant since the H7 port will use float throughout (like RP2350).
4. **Block size increase** (384 or 512 samples): reduces ISR overhead amortization cost; tradeoff is increased latency.

---

## 12. I2C Target

The STM32H7 I2C peripheral in slave mode differs from RP2040's in one important way: **the address match fires the ADDR interrupt and holds SCL low until the CPU clears the ADDR flag** (RM0468 §45.4.4). On RP2040, the I2C hardware handles the ACK/stretch autonomously with less ISR latency sensitivity.

**Gotchas specific to H723 I2C slave:**

1. **ADDR ISR latency:** if the ADDR flag is not cleared within approximately one byte period (~10 µs at 400 kHz), the master may time out. The ISR must be prioritized appropriately and must not be masked when the audio SAI DMA IRQ fires.

2. **HAL_I2C_Mem_Read_DMA known bug:** on STM32H723 specifically, `HAL_I2C_Mem_Read_DMA` returns success but fails to execute the repeated start ([community report](https://community.st.com/t5/stm32-mcus-embedded-software/hal-i2c-mem-read-dma-not-working-on-stm32h723/td-p/591558)). Use blocking `HAL_I2C_Mem_Read` or `HAL_I2C_Slave_Seq_Receive_DMA` with the sequential API instead.

3. **Silicon errata:** some STM32H7 revisions have an I2C slave DMA stall bug (SCL held low after a DMA transfer completes). Check the errata sheet for the specific silicon revision of your board.

4. **NOSTRETCH mode:** leave NOSTRETCH=0 (clock stretching enabled). The hardware stretches SCL while the CPU processes the address match and prepares the data register.

**Implementation:** use `HAL_I2C_EnableListen_IT()` + `HAL_I2C_AddrCallback()` / `HAL_I2C_SlaveRxCpltCallback()` pattern. The existing `i2c_target.c` logic (register-map style access) is platform-agnostic; replace only the 3 HAL call sites in the init and ISR glue.

---

## 13. Repository Strategy

### Recommendation: monorepo with a hardware abstraction layer split

```
DSPi/
├── firmware/
│   ├── dsp_core/           ← NEW: portable DSP algorithms (~60% of LOC)
│   │   ├── dsp_pipeline.c/h
│   │   ├── audio_pipeline.c/h
│   │   ├── leveller.c/h
│   │   ├── loudness.c/h
│   │   ├── crossfeed.c/h
│   │   ├── resampler.c/h
│   │   ├── resampler_taps.h
│   │   └── config_portable.h  ← MAX_BANDS, MatrixMixer, Biquad structs
│   │
│   ├── usb_class/          ← NEW: portable UAC1 TinyUSB class driver
│   │   ├── usb_audio.c/h       (platform-agnostic UAC1 logic)
│   │   ├── usb_descriptors.c/h
│   │   ├── vendor_commands.c/h
│   │   ├── bulk_params.c/h
│   │   └── notify.c/h
│   │
│   ├── DSPi/               ← existing RP2040/RP2350 platform layer (unchanged)
│   │   ├── main.c
│   │   ├── audio_input.c/h    ← platform HAL wrapper
│   │   ├── flash_storage.c/h  ← platform HAL wrapper
│   │   ├── pdm_generator.c/h  ← platform-specific (PIO)
│   │   └── CMakeLists.txt
│   │
│   ├── STM32/              ← NEW: STM32H723 platform layer
│   │   ├── main_stm32.c
│   │   ├── audio_input_stm32.c   ← SPDIFRX + ASRC glue
│   │   ├── audio_output_stm32.c  ← SAI1/2 + DMA management
│   │   ├── flash_storage_stm32.c ← SPI flash + LittleFS
│   │   ├── pdm_generator_stm32.c ← SPI + DMA sigma-delta output
│   │   ├── usb_platform_stm32.c  ← TinyUSB dwc2 + FIFO init
│   │   ├── i2c_target_stm32.c    ← STM32 I2C slave HAL glue
│   │   ├── clock_config.c        ← RCC + PLL + MPU init
│   │   └── CMakeLists.txt
│   │
│   └── pico-extras/        ← existing (SPDIF/I2S PIO libraries; unused by STM32)
│
├── vendor/
│   ├── stm32-cmake/        ← git submodule
│   ├── STM32CubeH7/        ← git submodule (or sparse checkout)
│   └── tinyusb/            ← git submodule (shared with RP build)
│
└── CMakeLists.txt          ← top-level, selects platform
```

**Platform abstraction boundary:** define a thin HAL interface in `dsp_core/platform_hal.h`:

```c
// Audio output: deliver one block to all output streams
void platform_audio_output_submit(int32_t (*buf_out)[AUDIO_BUFFER_SAMPLES],
                                   uint32_t n_outputs, uint32_t n_samples);
// Input: retrieve one block from the active input source
uint32_t platform_audio_input_consume(int32_t *buf_l, int32_t *buf_r,
                                       uint32_t n_samples);
// Flash
int platform_flash_write(uint32_t sector, const void *data, size_t len);
int platform_flash_read(uint32_t sector, void *data, size_t len);
```

This boundary means `dsp_pipeline.c`, `leveller.c`, `resampler.c`, all EQ code, and the USB class driver (`usb_audio.c`) never include pico-sdk headers and compile cleanly on any ARM toolchain.

The refactor is incremental: start by moving files into `dsp_core/` and verifying the RP builds still pass before writing a single line of STM32 code.

---

## 14. Milestone Plan

### M0 — Dev board + toolchain (estimated: 1 day)

- Install arm-none-eabi-gcc 13+, CMake 3.22+, OpenOCD or J-Link GDB Server on macOS.
- Clone NUCLEO-H723ZG starter project (ST's NUCLEO-H723ZG BSP example or a minimal hand-written CMakeLists using stm32-cmake).
- LED blink from `HAL_GPIO_TogglePin`, UART stdio via `HAL_UART_Transmit` on USART3 (ST-LINK virtual COM).
- SWD debug working in VS Code with cortex-debug.
- RTT `SEGGER_RTT_printf` working in DTCM.
- **Done when:** LED blinks at 1 Hz, printf appears over UART, J-Link RTT channels show output.

### M1 — Clock tree to spec (estimated: 0.5 days)

- Configure RCC in `clock_config.c`: PLL1 to 550 MHz, PLL2_P to 49.152 MHz (or 98.304 MHz), HSI48 + CRS for USB.
- Route PLL2_P to MCO1 pin (PA8 on NUCLEO).
- **Done when:** scope on MCO1 shows 49.152 MHz ±100 ppm; `SystemCoreClock` reads 550 MHz via DWT.

### M2 — TinyUSB vendor class, USB FS enumeration (estimated: 1 day)

- Add TinyUSB as submodule; configure `tusb_config.h` for dwc2 FS (H723).
- Set FIFO sizes per Section 4.
- Register a `usbd_app_driver_get_cb` stub that exposes one vendor bulk IN/OUT endpoint.
- USB HID descriptor ping-pong echo test.
- **Done when:** device enumerates as vendor class on macOS and Windows; `lsusb` shows correct VID/PID; echo test passes at full bulk rate.

### M3 — UAC1 IN (USB audio source), silence (estimated: 1–2 days)

- Port `usb_audio.c` custom class driver: replace pico-sdk audio pool calls with stub that delivers silence.
- Add UAC1 descriptors (`usb_descriptors.c`); verify host driver selection.
- Implement SOF IRQ callback that returns nominal feedback `(48000 << 14) / 1000` = `0xC0000`.
- **Done when:** device enumerates as UAC1 audio on macOS and Windows; host audio apps can select the device; output is silence.

### M4 — SAI1_A I2S master TX, 1 kHz tone (estimated: 1–2 days)

- Configure SAI1_A in I2S master mode: PLL2_P kernel clock, 24-bit left-justified, MCLK output enabled.
- DMA1 stream 0 circular ping-pong from AXI SRAM non-cacheable buffer.
- MPU region for DMA buffers configured.
- DSP fill callback generates a 1 kHz sine wave.
- **Done when:** scope shows 49.152 MHz MCLK, 3.072 MHz BCK, 48 kHz LRCLK; oscilloscope or DAC shows 1 kHz audio.

### M5 — SAI1_B synchronized slave, sample-aligned second channel (estimated: 1 day)

- Configure SAI1_B as internal synchronous slave to SAI1_A (SYNCEN=01).
- Arm both DMA channels; start SAI1_A after SAI1_B DMA is armed.
- Verify phase alignment: both channels output identical 1 kHz sines; measure phase offset on scope.
- **Done when:** phase offset between SAI1_A and SAI1_B outputs is < 1 sample (20.8 µs at 48 kHz) as measured on scope.

### M6 — Portable DSP core plumbed in (estimated: 2–3 days)

- Refactor source tree per Section 13: move `dsp_pipeline.c`, EQ, leveller, matrix mixer into `dsp_core/`.
- Verify RP2040 and RP2350 builds still pass with zero regressions.
- Wire the DSP pipeline into the STM32 SAI DMA callback: USB → ring buffer → DSP block → SAI ping-pong buffer.
- Remove the dual-core handshake (`Core1EqWork`) for the STM32 target; run all EQ in a single sequential pass.
- **Done when:** USB audio plays through DSP (including EQ bands) out of SAI1_A on H723; RP builds still pass.

### M7 — SPDIFRX input + ASRC (estimated: 2–3 days)

- Configure SPDIFRX peripheral; connect DMA_SPDIFRX_DT circular buffer.
- Port `audio_input_stm32.c` to read from SPDIFRX DMA ring, format-convert (24-bit + sign extension), feed into existing ASRC ring buffer.
- Port ASRC ratio update to use channel-status byte 3 from DMA_SPDIFRX_CS.
- Implement lock/unlock detection via SPDIFRX_SR.SYNCD.
- **Done when:** SPDIF source plays through the DSP pipeline; ASRC handles 44.1 kHz and 48 kHz input; lock/unlock callbacks fire correctly.

### M8 — SPDIF TX via SAI, single instance (estimated: 1–2 days)

- Configure SAI1_A in SPDIF protocol mode (PRTCFG=10).
- Write `spdif_update_subframe_h7()` (the 3-instruction version from Section 6).
- Port channel status byte array population (IEC 60958-3 byte 4 = 0x0B for 24-bit).
- DMA ping-pong from AXI SRAM non-cacheable buffer.
- **Done when:** SPDIF output received correctly by a receiver (TV, DAC, or Toslink analyzer); 1 kHz tone verified; channel status bits correct.

### M9 — Multi-instance SPDIF TX, all sample-aligned (estimated: 1–2 days)

- Bring up SAI1_B, SAI2_A, SAI2_B in SPDIF mode.
- Implement synchronized start sequence (all DMA armed before any SAI enabled).
- Verify inter-output alignment: all 4 SPDIF streams carry the same 1 kHz tone with < 1 sample phase offset.
- **Done when:** 4 SPDIF receivers all show identical audio; any source switch (preset load, rate change) triggers `complete_pipeline_reset()` equivalent that preserves alignment.

### M10 — PDM output (estimated: 1 day)

- Configure SPI1 in transmit-only mode at 12.288 MHz (= 256 × 48 kHz).
- DMA2 stream 2 circular from a 16-bit PDM bitstream buffer in AXI SRAM non-cacheable.
- Port `pdm_generator.c` sigma-delta loop to fill the SPI DMA buffer; this code is already platform-agnostic.
- **Done when:** subwoofer output audible and correlated with the SPDIF main outputs.

### M11 — Preset storage redesign (estimated: 2 days)

- Add W25Q SPI flash driver (or reuse WeAct board's existing QSPI/OctoSPI flash via HAL_OSPI).
- Port LittleFS (250-line block driver wrapping SPI erase/program/read).
- Replace `flash_range_erase/program` call sites in `flash_storage.c` with LittleFS file API.
- Bump `SLOT_DATA_VERSION` and add migration path from internal flash on first boot.
- **Done when:** 10 preset slots save/load correctly across power cycles; audio does not glitch during save.

### M12 — I2C target (estimated: 0.5 days)

- Configure I2C1 or I2C2 as slave with HAL listen + sequential API.
- Port `i2c_target.c` register-map handler to H7 ISR callbacks.
- **Done when:** I2C master (host MCU) can read/write registers via the existing command set.

### M13 — USB feedback PID, ASRC integration, full parity (estimated: 3–5 days)

- Wire the SOF IRQ feedback computation: replace PIO DMA word counter with SAI1 DMA transfer counter via `DMA_S0NDTR`.
- Tune `FILL_SERVO_KP` for the H7 ring buffer depth.
- Full vendor command pass: verify all `REQ_*` commands work (most require no changes).
- Loudness, leveller, crossfeed all verified with automated test tracks.
- System status packet (`REQ_GET_STATUS`): replace `cpu1_load` with a single CPU load estimate.
- **Done when:** USB audio plays at 48 kHz and 96 kHz without glitches over 24-hour soak test; host audio metering app shows correct levels; all vendor commands return expected responses.

### Milestone order rationale

M0–M3 are infrastructure; none is research-blocked. M4–M5 are the hardest mechanical tasks (cache coherency, DMA stall debugging). M6 is the most strategically important: it validates the monorepo split and ensures no regressions on RP. M7–M9 can parallelize after M6. M11 can be done before M7 if flash storage is needed sooner. M13 is last because it requires all prior subsystems working simultaneously.

---

## 15. Open Risks / Unknowns — Validate Early

1. **HSE crystal frequency on WeAct board:** The GitHub README does not document the HSE crystal value. **Validate before ordering.** If it is 8 MHz or 25 MHz instead of 24.576 MHz, the PLL plan changes and fractional mode is required for audio clocks.

2. **SAI4 BDMA + SRAM4 pin count:** Verify that you actually need SAI4. If SAI1 + SAI2 (4 sub-blocks total) suffice, SAI4's BDMA constraint is a non-issue. Only becomes a problem if you need a 5th or 6th independent audio DMA stream.

3. **SAI SPDIF mode simultaneous instances:** The RM describes PRTCFG=10 for a single sub-block. Confirm empirically that SAI1_A and SAI2_A can both be in SPDIF mode simultaneously with independent MCLK generators without one interfering with the other via the shared PLL2 kernel clock. The theory says yes; validate with silicon at M8–M9.

4. **Cache coherency with TinyUSB dwc2:** The dwc2 uses AHB DMA internally. If the USB packet buffer is in cached AXI SRAM, the CPU's written ISO OUT data may not be visible to DMA. This is the most common H7 USB audio failure mode. The non-cacheable MPU region approach in Section 5 prevents it, but verify with a cache-enabled test at M2 before M3.

5. **Flash erase time:** The 800 ms figure is from community reports on H743. Verify with actual H723 silicon (could vary). If the erase is longer than 800 ms, the internal-flash preset-save option becomes completely unusable for live use.

6. **TinyUSB dwc2 FS FIFO conflict with vendor EP:** the FIFO budget in Section 4 is tight when adding a vendor bulk IN endpoint for notifications (existing `VENDOR_EP_IN = 0x83`). A 64-byte bulk EP needs at least 2 × 64/4 = 32 additional TX FIFO words. This exceeds the 320-word pool with the 96 kHz ISO FIFO sizes. At M2 validate: either reduce ISO OUT FIFO (acceptable if running 48 kHz only), or disable the vendor bulk IN EP and fall back to polling via vendor control transfers.

7. **I2C DMA stall errata:** check the H723 errata sheet (ES0489) for I2C DMA-related errata before M12. The H743 had a documented stall on repeated-start that required a HAL workaround.

8. **PDM SPI output EMI:** at 12.288 MHz, the SPI MOSI line is a high-frequency single-ended signal. Verify layout rules (controlled impedance, no loops) before PCB tape-out. On the NUCLEO evaluation, use an external buffer or just probe the digital output—don't connect a raw SPI pin directly to a PDM microphone DAC without a series termination resistor.

9. **ASRC sample rate detection timing on SPDIFRX lock:** the elehobica library had a specific lock debounce (`SPDIF_RX_LOCK_DEBOUNCE_MS = 100`). Verify that SPDIFRX_SR.SYNCD stabilizes within a similar window before unmuting; ST's AN5073 does not specify a guaranteed lock time.

10. **Platform ID for vendor command REQ_GET_PLATFORM:** currently returns `PLATFORM_RP2040 = 0` or `PLATFORM_RP2350 = 1`. Add `PLATFORM_STM32H723 = 2` and update any host app that inspects this.

---

## Go / No-Go Verdict

**Go.** The port is well-scoped and the hardware is capable:

- The M7@550 MHz comfortably handles the full 9-channel DSP workload single-core.
- SAI SPDIF protocol mode eliminates the largest platform-specific component (PIO BMC encoder).
- SPDIFRX is more capable than the PIO receiver.
- ~60% of the firmware is already portable.
- TinyUSB on dwc2 is stable for UAC1 FS applications as of 2025.

The three hardest problems are engineering, not architectural unknowns:

1. D-cache coherency management for DMA buffers (solved with MPU non-cacheable regions, Section 5/9).
2. Flash sector size mismatch (solved with external SPI flash + LittleFS, Section 10).
3. SAI4/BDMA constraint (avoided by using only SAI1/SAI2, Section 5).

**Recommended first concrete step:** order one NUCLEO-H723ZG board and one J-Link EDU Mini. Confirm your arm-none-eabi-gcc toolchain is ≥ 13.2 (`arm-none-eabi-gcc --version`). Create the `firmware/STM32/` directory, add the stm32-cmake and STM32CubeH7 submodules, and work through M0 (LED blink) this week.

---

Sources:
- [NUCLEO-H723ZG product page](https://www.st.com/en/evaluation-tools/nucleo-h723zg.html)
- [STM32H723 datasheet DS13313 Rev 5](https://www.st.com/resource/en/datasheet/stm32h723zg.pdf)
- [WeActStudio/WeActStudio.MiniSTM32H723 GitHub](https://github.com/WeActStudio/WeActStudio.MiniSTM32H723)
- [ObKo/stm32-cmake GitHub](https://github.com/ObKo/stm32-cmake)
- [STMicroelectronics/STM32CubeH7 GitHub](https://github.com/STMicroelectronics/STM32CubeH7)
- [TinyUSB custom class driver Issue #467](https://github.com/hathach/tinyusb/issues/467)
- [TinyUSB changelog v0.20.0](https://docs.tinyusb.org/en/latest/_sources/info/changelog.rst.txt)
- [TinyUSB DWC2 driver DeepWiki](https://deepwiki.com/hathach/tinyusb/4.1-dwc2-usb-controller)
- [TinyUSB better ISO FIFO allocation Issue #540](https://github.com/hathach/tinyusb/issues/540)
- [dragonman225/stm32f469-usbaudio GitHub](https://github.com/dragonman225/stm32f469-usbaudio)
- [ST AN5073 — Receiving S/PDIF audio stream with STM32F4/F7/H7](https://www.st.com/resource/en/application_note/an5073-receiving-spdif-audio-stream-with-the-stm32f4f7h7-series-stmicroelectronics.pdf)
- [STM32H7 Peripheral SAI training slide deck](https://www.st.com/content/ccc/resource/training/technical/product_training/group0/d3/c0/b0/0e/fe/eb/40/a9/STM32H7-Peripheral-Serial-Audio-Interface_SAI/files/STM32H7-Peripheral-Serial-Audio-Interface_SAI.pdf/_jcr_content/translations/en.STM32H7-Peripheral-Serial-Audio-Interface_SAI.pdf)
- [STM32H7 SAI synchronization clarification — ST Community](https://community.st.com/t5/stm32-mcus-products/stm32h7-sai-synchronization-clarification/td-p/636823)
- [SAI inter-instance synchronization Linux kernel patch](https://patchwork.kernel.org/patch/10016817/)
- [ST AN4839 — Level 1 cache on STM32F7/H7](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf)
- [STM32H7 DMA not working guide — ST Community](https://community.st.com/t5/stm32-mcus/dma-is-not-working-on-stm32h7-devices/ta-p/49498)
- [SAI4 BDMA SRAM4 constraint — ST Community thread](https://community.st.com/t5/stm32-mcus-embedded-software/sai4-receiving-dma-busy-no-buffer-update-stm32h747i-disco/td-p/824999)
- [STM32H7 LittleFS on internal flash Issue #1016](https://github.com/littlefs-project/littlefs/issues/1016)
- [Zephyr STM32H723 UART interrupts disabled during flash — Discussion #59714](https://github.com/zephyrproject-rtos/zephyr/discussions/59714)
- [STM32H7 SAI SPDIF Linux RFC patch](https://www.mail-archive.com/linux-kernel@vger.kernel.org/msg1613697.html)
- [STM32H723 I2C SAI issue — ST Community](https://community.st.com/t5/stm32-mcus-products/stm32h723-i2s-or-sai/td-p/681752)
- [HAL_I2C_Mem_Read_DMA bug on STM32H723](https://community.st.com/t5/stm32-mcus-embedded-software/hal-i2c-mem-read-dma-not-working-on-stm32h723/td-p/591558)
- [SAI1-A and SAI4-A LQFP100 sync constraint — CubeMX community](https://community.st.com/t5/stm32cubemx-mcus/sai1-a-and-sai4-a-don-t-have-synchronous-slave-option-in-lqfp100/td-p/399060)
- [STM32H7 PLL 49.152 MHz community thread](https://community.st.com/t5/stm32-mcus-products/stm32h7-clock-scheme-49-152mhz-to-24mhz-and-max-mcu/td-p/702503)
- [CMSIS-DSP biquad cascade documentation](https://arm-software.github.io/CMSIS-DSP/main/group__BiquadCascadeDF1.html)
- [Arm Cortex-M comparison table](https://documentation-service.arm.com/static/6267de1c7e121f01fd22d677)
- [AN5224 — Introduction to DMAMUX for STM32](https://www.st.com/resource/en/application_note/an5224-introduction-to-dmamux-for-stm32-mcus-stmicroelectronics.pdf)
- [STM32H723 ULPI compatible devices — ST Community](https://community.st.com/t5/stm32-mcus-products/stm32h723-ulpi-compatible-devices/td-p/603736)
- [STM32 W25Qxx LittleFS — stm32world.com](https://stm32world.com/wiki/STM32_W25Qxx_LittleFS)
- [stm32-rs/stm32h7xx-hal SAI DMA example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/sai_dma_passthru.rs)
- [SEGGER J-Link product page](https://www.segger.com/products/debug-probes/j-link/)