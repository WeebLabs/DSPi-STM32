# DSPi STM32H723 Port

Bring-up tree for the STM32H723VGT6 (WeAct MiniSTM32H723 V1.2). See
`../../Documentation/Porting/STM32H723_first_steps.md` for the full plan,
pin map, clock plan, and milestone sequence. This README only covers
*how to build and flash*.

## Prerequisites

- `arm-none-eabi-gcc` ≥ 13.2 (`brew install --cask gcc-arm-embedded`)
- `cmake` ≥ 3.20
- `dfu-util` (`brew install dfu-util`) — for USB DFU flashing
- A USB-C cable, no debug probe needed for M0

Submodules (cloned into `vendor/` at the repo root):
- `vendor/stm32-cmake` — CMake glue for STM32 HAL/CMSIS
- `vendor/STM32CubeH7` — ST's HAL/CMSIS source. Nested submodules already
  initialised: `Drivers/CMSIS/Device/ST/STM32H7xx`, `Drivers/STM32H7xx_HAL_Driver`.

If you cloned the repo without `--recurse-submodules`, run:
```
git submodule update --init --depth 1 vendor/stm32-cmake vendor/STM32CubeH7
cd vendor/STM32CubeH7
git submodule update --init --depth 1 \
    Drivers/CMSIS/Device/ST/STM32H7xx \
    Drivers/STM32H7xx_HAL_Driver
cd -
```

## Build

```
cmake -S firmware/STM32 -B build-stm32h723 -DCMAKE_BUILD_TYPE=Release
cmake --build build-stm32h723 -j
```

Outputs land in `build-stm32h723/`:
- `DSPi_stm32h723.elf` (debug-friendly, with symbols)
- `DSPi_stm32h723.bin` (raw, for DFU)
- `DSPi_stm32h723.hex` (Intel HEX, for STM32CubeProgrammer)

## Flash via USB DFU

1. Hold **BOOT0**, tap **NRST**, release **BOOT0** ~0.5 s later. The board
   re-enumerates as `STMicroelectronics STM Device in DFU Mode`.
2. Use the helper script (it waits for DFU enumeration, flashes, and
   issues a USB reset to land cleanly in user firmware):
   ```
   firmware/STM32/scripts/flash.sh
   ```
   Equivalent raw `dfu-util` invocation:
   ```
   dfu-util -a 0 -s 0x08000000:leave -R -D build-stm32h723/DSPi_stm32h723.bin
   ```
   The H7's DFU bootloader (v0x011a) leaves enough USB peripheral state
   behind that `:leave` alone often wedges user firmware until a manual
   NRST press; the `-R` flag adds a USB-level reset that clears that
   residue. If the LED still doesn't come up after the script reports
   complete, tap **NRST** once.
3. Watch USART1 (PA9 TX, PA10 RX) at 115200 8N1 for the heartbeat.
   The on-board `MCO1` pin (PA8) outputs the HSE crystal at 25 MHz — scope
   it to confirm the clock tree is alive.

## What this milestone (M0) verifies

- Toolchain + linker produce a flashable binary
- DFU bootloader entry sequence works on this physical board
- `SystemClock_Config()` reaches 550 MHz SYSCLK without faulting
- PLL2 configures with the FRACN values from §0.5 of the plan
- Heartbeat LED on PB5 toggles at 1 Hz
- USART1 prints frequency report and a tick counter

If any of those fail, debug *before* moving to M1 (clock-tree verification
on a scope). Bring-up problems compound; nail each step before stacking
the next one on top.

## Pin reservations established by M0

| Pin | Function | Locked for |
|---|---|---|
| PA8 | MCO1 → HSE/1 (M0) → PLL2_P later | clock verification |
| PA9 / PA10 | USART1 TX/RX | logging, all milestones |
| PA11 / PA12 | USB FS D-/D+ | hard-wired to USB-C, used from M2 |
| PA13 / PA14 | SWDIO/SWCLK | header P3, used as soon as a probe is wired |
| PB5 | Heartbeat LED | M0 only — repurpose later if a free pin is needed |

All other pins remain available; see plan §0.4 for the full audio map.
