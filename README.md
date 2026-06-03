# DSPi for STM32H723

**DSPi** turns an inexpensive STM32H7 board into a very competent little digital
audio processor. It enumerates as a USB sound card with an on-board DSP engine,
giving you room correction, active crossovers, parametric EQ, time alignment,
loudness compensation, headphone crossfeed, and more — all running on the chip.

This repository is the **STM32H723 port** of DSPi. The original project targets
the Raspberry Pi RP2040/RP2350; this branch carries the same DSP pipeline and the
same USB control protocol onto the STM32H723VGT6 (a 550 MHz single-core
Cortex-M7), using its hardware SAI blocks for audio output and the dedicated
SPDIFRX peripheral for S/PDIF input.

> **Target board:** [WeAct MiniSTM32H723 (V1.2)](https://github.com/WeActStudio/MiniSTM32H7xx)
> — STM32H723VGT6, 25 MHz HSE, on-board W25Q64 SPI flash, USB-C.
> The firmware is hard-coded to this board's pin map and clock tree.

---

## Table of Contents

- [Port Status](#port-status)
- [Key Capabilities](#key-capabilities)
- [Hardware & Pin Map](#hardware--pin-map)
- [Audio Signal Chain](#audio-signal-chain)
- [DSP Features](#dsp-features)
- [Outputs (I2S / S/PDIF)](#outputs-i2s--spdif)
- [S/PDIF Input](#spdif-input)
- [User Presets](#user-presets)
- [Building from Source](#building-from-source)
  - [macOS](#building-on-macos)
  - [Windows](#building-on-windows)
- [Flashing the Firmware](#flashing-the-firmware)
  - [macOS](#flashing-on-macos)
  - [Windows](#flashing-on-windows)
- [Developer Reference](#developer-reference)
  - [USB Control Protocol](#usb-control-protocol)
  - [System Telemetry](#system-telemetry)
  - [Data Structures](#data-structures)
- [Differences from the RP2040/RP2350 Build](#differences-from-the-rp2040rp2350-build)
- [Specifications](#specifications)
- [License](#license)

---

## Port Status

The port is functional and runs the full DSP pipeline end-to-end. The device
enumerates on macOS, Windows, Linux, and iOS as **"Weeb Labs DSPi for STM32"**
(USB `VID 0x2E8B`, `PID 0xFEAA`) and is controlled by the same DSPi Console
application as the Pico builds. Firmware version reported by the device: **1.1.4**.

| Subsystem | State | Notes |
|---|---|---|
| USB Audio (UAC1) input | ✅ Working | 16-bit stereo, 48 kHz + 44.1 kHz, async feedback |
| Audio output (SAI) | ✅ Working | 4 sample-aligned stereo slots; per-slot I2S **or** S/PDIF, switchable at runtime |
| S/PDIF input (SPDIFRX) | ✅ Working | 24-bit end-to-end, 32–192 kHz detect, asynchronous sample-rate conversion |
| Input source switching | ✅ Working | Live USB ↔ S/PDIF |
| Full DSP pipeline | ✅ Working | Preamp, EQ, matrix mixer, crossfeed, leveller, loudness, delay, master volume |
| Preset storage | ✅ Working | 10 slots in on-board W25Q64 SPI flash |
| Diagnostics | ✅ Working | Peak/clip meters, CPU load, buffer stats, USB error counters, S/PDIF starvation |
| PDM subwoofer output | ⛔ Not implemented | The Pico's PIO-driven PDM modulator has no STM32 equivalent; the channel exists in the protocol/UI as a placeholder but produces no audio |
| Runtime output-pin remapping | ⛔ Not applicable | SAI output pins are fixed by the board layout |
| Software bootloader entry | ⛔ Not supported | Use the **BOOT0 + NRST** hardware sequence (see [Flashing](#flashing-the-firmware)) |
| 96 kHz USB input | ⛔ Not supported | USB tops out at 48 kHz; 96/176.4/192 kHz are reachable only via the S/PDIF input |

See [Differences from the RP2040/RP2350 Build](#differences-from-the-rp2040rp2350-build)
for the complete list of where this port diverges from the Pico firmware.

---

## Key Capabilities

* **USB Audio Interface** — Plug-and-play UAC1 sound card under macOS, Windows,
  Linux, and iOS. 16-bit PCM stereo input at 48 kHz or 44.1 kHz, with
  asynchronous SOF-based feedback for drift-free streaming.
* **Four Stereo Outputs (8 channels)** — Two SAI peripherals (SAI1 + SAI4)
  provide four independent, sample-aligned stereo slots. Each slot is switchable
  at runtime between **24-bit I2S** and **24-bit S/PDIF**, so one device can feed
  a mix of I2S DAC chips and Toslink receivers from the same pipeline.
* **S/PDIF Input** — The STM32H723's dedicated SPDIFRX peripheral decodes an
  incoming biphase-mark stream (up to 192 kHz) in hardware, carried at full
  24-bit depth and resampled to the output rate by a polyphase windowed-sinc
  ASRC whose ratio is servoed to the input — no PLL retuning.
* **Per-Channel Preamp** — Independent gain for each USB input channel (L/R),
  applied first in the pipeline.
* **Matrix Mixer** — Route either or both input channels to any output with
  per-crosspoint gain and phase invert.
* **Parametric EQ** — Up to 10 bands per channel with 6 filter types, using a
  single-precision float pipeline with a hybrid SVF/biquad architecture for
  superior low-frequency accuracy.
* **Volume Leveller** — RMS-based, stereo-linked, soft-knee upward compressor
  with optional 10 ms lookahead and a −6 dBFS safety limiter.
* **Loudness Compensation** — Volume-dependent EQ based on the ISO 226:2003
  equal-loudness contours.
* **Headphone Crossfeed** — BS2B crossfeed with interaural time delay; three
  classic presets plus fully custom parameters.
* **Master Volume** — Device-side output ceiling (−128…0 dB with a true-mute
  sentinel) applied at the end of the chain, independent of USB host volume.
  Persists independently of presets (default) or as part of each preset.
* **Per-Output Gain, Mute & Delay** — Independent control per output channel,
  with time alignment up to ~42 ms and automatic latency compensation.
* **10-Slot Preset System** — Save, load, and manage complete DSP
  configurations with user-defined names, stored in the on-board SPI flash.
* **Diagnostics** — Per-channel peak/clip metering, USB PHY error counters,
  buffer-fill statistics, S/PDIF DMA starvation counters, and CPU load.

---

## Hardware & Pin Map

The firmware targets the **WeAct MiniSTM32H723 V1.2** and assumes its on-board
peripherals. The camera and OSPI flash footprints are left unpopulated/disabled,
which frees Port E and Port D pins for the SAI audio outputs.

### Wiring guide

| Function | Pin | AF | Notes |
|---|---|---|---|
| **USB D− / D+** | `PA11` / `PA12` | AF10 | Hard-wired to the on-board USB-C connector |
| **Output Slot 0** (Out 1-2) | `PE6` (SAI1_SD_A) | AF6 | I2S or S/PDIF data |
| **Output Slot 1** (Out 3-4) | `PE3` (SAI1_SD_B) | AF6 | I2S or S/PDIF data |
| **Output Slot 2** (Out 5-6) | `PD11` (SAI4_SD_A) | AF10 | I2S or S/PDIF data |
| **Output Slot 3** (Out 7-8) | `PA0` (SAI4_SD_B) | AF10 | I2S or S/PDIF data |
| **I2S MCLK** (shared) | `PE2` (SAI1_MCLK_A) | AF6 | 256× Fs master clock (12.288 MHz @ 48 kHz) |
| **I2S LRCLK / FS** (shared) | `PE4` (SAI1_FS_A) | AF6 | Frame clock, = sample rate |
| **I2S BCK / SCK** (shared) | `PE5` (SAI1_SCK_A) | AF6 | Bit clock (3.072 MHz @ 48 kHz) |
| **S/PDIF Input** | `PD8` (SPDIFRX_IN) | AF9 | Biphase-mark input (WeAct header P1) |
| **Log UART** | `PA9` / `PA10` (USART1 TX/RX) | AF7 | 115200 8N1 firmware log |
| **SWD** | `PA13` / `PA14` (SWDIO/SWCLK) | AF0 | Debug header P3 |
| **Heartbeat LED** | `PB5` | GPIO | Active-high; wire an LED + series resistor |
| **Preset flash** | on-board W25Q64 on SPI3 | — | No wiring required |
| **BOOT0 / NRST** | on-board buttons | — | DFU entry sequence |

> **Notes.** All four I2S outputs share the single BCK/LRCLK/MCLK clock group on
> `PE5`/`PE4`/`PE2`, so multiple I2S DACs stay phase-locked. In S/PDIF mode a slot
> self-clocks (the clock is embedded in the biphase signal) and needs only its
> data pin. S/PDIF output is a logic-level signal — drive a Toslink transmitter
> (e.g. a Toshiba TX179) or a simple resistor divider into a coax input.
>
> The on-board blue LED (PE3) is repurposed as `SAI1_SD_B`, so the heartbeat
> indicator moved to `PB5`.

---

## Audio Signal Chain

DSPi processes audio in a linear, low-latency pipeline. All stages run on the
single Cortex-M7 core in a single-precision float path.

```
Input:  USB (16-bit PCM stereo, 48 / 44.1 kHz)   — or —   S/PDIF (24-bit, 32–192 kHz → ASRC)
    |
PASS 1: Per-Channel Preamp (independent L/R gain) + USB Volume
    |
PASS 2: Master EQ (10 bands per channel, Left/Right)
    |
PASS 2.5: Volume Leveller (RMS upward compression, optional)
    |
PASS 3: Headphone Crossfeed (BS2B + ITD, optional) + Master Peak Metering
    |
        Loudness Compensation (volume-dependent EQ, optional)
    |
PASS 4: Matrix Mixer (2 inputs × outputs, per-crosspoint gain & phase)
    |
PASS 5: Per-Output EQ → Gain/Mute → Delay → Output Gain × Master Volume
    |
    +-- Out 1-2 --> SAI1_A  (slot 0, I2S or S/PDIF)  — PE6
    +-- Out 3-4 --> SAI1_B  (slot 1, I2S or S/PDIF)  — PE3
    +-- Out 5-6 --> SAI4_A  (slot 2, I2S or S/PDIF)  — PD11
    +-- Out 7-8 --> SAI4_B  (slot 3, I2S or S/PDIF)  — PA0
                  (shared I2S clocks: MCLK PE2 / LRCLK PE4 / BCK PE5)
```

> The device presents 11 logical channels to the Console (2 master + 8 output +
> 1 PDM placeholder) to stay wire-compatible with the RP2350 build. The PDM
> channel is accepted by the protocol but has **no hardware behind it on STM32**
> and produces silence.

### Signal Chain Details

1. **Input.** USB delivers 16-bit PCM stereo at 48/44.1 kHz. The S/PDIF input
   delivers 24-bit audio at any standard rate (32–192 kHz) and is asynchronously
   resampled to the active output rate.
2. **Per-Channel Preamp (PASS 1).** Independent dB gain for the Left and Right
   input channels, applied before all downstream processing.
3. **Master EQ (PASS 2).** Up to 10 bands of parametric EQ per channel
   (peaking, low/high shelf, low/high pass, notch).
4. **Volume Leveller (PASS 2.5).** Optional feedforward, stereo-linked, single-band
   RMS upward compressor with soft knee, configurable speed/ceiling/gate, optional
   10 ms lookahead, and a −6 dBFS gain-reduction safety limiter.
5. **Headphone Crossfeed (PASS 3).** Optional BS2B crossfeed with interaural time
   delay. Master peak metering taps this stage.
6. **Loudness Compensation.** Optional ISO 226:2003 equal-loudness EQ driven by the
   USB host volume position.
7. **Matrix Mixer (PASS 4).** Routes the two input channels to all outputs, each
   crosspoint with enable, gain (−inf…+12 dB), and phase invert.
8. **Output EQ (PASS 5).** Independent 10-band EQ per output channel. Filters below
   Fs/7.5 use SVF topology for low-frequency accuracy; higher frequencies use TDF2
   biquad.
9. **Gain / Mute / Delay.** Per-output gain (−inf…+12 dB), soft mute, and delay up
   to ~42 ms (2048 samples at 48 kHz), with automatic path latency compensation.
10. **Master Volume.** Device-side output ceiling folded into the per-output
    multiplier, independent of USB host volume and DSP processing.
11. **Output encoding.** Each slot emits 24-bit I2S (left-justified, MSB-first,
    32-bit frames) or 24-bit S/PDIF (hardware biphase-mark encoding via the SAI).

---

## DSP Features

The DSP feature set is shared with the RP2350 build and runs unchanged on the
STM32's float pipeline. The summaries below are STM32-accurate; per-feature wire
formats live under [`Documentation/Features/`](Documentation/Features/).

### Matrix Mixer
Routes the USB/S/PDIF stereo input to all output channels. Each crosspoint has
enable/disable, gain (−inf…+12 dB), and phase invert. Each output channel also has
enable (disabled outputs skip all processing to save CPU), gain, mute, and delay.

### Parametric Equalization
10 bands per channel, six filter types (Flat, Peaking, Low Shelf, High Shelf, Low
Pass, High Pass; Notch is also supported in firmware). The float pipeline uses a
hybrid topology: filters below Fs/7.5 (~6.4 kHz at 48 kHz) use the Cytomic SVF
(linear trapezoid) topology for numerical accuracy at low frequencies; higher
frequencies use a transposed-direct-form-II biquad. Flat/bypassed bands cost zero
CPU.

### Loudness Compensation
ISO 226:2003 equal-loudness EQ that adapts to the current volume. Configurable
reference SPL (40–100 dB) and intensity (0–200%). Coefficient tables for all 91
volume steps are precomputed and double-buffered for glitch-free updates.

### Headphone Crossfeed
BS2B crossfeed with a complementary filter design (direct = input − lowpass(input),
so mono passes through at unity). Optional interaural time delay (~220 µs all-pass).

| Preset | Cutoff | Feed Level | Character |
|--------|--------|------------|-----------|
| Default | 700 Hz | 4.5 dB | Balanced, most popular |
| Chu Moy | 700 Hz | 6.0 dB | Stronger spatial effect |
| Jan Meier | 650 Hz | 9.5 dB | Subtle, natural |
| Custom | 500–2000 Hz | 0–15 dB | User-defined |

### Volume Leveller
Feedforward, stereo-linked, single-band RMS upward compressor: boosts content below
the threshold, leaves louder content untouched. RMS detection, soft knee, −6 dBFS
gain-reduction safety limiter, optional 10 ms lookahead, configurable speed,
max-gain ceiling, and noise gate.

### Per-Channel Preamp
Independent dB gain for the Left and Right input channels, applied at PASS 1. A
legacy single-value command (sets both channels) remains for compatibility.

### Master Volume
Device-side output ceiling, −128…0 dB (−128 = true mute). Independent of USB host
volume (they multiply) and of DSP processing. Two persistence modes: **Independent**
(default — a stand-alone device setting saved to the directory, unaffected by preset
switching) or **With preset** (saved/restored as part of each preset). Power-on
default is −20 dB (`MASTER_VOL_DEFAULT_DB`).

---

## Outputs (I2S / S/PDIF)

Four stereo output slots are produced by two SAI peripherals:

- **SAI1** (D2 domain, DMA1) drives slots 0–1 on `PE6` / `PE3`.
- **SAI4** (D3 domain, BDMA, buffers in SRAM4) drives slots 2–3 on `PD11` / `PA0`.

All four sub-blocks are clock-aligned: SAI1_A is the asynchronous master, and the
other three sub-blocks slave to it (internally or via the SAI GCR external-sync
path), so every output emits the same sample on the same bit-clock edge.

Each slot is switchable at runtime between **S/PDIF** (default) and **I2S**,
independently, via `REQ_SET_OUTPUT_TYPE`. Switching triggers a hot-swap that tears
down the SAI blocks, reconfigures them, and restarts with aligned clocks.

- **I2S format:** 24-bit data, left-justified, MSB-first, 32-bit frames. Drop-in to
  most standard I2S DACs (PCM5102, ES9038Q2M, etc.). All I2S slots share one
  BCK/LRCLK/MCLK group (`PE5`/`PE4`/`PE2`); MCLK runs at 256× Fs.
- **S/PDIF format:** IEC 60958 biphase-mark, encoded in SAI hardware. A S/PDIF slot
  is always its own clock master and needs only its data pin.
- **Dependency rule:** an I2S slot inherits the shared clock group, so the firmware
  coerces an invalid I2S choice down to S/PDIF when its clock parent is in S/PDIF
  mode. (S/PDIF slots have no parent dependency.)

---

## S/PDIF Input

The STM32H723's dedicated **SPDIFRX** peripheral decodes the incoming biphase-mark
stream on `PD8` (AF9), recovering audio, clock, and IEC 60958 channel status in
hardware — no external comparator needed.

- **Bit depth:** full 24-bit, sign-extended and carried end-to-end (no 16-bit
  truncation).
- **Rate detection:** 32, 44.1, 48, 88.2, 96, 176.4, and 192 kHz, inferred from the
  symbol width and snapped to the nearest standard rate within ±2%.
- **Lock state machine:** INACTIVE → ACQUIRING → LOCKED → RELOCKING.
- **Rate conversion (async SRC):** a polyphase Kaiser-windowed resampler converts the
  detected input rate to the SAI output rate (e.g. 96 kHz → 48 kHz). A ~100 Hz PI loop
  watches the input ring's fill against a ~10 ms target latency and continuously trims
  the **resampler ratio** to hold it there, so the source/sink rate difference and any
  drift are absorbed entirely in software. The SAI output clock runs free at its
  nominal rate — **PLL2 is not servoed**. (The H7 has no hardware route from the
  recovered S/PDIF clock to the PLL, so this software ASRC *is* the locking mechanism;
  there is no hardware-synchronous clock recovery.)
- **Source switching:** the active input (USB or S/PDIF) is selected with
  `REQ_SET_INPUT_SOURCE`; the actual start/stop is deferred to the main loop, not the
  USB ISR.

The SPDIFRX kernel clock is sourced from PLL3_R (120 MHz), independent of the SAI
audio PLL (PLL2), so it is exact and never moves with the audio rate.

---

## User Presets

A 10-slot preset system stores complete DSP configurations in the on-board W25Q64
SPI flash. A preset is always active — there is no "no preset" state.

* Each slot stores the full DSP state: per-channel preamp, EQ bands, delays,
  loudness, leveller, crossfeed, matrix mixer, output gains/mutes, per-slot output
  type (S/PDIF or I2S), master volume (in *With preset* mode), and per-channel names
  (up to 31 characters).
* **Startup configuration:** load a specific default slot or whichever slot was last
  active.
* **Preset-switch mute:** output is briefly muted (~10 ms) during transitions.
* **Legacy commands** (0x51–0x53) redirect through the preset system, operating on
  the active slot.
* **Bulk parameter transfer:** the entire DSP state can be read or written in a
  single control transfer (~2.9 KB) for fast host synchronization.

---

## Building from Source

The build uses CMake with the `arm-none-eabi` GCC toolchain and the bundled
[`stm32-cmake`](https://github.com/ObKo/stm32-cmake) glue and ST HAL/CMSIS sources.
TinyUSB is reused from the `pico-sdk` submodule, so a recursive checkout is required.

These instructions were verified end-to-end. The build produces:

- `build-stm32h723/DSPi_stm32h723.elf` — with symbols (for debugging)
- `build-stm32h723/DSPi_stm32h723.bin` — raw image (for DFU)
- `build-stm32h723/DSPi_stm32h723.hex` — Intel HEX (for STM32CubeProgrammer)

### Clone with submodules (all platforms)

```bash
git clone --recurse-submodules https://github.com/WeebLabs/DSPi-STM32.git
cd DSPi-STM32
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

> The required submodules are `vendor/stm32-cmake`, `vendor/STM32CubeH7` (with its
> nested `CMSIS/Device/ST/STM32H7xx` and `STM32H7xx_HAL_Driver` checkouts), and
> `firmware/pico-sdk` (used only for its bundled TinyUSB under
> `firmware/pico-sdk/lib/tinyusb`). `--recurse-submodules` fetches all of them.

### Building on macOS

1. **Install the toolchain** (Homebrew):
   ```bash
   xcode-select --install                       # git + build basics, if not present
   brew install --cask gcc-arm-embedded         # arm-none-eabi-gcc
   brew install cmake dfu-util                   # build + flash tools
   ```
   Verify:
   ```bash
   arm-none-eabi-gcc --version    # expect 13.2 or newer
   cmake --version                # expect 3.20 or newer
   ```

2. **Configure and build:**
   ```bash
   cmake -S firmware/STM32 -B build-stm32h723 -DCMAKE_BUILD_TYPE=Release
   cmake --build build-stm32h723 -j
   ```

   The first `cmake` invocation downloads nothing — it just locates the toolchain
   and the bundled HAL. The build finishes with a size summary; the three output
   files land in `build-stm32h723/`.

### Building on Windows

Use the **Arm GNU Toolchain** plus CMake and Ninja. (Ninja avoids needing a `make`
on Windows; the Makefile generator works too if you prefer it.)

1. **Install the tools:**
   - [Git for Windows](https://git-scm.com/download/win)
   - [Arm GNU Toolchain (`arm-none-eabi`)](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
     — during install, tick **"Add path to environment variable"**.
   - [CMake](https://cmake.org/download/) — choose **"Add CMake to the system PATH"**.
   - [Ninja](https://github.com/ninja-build/ninja/releases) — unzip `ninja.exe`
     somewhere on your `PATH` (e.g. the CMake `bin` folder).

   Verify in a fresh PowerShell / Command Prompt:
   ```powershell
   arm-none-eabi-gcc --version
   cmake --version
   ninja --version
   ```

2. **Configure and build** (from the repo root):
   ```powershell
   cmake -S firmware/STM32 -B build-stm32h723 -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build-stm32h723
   ```

   Outputs appear in `build-stm32h723\` exactly as on macOS.

> **Tip:** if CMake can't find the compiler, make sure the Arm toolchain `bin`
> directory is on your `PATH` (re-open the terminal after installing), or pass
> `-DTOOLCHAIN_PREFIX="C:/Program Files (x86)/Arm GNU Toolchain .../bin/arm-none-eabi"`.

---

## Flashing the Firmware

The STM32H723 has a built-in ROM **USB DFU bootloader** — no external probe or UF2
drive is involved. Software-triggered bootloader entry is intentionally not
supported on this port (it proved unreliable on the H7 ROM), so always use the
hardware button sequence:

> **Enter DFU mode:** hold **BOOT0**, tap **NRST**, then release **BOOT0** about half
> a second later. The board re-enumerates as
> *"STMicroelectronics STM Device in DFU Mode"* (USB `0483:DF11`).

### Flashing on macOS

Use the helper script (it waits for DFU enumeration, flashes, and issues a USB reset
so the board lands cleanly in user firmware):

```bash
firmware/STM32/scripts/flash.sh
```

Equivalent raw `dfu-util` command:

```bash
dfu-util -a 0 -s 0x08000000:leave -R -D build-stm32h723/DSPi_stm32h723.bin
```

> The H7's DFU bootloader leaves enough USB state behind that `:leave` alone often
> wedges the firmware until a manual reset; the `-R` flag adds a USB-level reset that
> clears it. If the heartbeat LED still doesn't come up, tap **NRST** once.

After flashing, the board re-enumerates as **"Weeb Labs DSPi for STM32"**. Watch
USART1 (`PA9` TX, 115200 8N1) for the boot log, and launch the DSPi Console to
configure it.

### Flashing on Windows

Two reliable options:

**Option A — STM32CubeProgrammer (recommended, no driver hassle).**
1. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).
   It bundles the STM32 DFU (USB) driver.
2. Put the board in DFU mode (BOOT0 + NRST sequence above).
3. **GUI:** open STM32CubeProgrammer, pick **USB** in the connection dropdown,
   click the refresh icon and select the DFU port, **Connect**, then on the *Erasing
   & Programming* tab choose `build-stm32h723\DSPi_stm32h723.hex` (HEX carries its own
   address; for the `.bin` set start address `0x08000000`), tick **Run after
   programming**, and **Start Programming**.
4. **CLI** (equivalent):
   ```powershell
   STM32_Programmer_CLI -c port=usb1 -w build-stm32h723\DSPi_stm32h723.hex -rst
   ```

**Option B — dfu-util + Zadig.**
1. Download [dfu-util for Windows](https://dfu-util.sourceforge.net/) and put it on
   your `PATH`.
2. Put the board in DFU mode, then use [Zadig](https://zadig.akeo.ie/) once to assign
   the **WinUSB** driver to the `STM32 BOOTLOADER` device (`0483:DF11`) so `dfu-util`
   can access it.
3. Flash:
   ```powershell
   dfu-util -a 0 -s 0x08000000:leave -R -D build-stm32h723\DSPi_stm32h723.bin
   ```

If the heartbeat LED doesn't come up after programming, tap **NRST** once.

---

## Developer Reference

The control protocol is identical to the RP2040/RP2350 firmware, so existing host
tooling works unchanged. A handful of commands behave differently on STM32 — those
are flagged in the table and detailed under
[Differences](#differences-from-the-rp2040rp2350-build).

Configuration is performed over **Interface 2** (Vendor) using control transfers on
Windows, and over **Interface 0** on macOS. The device supports WinUSB/WCID for
driverless installation on Windows. A bulk IN endpoint (`0x83`) carries asynchronous
device→host notifications (peak meters, source/lock changes).

### USB Control Protocol

**Request Table**

| Code | Name | Direction | Payload | Description |
| :--- | :--- | :--- | :--- | :--- |
| `0x42` | `REQ_SET_EQ_PARAM` | OUT | 16 bytes | Upload filter parameters |
| `0x43` | `REQ_GET_EQ_PARAM` | IN | 16 bytes | Read filter parameters |
| `0x44` | `REQ_SET_PREAMP` | OUT | 4 bytes | Set global gain (float dB) |
| `0x45` | `REQ_GET_PREAMP` | IN | 4 bytes | Get global gain |
| `0x46` | `REQ_SET_BYPASS` | OUT | 1 byte | Bypass Master EQ (1=On, 0=Off) |
| `0x47` | `REQ_GET_BYPASS` | IN | 1 byte | Get bypass state |
| `0x48` | `REQ_SET_DELAY` | OUT | 4 bytes | Set channel delay (float ms) |
| `0x49` | `REQ_GET_DELAY` | IN | 4 bytes | Get channel delay |
| `0x50` | `REQ_GET_STATUS` | IN | 4-12 bytes | Get system statistics (wValue selects field) |
| `0x51` | `REQ_SAVE_PARAMS` | IN | 1 byte | Save to active preset slot |
| `0x52` | `REQ_LOAD_PARAMS` | IN | 1 byte | Reload active preset slot |
| `0x53` | `REQ_FACTORY_RESET` | IN | 1 byte | Reset live state to defaults |
| `0x54`–`0x57` | `REQ_*_CHANNEL_GAIN/MUTE` | — | — | Per-master-channel gain & mute |
| `0x58`–`0x5D` | `REQ_*_LOUDNESS*` | — | — | Loudness enable / reference SPL / intensity |
| `0x5E`–`0x67` | `REQ_*_CROSSFEED*` | — | — | Crossfeed enable / preset / freq / feed / ITD |
| `0x70`/`0x71` | `REQ_*_MATRIX_ROUTE` | — | 8 bytes | Matrix crosspoint set/get |
| `0x72`–`0x79` | `REQ_*_OUTPUT_*` | — | — | Per-output enable / gain / mute / delay |
| `0x7A`/`0x7B` | `REQ_GET_CORE1_MODE/CONFLICT` | IN | 1 byte | Always `0` (single-core; see notes) |
| `0x7C`/`0x7D` | `REQ_*_OUTPUT_PIN` | IN | 1 byte | **No-op on STM32** (pins fixed) |
| `0x7E` | `REQ_GET_SERIAL` | IN | variable | Unique board serial (from H7 UID) |
| `0x7F` | `REQ_GET_PLATFORM` | IN | 1 byte | Returns `2` = STM32H723 |
| `0x83` | `REQ_CLEAR_CLIPS` | OUT | — | Clear clip detection latches |
| `0x90`–`0x9A` | `REQ_PRESET_*` | — | — | Save / load / delete / name / directory / startup |
| `0x9B`/`0x9C` | `REQ_*_CHANNEL_NAME` | — | 32 bytes | Per-channel name set/get |
| `0xA0`/`0xA1` | `REQ_*_ALL_PARAMS` | — | ~2896 bytes | Bulk read/write entire DSP state |
| `0xB0`–`0xB3` | `REQ_*_BUFFER_STATS` / `REQ_*_USB_ERROR_STATS` | — | — | Diagnostics counters |
| `0xB4`–`0xBF` | `REQ_*_LEVELLER_*` | — | — | Leveller enable / amount / speed / max-gain / lookahead / gate |
| `0xC0`/`0xC1` | `REQ_*_OUTPUT_TYPE` | — | 1 byte | Per-slot S/PDIF (0) or I2S (1) |
| `0xC2`–`0xC9` | `REQ_*_I2S_BCK_PIN` / `REQ_*_MCK_*` | — | — | I2S clock config (pins fixed on STM32) |
| `0xD0`/`0xD1` | `REQ_*_PREAMP_CH` | — | 4 bytes | Per-channel preamp (wValue=channel) |
| `0xD2`–`0xD7` | `REQ_*_MASTER_VOLUME*` | — | — | Master volume value / mode / save / get-saved |
| `0xD8`/`0xD9` | `REQ_*_BAND_BYPASS` | — | 1 byte | Per-band bypass (wValue=(ch<<8)\|band) |
| `0xE0`/`0xE1` | `REQ_*_INPUT_SOURCE` | — | 1 byte | Select USB or S/PDIF input |
| `0xE2`–`0xE5` | `REQ_*_SPDIF_RX_*` | — | — | S/PDIF RX status / channel status / pin |
| `0xF0` | `REQ_ENTER_BOOTLOADER` | IN | 1 byte | **Unsupported on STM32** (STALLs — use BOOT0+NRST) |

The full opcode list (including the individual sub-codes collapsed above) lives in
[`firmware/STM32/Core/Inc/config.h`](firmware/STM32/Core/Inc/config.h).

### System Telemetry

`REQ_GET_STATUS` (0x50) returns data selected by `wValue`. Highlights:

| wValue | Returns | Description |
| :--- | :--- | :--- |
| `0`–`2`, `9` | packed | Per-channel peaks + CPU0/CPU1 load (CPU1 always 0) |
| `10`–`12` | uint32 | USB packet count / alt setting / mounted state |
| `13` | uint32 | System clock frequency (Hz) — 550 MHz |
| `14` | — | Vdda — **STALLs on STM32** (Vdda is pin-bonded to 3V3; Console hides the field) |
| `15` | uint32 | Sample rate (Hz) |
| `16` | int32 | Junction temperature (centi-°C, internal sensor via ADC3) |
| `17`–`21` | uint32 | S/PDIF DMA starvation counters (total + per slot) |

A starvation event means the S/PDIF DMA needed a buffer but the pool was empty, so a
silence buffer was substituted for that transfer.

### Data Structures

**Filter Packet (16 bytes):**
```c
struct __attribute__((packed)) {
    uint8_t channel;  // 0-10 (2 master + 8 output + 1 PDM placeholder)
    uint8_t band;     // 0-9
    uint8_t type;     // 0=Flat, 1=Peak, 2=LS, 3=HS, 4=LP, 5=HP, 6=Notch
    uint8_t bypass;   // exactly 1 = bypassed; any other value = active
    float freq;       // Hz
    float Q;
    float gain_db;
}
```

**Matrix Route Packet (8 bytes):**
```c
struct __attribute__((packed)) {
    uint8_t input;          // 0-1 (USB/S/PDIF L/R)
    uint8_t output;         // 0-8
    uint8_t enabled;        // 0 or 1
    uint8_t phase_invert;   // 0 or 1
    float gain_db;          // -inf to +12 dB
}
```

In-depth specs for each subsystem are kept under
[`Documentation/Features/`](Documentation/Features/), and the full STM32 bring-up
plan (clock tree, pin map, milestone history) is in
[`Documentation/Porting/STM32H723_first_steps.md`](Documentation/Porting/STM32H723_first_steps.md).

---

## Differences from the RP2040/RP2350 Build

If you're coming from the Pico firmware or its README, note these STM32-specific
changes:

| Area | RP2040 / RP2350 | STM32H723 port |
|---|---|---|
| **Core(s)** | Dual-core; Core 1 runs an EQ worker or the PDM modulator | Single Cortex-M7; all DSP on one core. `Core1` queries always return `0`; `cpu1_load` is always `0` |
| **Outputs** | Up to 4 stereo slots via PIO (RP2350) | 4 stereo slots via SAI1 + SAI4 (sample-aligned) |
| **PDM subwoofer** | Software delta-sigma on PIO1 | **Not implemented** (no PIO equivalent); channel is a silent placeholder |
| **USB input** | 16/24-bit, 44.1 / 48 / 96 kHz | 16-bit, 48 / 44.1 kHz only |
| **S/PDIF input** | PIO-based receiver | Hardware SPDIFRX peripheral, 24-bit, up to 192 kHz |
| **Output pin remap** | Any output pin reassignable at runtime | Pins fixed by board layout; `REQ_SET_OUTPUT_PIN` is a no-op |
| **Max delay** | 21 ms (RP2040) / 42 ms (RP2350) | ~42 ms (2048 samples @ 48 kHz) |
| **Bootloader entry** | UF2 drive; software `REQ_ENTER_BOOTLOADER` | ROM USB DFU; software entry **unsupported** — use BOOT0 + NRST |
| **Flashing** | Drag-drop `.uf2` | `dfu-util` / STM32CubeProgrammer over DFU |
| **Platform ID** | `0` / `1` | `2` (STM32H723) |

---

## Specifications

| Category | Specification |
|---|---|
| **Board** | WeAct MiniSTM32H723 V1.2 |
| **MCU** | STM32H723VGT6, single-core Cortex-M7 @ 550 MHz, VOS0 |
| **Flash / RAM** | 1 MB internal flash; DTCM + AXI SRAM + SRAM4 (D3) |
| **HSE** | 25 MHz crystal |
| **Audio clock** | PLL2_P ≈ 49.152 MHz (fixed per output rate; not servoed) |
| **USB clock** | 48 MHz via PLL3_Q |
| **SPDIFRX clock** | PLL3_R = 120 MHz |
| **USB audio** | UAC1, 16-bit stereo, 48 / 44.1 kHz, async feedback |
| **Outputs** | 4 stereo slots (8 ch); per-slot 24-bit I2S or S/PDIF, runtime-switchable |
| **S/PDIF input** | SPDIFRX on PD8, 24-bit, 32–192 kHz, polyphase ASRC |
| **EQ** | 10 bands/channel, float, hybrid SVF/biquad |
| **Logical channels** | 11 (2 master + 8 output + 1 PDM placeholder) |
| **Preset storage** | 10 slots in on-board W25Q64 (SPI3) |
| **Firmware version** | 1.1.4 |
| **Toolchain** | arm-none-eabi-gcc ≥ 13.2, CMake ≥ 3.20 |

---

## License

This project is licensed under the GNU General Public License v3.0. See
[LICENSE](LICENSE) for details.
</content>
</invoke>
