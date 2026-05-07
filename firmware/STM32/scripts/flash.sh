#!/usr/bin/env bash
#
# DSPi STM32H723 — DFU flash + clean restart
#
# Usage:  firmware/STM32/scripts/flash.sh [path/to/firmware.bin]
# Default firmware path: build-stm32h723/DSPi_stm32h723.bin
#
# Why this script exists: STM32H7's DFU bootloader (v0x011a) leaves the USB
# peripheral in an intermediate state after `:leave` that often wedges the
# user firmware on startup until a hardware reset. Combining `:leave` with
# `dfu-util -R` (issue USB reset signalling at end) clears that state on
# the host side; if it's still flaky we fall through to instructing the
# user to tap NRST.

set -euo pipefail

FW="${1:-build-stm32h723/DSPi_stm32h723.bin}"

if [[ ! -f "$FW" ]]; then
    echo "ERROR: firmware not found at $FW" >&2
    echo "Run 'cmake --build build-stm32h723' first." >&2
    exit 1
fi

# Wait up to 10 s for DFU enumeration so we don't race the BOOT0+NRST tap.
echo "Waiting for board in DFU mode (hold BOOT0, tap NRST, release BOOT0)..."
for i in $(seq 1 20); do
    if dfu-util -l 2>/dev/null | grep -q "Internal Flash"; then
        echo "  found."
        break
    fi
    if (( i == 20 )); then
        echo "ERROR: timed out waiting for DFU device (0483:df11)" >&2
        echo "Confirm the board is in DFU mode: dfu-util -l" >&2
        exit 1
    fi
    sleep 0.5
done

# Flash.  -s :leave triggers the bootloader's jump-to-app on completion.
# -R adds a USB-level reset on top, which is what makes the H7 land in
# user firmware reliably without a manual NRST press.
echo "Flashing $FW ..."
dfu-util -a 0 -s 0x08000000:leave -R -D "$FW"

echo
echo "Flash complete.  Board should now be running user firmware."
echo "If LED still doesn't blink, tap NRST once."
