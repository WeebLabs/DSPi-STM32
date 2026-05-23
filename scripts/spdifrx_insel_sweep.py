#!/usr/bin/env python3
"""Sweep SPDIFRX INSEL through all four values and check whether any
of them sees signal (WIDTH5 != 0 or SYNCD=1). Use with the receiver
wired to ONE pin — the sweep tells us which INSEL value actually maps
to that pin on this silicon.

Wire receiver OUT to whichever silkscreen pin you've chosen, run this,
look for the INSEL value that produces WIDTH5 != 0 in SR.
For STM32H723 PD8 (WeAct header P1 pin 40), the datasheet pin table calls
this SPDIFRX1_IN2, but RM0468/HAL select it with INSEL=1.
"""
import sys, struct, time
import usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")


def set_insel(value):
    """Issue vendor IN 0xF2 with wValue = INSEL. Returns new CR."""
    raw = dev.ctrl_transfer(0xC0, 0xF2, value, 0, 4, timeout=1000)
    return struct.unpack("<I", bytes(raw))[0]


def read_status():
    """Issue vendor IN 0xF4 and return (CR, SR)."""
    raw = dev.ctrl_transfer(0xC0, 0xF4, 0, 0, 32, timeout=1000)
    cr, sr = struct.unpack("<II", bytes(raw)[:8])
    return cr, sr


print("Sweeping INSEL 0..3 — wait ~500 ms per value for SYNC to engage")
print("Looking for WIDTH5 != 0 (peripheral measured at least one pulse)")
print()

best = None
for insel in range(4):
    cr_new = set_insel(insel)
    print(f"INSEL={insel}  CR=0x{cr_new:08x}  ", end="", flush=True)
    # give the peripheral a moment to detect pulses
    time.sleep(0.5)
    cr, sr = read_status()
    syncd  = (sr >> 5) & 1
    terr   = (sr >> 8) & 1
    width5 = (sr >> 16) & 0x7FFF
    print(f"SR=0x{sr:08x}  SYNCD={syncd}  TERR={terr}  WIDTH5={width5}")
    if width5 > 0 and best is None:
        best = (insel, width5)

print()
if best:
    insel, width5 = best
    print(f"✓ INSEL={insel} sees signal (WIDTH5={width5})")
    print(f"  Set this in firmware: hspdif.Init.InputSelection = "
          f"SPDIFRX_INPUT_IN{insel}")
else:
    print("✗ No INSEL value sees signal. Issue isn't INSEL mapping —")
    print("  signal isn't electrically reaching ANY SPDIFRX-capable pin")
    print("  from this wire position.")
