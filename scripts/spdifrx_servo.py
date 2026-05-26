#!/usr/bin/env python3
"""Poll the SPDIFRX FRACN servo state. Run this repeatedly while a
SPDIF source is locked to verify the servo is actually doing work:

- fill should hover near target (~512 frames). If it drifts to ring
  ends (0 or 1024), servo isn't keeping up.
- err = fill - target. Within ±2 = deadband, no servo updates.
- int_acc = integral accumulator = current FRACN offset from nominal
  (2490). Clamped to ±160 = ±200 ppm pull range.
- current_fracn = last value actually written to PLL2FRACR.
- fracn_writes = total pll2_fracn_write calls. Should slowly climb
  during convergence, then plateau when in deadband. If it spikes
  by tens per second, the servo is thrashing.

Run as `watch -n 0.5 python3 scripts/spdifrx_servo.py` for live view.
"""
import sys, struct
import usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")

raw = dev.ctrl_transfer(0xC0, 0xF7, 0, 0, 32, timeout=1000)
fill, err, int_acc, cur_fracn, writes, widx, ridx, _ = \
    struct.unpack("<iiiiIIII", bytes(raw))

# FRACN sensitivity: 1 step at PLL2_P (49.152 MHz / DIVP=10) = ~1.24 ppm
FRACN_NOMINAL = 2490
PPM_PER_STEP  = 1.24
offset_ppm = (cur_fracn - FRACN_NOMINAL) * PPM_PER_STEP

print(f"ring  widx={widx:>10}  ridx={ridx:>10}  fill={fill:>5} frames")
print(f"err   {err:+d} frames  (deadband ±8, slew ±2/100ms)")
print(f"accum {int_acc:+d} steps  (clamped ±160 = ±200 ppm)")
print(f"FRACN {cur_fracn}  (nominal {FRACN_NOMINAL}, offset {offset_ppm:+.2f} ppm, "
      f"write threshold ±4)")
print(f"writes since boot: {writes}")
print()
if fill < 64:
    print("⚠ ring nearly EMPTY — consumer faster than producer / DMA stalled")
elif fill > 960:
    print("⚠ ring nearly FULL  — producer faster than consumer / consumer stalled")
elif abs(err) <= 8:
    print("✓ in deadband — servo idle, locked")
else:
    print(f"… servo correcting toward target")
