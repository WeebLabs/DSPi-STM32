#!/usr/bin/env python3
"""Dump per-stage cycle counters from fill_half (vendor 0xEC).

Each probe shows the cycles spent in each stage since the LAST probe,
along with the number of fill_half calls in the window. Run twice in
different states (e.g., PEQ active vs PEQ bypassed) to see which
stage's cycles change.

Stages:
  0 = input pop (SPDIF resampler or USB ring)
  1 = preamp + matrix snapshot + param prep
  2 = loudness shelves
  3 = per-input PEQ (channels 0, 1)
  4 = crossfeed + leveller
  5 = matrix mixer
  6 = per-output PEQ (channels 2..9)
  7 = per-output delay + output gain + format convert → SAI buffer
"""
import sys, struct, usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")

raw = dev.ctrl_transfer(0xC0, 0xEC, 0, 0, 36, timeout=1000)
v = struct.unpack("<9I", bytes(raw))
calls = v[0]
stages = v[1:9]

print(f"fill_half calls since last probe: {calls}")
if calls == 0:
    sys.exit("(no calls — fill_half not running)")
total_cycles = sum(stages)
budget = 2_200_000  # cycles per fill_half budget at 48k × 192 frames

names = [
    "0 input-pop",
    "1 preamp+snap",
    "2 loudness",
    "3 input-PEQ",
    "4 cf+leveller",
    "5 matrix-mix",
    "6 output-PEQ",
    "7 delay+gain+conv",
]
print(f"{'stage':<22} {'cyc/call':>10} {'%/call':>8} {'%total':>8}")
for name, cyc in zip(names, stages):
    cyc_per_call = cyc // calls
    pct_per_call = 100 * cyc / (calls * budget)
    pct_of_total = 100 * cyc / total_cycles if total_cycles else 0
    print(f"  {name:<20} {cyc_per_call:>10}   {pct_per_call:>5.2f}%  {pct_of_total:>5.1f}%")
print(f"  {'TOTAL':<20} {total_cycles // calls:>10}   "
      f"{100 * total_cycles / (calls * budget):>5.2f}%  100.0%")
