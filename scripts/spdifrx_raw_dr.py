#!/usr/bin/env python3
"""Snapshot 16 consecutive SPDIFRX_DR words and decode them. Validates
the bit-position assumptions used by the demux:
  bits[31:30] reserved
  bits[29:28] PT  (10=B, 00=M=left, 01=W=right, 11=invalid)
  bit[27]     C   (channel status)
  bit[26]     U   (user)
  bit[25]     V   (validity, 0=valid PCM)
  bit[24]     PE  (parity error)
  bits[23:0]  audio sample, MSB at bit 23

If the actual bit layout is different (e.g. audio in [27:4] with metadata
at [3:0] = "raw" DRFMT=11 format), the audio column will look like
garbage / metadata mixed in, and PT will be in the wrong place.
"""
import sys, struct
import usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")

raw = dev.ctrl_transfer(0xC0, 0xF8, 0, 0, 64, timeout=1000)
words = struct.unpack("<16I", bytes(raw))

PT_NAME = {0b00: "M(L)", 0b01: "W(R)", 0b10: "B(L)", 0b11: "INV"}

print(f"{'idx':>3} {'word':>10} {'PT':>5} {'C':>2} {'U':>2} {'V':>2} "
      f"{'PE':>2} {'audio[23:0]':>11} {'as int24':>10}")
for i, w in enumerate(words):
    pt   = (w >> 28) & 0b11
    c    = (w >> 27) & 1
    u    = (w >> 26) & 1
    v    = (w >> 25) & 1
    pe   = (w >> 24) & 1
    audio_u24 = w & 0xFFFFFF
    # sign-extend 24-bit
    audio_s24 = audio_u24 if audio_u24 < 0x800000 else audio_u24 - 0x1000000
    print(f"{i:>3} 0x{w:08x} {PT_NAME[pt]:>5} "
          f"{c:>2} {u:>2} {v:>2} {pe:>2} "
          f"0x{audio_u24:06x}  {audio_s24:>10}")

print()
# Expected pattern: alternating L,R,L,R... (PT=00 or 10 then PT=01)
pts = [(w >> 28) & 0b11 for w in words]
alternating = all((p in (0b00, 0b10)) == (i % 2 == 0) for i, p in enumerate(pts))
print("alternates L,R,L,R…:", "✓ yes" if alternating else "✗ NO — PT decode wrong")

# Count PT distribution
from collections import Counter
print("PT distribution:", dict(Counter(PT_NAME[p] for p in pts)))

# Check V bits — if many are 1 (invalid), samples will be muted by demux
v_count = sum((w >> 25) & 1 for w in words)
print(f"V=1 (invalid) count: {v_count}/16  "
      f"({'demux is muting these' if v_count > 0 else 'all valid PCM'})")

# Check PE bits
pe_count = sum((w >> 24) & 1 for w in words)
print(f"PE=1 (parity err) count: {pe_count}/16")
