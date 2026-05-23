#!/usr/bin/env python3
"""Force PD8 / WeAct P1 pin 40 to plain INPUT + internal pull-up (AF disconnected, SPDIFEN=00).
If pin still reads LOW with nothing connected externally, the issue is
board-side or shared with some other GPIO config — NOT the SPDIFRX AF
routing. If pin reads HIGH, AF9 SPDIFRX is what's sinking the line.
Note: on the WeAct board, PD8 is connector P1 pin 40, labelled D8.
"""
import sys, struct
import usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")

raw = dev.ctrl_transfer(0xC0, 0xF6, 0, 0, 20, timeout=1000)
moder, pupdr, afr1, idr, cr = struct.unpack("<IIIII", bytes(raw))

print(f"GPIOD->MODER   = 0x{moder:08x}")
print(f"  PD8 mode     = {(moder >> 16) & 0x3}  (expect 0 = INPUT)")
print(f"GPIOD->PUPDR   = 0x{pupdr:08x}")
print(f"  PD8 pull     = {(pupdr >> 16) & 0x3}  (expect 1 = pull-up)")
print(f"GPIOD->AFR[1]  = 0x{afr1:08x}")
print(f"  PD8 AF       = {afr1 & 0xF}  (expect 0 = disconnected)")
print(f"GPIOD->IDR     = 0x{idr:08x}")
print(f"  PD8 reads    = {(idr >> 8) & 1}  (1 = pull-up wins, 0 = something sinks it)")
print(f"SPDIFRX->CR    = 0x{cr:08x}")
print(f"  SPDIFEN      = {cr & 0x3}  (expect 0 = peripheral disabled)")
print()
print("Multimeter on PD8 / WeAct P1 pin 40 to GND:")
print("  ~3.3 V → AF9 SPDIFRX routing was sinking the line")
print("  ~0 V   → Something else sinks PD8 even with AF disconnected and SPDIFRX off")
