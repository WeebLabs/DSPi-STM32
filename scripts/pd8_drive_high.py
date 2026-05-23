#!/usr/bin/env python3
"""Drive PD8 / WeAct P1 pin 40 as a plain GPIO push-pull output HIGH, bypassing AF9 SPDIFRX.
If you then measure 0 V on the PD8 / WeAct P1 pin 40, the pin is electrically
damaged. If you measure 3.3 V, the silicon is healthy and the SPDIFRX AF
config is the problem.
Note: on the WeAct board, PD8 is connector P1 pin 40, labelled D8.
"""
import sys, struct
import usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")

raw = dev.ctrl_transfer(0xC0, 0xF5, 0, 0, 16, timeout=1000)
moder, pupdr, odr, idr = struct.unpack("<IIII", bytes(raw))

print(f"GPIOD->MODER  = 0x{moder:08x}")
print(f"  PD8 mode    = {(moder >> 16) & 0x3}  (expect 1 = OUTPUT)")
print(f"GPIOD->PUPDR  = 0x{pupdr:08x}")
print(f"  PD8 pull    = {(pupdr >> 16) & 0x3}  (expect 0 = none)")
print(f"GPIOD->ODR    = 0x{odr:08x}")
print(f"  PD8 ODR     = {(odr >> 8) & 1}  (expect 1)")
print(f"GPIOD->IDR    = 0x{idr:08x}")
print(f"  PD8 IDR     = {(idr >> 8) & 1}  "
      f"(1 = pin actually reads high, 0 = pin is being held LOW)")
print()
print("Now measure PD8 / WeAct P1 pin 40 with a multimeter:")
print("  ~3.3 V → pin is healthy, AF8 SPDIFRX config is the issue")
print("  ~0 V   → pin is silicon-damaged, pick a different SPDIFRX pin")
