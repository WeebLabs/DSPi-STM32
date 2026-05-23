#!/usr/bin/env python3
"""SPDIFRX + RCC/PLL2 register dump for DSPi STM32.
Requires pyusb + libusb."""
import sys, struct, time
import usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")

def vget(req, length):
    return bytes(dev.ctrl_transfer(0xC0, req, 0, 0, length, timeout=1000))

# --- 0xF4: SPDIFRX peripheral state -----------------------------------------
cr, sr, imr, kclk_hal, afr1, moder, dir_reg, pd8_state = struct.unpack(
    "<IIIIIIII", vget(0xF4, 32))

print(f"SPDIFRX->CR    = 0x{cr:08x}")
print(f"  SPDIFEN[1:0] = {cr & 0x3}  (0=disabled  1=SYNC  2=reserved  3=RCV)")
print(f"  RXDMAEN      = {(cr >> 2) & 1}  (audio DMA enable)")
print(f"  RXSTEO       = {(cr >> 3) & 1}  (stereo mode)")
print(f"  DRFMT[5:4]   = {(cr >> 4) & 0x3}  (0=LSB,24-bit-in-bits[23:0])")
print(f"  PMSK         = {(cr >> 6) & 1}  (parity mask, 0=keep PE bit)")
print(f"  VMSK         = {(cr >> 7) & 1}  (validity mask)")
print(f"  CUMSK        = {(cr >> 8) & 1}  (CS/user mask)")
print(f"  PTMSK        = {(cr >> 9) & 1}  (preamble type mask)")
print(f"  CBDMAEN      = {(cr >> 10) & 1} (CS/UB DMA enable)")
print(f"  CHSEL        = {(cr >> 11) & 1} (channel select for CS DMA)")
print(f"  NBTR[13:12]  = {(cr >> 12) & 0x3} (retries: 0=none 1=3 2=15 3=63)")
print(f"  WFA          = {(cr >> 14) & 1} (1=wait for activity)")
print(f"  INSEL[17:16] = {(cr >> 16) & 0x3} "
      f"(expect 1 for PD8 / WeAct P1 pin 40; datasheet labels the pin IN2)")
print()
print(f"SPDIFRX->SR    = 0x{sr:08x}")
for i, name in enumerate(["RXNE","CSRNE","PERR","OVR","SBD","SYNCD","FERR","SERR","TERR"]):
    print(f"  {name:<6} bit{i:<2} = {(sr >> i) & 1}")
print(f"  WIDTH5       = {(sr >> 16) & 0x7FFF}  (5 S/PDIF symbols in kclk ticks)")
print()
print(f"SPDIFRX->IMR   = 0x{imr:08x}")
print(f"kclk (HAL)     = {kclk_hal:,} Hz  (HAL bug: returns 0 for SPDIFRX)")
print(f"GPIOD->AFR[1]  = 0x{afr1:08x}  → PD8 AF = {afr1 & 0xF} (expect 9)")
print(f"GPIOD->MODER   = 0x{moder:08x}  → PD8 mode = {(moder >> 16) & 0x3} (expect 2=AF)")
print(f"GPIOD->PUPDR   = 0x{dir_reg:08x}  → PD8 pull = {(dir_reg >> 16) & 0x3} "
      f"(0=none 1=pull-up 2=pull-down)")
print(f"PD8 live state = {pd8_state & 1}  (HAL_GPIO_ReadPin)")

# --- 0xE2: user-facing SPDIF status packet ----------------------------------
print()
print("=" * 60)
status = vget(0xE2, 16)
state, source, locks, losses, rate, parity, fill, reserved = struct.unpack(
    "<BBBBIIHH", status)
print(f"REQ_GET_SPDIF_RX_STATUS = {status.hex()}")
print(f"  state        = {state}  (0=inactive 1=acquiring 2=locked 3=relocking)")
print(f"  input_source = {source}  (0=USB 1=SPDIF)")
print(f"  locks/losses = {locks}/{losses}")
print(f"  sample_rate  = {rate} Hz")
print(f"  parity_errs  = {parity}")
print(f"  fifo_fill    = {fill}%")

# --- Short live-input sample -------------------------------------------------
vals = []
widths = []
sync_samples = 0
last_sr = 0
for _ in range(1000):
    sample = struct.unpack("<IIIIIIII", vget(0xF4, 32))
    _, sample_sr, _, _, _, _, _, sample_pd8 = sample
    vals.append(sample_pd8 & 1)
    widths.append((sample_sr >> 16) & 0x7FFF)
    sync_samples += 1 if (sample_sr & (1 << 5)) else 0
    last_sr = sample_sr
    time.sleep(0.001)

transitions = sum(1 for a, b in zip(vals, vals[1:]) if a != b)
print()
print("1s PD8/status sample:")
print(f"  final SR      = 0x{last_sr:08x}")
print(f"  PD8 high/low  = {sum(vals)}/{len(vals) - sum(vals)}")
print(f"  PD8 edges     = {transitions}")
print(f"  SYNCD samples = {sync_samples}")
print(f"  WIDTH5 nonzero= {sum(1 for w in widths if w)}")
print(f"  WIDTH5 max    = {max(widths)}")

# --- 0xF3: RCC + PLL2 clock tree --------------------------------------------
print()
print("=" * 60)
rcc_cr, pllcfgr, pll2divr, pll2fracr, d2ccip1r, apb1lenr, cfgr = \
    struct.unpack("<IIIIIII", vget(0xF3, 28))

print(f"RCC->CR        = 0x{rcc_cr:08x}")
print(f"  PLL2ON       = {(rcc_cr >> 26) & 1}")
print(f"  PLL2RDY      = {(rcc_cr >> 27) & 1}  (1 = locked)")
print()
print(f"RCC->PLLCFGR   = 0x{pllcfgr:08x}")
print(f"  PLL2FRACEN   = {(pllcfgr >> 4) & 1}  (FRACN modulator)")
print(f"  PLL2DIVPEN   = {(pllcfgr >> 19) & 1}  (P output enabled — SAI)")
print(f"  PLL2DIVQEN   = {(pllcfgr >> 20) & 1}  (Q output enabled)")
print(f"  PLL2DIVREN   = {(pllcfgr >> 21) & 1}  (R output enabled — SPDIFRX)")
print()
print(f"RCC->PLL2DIVR  = 0x{pll2divr:08x}")
print(f"  DIVN2[8:0]   = {(pll2divr & 0x1FF) + 1}  (multiplier)")
print(f"  DIVP2[15:9]  = {((pll2divr >> 9) & 0x7F) + 1}")
print(f"  DIVQ2[22:16] = {((pll2divr >> 16) & 0x7F) + 1}")
print(f"  DIVR2[30:24] = {((pll2divr >> 24) & 0x7F) + 1}")
print()
print(f"RCC->PLL2FRACR = 0x{pll2fracr:08x}  FRACN={(pll2fracr >> 3) & 0x1FFF}")
print()
print(f"RCC->D2CCIP1R  = 0x{d2ccip1r:08x}")
print(f"  SPDIFSEL[21:20] = {(d2ccip1r >> 20) & 0x3}  "
      f"(0=PLL1Q  1=PLL2R  2=PLL3R  3=HSI)")
print(f"  SAI1SEL[2:0]    = {d2ccip1r & 0x7}  (0=PLL1Q  1=PLL2P  …)")
print()
print(f"RCC->APB1LENR  = 0x{apb1lenr:08x}")
print(f"  SPDIFRXEN bit 16 = {(apb1lenr >> 16) & 1}  (peripheral clock gate)")
print()
print(f"RCC->CFGR      = 0x{cfgr:08x}")

# Computed PLL2_R freq
divn = ((pll2divr) & 0x1FF) + 1
divp = ((pll2divr >> 9) & 0x7F) + 1
divr = ((pll2divr >> 24) & 0x7F) + 1
fracn = (pll2fracr >> 3) & 0x1FFF
# HSE = 25 MHz, DIVM2 = 5
ref = 25_000_000 // 5
vco = int(ref * (divn + fracn / 8192.0))
pll2p = vco // divp
pll2r = vco // divr
print()
print(f"  → PLL2_P ≈ {pll2p:,} Hz  (SAI kernel clock)")
print(f"  → PLL2_R ≈ {pll2r:,} Hz  (SPDIFRX kernel clock if SPDIFSEL=01)")
