#!/usr/bin/env python3
"""Continuously poll SPDIFRX servo + status for ~40 s, log anything
that changes from one sample to the next. Used to chase intermittent
DAC dropouts — anomalies that fire every few seconds will show up in
the diff stream.
"""
import sys, struct, time
import usb.core

VID, PID = 0x2E8B, 0xFEAA
dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    sys.exit(f"No device {VID:04x}:{PID:04x} found")


def get_servo():
    raw = dev.ctrl_transfer(0xC0, 0xF7, 0, 0, 40, timeout=1000)
    fill, err, int_acc, cur_fracn, writes, widx, ridx, \
        underruns, peak_cyc, budget_cyc = \
        struct.unpack("<iiiiIIIIII", bytes(raw))
    return dict(fill=fill, err=err, int_acc=int_acc, fracn=cur_fracn,
                writes=writes, widx=widx, ridx=ridx,
                underruns=underruns, peak_cyc=peak_cyc,
                budget_cyc=budget_cyc)


def get_status():
    raw = dev.ctrl_transfer(0xC0, 0xE2, 0, 0, 16, timeout=1000)
    state, src, lock_count, loss_count, fs, perr, fifo, rsv = \
        struct.unpack("<BBBBIIHH", bytes(raw))
    return dict(state=state, src=src, locks=lock_count, losses=loss_count,
                fs=fs, perr=perr)


def get_sr():
    """Raw SPDIFRX->SR + GPIO live state."""
    raw = dev.ctrl_transfer(0xC0, 0xF4, 0, 0, 32, timeout=1000)
    cr, sr = struct.unpack("<II", bytes(raw)[:8])
    return dict(cr=cr, sr=sr,
                syncd=(sr >> 5) & 1, serr=(sr >> 7) & 1,
                terr=(sr >> 8) & 1, ferr=(sr >> 6) & 1,
                ovr=(sr >> 3) & 1)


STATE_NAME = {0: "INACTIVE", 1: "ACQUIRING", 2: "LOCKED", 3: "RELOCKING"}


def fmt(s, st, raw):
    pct = 100 * s['peak_cyc'] / s['budget_cyc'] if s['budget_cyc'] else 0
    return (f"fill={s['fill']:>4} err={s['err']:+4d} "
            f"acc={s['int_acc']:+4d} fracn={s['fracn']} "
            f"writes={s['writes']:>5} under={s['underruns']:>3} "
            f"peak={pct:.0f}% locks={st['locks']} losses={st['losses']} "
            f"perr={st['perr']} state={STATE_NAME[st['state']]} "
            f"SR=0x{raw['sr']:08x}")


print("Sampling every 100 ms for ~40 s. Reporting changes only.\n")

interval = 0.1
duration = 40.0
samples = int(duration / interval)

prev_s = get_servo()
prev_st = get_status()
prev_raw = get_sr()
t0 = time.time()
print(f"t=0.0  baseline: {fmt(prev_s, prev_st, prev_raw)}")

anomalies = []

for i in range(samples):
    time.sleep(interval)
    t = time.time() - t0
    s = get_servo()
    st = get_status()
    raw = get_sr()

    diffs = []
    if s['writes'] != prev_s['writes']:
        diffs.append(f"WRITES +{s['writes'] - prev_s['writes']}")
    if s['underruns'] != prev_s['underruns']:
        diffs.append(f"⚠ UNDERRUN +{s['underruns'] - prev_s['underruns']}")
    if s['peak_cyc'] > s['budget_cyc']:
        pct = 100 * s['peak_cyc'] / s['budget_cyc']
        diffs.append(f"⚠ CPU SPIKE {pct:.0f}% of budget (peak={s['peak_cyc']})")
    if s['fracn'] != prev_s['fracn']:
        diffs.append(f"FRACN {prev_s['fracn']}→{s['fracn']} "
                     f"(Δ{s['fracn']-prev_s['fracn']:+d})")
    if abs(s['fill'] - prev_s['fill']) > 32:
        diffs.append(f"FILL JUMP {prev_s['fill']}→{s['fill']}")
    if st['locks'] != prev_st['locks']:
        diffs.append(f"LOCK +{st['locks'] - prev_st['locks']}")
    if st['losses'] != prev_st['losses']:
        diffs.append(f"LOSS +{st['losses'] - prev_st['losses']}")
    if st['perr'] != prev_st['perr']:
        diffs.append(f"PARITY +{st['perr'] - prev_st['perr']}")
    if st['state'] != prev_st['state']:
        diffs.append(f"STATE {STATE_NAME[prev_st['state']]}"
                     f"→{STATE_NAME[st['state']]}")
    if raw['serr'] != prev_raw['serr'] or raw['terr'] != prev_raw['terr'] \
            or raw['ferr'] != prev_raw['ferr'] or raw['ovr'] != prev_raw['ovr']:
        diffs.append(f"SR ERR-BITS serr={raw['serr']} terr={raw['terr']} "
                     f"ferr={raw['ferr']} ovr={raw['ovr']}")
    if raw['syncd'] != prev_raw['syncd']:
        diffs.append(f"SYNCD {prev_raw['syncd']}→{raw['syncd']}")

    if diffs:
        print(f"t={t:5.2f}  " + "  ".join(diffs))
        anomalies.append((t, diffs))

    prev_s, prev_st, prev_raw = s, st, raw

t_end = time.time() - t0
print(f"\nt={t_end:.2f}  final: {fmt(prev_s, prev_st, prev_raw)}")
print(f"\nTotal anomaly events: {len(anomalies)}")
write_events = sum(1 for _, ds in anomalies for d in ds if d.startswith("WRITES"))
print(f"  FRACN writes during window: {write_events}")
loss_events = sum(1 for _, ds in anomalies for d in ds if d.startswith("LOSS"))
print(f"  Lock losses:                {loss_events}")
parity_events = sum(1 for _, ds in anomalies
                    for d in ds if d.startswith("PARITY"))
print(f"  Parity error bursts:        {parity_events}")
serr_events = sum(1 for _, ds in anomalies
                  for d in ds if d.startswith("SR ERR-BITS"))
print(f"  SR error-bit changes:       {serr_events}")
fill_jumps = sum(1 for _, ds in anomalies
                 for d in ds if d.startswith("FILL JUMP"))
print(f"  Ring fill jumps (>32 frames): {fill_jumps}")
