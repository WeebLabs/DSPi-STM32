# Runtime Output Type Switch — Console Spec

*Last updated: 2026-05-10*

## Purpose

Define how DSPi Console interacts with firmware to switch each of the four
stereo output slots between **I2S** and **S/PDIF** at runtime, including the
hardware-derived dependency rules the UI must surface or honour.

This spec covers the firmware contract that ships in the STM32H723 build
(M12 phases 3 + 4). The RP2040/RP2350 builds expose the same vendor
command surface but have different hardware constraints — see
`i2s_output_spec.md` for those.

---

## 1. Vendor Command Surface

### `REQ_GET_OUTPUT_TYPE` — `bRequest = 0xC1`

Read the current type of one slot.

| Field         | Value                                                  |
|---------------|--------------------------------------------------------|
| bmRequestType | `0xC0` (Vendor IN, Device-recipient)                   |
| bRequest      | `0xC1`                                                 |
| wValue        | slot index (0..3)                                      |
| wIndex        | 0 (reserved)                                           |
| wLength       | 1                                                      |
| Direction     | IN — device returns 1 byte                             |

**Response (1 byte):** `0` = S/PDIF, `1` = I2S.

Slot index ≥ `NUM_SPDIF_INSTANCES` returns a STALL.

### `REQ_SET_OUTPUT_TYPE` — `bRequest = 0xC0`

Set one slot's type. Sent as a **vendor IN with side effects** —
following the established RP-firmware convention, the request both
applies the change AND returns a 1-byte status. There is no separate
data stage.

| Field         | Value                                                  |
|---------------|--------------------------------------------------------|
| bmRequestType | `0xC0` (Vendor IN, Device-recipient)                   |
| bRequest      | `0xC0`                                                 |
| wValue        | `(new_type << 8) | slot_index`                         |
| wIndex        | 0 (reserved)                                           |
| wLength       | 1                                                      |
| Direction     | IN — device returns 1 byte                             |

**Response (1 byte):**

| Status | Meaning                                                       |
|--------|---------------------------------------------------------------|
| `0x00` | Success — change accepted (or no-op if already that type)     |
| `0xFF` | Rejected — invalid `slot_index` (≥ 4) or `new_type` (>1)      |

**Side effects:**

1. If the value is valid AND differs from the current type, firmware
   updates `output_types[slot]` immediately.
2. Firmware **may auto-coerce dependent slots** down to S/PDIF — see
   §2 below. Each coerced slot is also updated and notified.
3. Firmware fires a `PARAM_CHANGED` notification (see
   `notification_protocol_v2_spec.md`) for every changed slot —
   the requested one AND any cascade-coerced ones.
4. Firmware queues a SAI hardware re-init (the *hot-swap*); audio
   re-initialises in main-loop context within ~1 ms. There is no
   reboot required, but expect a single audible click on each
   affected slot as the SAI peripheral tears down and comes back up
   in the new mode.

> **Important for Console:** the SET succeeds even when other slots
> are auto-coerced. The cascaded changes arrive as separate
> `PARAM_CHANGED` events on the notify endpoint — Console must
> consume those and update its UI to reflect the actual applied
> state, not just the slot the user asked for.

---

## 2. Hardware Dependency Rules (STM32H723 build)

### Why these exist

The four output slots map to four SAI sub-blocks:

| Slot | SAI sub-block | SD pin | I2S clock-source pins              |
|------|---------------|--------|------------------------------------|
| 0    | SAI1_A        | PE6    | PE2 (MCLK), PE5 (BCK), PE4 (LRCLK) |
| 1    | SAI1_B        | PE3    | (none — borrows from SAI1_A)       |
| 2    | SAI4_A        | PD11   | (none — borrows from SAI1_A)       |
| 3    | SAI4_B        | PA0    | (none — borrows from SAI4_A)       |

Only slot 0 has dedicated I2S clock pins. Slots 1/2/3 borrow clocks
from a "parent" slot via the SAI peripheral's internal sync mesh. If
the parent is in S/PDIF mode, no I2S clocks exist at the dependent
slot's frequency, so the dependent slot cannot operate as I2S.

S/PDIF outputs do not have this constraint: the S/PDIF subframe
embeds clock recovery in its biphase-mark encoding, so a S/PDIF slot
self-clocks regardless of what neighbouring slots are doing.

### The rules

| Rule                              | Affects | Justification                                           |
|-----------------------------------|---------|---------------------------------------------------------|
| Slot 1 = I2S **requires** slot 0 = I2S | Slot 1  | SAI1_B sync-internal to SAI1_A's clock generator        |
| Slot 2 = I2S **requires** slot 0 = I2S | Slot 2  | SAI4_A sync-external to SAI1's broadcast clock          |
| Slot 3 = I2S **requires** slot 2 = I2S | Slot 3  | SAI4_B sync-internal to SAI4_A's clock generator        |

There is no constraint on S/PDIF slots — any slot can be S/PDIF
regardless of any other slot's mode.

### Allowed combinations

```
0  1  2  3
─────────────
I  I  I  I    ✓  cross-peripheral sync: all four sample-aligned
I  I  I  S    ✓
I  I  S  I    ✗  (slot 3 I2S needs slot 2 I2S)
I  I  S  S    ✓
I  S  I  I    ✓
I  S  I  S    ✓
I  S  S  I    ✗  (slot 3 I2S needs slot 2 I2S)
I  S  S  S    ✓
S  I  ?  ?    ✗  (slot 1 I2S needs slot 0 I2S)
S  ?  I  ?    ✗  (slot 2 I2S needs slot 0 I2S)
S  S  S  I    ✗  (slot 3 I2S needs slot 2 I2S)
S  S  S  S    ✓
```

### Firmware behaviour for invalid combinations

Firmware **never refuses** a SET that produces an invalid combination.
Instead it applies the requested change and **cascade-coerces** any
now-invalid I2S slots down to S/PDIF in dependency order:

1. If after the SET, slot 1 is I2S and slot 0 is S/PDIF → slot 1 coerced to S/PDIF.
2. If after the SET, slot 2 is I2S and slot 0 is S/PDIF → slot 2 coerced to S/PDIF.
3. If after the SET, slot 3 is I2S and slot 2 is S/PDIF → slot 3 coerced to S/PDIF.

Each coerced slot fires its own `PARAM_CHANGED` notification.

This means setting slot 0 from I2S to S/PDIF can produce **up to three
additional notifications** as slots 1/2/3 cascade down. Console must
not assume `REQ_SET_OUTPUT_TYPE` only changes the slot it asked about.

---

## 3. Bulk SET Behaviour (`REQ_SET_ALL_PARAMS`)

When Console writes a full `WireBulkParams` (e.g., loading a preset),
the `i2s_config.output_types[]` array is applied wholesale. The same
sanitization runs after the wholesale write — any cascade-coerced
slots fire notifications, and the SAI re-init runs once for the whole
update (not once per slot).

A bulk SET that contains an invalid combination still applies cleanly,
just with the invalid I2S choices silently coerced down to S/PDIF.
Console should ideally validate the combination before sending the
bulk SET so the user sees the constraint in their UI rather than
seeing notifications cascade in after the fact.

---

## 4. Persistence

`output_types[]` is part of the saved preset payload (since
`SLOT_DATA_VERSION = 9`). On preset load, the array is restored from
flash, sanitized as described above, and the SAI hot-swap fires
exactly once for the new configuration.

The factory-default value on the STM32H723 build is `{I2S, I2S, I2S, I2S}`
(all four slots I2S) so that first boot before any preset has been
saved produces the verified all-I2S sample-aligned configuration.
The RP builds default to all S/PDIF — this is a per-platform default
and Console should not hardcode either.

---

## 5. Recommended Console UX

### A. Show dependencies in the UI

Render the four slot-type selectors as a chain or grouped layout that
makes the "slot 0 governs the others" relationship visible. Option
suggestions:

- **Grouped switches** with disabled-state visuals: when slot 0 is
  set to S/PDIF, the I2S radio buttons for slots 1/2/3 become greyed
  out with a tooltip explaining the constraint.
- **Indented layout**: slot 0 at the top level, slot 1 indented under
  slot 0, slot 2 indented under slot 0, slot 3 indented under slot 2.
- **Single mode selector**: "Output mode" presents preset combinations
  ("4× I2S", "2× I2S + 2× S/PDIF", "4× S/PDIF") and hides the per-slot
  detail. Power-users can expand to per-slot if they want it.

### B. Warn before destructive cascades

When the user picks slot 0 = S/PDIF while any of slots 1/2/3 are I2S,
show a confirmation dialog:

> Switching slot 0 to S/PDIF will also switch slots 1, 2 and 3 to
> S/PDIF — they share clock pins with slot 0 and can't run I2S
> independently on this hardware. Continue?

### C. Reflect cascaded notifications

The notify endpoint will fire one `PARAM_CHANGED` per coerced slot
*after* the slot the user changed. Console should treat these as the
authoritative state and update the UI to match — do **not** roll back
to the user's clicked value. The hardware *did* coerce.

### D. Indicate the re-init click

A small banner ("Reconfiguring outputs…") for ~250 ms after a type
change is helpful so users don't think the audible click on PE-pins
is a fault.

---

## 6. Diagnostic Helpers

Vendor command `0xFA` (vendor IN, wValue=0, wLength=4) returns 4 bytes
with each slot's live `SAI_xCR1.PRTCFG` field decoded:

- `0x00` = I2S (free protocol with manual frame config)
- `0x01` = S/PDIF
- (other values reserved)

Console can poll this after a SET to verify the firmware actually
re-initialised the hardware to the requested mode. Useful for
self-test and for in-field diagnostics.

---

## 7. Summary for Console Implementation

1. SET via vendor IN at `bRequest=0xC0`, wValue = `(type<<8) | slot`.
2. Read 1-byte status — `0x00` success, `0xFF` invalid args.
3. After every SET, listen for additional `PARAM_CHANGED` events
   covering the same `i2s_config.output_types` field at OTHER slot
   offsets — those are auto-coerced cascades, treat as authoritative.
4. Validate the combination user-side too (using the rules in §2)
   before sending, so the UI can warn before triggering a cascade.
5. Don't expect REJECT — firmware accepts all combinations and
   silently coerces. Console is responsible for surfacing the
   constraint to the user.
6. After a SET, expect ~1 ms of audible click on the affected slots
   as the SAI peripheral re-initialises. Other slots whose type
   didn't change still re-init briefly (the SAI sync chain may need
   to re-form), so all four slots glitch on any type change. This is
   expected.
