# LagariMotors — Hardware Setup & Bring-Up Guide

Everything about the physical side: drive parameters, wiring, first-run
verification, scaling, and position limits. For the command protocol see
[PROTOCOL.md](PROTOCOL.md); for install-and-run see the
[README](../README.md).

Library: https://github.com/ailagari/LagariMotors
Author: LAGARI A <lagariscience@gmail.com> · License: MIT

---

## Wiring

- RS-485 **A/B daisy-chain** from the Controllino's built-in RS-485 (Serial3)
  to every drive, **120 Ω termination at the last drive**.
- Ethernet from the Controllino to the same switch/subnet as the PC.

## Drive setup (once per drive, on the front panel)

> **⚠ Factory default is CANopen, not Modbus!** Per the IHSS60-R/RC manual,
> P43 defaults to **1** (CANopen). If a drive never answers (`I` reports NOT
> DETECTED, green LED flickering), set **P43 = 0** (RS-485/Modbus) and
> restart the drive. This is the most common first-run issue.

- **P43 = 0** — RS-485 / Modbus-RTU mode (factory default is 1 = CANopen!). Restart required.
- Unique **slave ID** 1..N (P40 / rotary switches: `ID = S2×16 + S1`)
- **Baud** 115200 (P41 / BD switch = 7). Restart required.
- Sensor polarity **P42** (0 = PNP, 1 = NPN; factory default 1 = NPN). Restart required.
- **P19 (speed smoothness, 0–10)** — for geared mechanisms set **7** (no
  restart needed; 5–8 all work, 7 is field-proven across many installations).
  It rounds the corners of the accel/decel ramp (S-curve effect) so backlash
  is taken up softly. Combine with gentle ramps (`AD:0.1`) for the kindest
  drivetrain treatment. Factory default 2 — and check it: drives are
  sometimes found at 0 (no smoothing at all).
- **P17 (pulses/revolution)** — sets how many steps = 360°. **Factory default
  is setting 2 = 1600 steps/rev.** Options: 0=user-defined (via P20), 1=800,
  2=1600, 3=3200, 4=6400, 5=12800, 6=25600, 7=51200, 8=1000, 9=2000,
  **10=4000**, 11=5000, 12=8000, 13=10000, 14=20000, 15=40000. Panel-set
  only (restart required), not reachable over Modbus. Positions in this
  library are raw steps, so `P:4000` = one full turn only when P17 = 4000.

## Units

| Quantity | Unit |
|---|---|
| velocity | rev/s (rps) |
| accel / decel | rps/s |
| position | encoder steps (steps/rev = drive P17) |

Register scaling (×10 per the JMC manual) is handled inside the library.

## Verify speed scaling on first bring-up

Send `V:1` then `VS` — the shaft must turn exactly **60 revolutions in 60 s**
(then `B` to stop).

- Exactly 60 → scaling is correct, decimals work (`V:0.3` = 18 RPM).
- ~600 revs (10× fast) → that drive uses whole-rps units: add
  `jmc.setDriveP45(0)` (speeds then resolve in whole rps only).
- No movement at all → run `DG:1` and check `vel6081` — if it reads 0, the
  commanded speed didn't reach the drive; report the full DG output.

## Scaling to 32 motors

- `jmc.setMotorCount(n)` accepts 1–32 (slave IDs 1..n) — set it per project in
  the sketch. For commissioning you can also change it live over UDP with
  `MC:n` (reverts to the sketch value on reboot).
- **Synchronised launch is broadcast-based**: when a `P:` command addresses
  every motor (and always for `VS`), the arm+start are sent as two single
  broadcast frames (slave 0) — all drives start in the same instant, with no
  per-motor stagger even at 32 motors. Partial selections launch per motor
  (~5 ms apart each).
- Monitor sweep time scales with the configured count (≈ count × 13 ms), so
  **set `setMotorCount` to the number of drives actually installed**: 6 motors
  → limit events in <100 ms; 32 motors → <450 ms.
- Boot stays fast with missing drives: every slot gets a 12 ms presence probe
  first; only responding drives get the full initialisation.

## Position range & travel limits

**Hard limit: ±2,147,483,647 steps** (signed 32-bit — the width of the drive's
position registers 0x607A/0x6064). The library carries the full range
end-to-end: `long` positions, 32-bit Modbus transfers, and `P:` commands parse
10-digit values, positive or negative. A single move may span the whole range —
the drive limits the counter width, not the move length.

What that means in revolutions depends on the drive's P17 (steps/rev):

| P17 setting | Steps/rev | Max travel from zero (each direction) |
|---|---|---|
| 2 (factory default) | 1,600 | ≈ 1,342,000 revolutions |
| 10 | 4,000 | ≈ 536,800 revolutions |
| 7 (largest preset) | 51,200 | ≈ 41,900 revolutions |

For any positioning mechanism this is effectively unlimited — at 4,000
steps/rev and 18 RPM you would have to run ~20 days in one direction to reach
the limit.

**Reliability notes:**

1. **Counter wrap.** An application that rotates endlessly in one direction
   (e.g. a display turntable) will eventually pass +2,147,483,647 and wrap to
   negative — the drive gives no warning. Use **velocity mode** (`VS`) for
   endless rotation (it consumes no position range), or periodically re-zero
   at a known point (`Z` / homing) to pull the counter back.
2. **Accuracy is closed-loop, not count-limited.** These are stepper-servos
   with encoders: every step in the range is addressed exactly. If the motor
   physically can't follow (jam, overload), the drive raises the over-position
   fault (status bit13 / 0x1001 bit5, threshold set by drive P16) and the
   library pushes it as `EVENT:FAULT`.
3. **Practical rule:** treat ±2 billion steps as the absolute envelope, set
   your working zero with `Z`/`ZO` at commissioning, and use velocity mode for
   anything that spins forever.

## Advanced use (direct API)

`JMCController::motor(i)` returns the underlying `JMCMotor*` for direct API
calls (`moveToPosition`, `performHoming`, `getDetailedStatus`, …), and
`busRef()` exposes the raw Modbus transport (`readHolding`, `writeSingle`,
`writeMultiple`) — every call returns a `JMCResult` so comms failures are
never silent.

## Recommended bring-up checks on any new installation

1. `I` → all drives detected
2. `P:` move → motor moves, position reads back
3. Speed verification: `V:1` + `VS` → exactly 60 revs in 60 s → `B`
4. Trip a limit switch → `EVENT:LIMIT` packet arrives at the PC
