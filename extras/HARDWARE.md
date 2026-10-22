# JMCMotors — Motor Setup, Wiring & Bring-Up Guide

Everything on the physical side, in the order you do it: what to buy, how to
wire it, which drive parameters to set on each motor, how to configure the
sketch for your motors, and how to commission each axis with the UDP
commands. For the command protocol see [PROTOCOL.md](PROTOCOL.md); for
install-and-run see the [README](../README.md).

Library: https://github.com/ailagari/JMCMotors
Author: LAGARI A <lagariscience@gmail.com> · License: MIT

---

## Contents

1. [What you need](#1-what-you-need)
2. [Wiring](#2-wiring)
3. [Drive parameters (front panel, once per motor)](#3-drive-parameters-front-panel-once-per-motor)
4. [Assigning slave IDs](#4-assigning-slave-ids)
5. [Configuring the sketch for your motors](#5-configuring-the-sketch-for-your-motors)
6. [First power-up](#6-first-power-up)
7. [Commissioning each motor](#7-commissioning-each-motor)
8. [Units and scaling](#8-units-and-scaling)
9. [Scaling to 32 motors](#9-scaling-to-32-motors)
10. [Position range and travel limits](#10-position-range-and-travel-limits)
11. [Troubleshooting](#11-troubleshooting)
12. [Advanced use (direct C++ API)](#12-advanced-use-direct-c-api)

---

## 1. What you need

| Item | Notes |
|---|---|
| **Controllino MAXI** (or another Controllino with built-in RS-485 and Ethernet) | the library drives the RS-485 transceiver through the CONTROLLINO library on `Serial3` |
| **JMC integrated servo/stepper drives**, IHSS or IHT "-RC" series | the RC suffix is the RS-485/Modbus variant (e.g. IHSS60-RC) — the drives this library was verified against |
| **Motor power supply** per the drive datasheet | separate from the Controllino's 12/24 V logic supply; size it for all motors at once |
| **RS-485 cable**: twisted pair, shielded | one pair for A/B, daisy-chained; a 120 Ω resistor for the last drive |
| **Ethernet** cable + a switch or a direct cable to the PC | the Controllino needs a static IP on the PC's subnet |
| **Limit / origin switches** (optional) | PNP or NPN, wired to the drive's inputs; polarity is a drive parameter (P42) |
| A UDP tool on the PC | SocketTest, Packet Sender, `ncat -u`, or your own application |

---

## 2. Wiring

### Power

- Each drive gets motor power from the motor supply, within the voltage range
  on its datasheet. Keep the Controllino on its own logic supply.
- Common ground between the Controllino and the drives is recommended so the
  RS-485 signal stays within its common-mode range (run a GND wire alongside
  the A/B pair if the supplies are separate).

### RS-485 bus

```
Controllino RS-485 (Serial3)
   A ───┬─────────┬─────────┬──── … ──┐
   B ───┼────┬────┼────┬────┼──── … ──┼──┐
        │    │    │    │    │         │  │
      drive 1   drive 2   drive 3 … drive N   [120 Ω across A/B at drive N]
```

- **Daisy-chain** A to A, B to B, from the Controllino through every drive.
  No star topology, no long stubs.
- **120 Ω termination across A/B at the far end** (the last drive). Check
  your Controllino model's manual for whether its RS-485 port needs a
  resistor at the near end too.
- If nothing answers at all, swap A and B once — labelling differs between
  vendors.
- All drives on one bus share one baud rate (115200, see §3).

### Sensors

- Wire limit switches and the origin switch to the drive's digital inputs as
  the drive manual shows (CW limit, CCW limit, origin). The library reads
  them from the drive's status word (bits 14, 15, 11) — nothing is wired to
  the Controllino.
- Set the input polarity on the drive (**P42**: 0 = PNP, 1 = NPN) to match the
  sensors; a wrong polarity shows as a permanently "active" limit.

### Ethernet

- Controllino → the same switch as the PC, or a direct cable. The library
  uses a static IP (no DHCP), so pick a free address on the PC's subnet.

---

## 3. Drive parameters (front panel, once per motor)

These are set with the buttons and display on each drive (see the IHSS/IHT
manual for the menu keys). Parameters marked *restart* only take effect after
the drive is power-cycled.

> **⚠ Factory default is CANopen, not Modbus!** Per the IHSS60-R/RC manual,
> P43 defaults to **1** (CANopen). A drive on that setting never answers:
> `I` reports NOT DETECTED and the green LED flickers. Set **P43 = 0** and
> restart the drive. This is the most common first-run issue.

| Parameter | Set to | What it does | Restart? |
|---|---|---|---|
| **P43** | **0** | fieldbus mode: 0 = RS-485 / Modbus-RTU, 1 = CANopen (factory default) | yes |
| **P40** | 1, 2, 3 … | Modbus slave ID — unique per drive, consecutive from 1 (see §4) | yes |
| **P41** | **7** | baud rate code; 7 = 115200, which `begin()` uses by default | yes |
| **P42** | 0 or 1 | sensor input polarity: 0 = PNP, 1 = NPN (factory default 1) | yes |
| **P17** | your choice (see below) | pulses per revolution = how many steps make 360° | yes |
| **P19** | **7** for geared loads (default 2) | speed smoothness 0–10: rounds the corners of the accel/decel ramp (S-curve effect) so backlash is taken up softly | no |
| **P16** | drive default unless you see false over-position faults | following-error threshold that raises the over-position fault (status bit 13) | — |
| **P45** | leave default | velocity unit; only matters if a drive runs 10× too fast (see §8) | — |

**Choosing P17 (steps per revolution).** The library works in raw steps, so
`P:4000` is one full turn only when P17 = 4000. Factory default is setting
**2 = 1600 steps/rev**. Options: 0 = user-defined (via P20), 1 = 800,
2 = 1600, 3 = 3200, 4 = 6400, 5 = 12800, 6 = 25600, 7 = 51200, 8 = 1000,
9 = 2000, **10 = 4000**, 11 = 5000, 12 = 8000, 13 = 10000, 14 = 20000,
15 = 40000. Pick one value and use it on every drive of the same axis type;
4000 (setting 10) gives round numbers (1 step = 0.09°). P17 is panel-set
only — it is not reachable over Modbus.

**P19 in practice.** Values 5–8 all work; 7 is field-proven across many
installations. Check it on every drive: units are sometimes found at 0 (no
smoothing at all). Combine with a gentle ramp (`ACCA:0.1`, `DECA:0.1`) for the kindest
drivetrain treatment.

---

## 4. Assigning slave IDs

The controller manages slave IDs **1 .. N** where N is `setMotorCount()` (or
the runtime `MC`). Motor number in every command equals the slave ID: motor 3
is slave ID 3.

- Set the ID with **P40**, or on drives that have rotary switches:
  `ID = S2 × 16 + S1` (S1 = low digit, S2 = high digit; both 0 → ID 0, which
  is the broadcast address and must not be used).
- IDs must be **unique and gap-free** from 1. A gap (IDs 1, 2, 4) makes
  motor 3 permanently "NOT DETECTED" and costs a probe on every monitor
  sweep.
- Label each drive with its ID. The ID is the only thing that distinguishes
  motors on the bus.

---

## 5. Configuring the sketch for your motors

Open `File → Examples → JMCMotors → JMCMotors_6Motor` and edit the two
"EDIT ME" blocks.

### Network

```cpp
IPAddress CONTROLLINO_IP  (192, 168, 1, 101);   // this board, static
uint16_t  CONTROLLINO_PORT = 8001;
IPAddress PC_IP   (192, 168, 1, 100);           // where events go
uint16_t  PC_PORT  = 8000;
byte CONTROLLINO_MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
```

One static IP and one unique MAC per board. `PC_IP` must be the PC's real
address on that subnet — events only go there.

### Motor count

```cpp
const uint8_t MOTOR_COUNT = 6;    // slave IDs 1..6, up to 32
```

Set it to the number of drives **actually installed**: the event monitor's
sweep time grows with the count (§9).

### Speeds and ramps — global or per motor

```cpp
const bool PER_MOTOR_SETTINGS = false;

// GLOBAL (PER_MOTOR_SETTINGS = false): every motor gets these
float GLOBAL_ACCEL      = 0.5;   // rps/s
float GLOBAL_DECEL      = 0.5;   // rps/s
float GLOBAL_VELOCITY   = 0.3;   // rps  (0.3 rps = 18 RPM)
float GLOBAL_HOMING_VEL = 0.3;   // rps

// PER-MOTOR (PER_MOTOR_SETTINGS = true): one row per motor
MotorSettings MOTOR[MOTOR_COUNT] = {
  // velocity, accel, decel, homingVel
  { 0.3, 0.5, 0.5, 0.3 },   // motor 1
  { 0.3, 0.5, 0.5, 0.3 },   // motor 2
  …
};
```

Values in the sketch are **re-applied on every boot and whenever a drive
comes back online**, so they survive power failures. Values changed at
runtime with `V`, `AD`, `HV`, `HO`, `MC` last only until the next reboot —
use them to find the right numbers, then copy them into the sketch. The
table may have fewer rows than `MOTOR_COUNT`: motors without a row (or with
a 0 in a row) use the `GLOBAL_*` values.

### Homing offset and drive-unit quirk (optional)

```cpp
// jmc.setMotorHomingOffset(1, 1000); // steps past the sensor, per motor (like HO)
// jmc.setDriveP45(0);                // only if a motor runs 10x faster than commanded
```

### Starting point for a geared axis

| Setting | Suggested | Why |
|---|---|---|
| velocity | 0.3 rps (18 RPM) | gentle; raise once the mechanism is proven |
| accel / decel | 0.1–0.3 rps/s | 1–3 s ramps take up backlash softly |
| homing velocity | 0.2 rps | slow enough to stop precisely on the sensor |
| drive P19 | 7 | S-curve smoothing |

---

## 6. First power-up

1. Power the drives, then the Controllino. Open the Arduino Serial Monitor at
   115200 baud: `begin()` prints one line per slot,
   `JMC motor 1 -> OK` / `JMC motor 2 -> TIMEOUT`, then `6 motor(s) detected`.
2. On the PC, listen on `PC_PORT` (e.g. `ncat -u -l 8000`). Within a few
   seconds of boot `Motor ready` must arrive. If it does not: check `PC_IP`,
   the subnet, and the Windows Firewall (inbound UDP is blocked on networks
   marked "Public").
3. Send `IA` to `CONTROLLINO_IP:CONTROLLINO_PORT`. Expect
   `Total motors detected: N of N` and `SUCCESS: All expected motors detected
   and initialized!`. A `NOT DETECTED (TIMEOUT)` line names the slot to fix
   (P43, ID, baud, wiring — §11).
4. Send `SA:COMPACT`. Every motor should show `RUN|STP|TR` (enabled, standing,
   at target) and `POS=0`.

---

## 7. Commissioning each motor

Do this once per axis, with the mechanism free to move and someone at the
power switch. Replace `1` with the motor's slot where shown.

### 7.1 Direction and basic motion

```
V:0.3                 → Set Motor 1 V: 0.30
PR:4000               → OK              one revolution forward at P17 = 4000
P:?                   → M1=4000
PR:-4000              → OK              back
```

If "forward" is the wrong way for your mechanism, swap the direction on the
drive (direction parameter in the drive manual) rather than negating every
command.

### 7.2 Speed-scale verification (once per drive type)

```
V:1                   1 rps commanded (motor 1; VA:1 for all)
VSA                   run; count shaft revolutions for 60 s
B                     stop
```

- Exactly 60 revolutions → scaling is correct; decimals work (`V:0.3` = 18 RPM).
- About 600 (10× fast) → that drive uses whole-rps units: add
  `jmc.setDriveP45(0)` to the sketch (speeds then resolve in whole rps only).
- No movement → `DG:M1` and look at `vel6081`: if it reads `0,0` the speed
  did not reach the drive; report the full `DG` output.

### 7.3 Ramps

```
ACC:0.2               → Set ACC: M1=0.20 rps/s        (ACCA:0.2 for all; DEC / DECA likewise)
P:20000               → OK
```

Listen and watch for backlash clunk at start/stop; lower `AD` or raise P19
until it is gone. Then copy the values into the sketch.

### 7.4 Zero reference

Move the axis to its working origin by hand (`L:M1` frees the shaft; the
next command that needs torque re-enables the drive) or with `PR`, then:

```
Z:M1                  → Motor 1 set to zero position
ZS:M1                 → ZERO STATUS: M1:ZERO(0) | ALL_ZERO_SET
```

Or, if the origin is a known coordinate, `ZO:<value>`.

### 7.5 Homing (if the axis has an origin or limit switch)

```
HV:0.2                → Set HV: M1=0.20 rps
HO:1000               → HO: M1=1000 steps after sensor (used at next homing)
H3:M1                 → Homing method 3 started for M1      (3 = origin switch via CW)
                      ← EVENT:ORIGIN:M1:ON POS=… ST=…      (switch found)
                      ← EVENT:HOMED:M1 POS=0 ST=0x1437       (done)
LS:M1                 → LIMITS: M1:CW=0 CCW=0 SW=0 HOMED=1 MODE=HOM
P:?                   → M1=0
```

Methods 1/2 home to the CW/CCW limit switch, 3/4 to the origin switch
(reversing at the CW/CCW limit if they meet it first), 5–12 are the fast
back-off variants; 9–12 accept a negative `HO`. The full table is in
[PROTOCOL.md](PROTOCOL.md#h1--h12--start-homing). Completion arrives as
`EVENT:HOMED`, failure as `EVENT:HOMING_ERROR` (check switch wiring and
polarity, lower `HV`, then `R:M1` and retry). `LS` shows the live state
(`HOMED=BUSY` while running).

To home every axis in one go: `HVA:0.2`, `HOA:500`, `H3A` — one `EVENT:HOMED`
per motor follows.

### 7.6 Limit and origin switches

With the motor standing, press each switch by hand. The PC listening on
`PC_PORT` must receive:

| Switch | On press | On release |
|---|---|---|
| CW limit | `EVENT:LIMIT:M1:CW POS=… ST=…` | `EVENT:LIMIT:M1:CW_CLEARED` |
| CCW limit | `EVENT:LIMIT:M1:CCW POS=… ST=…` | `EVENT:LIMIT:M1:CCW_CLEARED` |
| origin (SW) | `EVENT:ORIGIN:M1:ON POS=… ST=…` | `EVENT:ORIGIN:M1:OFF` |

`LS:M1` shows the same three inputs as `CW=`, `CCW=`, `SW=`. If an input
reads active with the switch *released*, the polarity (P42) is wrong or the
wire is open.

### 7.7 Know the fault path

When a drive faults you will see `EVENT:FAULT:M1 ST=… ERR=… <name>` on
`PC_PORT`. The routine is: `SD:M1` to read the decoded cause, fix it (jam,
load, wiring, supply), then `R:M1`; `EVENT:FAULT_CLEARED:M1` confirms the
drive is back. Make sure your PC-side application handles this sequence
before the installation goes live.

### 7.8 Write it down

Copy the final `V`, `AD`, `HV`, `HO` values into the sketch's per-motor table
(or `setMotionDefaults` / `setMotorHomingOffset`) so they survive power loss,
and re-flash.

---

## 8. Units and scaling

| Quantity | Unit |
|---|---|
| velocity | rev/s (rps) — 0.1 rps resolution on the drive |
| accel / decel | rps/s — 0.1 rps/s resolution |
| position | encoder steps (steps/rev = drive P17) |

Register scaling (×10 per the JMC Modbus manual) is handled inside the library
(`libScale=10` in `DG`). Drives whose P45 is set to whole-rps units need
`setDriveP45(0)` in the sketch (`libScale=1`).

---

## 9. Scaling to 32 motors

- `jmc.setMotorCount(n)` accepts 1–32 (slave IDs 1..n). For commissioning
  you can also change it live with `MC:n` (reverts to the sketch value on
  reboot).
- **Synchronised launch is broadcast-based**: when a `P:` command addresses
  every motor (and always for `PA` and `VSA`), the arm + start are sent as two
  single broadcast frames (slave 0) — all drives start in the same instant,
  with no per-motor stagger even at 32 motors. Partial selections launch per
  motor (~5 ms apart each).
- Monitor sweep time scales with the configured count (≈ count × 13 ms), so
  **set `setMotorCount` to the number of drives actually installed**: 6 motors
  → limit events in <100 ms; 32 motors → <450 ms.
- Boot stays fast with missing drives: every slot gets a 12 ms presence probe
  first; only responding drives get the full initialisation.
- Bus timing: 115200 baud, 30 ms response timeout with one retry per
  transaction, 350 µs inter-frame gap.

---

## 10. Position range and travel limits

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
   negative — the drive gives no warning. Use **velocity mode** (`VSA`) for
   endless rotation (it consumes no position range), or periodically re-zero
   at a known point (`Z` / homing) to pull the counter back.
2. **Accuracy is closed-loop, not count-limited.** These are stepper-servos
   with encoders: every step in the range is addressed exactly. If the motor
   physically can't follow (jam, overload), the drive raises the over-position
   fault (status bit 13 / 0x1001 bit 5, threshold set by drive P16) and the
   library pushes it as `EVENT:FAULT`.
3. **Power cycle resets the counter.** A drive that loses power comes back at
   position 0 (`EVENT:ONLINE … (re-initialised)`). Re-home or `ZO` before
   trusting coordinates. The Controllino rebooting (`X`) does not touch the
   drives' counters.
4. **Practical rule:** treat ±2 billion steps as the absolute envelope, set
   your working zero with `Z`/`ZO` at commissioning, and use velocity mode for
   anything that spins forever.

---

## 11. Troubleshooting

| Symptom | Check | Fix |
|---|---|---|
| No `Motor ready`, no replies at all | `PC_IP` is the PC's real IP on the Controllino's subnet; Windows Firewall allows inbound UDP for your tool | fix the address / allow the app; reset the Controllino while listening on `PC_PORT` |
| Replies arrive but no `EVENT:` messages | events go **only** to `PC_IP:PC_PORT`, replies also echo to the sender | listen on `PC_PORT` |
| `INVALID COMMAND` | head must be upper case; one command per packet | `P:1000`, not `p:1000` |
| `I`: `NOT DETECTED (TIMEOUT)` on every motor | P43 = 0 on each drive? baud P41 = 7? A/B swapped? termination? | set P43 = 0 and **restart the drive**; swap A/B once; add 120 Ω at the far end |
| `NOT DETECTED` on one motor only | its slave ID (P40 / rotary) matches its slot? duplicate ID? power? | set a unique ID 1..N, restart the drive |
| `NOT DETECTED (CRC)` or `BADFRAME` | noise on the bus, missing termination, long stubs | shielded twisted pair, terminate, common ground |
| Motor replies `OK` but does not move | `DG:n` → `vel6081=0,0`? `mode6061` not 1? `err1001` non-zero? `WRITE10=EXCEPTION`? | send `V`, `R`, or check the drive's enable input; report the `DG` line |
| Motor runs 10× too fast | drive P45 velocity unit | `jmc.setDriveP45(0)` in the sketch |
| A limit shows `*** ACTIVE ***` with nothing pressed | P42 polarity, or the input wire is open | set P42 to match PNP/NPN sensors; check wiring |
| `EVENT:HOMING_ERROR` after `H<n>` | the method's switch is not wired or has the wrong polarity, `HV` too fast for the method's limit, `HO` pushes past a limit | check `LS` while pressing the switches, fix P42, lower `HV`, `R:…`, retry |
| `EVENT:FAULT … OVERPOS` on normal moves | mechanism binds, ramps too steep, P16 too tight | lower `AD`, raise P19, check the load, adjust P16 |
| `EVENT:FAULT … CURRENT` at start | stall, short, wrong motor phase wiring | check the motor cable and load |
| `EVENT:OFFLINE` / `ONLINE` flapping | loose RS-485 connector, marginal power | reseat connectors; check the motor supply under load |
| `P` after `VS` answers late or not at all | the controller is braking the jog on the `DEC` ramp first (`OK` is sent when the move starts) | send `B` before repositioning, or use a steeper decel |
| Settings forgotten after power loss | `V`/`AD`/`HV`/`HO`/`MC` are runtime only | put them in the sketch |
| Positions are 0 after a drive power cycle | the drive's counter reset | re-home or `ZO:<known value>` |
| `L:2` loosened motor 1 | selector commands are positional | `L:,1` for motor 2 |

---

## 12. Advanced use (direct C++ API)

`JMCController::motor(i)` (0-based) returns the underlying `JMCMotor*` for
direct calls (`moveToPosition`, `performHoming`, `getDetailedStatus`,
`quickStop`, …), and `busRef()` exposes the raw Modbus transport
(`readHolding`, `writeSingle`, `writeMultiple`). Every call returns a
`JMCResult` (`JMC_OK`, `JMC_ERR_TIMEOUT`, `JMC_ERR_CRC`, `JMC_ERR_EXCEPTION`,
`JMC_ERR_BADFRAME`, `JMC_ERR_PARAM`) so comms failures are never silent. See
[src/JMCMotors.h](../src/JMCMotors.h) for the full declarations.
