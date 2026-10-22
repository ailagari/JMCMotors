# JMCMotors — UDP Command Reference (v0.5)

The complete reference for the text protocol implemented by this library: the
one grammar every command follows, every command with its arguments, what each
one **triggers** inside the controller and on the RS-485 bus, every reply it can
produce, the unsolicited **events** the controller pushes on its own, and the
status/error words you will see in those replies.

Library: https://github.com/ailagari/JMCMotors
Author: LAGARI A <lagariscience@gmail.com> · License: MIT

Companion documents: [HARDWARE.md](HARDWARE.md) (drive setup, wiring, bring-up)
· [README](../README.md) (install and quick start).

---

## Contents

1. [Transport and addressing](#1-transport-and-addressing)
2. [The grammar: one rule for every command](#2-the-grammar-one-rule-for-every-command)
3. [Reply types and multi-packet markers](#3-reply-types-and-multi-packet-markers)
4. [Motion commands](#4-motion-commands) — `P` `PR`
5. [Speed and ramp commands](#5-speed-and-ramp-commands) — `V` `VS` `ACC` `DEC`
6. [Stopping commands](#6-stopping-commands) — `B` `B1` `B2` `B3` `L` `BS`
7. [Zero and homing commands](#7-zero-and-homing-commands) — `Z` `ZO` `ZS` `HV` `HVZ` `HACC` `HO` `H1`…`H12`
8. [Status and diagnostic commands](#8-status-and-diagnostic-commands) — `S` `SD` `LS` `DG`
9. [Controller commands](#9-controller-commands) — `I` `R` `MC` `SAVE` `LOAD` `VER` `PING` `X`
10. [Events: what the controller sends on its own](#10-events-what-the-controller-sends-on-its-own)
11. [Trigger and response matrix](#11-trigger-and-response-matrix)
12. [Status word, drive states and error register](#12-status-word-drive-states-and-error-register)
13. [Error message catalogue](#13-error-message-catalogue)
14. [Recipes and sequences](#14-recipes-and-sequences)
15. [Changes from 0.4](#15-changes-from-04)

---

## 1. Transport and addressing

| Item | Value |
|---|---|
| Protocol | UDP, plain ASCII text, one command per datagram |
| Controller listens on | `CONTROLLINO_IP:CONTROLLINO_PORT` (sketch: `setNetwork()`, or `setStationId(n)` → `192.168.1.(100+n):8001`) |
| Command replies go to | the **sender** of the command (its IP and source port) **and** the configured `PC_IP:PC_PORT` (sent once if those are the same) |
| Events and `Motor ready` go to | `PC_IP:PC_PORT` **only** — fixed for the life of the sketch |
| Packet size | up to 511 characters (enough for 32 positions) |
| Terminator | none needed; leading/trailing whitespace and line endings are trimmed |
| Case | command heads may be upper or lower case (`p:1000` works); values are case-insensitive too |
| Authentication | none — any host that can reach the port can command the motors |
| Unknown command | reply `ERR:unknown command <HEAD>` |

### Static addressing, no pairing

There is no discovery, beacon or auto-adopt. Both addresses are fixed in the
sketch:

```cpp
jmc.setNetwork(mac,
               controllinoIp, controllinoPort,   // this board
               pcIp,          pcPort);           // where events go
```

- **Controller side** — the board's own static IP/port. `setStationId(n)` is a
  shortcut that derives a unique MAC (`DE:AD:BE:EF:00:<n>`) and IP
  (`192.168.1.100+n`) from one number.
- **PC side** — every `EVENT:` and the boot message go here, always. Because
  replies are *also* echoed to the sender, any UDP tool sees its answers
  without listening on `pcPort`. To receive **events** you must listen on
  `PC_IP:PC_PORT`.

### Boot message

When `begin()` has probed and initialised every configured motor slot, the
controller sends one datagram to `PC_IP:PC_PORT`:

```
Motor ready
```

It is sent regardless of how many drives answered — use `IA` for the
per-motor detection report, `VER` to confirm the firmware version.

---

## 2. The grammar: one rule for every command

```
CMD:<slots>        individual motors
CMDA[:value]       all motors   (the A suffix; the colon is optional with no value)
```

| Part | Rule | Examples |
|---|---|---|
| `CMD` | upper-case word; stop type and homing method are the digit in the head | `P` `V` `B2` `H3` `LS` |
| `<slots>` positional | field N = motor N, an **empty field skips** that motor; a single value with **no comma = motor 1** | `P:1000,,3000` (M1, M3) · `V:0.5` (M1) · `B2:1,,1` |
| `<slots>` by name | motor names instead of counting: `M3=500` for a value, `M3` for a selector or a read; any order | `P:M17=1000,M3=500` · `L:M2` · `V:M3` |
| `?` | read instead of write; in a slot, or `M3=?` / `M3` by name | `P:?` (M1) · `P:,,?` (M3) · `V:M3` · `HO:?` |
| `CMDA` | every motor; one value (or `?`) applies to all | `PA:5000` · `VA:0.5` · `BA` · `LA` · `HOA:?` |
| options | status format after a second colon | `S:1,,1:COMPACT` · `SA:FLAGS` |
| bare `CMD` | an error (`ERR:L needs motor slots …`) — **except the stop commands**: `B`, `B1`, `B2`, `B3` with nothing after them stop every motor, because a stop is never refused | |
| controller commands | no slots, no A form | `MC:12` `SAVE` `LOAD` `VER` `PING` `X` |

Selector commands (anything selects: `B2:1,,1`, `L:,1`) read a slot as a
flag; value commands (`P`, `V`, …) read it as a number. Both accept the
`M<n>` form, which is the easier one to generate from software and the only
practical one past a dozen motors (`P:,,,,,,,,,,,,,,,,1000` versus
`P:M17=1000`).

### Units

| Quantity | Unit | Notes |
|---|---|---|
| Position | encoder **steps**, signed 32-bit (±2,147,483,647) | steps per revolution = drive parameter **P17** |
| Velocity | **rps** (revolutions per second), decimals allowed | 0.1 rps resolution on the drive; 0.3 rps = 18 RPM; negative = reverse jog |
| Accel / decel | **rps/s** | 0.1 rps/s resolution |
| Homing offset | steps | sign = direction past the sensor |

### Parsing rules worth knowing

- A value that is not a number parses as **0** (`P:abc` moves motor 1 to step
  0). Validate on the PC side.
- A motor name above the configured count (`MC`) is refused:
  `ERR:P motor must be M1..M6`.
- Slots beyond the configured motor count are ignored.
- Decimals use a dot: `V:0.5`. A comma is always a field separator.
- The motor count is one variable: `setMotorCount(n)` in the sketch, `MC:n`
  live, or the count stored by `SAVE`; 1–32. Every command, detection and
  the event monitor follow it.

### Per-motor ↔ all-motors at a glance

| Action | Individual | All motors |
|---|---|---|
| absolute move | `P:1000,,3000` · `P:M17=1000` | `PA:5000` |
| relative move | `PR:-500,,200` | `PRA:-500` |
| read position | `P:?` · `P:,,?` · `P:M3` | `PA:?` |
| run speed | `V:0.5,,0.3` · `V:?` | `VA:0.5` · `VA:?` |
| jog | `VS:1,,1` | `VSA` |
| acceleration · deceleration | `ACC:0.2,,0.5` · `DEC:0.1` | `ACCA:0.3` · `DECA:0.1` |
| stop (type 0–3) | `B:,1` · `B2:1,,1` | `BA` · `B2A` (also bare `B` … `B3`) |
| loosen brake | `L:,1` | `LA` |
| zero here · position = value | `Z:1,,1` · `ZO:5000,,100` | `ZA` · `ZOA:5000` |
| homing speed · zero-find speed · homing ramp · offset | `HV:0.2` · `HVZ:0.05` · `HACC:0.2` · `HO:1000,,-800` | `HVA:0.2` · `HVZA:0.05` · `HACCA:0.2` · `HOA:1000` |
| start homing | `H3:1,,1` | `H3A` |
| status | `S:1,,1:COMPACT` `SD:,,1` `LS:M1,M3` `BS:,1` `ZS:,1` `DG:M3` | `SA` `SDA` `LSA` `BSA` `ZSA` `DGA` |
| detect · reset | `I:1,,1` · `R:,,1` | `IA` · `RA` |

---

## 3. Reply types and multi-packet markers

| Reply type | Looks like | Produced by |
|---|---|---|
| **Acknowledge** | `OK` | `P` `PR` `PA` `PRA` when the move has been launched |
| **Confirmation** | a sentence naming what was done, e.g. `Fast brake applied to motor 1,3` | stops, `L`, `Z`, `ZO`, `R`, `VS`, `H<n>`, settings |
| **Data line** | `M1=10000,M3=200` · `STATUS:M1=0x1437,…` · `V: M1=0.50,M3=0.30` | reads (`?`), `S`, `BS`, `ZS`, `LS` |
| **Multi-line packet** | several lines in one datagram | `I`, `SD`, `S:…:DETAIL`, `DG` |
| **One packet per motor** | N separate datagrams | `SD`, `S:…:DETAIL`, `DG` with more than one motor |
| **Warning** | `WARN: M1=10000,M2=ERROR` | a data reply where at least one motor did not answer on RS-485 |
| **Error** | `ERR:<reason>` | bad arguments; nothing was sent to the drives |
| **Unknown** | `ERR:unknown command PX` | head not recognised |
| **Deferred** | `OK` arrives later | `P`/`PR` sent while motors are in velocity mode (see §4) |
| **No further reply** | — | `X` answers once, then reboots |

**Multi-packet markers.** Whenever one command answers with more than one
datagram, every packet but the last ends with `:MORE` and the last ends with
`:END`. A single-packet reply carries no marker. This happens for the
per-motor-packet commands (`SD`, `DG`, `S:…:DETAIL`) when more than one motor
is selected, and for any list reply that grows past about 300 characters
(from roughly 16 motors up), in which case each continuation packet repeats
the reply's header (`STATUS:`, `LIMITS: `, …) and starts a fresh list:

```
LIMITS: M1:CW=0 … ,M12:CW=0 CCW=0 SW=0 HOMED=0 MODE=POS:MORE
LIMITS: M13:CW=0 … ,M24:…:MORE
LIMITS: M25:CW=0 … ,M32:…:END
```

Per-motor comms failures inside a data reply show as `ERROR` for that motor
(`S`, `P:?`, `BS`, `ZS`, `LS`) or `NOT RESPONDING (<reason>)` (`SD`, `I`). The
reason is one of the bus result names: `TIMEOUT`, `CRC`, `EXCEPTION`,
`BADFRAME`, `PARAM`.

---

## 4. Motion commands

### `P` — absolute position move · `PA` — all motors

| Send | Effect | Reply |
|---|---|---|
| `P:10000` | motor 1 → step 10000 | `OK` |
| `P:1000,2000,3000` | M1, M2, M3 launched together | `OK` |
| `P:500,,1500` · `P:M1=500,M3=1500` | M1 → 500, M3 → 1500, skip M2 | `OK` |
| `P:-40000` | negative targets are fine | `OK` |
| `PA:5000` | every motor → 5000, broadcast start, at motor 1's `V` speed | `OK` |
| `P:?` · `P:,,?` · `P:?,?` · `P:M3` | **read** position of M1 · M3 · M1+M2 · M3 | `M1=10000` · `M3=200` · `M1=10000,M2=2000` |
| `PA:?` | read every motor | `M1=…,M2=…,…` |

**What it triggers**

1. For each addressed motor the controller writes the target (0x607A) and the
   motor's stored velocity (`V`) into the drive — *prepare*.
2. Then it launches. `PA`, and a `P` that addresses every motor, launch with
   **broadcast** frames (slave ID 0) so all drives start in the same instant;
   a subset launches per motor (about 5 ms apart each). `PA` also *prepares*
   by broadcast, so all motors run at **motor 1's `V` speed**.
3. The new target takes effect **immediately** (control-word bit 5): a `P`
   sent mid-move abandons the move in flight and heads to the new target.
4. `OK` means "launched", not "arrived". Watch `S:n:COMPACT` (`TR` = target
   reached) to know when the motor is there.

**If the motors are in velocity mode (`VS`)**: changing a drive's mode
register while the shaft turns stops it dead, so the controller first brakes
every addressed motor on its `DEC` ramp, waits for standstill (polling every
50 ms, up to 20 s), then launches all of them together. During the wait the
controller keeps answering other commands and pushing events; **`OK` is sent
when the move actually starts**. A `B`, `VS`, `I` or `R` received during the
wait cancels the queued move. Stopping with `B` *before* repositioning avoids
the wait.

**Errors**: `ERR:P needs values (e.g. P:1000,,3000 or P:M3=1000) - PA:<value> = all motors`
· `ERR:P motor must be M1..M6 (e.g. P:M3=1000)` · `ERR:PA takes one value for all
motors (e.g. PA:1000)` · `ERR:P cannot mix values and ?`.

### `PR` — relative position move · `PRA` — all motors

Same syntax, launch sequence, deferral behaviour and errors as `P` (with
`PR` in the messages), but every value is a **delta from the motor's current
position**: `PR:1000` forward 1000, `PR:-1000` back, `PR:500,,-500`
synchronised, `PRA:-250` every motor back 250. The offset is applied inside
the drive (control-word bit 6), so there is no read-modify-write race even if
the motor is still moving. `PR:?` and `PRA:?` read positions, identical to
`P:?` / `PA:?`.

---

## 5. Speed and ramp commands

`V`, `ACC`, `DEC`, `HV`, `HVZ` and `HACC` are the six per-motor float settings
and share one behaviour: positional slots or `M<n>=value`, `?` reads, the
`A` form applies one value to every motor. Runtime values live in RAM and are
lost on reboot **unless stored with `SAVE`**; values set in the sketch are
re-applied on every boot and whenever a drive comes back online.

### `V` — run velocity per motor (rps) · `VA`

| Send | Effect | Reply |
|---|---|---|
| `V:0.5` | motor 1 = 0.5 rps (stored; written at the next move or jog) | `Set V: M1=0.50 rps` |
| `V:0.5,1,0.3` · `V:M1=0.5,M3=0.3` | per motor | `Set V: M1=0.50,M2=1.00,M3=0.30 rps` |
| `V:-0.5` | negative = reverse direction for `VS` | `Set V: M1=-0.50 rps` |
| `VA:1` | every motor; also **broadcast to the drives now**, so a running jog changes speed at once | `VA: all motors = 1.00 rps (BROADCAST)` |
| `V:?` · `V:,,?` · `V:M3` · `VA:?` | read | `V: M1=0.50 rps` · `V: M1=0.30,M2=0.30,… rps` |

Default 0.3 rps = 18 RPM. Range 0.01–1000 rps in magnitude. **Errors**:
`ERR:V needs values (…)` · `ERR:V range 0.01..1000.0 rps (M2)` · `ERR:V cannot
mix values and ?`.

### `VS` — start velocity mode (endless rotation) · `VSA`

| Send | Effect | Reply |
|---|---|---|
| `VSA` | all motors jog at their `V` speeds, broadcast start | `VS: velocity mode started (stop with B)` |
| `VS:1,,1` · `VS:M1,M3` | motors 1 and 3 only | `VS: velocity mode started for M1,3 (stop with B)` |

**What it triggers**: cancels any deferred move, writes each selected motor's
own speed and switches it to velocity mode, then one broadcast enable + start
when every motor is selected, or a per-motor enable + start for a subset.
Velocity mode consumes no position range: use it for anything that spins
forever. **Leaving it**: `B` (any type) stops it; `P`/`PR`, `Z`, `ZO` and
`H<n>` brake on the `DEC` ramp first (see §4). **Errors**: `ERR:VS needs motor
slots (e.g. VS:1,,1 or VS:M1,M3) - VSA = all motors`.

### `ACC` — acceleration (rps/s) · `ACCA`, and `DEC` — deceleration · `DECA`

| Send | Effect | Reply |
|---|---|---|
| `ACC:0.2,,0.5` | M1 accel 0.2, M3 0.5 rps/s, written to the drives now | `Set ACC: M1=0.20,M3=0.50 rps/s` |
| `DEC:0.1` | motor 1 decel 0.1 rps/s | `Set DEC: M1=0.10 rps/s` |
| `ACCA:0.3` · `DECA:0.1` | every motor | `ACCA: all motors = 0.30 rps/s` |
| `ACC:?` · `DEC:M3` · `ACCA:?` | read | `ACC: M1=0.50 rps/s` · `ACC: M1=0.50,M2=0.50,… rps/s` |

Setting `DEC` also refreshes the quick-stop ramp used by `B2`. Ramp time =
speed ÷ ramp (0.3 rps at 0.1 rps/s = 3 s). For geared mechanisms pair a slow
ramp with drive parameter **P19 = 7**. Default 0.5 rps/s each. Range
0.1–6553.5. **Errors**: `ERR:ACC range 0.1..6553.5 rps/s (M2)` and the
generic ones above.

---

## 6. Stopping commands

### `B`, `B1`, `B2`, `B3` — stop

The **stop type is the digit in the head**; the argument selects motors.

| Head | Stop type | How the drive stops |
|---|---|---|
| `B` (= `B0`) | controlled stop | halt bit: ramps down on the normal `DEC` ramp; the jog setpoint is zeroed so the motor cannot resume when halt is released |
| `B1` | quick stop, decel ramp | quick-stop code 1: decelerate on the normal ramp, then hold |
| `B2` | fast brake | quick-stop code 2: decelerate on the quick-stop ramp |
| `B3` | immediate | quick-stop code 3: stop at once, no ramp — the emergency stop |

| Send | Effect | Reply |
|---|---|---|
| `B` · `BA` | controlled stop, all motors | `All motors stopped with deceleration` |
| `B:,1` · `B:M2` | controlled stop, motor 2 | `Motor 2 stopped with deceleration` |
| `B1` · `B1A` · `B1:1,,1` | quick stop on decel ramp, all / motors 1 and 3 | `Deceleration brake applied to all motors` / `… to motor 1,3` |
| `B2` · `B2A` · `B2:M1,M3` | fast brake | `Fast brake applied to all motors` / `… to motor 1,3` |
| `B3` · `B3A` · `B3:,1` | immediate stop | `Immediate brake applied to all motors` / `… to motor 2` |

**The emergency exception.** Every other command refuses a bare head, but
`B`, `B1`, `B2` and `B3` with nothing after them stop **all** motors, so a
stop is never refused for a missing selector.

**What it triggers**: cancels any move waiting for a ramp-down ("stop wins"),
then sends the stop to each selected motor. After a `B` the status word shows
`HALT` (bit 8); after a quick stop the drive sits in "Quick stop active". The
next `P` or `VS` re-enables automatically. `BS` reports the result.
**Errors**: `ERR:B type is B, B1, B2 or B3` · `ERR:B motor must be M1..M6`.

### `L` — loosen brakes (release torque) · `LA`

| Send | Effect | Reply |
|---|---|---|
| `LA` | all motors | `All motor brakes loosened` |
| `L:,1` · `L:M2` | motor 2 | `Motor 2 brake loosened` |
| `L:1,,1` | motors 1 and 3 | `Motor 1,3 brake loosened` |

Opens the brake and **disables motor torque**: a loosened axis will not hold
position. The next move re-enables the drive. **Errors**: `ERR:L needs motor
slots (e.g. L:1,,1 or L:M1,M3) - LA = all motors`.

### `BS` — brake status · `BSA`

```
Send:  BSA
Reply: BRAKE STATUS: M1:FREE/RUN,M2:BRAKE/STOP,M3:ERROR | SUMMARY: MIXED
Send:  BS:,1
Reply: BRAKE STATUS: M2:BRAKE/STOP | SUMMARY: BRAKED
```

| Token | Meaning |
|---|---|
| `BRAKE` | drive is halted (bit 8) **or** not enabled (bit 2 clear) |
| `FREE` | enabled and not halted |
| `RUN` / `STOP` | motor running (bit 9) / standing |
| `ERROR` | no RS-485 answer |
| `SUMMARY` | `BRAKED`, `FREE`, `MIXED`, or `ERROR` (none of the selected answered) |

---

## 7. Zero and homing commands

### `Z` — current position becomes 0 · `ZA`

| Send | Effect | Reply |
|---|---|---|
| `ZA` | all motors read 0 where they stand | `All motors set to zero position` |
| `Z:1,,1` · `Z:M1,M3` | motors 1 and 3 | `Motor 1,3 set to zero position` |

**What it triggers**: a jogging motor is braked to standstill first (blocking,
max 5 s); then, per motor, homing method 0 "set current position", a 20 ms
pause, and back to position mode. The shaft does not move. The controller
marks the motor as zero-set for `ZS`.

### `ZO` — current position becomes a value · `ZOA`

| Send | Effect | Reply |
|---|---|---|
| `ZO:10000` | where motor 1 is now reads 10000 | `ZO: M1=10000 (current position redefined)` |
| `ZO:10000,,500` · `ZO:M1=10000,M3=500` | M1 reads 10000, M3 reads 500 | `ZO: M1=10000,M3=500 (current position redefined)` |
| `ZOA:5000` | every motor reads 5000 | `ZO: M1=5000,M2=5000,… (current position redefined)` |

Use it to restore a known coordinate after a drive power cycle without
re-homing. Internally `Z` and `ZO` run homing method 0 with the value in the
drive's home-offset register — the same register `HO` uses — which is why
every `H<n>` re-writes the stored `HO` first. **Errors**: `ERR:ZO needs
values (…)` · `ERR:ZO takes values, not ? (read positions with P:?)`.

### `ZS` — zero status · `ZSA`

```
Send:  ZSA
Reply: ZERO STATUS: M1:ZERO(0),M2:NOT_SET(1500),M3:ERROR | ZERO_NOT_SET
```

`ZERO` means a `Z` or `ZO` was applied to that motor since boot. **Homing does
not set this flag**; the homing state is in `LS` and `SD`. The number in
brackets is the live position.

### `HV` — homing search speed (rps) · `HVA`

| Send | Effect | Reply |
|---|---|---|
| `HV:0.3,,0.5` · `HV:M3=0.5` | per motor; written to the drive now (0x6099) | `Set HV: M1=0.30,M3=0.50 rps` |
| `HVA:0.2` | every motor | `HVA: all motors = 0.20 rps` |
| `HV:?` · `HVA:?` | read | `HV: M1=0.30 rps` |

Default 0.3 rps. Keep it under the chosen method's maximum (§ `H<n>`).

### `HVZ` — homing zero-find speed (rps) · `HVZA`

The slow final approach after the switch has been found (0x609B). Unset, it
**follows `HV`**; set it lower than `HV` for repeatable zeros with the fast
methods 5–12.

| Send | Effect | Reply |
|---|---|---|
| `HVZ:0.05` · `HVZA:0.05` | motor 1 / every motor | `Set HVZ: M1=0.05 rps` · `HVZA: all motors = 0.05 rps` |
| `HVZ:?` · `HVZA:?` | read (shows the `HV` value while unset) | `HVZ: M1=0.05 rps` |

### `HACC` — homing acceleration (rps/s) · `HACCA`

The drive's own homing ramp (0x609A). Unset, it **follows the motor's `ACC`**.

| Send | Effect | Reply |
|---|---|---|
| `HACC:0.2,,0.5` · `HACCA:0.2` | per motor / every motor | `Set HACC: M1=0.20,M3=0.50 rps/s` · `HACCA: all motors = 0.20 rps/s` |
| `HACC:?` · `HACCA:?` | read | `HACC: M1=0.20 rps/s` |

### `HO` — homing offset (steps past the sensor) · `HOA`

| Send | Effect | Reply |
|---|---|---|
| `HO:1000` | motor 1 stops 1000 steps past the sensor | `HO: M1=1000 steps after sensor (used at next homing)` |
| `HO:1000,,-800` · `HO:M3=-800` | per motor; negative = the other direction | `HO: M1=1000,M3=-800 steps after sensor (used at next homing)` |
| `HOA:500` | every motor | `HO: M1=500,M2=500,… steps after sensor (used at next homing)` |
| `HO:?` · `HO:M3` · `HOA:?` | read stored offsets | `HO: M1=1000` · `HO: M1=1000,M2=0,…` |

Stored in the controller, written to the drive now, and **re-written before
every `H<n>` and every drive re-initialisation**.

### `H1` … `H12` — start homing · `H<n>A`

| Send | Effect | Reply |
|---|---|---|
| `H3A` | home all motors, method 3 | `Homing method 3 started for all motors` |
| `H3:1,,1` · `H3:M1,M3` | motors 1 and 3 | `Homing method 3 started for M1,3` |
| `H1:,1` | motor 2, method 1 | `Homing method 1 started for M2` |

The drive supports homing methods 0–12 (JMC Modbus manual V2.2, §6.3).
Method 0 is what `Z`/`ZO` use. The three inputs involved are the **CW limit**,
the **CCW limit** and the **origin switch** ("SW mechanical origin", status
bit 11). "Left" = towards the CW limit, "right" = towards the CCW limit.

| Method | Uses | Behaviour (per the JMC manual) | Max homing speed |
|---|---|---|---|
| 1 / 2 | CW / CCW limit | move to the limit and stop there | 300 rpm (5 rps) |
| 3 / 4 | origin switch, CW / CCW limit as fallback | move to the origin switch; reverse at the limit if met first | 300 rpm |
| 5 / 6 | CW / CCW limit | fast to the limit, back off, re-approach slowly | 1200 rpm |
| 7 / 8 | origin switch, CW / CCW limit as fallback | fast to the origin, back off, re-approach slowly | 1200 rpm |
| 9 / 10 | CW / CCW limit | as 5 / 6, `HO` may be positive or negative | 1200 rpm |
| 11 / 12 | origin switch, CW / CCW limit as fallback | as 7 / 8, signed `HO` | 1200 rpm |

Methods that end on a limit switch also raise the normal `EVENT:LIMIT:…` on
the way — that is expected.

**What it triggers**, per selected motor: brake to standstill if jogging
(blocking, max 5 s) → re-write `HV`, `HVZ`, `HACC` and `HO` → switch to homing
mode → write the method → pulse the start bit → arm the event monitor. The
reply means **homing has started**. Completion and failure are pushed as
events: `EVENT:HOMED:M1 POS=0 ST=…` (status bit 12 in homing mode) and
`EVENT:HOMING_ERROR:M1 ST=…` (bit 13). `LS` shows `HOMED=BUSY` meanwhile. The
drive stays in homing mode until the next move command, which switches it
back to position mode automatically. **Errors**: `ERR:homing method H1..H12`
· `ERR:H needs motor slots (e.g. H:1,,1 or H:M1,M3) - HA = all motors` (read
`H3:…` / `H3A`).

---

## 8. Status and diagnostic commands

### `S` — status word in six formats · `SA`

`S:<slots>[:FORMAT]` or `SA[:FORMAT]`. Formats: `RAW` (default) `COMPACT`
`STATE` `BITS` `FLAGS` `DETAIL`, case-insensitive.

| Format | Send | Reply | Use when |
|---|---|---|---|
| `RAW` | `SA` · `S:1,,1` | `STATUS:M1=0x1437,M3=0x1637` | logging / machine parsing |
| `COMPACT` | `S:M1:COMPACT` | `M1:RUN\|STP\|TR\|POS=10000\|VEL=0.0` | polling a move until `TR` |
| `STATE` | `SA:STATE` | `STATES:M1:Device enable,M2:Fault alarm` | which drive state each motor is in |
| `BITS` | `S:M1:BITS` | `M1:0001 0100 0011 0111` | comparing with the manual's bit table |
| `FLAGS` | `S:M1:FLAGS` | `M1:READY\|INIT\|ENABLED\|WORKING\|QSTOP\|REACHED\|SETPOINT_OK` | named bits at a glance |
| `DETAIL` | `SA:DETAIL` | one multi-line packet **per motor**, with `:MORE`/`:END` | a quick dump without `SD`'s prose |

`COMPACT` tokens: `RUN` (enabled) / `OFF` (disabled) / `ERR` (fault); `MOV` /
`STP`; `TR` (target reached) / `NR`.

`FLAGS` names bits 10, 12 and 13 **according to the motor's current mode**
(§12): position `REACHED` / `SETPOINT_OK` / `POSERR`, velocity `AT_SPEED` /
`VEL_ZERO` / `MAX_ACCEL`, homing `HOME_REACHED` / `HOMED` / `HOME_ERROR`.

`DETAIL` packet layout:

```
M1 (ID 1)
  Status: 0x1437 (Device enable)
  Mode:   position
  Flags:  READY|INIT|ENABLED|WORKING|QSTOP|REACHED|SETPOINT_OK
  Bits:   0001 0100 0011 0111
  Pos:    10000
  Vel:    0.0 rps
  Error reg 0x1001 = 0x2        ← only when the fault bit is set
```

**Errors**: `ERR:S needs motor slots (…) - SA = all motors` · `ERR:S formats
RAW|COMPACT|DETAIL|STATE|BITS|FLAGS`.

### `SD` — detailed status in plain language · `SDA`

One packet per motor (`:MORE`/`:END` markers when more than one). `SDA` = all,
`SD:,,1` / `SD:M3` = motor 3.

```
=== MOTOR 1 (Slave ID 1) Status 0x1437 ===
State: Device enable
Mode: position
Motor: ENABLED, stopped
Position: 10000 steps (target REACHED)
Velocity: 0.0 rps
Homing: homed | Origin switch (SW): off
Limits: CW clear | CCW clear
Health: OK, no faults
```

Variations: `Motor: ENABLED, RUNNING, HALT active, QUICK-STOP active`;
`State: … (entering initialization)`; `Homing:` reads the drive live in homing
mode (`IN PROGRESS`, `complete`, `ERROR`) and the latched result otherwise
(`homed`, `not homed since boot`, `ERROR (last run failed)`); `Limits: CW ***
ACTIVE *** | CCW clear`; `Origin switch (SW): ON`; bit-13 line by mode
(`OVER-POSITION ERROR (bit13)`, `MAX ACCELERATION reached (bit13)`, `HOMING
ERROR (bit13)`); `WARNING active (bit7)`. On a fault the last line becomes
`FAULT: over-current (0x1001=0x2) -> send R:M1 to reset` (names:
`over-current`, `voltage-error`, `over-temperature`, `communication-error`,
`over-position-protect`, `motor-phase-loss`, `generic-error`, `unknown`). A
dead drive: `=== MOTOR 2 === NOT RESPONDING (TIMEOUT) - check power, slave ID,
RS485 wiring`.

### `LS` — limit switches, origin switch and homing state · `LSA`

```
Send:  LSA
Reply: LIMITS: M1:CW=0 CCW=0 SW=1 HOMED=1 MODE=POS,M2:CW=1 CCW=0 SW=0 HOMED=BUSY MODE=HOM,M3:ERROR
```

| Field | Values | Source |
|---|---|---|
| `CW` / `CCW` | `1` = that limit-switch input is active | status bits 14 / 15 |
| `SW` | `1` = the origin (mechanical-origin) switch input is active | status bit 11 |
| `HOMED` | `0` never homed since the drive was last initialised · `BUSY` homing in progress · `1` completed · `ERR` the drive reported a homing error | bit 12 / 13 in homing mode, latched afterwards |
| `MODE` | `POS`, `VEL` or `HOM` — the mode the controller last commanded | controller |

### `DG` — low-level register diagnosis · `DGA`

`DG:M1` (or `DG:1`) reads back what the drive actually holds and live-tests
both Modbus write function codes; `DGA` does every motor, one packet each.
Harmless: the write tests re-write values already in the drive.

```
DG M1: fmt6000=0 st6041=0x1437 mode6061=1 err1001=0x0
tgt607A=0,10000 vel6081=0,3 acc6083=5 dec6084=5 pos6064=0,10000
WRITE06=OK WRITE10=OK libScale=10
```

| Field | Register | Meaning |
|---|---|---|
| `fmt6000` | 0x6000 | 32-bit word order (0 expected) |
| `st6041` | 0x6041 | status word |
| `mode6061` | 0x6061 | mode the drive is in: 1 position, 3 velocity, 6 homing |
| `err1001` | 0x1001 | error register (§12) |
| `tgt607A` · `pos6064` | 0x607A · 0x6064 | target / actual position as `hi,lo` words |
| `vel6081` | 0x6081 | profile velocity as `hi,lo`, drive units (rps × 10) |
| `acc6083` / `dec6084` | 0x6083 / 0x6084 | ramps in drive units |
| `WRITE06` / `WRITE10` | — | single / multi-register write test; `EXCEPTION(exc2)` = the drive rejected the frame |
| `libScale` | — | the library's velocity scale (10, or 1 after `setDriveP45(0)`) |

Reading it when a motor "replies OK but does not move": `tgt607A` unchanged
after `P` → writes not landing (check `WRITE10`); `vel6081=0,0` → velocity 0,
send `V`; `mode6061` wrong → `R`; `err1001` non-zero → fault, `R`.

---

## 9. Controller commands

### `I` — detect and initialise · `IA`

`IA` = every configured slot, `I:1,,1` / `I:M1,M3` = those motors.

```
MOTOR INITIALIZATION:
Motor 1 (Slave ID 1): DETECTED - Initialized in Position mode
  Status: 0x1437 (Device enable)
  Position: 10000
Motor 2 (Slave ID 2): NOT DETECTED (TIMEOUT)
  Check: Power, Address, RS485 wiring
…
Total motors detected: 5 of 6
```

When every selected motor answers, a final line `SUCCESS: All expected motors
detected and initialized!` is appended. With many motors the report spans
several packets (`:MORE` … `:END`); the `Total …` line is in the last one.
**What it triggers**: cancels any deferred move; per motor: enable sequence,
position mode, that motor's ramps and homing parameters; marks the motor
online/offline for the monitor. The homed flag is kept; nothing moves.

### `R` — alarm reset and re-initialise · `RA`

| Send | Effect | Reply |
|---|---|---|
| `RA` | all motors | `All motors reset complete` |
| `R:1` · `R:M1` | motor 1 | `Motor 1 reset complete` |
| `R:,,1` · `R:M3` | motor 3 | `Motor 3 reset complete` |

A motor that fails to come back is tagged `Motor 2(FAIL) reset complete`.
Clears a drive fault, re-runs init with the motor's ramps and homing
parameters, keeps the homed flag, and clears the fault flag in the monitor so
a persisting fault is reported again as a new `EVENT:FAULT`.

### `MC` — motor count

| Send | Reply |
|---|---|
| `MC:?` or `MC` | `MC: 6 motors (slave IDs 1..6)` |
| `MC:12` | `MC: 12 motors, 7 online (SAVE to keep across reboot)` |

Growing the count probes and initialises the new slots at once. Kept across
reboot only by `SAVE`; otherwise the sketch's `setMotorCount()` applies.
**Errors**: `ERR:MC range 1..32`.

### `SAVE` · `LOAD` — settings in EEPROM

| Send | Effect | Reply |
|---|---|---|
| `SAVE` | store `V`, `ACC`, `DEC`, `HV`, `HVZ`, `HACC`, `HO` of every motor and the motor count in the Controllino's EEPROM; applied automatically at every boot, **overriding the sketch values** | `SAVE: 6 motors stored (V, ACC, DEC, HV, HVZ, HACC, HO, MC)` |
| `SAVE:?` | is anything stored? | `SAVE: settings stored (applied at boot; SAVE:CLEAR to forget)` · `SAVE: nothing stored (sketch values in use)` |
| `SAVE:CLEAR` | forget the stored block; the sketch values apply at the next boot | `SAVE: cleared - sketch values apply at next boot` |
| `LOAD` | re-apply the stored block now (after experimenting at runtime) | `LOAD: 6 motors loaded and applied` |

The block is CRC-protected; a corrupt or absent block is ignored at boot
(`ERR:LOAD nothing stored (or checksum failed)` on `LOAD`). Flashing a new
sketch does not erase EEPROM, so after changing the sketch's values send
`SAVE:CLEAR` or `SAVE` again. **Errors**: `ERR:SAVE use SAVE, SAVE:? or
SAVE:CLEAR` · `ERR:SAVE EEPROM too small`.

### `VER` · `PING` · `X`

| Send | Reply | Notes |
|---|---|---|
| `VER` | `JMCMotors 0.5.0` | check firmware compatibility from the PC |
| `PING` | `PONG` | cheap link check; no bus traffic |
| `X` | `System restart initiated...` then `Motor ready` on PC_PORT | soft reset; runtime settings revert to the sketch (or the `SAVE`d block); drive counters untouched |

---

## 10. Events: what the controller sends on its own

A background monitor reads **one motor's status word every 8 ms**, cycling
through the configured motors, and pushes a datagram to `PC_IP:PC_PORT`
whenever something **changes**: one message when a condition appears, one
when it clears, never a repeat while it persists.

| Trigger (what happened on the drive) | Event | What to do |
|---|---|---|
| CW limit input goes active (bit 14 rises) | `EVENT:LIMIT:M2:CW POS=48211 ST=0x4677` | the drive has already stopped that direction; back off with `PR`, or re-home |
| CW limit input released | `EVENT:LIMIT:M2:CW_CLEARED` | — |
| CCW limit input goes active (bit 15) / released | `EVENT:LIMIT:M2:CCW POS=-120 ST=0x8677` / `EVENT:LIMIT:M2:CCW_CLEARED` | as above |
| Origin switch input goes active (bit 11) / released | `EVENT:ORIGIN:M2:ON POS=1012 ST=0x1c37` / `EVENT:ORIGIN:M2:OFF` | informational; methods 3/4/7/8/11/12 stop here |
| Homing run completes (bit 12 in homing mode, after `H<n>`) | `EVENT:HOMED:M2 POS=0 ST=0x1437` | position is valid; safe to move |
| Homing run fails (bit 13 in homing mode) | `EVENT:HOMING_ERROR:M2 ST=0x3437 (check limit/origin wiring, HV, HO)` | check the switches and P42 polarity, lower `HV`, `R:…`, retry |
| Fault bit (bit 3) rises / clears | `EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT` / `EVENT:FAULT_CLEARED:M3` | `SD` for the cause, fix it, `R:…` |
| Two consecutive status reads fail | `EVENT:OFFLINE:M4` | check power / RS-485 / slave ID; retried every 3 s |
| An offline drive answers again | `EVENT:ONLINE:M4 ST=0x1437 (re-initialised)` | re-initialised automatically (ramps, homing parameters, `HO`); a power-cycled drive is back at **position 0** — re-home or `ZO` |

`ERR=` in a fault event is the raw 0x1001 register followed by the decoded
names `CURRENT`, `VOLTAGE`, `TEMP`, `COMM`, `OVERPOS`, `PHASE`, or `GENERIC`.
**Latency**: about *count × 13 ms* (6 motors → under 100 ms; 32 → under
450 ms). **Not an event**: target reached and the end of a `B` ramp — poll
`S:n:COMPACT` for `TR`.

---

## 11. Trigger and response matrix

| Command | Writes to drives? | Stored in controller? | Cancels a pending move? | Brakes a jog first? | Reply |
|---|---|---|---|---|---|
| `P` / `PR` / `PA` / `PRA` | yes: target + velocity, enable, start (broadcast for all) | mode = position | supersedes it | yes, non-blocking, `OK` deferred | `OK` |
| `P:?` `PA:?` | reads position | — | no | no | `M1=…` / `WARN: …` |
| `V` | **no** (used at next move) | yes | no | no | `Set V: M1=…` |
| `VA` | yes, broadcast velocity | yes | no | no | `VA: all motors = … (BROADCAST)` |
| `VS` / `VSA` | yes: mode, velocity, start | mode = velocity | yes | — | `VS: velocity mode started …` |
| `ACC` / `DEC` (+A) | yes (DEC also quick-stop ramp) | yes | no | no | `Set ACC: M1=…` / `ACCA: all motors = …` |
| `B` `B1` `B2` `B3` (+A, bare) | yes: halt / quick-stop | — | yes | — | `… stopped with deceleration` / `… brake applied to …` |
| `L` / `LA` | yes: brake open (torque off) | — | no | no | `… brake(s) loosened` |
| `BS` / `BSA` | reads status | — | no | no | `BRAKE STATUS: …` |
| `Z` / `ZA`, `ZO` / `ZOA` | yes: homing method 0 + mode | zero flag, mode = position | no | yes, blocking ≤ 5 s | `… set to zero position` / `ZO: … (current position redefined)` |
| `ZS` / `ZSA`, `LS` / `LSA` | reads status / position | — | no | no | `ZERO STATUS: …` / `LIMITS: …` |
| `HV` `HVZ` `HACC` (+A) | yes: 0x6099 / 0x609B / 0x609A | yes | no | no | `Set HV: M1=…` etc. |
| `HO` / `HOA` | yes: 0x607C | yes | no | no | `HO: … (used at next homing)` |
| `H<n>` / `H<n>A` | yes: homing params, mode, method, start | mode = homing, armed | no | yes, blocking ≤ 5 s | `Homing method n started for …` then `EVENT:HOMED` / `EVENT:HOMING_ERROR` |
| `S` / `SA` | reads status (+ pos/vel for COMPACT/DETAIL) | — | no | no | per format |
| `SD` / `SDA`, `DG` / `DGA` | reads registers (DG: 2 harmless writes) | — | no | no | one packet per motor |
| `I` / `IA` | yes: full init | online flags, mode = position | yes | — | `MOTOR INITIALIZATION: …` |
| `R` / `RA` | yes: alarm reset + init | online/fault flags, mode = position | yes | — | `… reset complete` |
| `MC` | yes, for newly added slots | yes (RAM) | no | no | `MC: …` |
| `SAVE` / `LOAD` | LOAD: ramps + homing params to online drives | EEPROM / RAM | no | no | `SAVE: …` / `LOAD: …` |
| `VER` / `PING` | no | — | no | no | `JMCMotors 0.5.0` / `PONG` |
| `X` | no | all RAM cleared | yes (reboot) | — | `System restart initiated...` then `Motor ready` |

---

## 12. Status word, drive states and error register

### Status word 0x6041 (what `S`, `SD`, `LS`, events and `BS` decode)

Bits 0–9, 11, 14 and 15 mean the same in every mode. **Bits 10, 12 and 13 are
redefined by the drive per operation mode** (manual, 0x6041 table), which is
why the controller tracks the mode it last commanded and names them
accordingly.

| Bit | Mask | Flag name (`S:…:FLAGS`) | Meaning when 1 |
|---|---|---|---|
| 0 | 0x0001 | `READY` | ready to initialise |
| 1 | 0x0002 | `INIT` | initialisation complete |
| 2 | 0x0004 | `ENABLED` | motor enabled (torque on) |
| 3 | 0x0008 | `FAULT` | drive fault — read 0x1001 (`SD`, `DG`), then `R` |
| 4 | 0x0010 | `WORKING` | drive working normally |
| 5 | 0x0020 | `QSTOP` | quick-stop *not* active (part of the "Device enable" pattern) |
| 6 | 0x0040 | `INITSTATE` | entering initialisation |
| 7 | 0x0080 | `WARN` | warning |
| 8 | 0x0100 | `HALT` | halted (after `B`) |
| 9 | 0x0200 | `RUN` | motor running |
| 10 | 0x0400 | `REACHED` · `AT_SPEED` · `HOME_REACHED` | position: target reached · velocity: target speed reached · homing: homing position reached |
| 11 | 0x0800 | `SWORIGIN` | origin-switch (SW) input active — `SW=1` in `LS` |
| 12 | 0x1000 | `SETPOINT_OK` · `VEL_ZERO` · `HOMED` | position: ready for a new set-point · velocity: speed is 0 · homing: **homing complete** |
| 13 | 0x2000 | `POSERR` · `MAX_ACCEL` · `HOME_ERROR` | position: over-position error · velocity: maximum acceleration · homing: **homing error** |
| 14 | 0x4000 | `CWLIM` | CW limit-switch input active — `CW=1` in `LS` |
| 15 | 0x8000 | `CCWLIM` | CCW limit-switch input active — `CCW=1` in `LS` |

Typical values: `0x1437` enabled and idle at target · `0x1637` enabled and
running · `0x4437` idle with the CW limit pressed · `0x1c37` idle on the
origin switch.

### Drive state (low nibble, shown by `S:…:STATE` and `SD`)

| Low 4 bits | State name |
|---|---|
| 0x0 | Uninitialized |
| 0x1 | Initialized |
| 0x3 | Power on |
| 0x7 | Device enable (bit 5 set) · Quick stop active (bit 5 clear) |
| 0x8 | Error state |
| 0xF | Fault alarm |

### Error register 0x1001 (`ERR=` in events, `FAULT:` line in `SD`)

| Event name | `SD` name | Typical cause |
|---|---|---|
| `CURRENT` | over-current | short, stall, wrong motor wiring |
| `VOLTAGE` | voltage-error | supply too low/high, regen on fast decel |
| `TEMP` | over-temperature | drive too hot |
| `COMM` | communication-error | fieldbus trouble on the drive side |
| `OVERPOS` | over-position-protect | following error above drive P16 — jam or overload |
| `PHASE` | motor-phase-loss | motor cable |
| `GENERIC` | generic-error | none of the above bits set |

---

## 13. Error message catalogue

| Message | Cause |
|---|---|
| `ERR:unknown command <HEAD>` | head not recognised |
| `ERR:<CMD> needs motor slots (e.g. <CMD>:1,,1 or <CMD>:M1,M3) - <CMD>A = all motors` | selector command with nothing after the colon (not `B`…`B3`) |
| `ERR:<CMD> needs values (e.g. <CMD>:1000,,3000 or <CMD>:M3=1000) - <CMD>A:<value> = all motors` | value command with no argument |
| `ERR:<CMD>A takes one value for all motors (e.g. <CMD>A:1000)` | a comma list after an `A` form |
| `ERR:<CMD> motor must be M1..M<n>` | `M` name out of range or malformed |
| `ERR:<CMD> cannot mix values and ?` | `P:1000,?` |
| `ERR:V range 0.01..1000.0 rps (M2)` · `ERR:ACC range 0.1..6553.5 rps/s (M2)` (same for `DEC`, `HV`, `HVZ`, `HACC`) | value out of range in that slot; nothing written |
| `ERR:ZO takes values, not ? (read positions with P:?)` | `ZO:?` |
| `ERR:B type is B, B1, B2 or B3` | `B4`, `B9`… |
| `ERR:homing method H1..H12` | `H0`, `H13`… |
| `ERR:S formats RAW\|COMPACT\|DETAIL\|STATE\|BITS\|FLAGS` | misspelt format |
| `ERR:MC range 1..32` | count out of range |
| `ERR:SAVE use SAVE, SAVE:? or SAVE:CLEAR` · `ERR:SAVE EEPROM too small` | bad `SAVE` argument / no room |
| `ERR:LOAD nothing stored (or checksum failed)` | `LOAD` with no valid block |

`WARN:` is not an error: the command succeeded for the motors listed with
values; those marked `ERROR` did not answer on the bus.

---

## 14. Recipes and sequences

### First contact after flashing

```
                      ← Motor ready            (arrives on PC_PORT at boot)
VER                   → JMCMotors 0.5.0
IA                    → MOTOR INITIALIZATION: … Total motors detected: 6 of 6
PA:?                  → M1=0,M2=0,…
SA:COMPACT            → M1:RUN|STP|TR|POS=0|VEL=0.0,M2:…
```

### A typical positioning cycle

```
VA:0.5                → VA: all motors = 0.50 rps (BROADCAST)
ACCA:0.3              → ACCA: all motors = 0.30 rps/s
DECA:0.3              → DECA: all motors = 0.30 rps/s
P:4000,8000,12000     → OK                            launched, synchronised
SA:COMPACT            → …|MOV|NR|…                    still moving
SA:COMPACT            → …|STP|TR|…                    arrived
PA:0                  → OK
```

### Jog, then position

```
V:1                   → Set V: M1=1.00 rps
VS:M1                 → VS: velocity mode started for M1 (stop with B)
B                     → All motors stopped with deceleration     (avoids the deferred wait)
P:0                   → OK
```

### Commission a homing axis

```
HV:0.2                → Set HV: M1=0.20 rps           search speed
HVZ:0.05              → Set HVZ: M1=0.05 rps          slow final approach
HO:1000               → HO: M1=1000 steps after sensor (used at next homing)
H3:M1                 → Homing method 3 started for M1
                      ← EVENT:ORIGIN:M1:ON POS=-3120 ST=0x1e37   (switch found)
                      ← EVENT:HOMED:M1 POS=0 ST=0x1437           (done)
LS:M1                 → LIMITS: M1:CW=0 CCW=0 SW=0 HOMED=1 MODE=HOM
```

If `EVENT:HOMING_ERROR:M1 …` arrives instead, check the switch wiring and
polarity (drive P42), lower `HV`, then `R:M1` and retry.

### Home every axis, then keep the settings

```
HVA:0.2  HVZA:0.05  HOA:500  H3A        one EVENT:HOMED per motor follows
ACC:0.2,0.2,0.5,0.5,0.1,0.1             per-axis ramps
SAVE                  → SAVE: 6 motors stored (V, ACC, DEC, HV, HVZ, HACC, HO, MC)
```

### Recover from a fault

```
                      ← EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT
SD:M3                 → === MOTOR 3 … FAULT: over-current (0x1001=0x2) -> send R:M3 to reset
   (fix the cause: jam, wiring, load)
R:M3                  → Motor 3 reset complete
                      ← EVENT:FAULT_CLEARED:M3
```

### Handle a limit switch

```
                      ← EVENT:LIMIT:M2:CW POS=48211 ST=0x4677
PR:M2=-2000           → OK                            back off the switch
                      ← EVENT:LIMIT:M2:CW_CLEARED
```

### Drive power-cycled while the controller kept running

```
                      ← EVENT:OFFLINE:M4
                      ← EVENT:ONLINE:M4 ST=0x1437 (re-initialised)
ZS:M4                 → ZERO STATUS: M4:ZERO(0) | ALL_ZERO_SET      counter is 0 now
H3:M4  (or ZO:M4=<known>)
```

### Emergency

```
B3                    → Immediate brake applied to all motors
LA                    → All motor brakes loosened      (only if the shafts must be free)
RA                    → All motors reset complete      when safe to resume
```

---

## 15. Changes from 0.4

| 0.4 | 0.5 | Why |
|---|---|---|
| bare `B`, `L`, `Z`, `R`, `I`, `VS`, `BS`, `ZS`, `LS`, `SD`, `DG` = all motors | `BA`, `LA`, `ZA`, `RA`, `IA`, `VSA`, `BSA`, `ZSA`, `LSA`, `SDA`, `DGA`; bare selector = error, **except bare `B`…`B3` which still stop all** | one rule: the `A` suffix is the only all-motors form |
| `all` keyword | removed | same |
| `AD:acc/dec`, `ADA` | `ACC`, `ACCA`, `DEC`, `DECA` — each like `V` | acceleration and deceleration are two settings |
| `S:0`, `S:2,COMPACT`, `DG:1` by motor number | `SA`, `S:,1:COMPACT`, `S:M2:COMPACT`, `DG:M1` | slots and names everywhere; format after a second colon |
| `V:?`, `HO:?` read every motor | `V:?` reads motor 1 (`V:,,?`, `V:M3`), `VA:?` / `HOA:?` read all | `?` follows the slot rule like `P:?` |
| `Set Motor 1 V: 0.50`, `Set V: 0.50,1.00` | `Set V: M1=0.50 rps`, `Set V: M1=0.50,M2=1.00 rps` | labelled values, one format |
| — | `M3=500` / `M1,M3` motor names in any slot command | software-friendly, no slot counting |
| — | `HVZ`, `HACC` (+A, `?`), `HV:?` | zero-find speed and homing ramp exposed |
| — | `SAVE`, `SAVE:?`, `SAVE:CLEAR`, `LOAD` | settings survive a reboot without re-flashing |
| — | `VER`, `PING`, lower-case heads, `ERR:unknown command <HEAD>`, `:MORE` / `:END` markers | PC-side conveniences |
| `INVALID COMMAND` | `ERR:unknown command <HEAD>` | names the head |
| `SD` hint `send R:1` | `send R:M1` | valid in the new grammar |
