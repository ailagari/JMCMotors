# JMCMotors — UDP Command Reference

The complete reference for the JMC_CONTROLLINO_V3 text protocol implemented by
this library: how commands are shaped, every command with its arguments, what
each one **triggers** inside the controller and on the RS-485 bus, every reply
it can produce, the unsolicited **events** the controller pushes on its own,
and the status/error words you will see in those replies.

Library: https://github.com/ailagari/JMCMotors
Author: LAGARI A <lagariscience@gmail.com> · License: MIT

Companion documents: [HARDWARE.md](HARDWARE.md) (drive setup, wiring, bring-up)
· [README](../README.md) (install and quick start).

---

## Contents

1. [Transport and addressing](#1-transport-and-addressing)
2. [Command grammar and command types](#2-command-grammar-and-command-types)
3. [Reply types](#3-reply-types)
4. [Motion commands](#4-motion-commands) — `P` `PR` `PA` `PRA`
5. [Speed and ramp commands](#5-speed-and-ramp-commands) — `V` `VA` `VS` `AD` `ADA`
6. [Stopping commands](#6-stopping-commands) — `B` `B1` `B2` `B3` `L` `BS`
7. [Zero and homing commands](#7-zero-and-homing-commands) — `Z` `ZO` `ZOA` `ZS` `HV` `HVA` `HO` `HOA` `H1`…`H12`
8. [Status and diagnostic commands](#8-status-and-diagnostic-commands) — `S` `SD` `LS` `DG`
9. [Administration commands](#9-administration-commands) — `I` `R` `MC` `X`
10. [Events: what the controller sends on its own](#10-events-what-the-controller-sends-on-its-own)
11. [Trigger and response matrix](#11-trigger-and-response-matrix)
12. [Status word, drive states and error register](#12-status-word-drive-states-and-error-register)
13. [Error message catalogue](#13-error-message-catalogue)
14. [Recipes and sequences](#14-recipes-and-sequences)

---

## 1. Transport and addressing

| Item | Value |
|---|---|
| Protocol | UDP, plain ASCII text, one command per datagram |
| Controller listens on | `CONTROLLINO_IP:CONTROLLINO_PORT` (sketch: `setNetwork()`, or `setStationId(n)` → `192.168.1.(100+n):8001`) |
| Command replies go to | the **sender** of the command (its IP and source port) **and** the configured `PC_IP:PC_PORT` (sent once if those are the same) |
| Events and `Motor ready` go to | `PC_IP:PC_PORT` **only** — fixed for the life of the sketch |
| Packet size | up to 511 characters (enough for a `P:` with 32 positions) |
| Terminator | none needed; leading/trailing whitespace and line endings are trimmed |
| Authentication | none — any host that can reach the port can command the motors |
| Unknown command | reply `INVALID COMMAND` |

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
  (`192.168.1.100+n`) from one number, so a fleet is configured by changing one
  digit per board. It does not change the PC address.
- **PC side** — every `EVENT:` and the boot message go here, always. Because
  replies are *also* echoed to the sender, any UDP tool (SocketTest, ncat,
  Packet Sender, your own app) sees its answers without listening on `pcPort`.
  To receive **events** you must listen on `PC_IP:PC_PORT`.

PC and controller must be able to reach each other directly (same subnet or a
routed path). There is no broadcast fallback.

### Boot message

When `begin()` has probed and initialised every configured motor slot, the
controller sends one datagram to `PC_IP:PC_PORT`:

```
Motor ready
```

This is the signal that commands will be accepted. It is sent regardless of
how many drives actually answered — use `I` to get the per-motor detection
report.

---

## 2. Command grammar and command types

A command is `HEAD` or `HEAD:ARGUMENTS`. The head is **upper-case** and
**case-sensitive** (`p:1000` is an `INVALID COMMAND`). Arguments are
comma-separated fields. The word `all` (any case) is accepted wherever a motor
selection is expected.

### 2.1 Units

| Quantity | Unit | Notes |
|---|---|---|
| Position | encoder **steps**, signed 32-bit (±2,147,483,647) | steps per revolution = drive parameter **P17** |
| Velocity | **rps** (revolutions per second), decimals allowed | resolution 0.1 rps on the drive (library scales ×10); 0.3 rps = 18 RPM |
| Accel / decel | **rps/s** | resolution 0.1 rps/s |
| Homing offset | steps | sign = direction past the sensor |

### 2.2 The six command shapes

Every command belongs to one of these shapes. Once you know the shape, you
know how its arguments are read. **Every motor command exists in a
per-motor form and an all-motors form**: selector commands take an empty
argument or `all` for everyone, and every value command has an `A`-suffixed
twin that applies one value to all motors (`PA`, `PRA`, `VA`, `ADA`, `ZOA`,
`HOA`, `HVA`).

| Shape | How arguments are read | Commands |
|---|---|---|
| **A. Bare** | no argument; acts on every motor or on the controller | `I` `X` `VS` `BS` `ZS` `LS` `SD` `DG` `B` `L` `Z` `R` `H<n>` |
| **B. Positional values** | field N is a **value for motor N**; empty field = skip that motor; a single value with **no comma = motor 1 only** | `P` `PR` `ZO` `HO` `V` `HV` `AD` |
| **C. Positional selectors** | field N is a **flag for motor N**; any non-empty text selects it, empty skips it; no argument or `all` = every motor | `B` `B1` `B2` `B3` `L` `Z` `R` `H<n>` `VS` `I` `BS` `ZS` `LS` |
| **D. Whole-bus value** | one value applied to **all** motors at once | `PA` `PRA` `VA` `ADA` `ZOA` `HOA` `HVA` `MC` |
| **E. Query** | `?` in a field reads instead of writes | `P:?` `PR:?` `PA:?` `SD:?` `V:?` `VA:?` `HO:?` `AD:?` `MC:?` |
| **F. Motor number** | the argument is a motor **number**, not a slot | `S:<n>` `SD:<n>` `DG:<n>` |

Per-motor ↔ all-motors at a glance:

| Action | One or some motors | All motors |
|---|---|---|
| absolute move | `P:1000,,3000` | `PA:1000` |
| relative move | `PR:-500,,200` | `PRA:-500` |
| run speed | `V:0.5,,0.3` | `VA:0.5` |
| jog (velocity mode) | `VS:1,,1` | `VS` |
| ramps | `AD:0.2,,0.5/0.2` | `ADA:0.2` |
| stop (any type) | `B2:1,,1` | `B2:` |
| loosen brake | `L:,1` | `L` |
| zero here | `Z:1,,1` | `Z` |
| position = value | `ZO:5000,,100` | `ZOA:5000` |
| homing speed | `HV:0.2,,0.3` | `HVA:0.2` |
| homing offset | `HO:1000,,-800` | `HOA:1000` |
| start homing | `H3:1,,1` | `H3` |
| reset / re-init | `R:,,1` · `I:,,1` | `R` · `I` |
| status | `S:3` `SD:3` `LS:,,1` `BS:,,1` `ZS:,,1` `DG:3` | `S:0` `SD` `LS` `BS` `ZS` `DG` |

### 2.3 Positional slots, explained once

Shapes B and C use the same positional rule as `P`: the first field is motor
1, the second is motor 2, and so on. An empty field skips that motor.

```
P:1000,,3000     M1 → 1000     M2 skipped     M3 → 3000
L:1,,1           M1 and M3 loosened (the "1" is only a flag)
L:,1             M2 only
L:2              M1 only!  (slot 1 is filled; "2" is just a flag)
B2:x,,x          fast brake M1 and M3 ("x" is as good as "1")
```

Why `L:2` is **not** motor 2: selector commands count slots, they do not read
numbers. Write `L:,1` for motor 2, exactly as you would write `P:,1000`.

Only the shape-F commands (`S`, `SD:<n>`, `DG`) take a plain motor number.

### 2.4 Parsing rules worth knowing

- A value field that is not a number parses as **0** (`P:abc` moves motor 1 to
  step 0; `P:all` does the same). Values are never rejected for being
  non-numeric, so validate on the PC side.
- Fields beyond the configured motor count (`MC`) are ignored.
- Decimal velocities use a dot: `V:0.5`. A comma is always a field separator;
  inside an `AD` slot, accel and decel are separated by `/`.
- `P:1000,` (trailing comma) and `P:1000` are equivalent: motor 1 only.
- **Motor count.** The number of motors is one variable: `setMotorCount(n)`
  in the sketch or `MC:n` at runtime, 1–32. Every command's slots, every
  all-motors form, detection and the event monitor follow it. Replies that
  list every motor are sent in **several packets** when they would exceed
  about 300 characters (roughly from 16 motors up); each continuation packet
  repeats the reply's header (`STATUS:`, `LIMITS: `, …) and starts a fresh
  comma-separated list, so concatenate on the PC side if you need one line.

---

## 3. Reply types

Every command produces exactly one of these outcomes.

| Reply type | Looks like | Produced by |
|---|---|---|
| **Acknowledge** | `OK` | `P` `PR` `PA` when the move has been launched |
| **Confirmation** | a sentence naming what was done, e.g. `Fast brake applied to motor 1,3` | `B*` `L` `Z` `ZO` `R` `V` `VA` `VS` `HV` `HO` `H<n>` `AD` `MC` `X` |
| **Data line** | `M1=10000,M3=200` · `STATUS:M1=0x1437,…` · `V: 0.30,0.30,…` | queries (`?`), `S`, `BS`, `ZS`, `DG` |
| **Multi-line packet** | several lines in one datagram | `I`, `SD`, `S:…,DETAIL`, `DG` |
| **One packet per motor** | N separate datagrams | `SD` (all motors), `S:0,DETAIL` |
| **Warning** | `WARN: M1=10000,M2=ERROR` | a data reply where at least one motor did not answer on RS-485 |
| **Error** | `ERR:<reason>` | bad arguments; nothing was sent to the drives |
| **Unknown** | `INVALID COMMAND` | head not recognised (check upper case) |
| **Deferred** | `OK` arrives later | `P` `PR` `PA` sent while motors are in velocity mode (see §4) |
| **No further reply** | — | `X` answers once, then reboots |

Per-motor comms failures inside a data reply show as `ERROR` for that motor
(`S`, `P:?`, `BS`, `ZS`) or `NOT RESPONDING (<reason>)` (`SD`, `I`). The reason
is one of the bus result names: `TIMEOUT`, `CRC`, `EXCEPTION`, `BADFRAME`,
`PARAM`.

---

## 4. Motion commands

### `P` — absolute position move

**Shape B** (positional values). Moves the addressed motors to absolute step
positions, synchronised.

| Send | Effect | Reply |
|---|---|---|
| `P:10000` | motor 1 → step 10000 | `OK` |
| `P:1000,2000,3000` | M1 → 1000, M2 → 2000, M3 → 3000, launched together | `OK` |
| `P:500,,1500` | M1 → 500, skip M2, M3 → 1500 | `OK` |
| `P:-40000` | negative targets are fine | `OK` |
| `P:?` · `P:,,?` · `P:?,?` | **query** — read position of M1 · M3 · M1+M2 | `M1=10000` · `M3=200` · `M1=10000,M2=2000` |

**What it triggers**

1. For each addressed motor the controller writes the target (0x607A) and the
   motor's stored velocity (`V`) into the drive — *prepare*.
2. Then it launches. If **every** configured motor is addressed, the launch is
   two **broadcast** frames (slave ID 0: enable, then start) so all drives
   start in the same instant. For a **partial** selection the launch is sent
   per motor (about 5 ms apart each) — broadcast would also retrigger the
   motors you did not address.
3. The drive is told that the new target takes effect **immediately**
   (control-word bit 5). A `P` sent mid-move abandons the move in flight and
   heads straight to the new target; it does not queue.
4. `OK` is sent once the start frames are out — it means "launched", not
   "arrived". Watch `S:n,COMPACT` (`TR` = target reached) or `SD` to know when
   the motor is there.

**If the motors are in velocity mode (`VS`)**: changing a drive's mode
register while the shaft turns stops it dead, so the controller first brakes
every addressed motor on its `AD` decel ramp, waits for standstill (polling
every 50 ms, up to 20 s), then launches all of them together. During the wait
the controller keeps answering other commands and pushing events; **`OK` is
sent when the move actually starts**. A `B`, `VS`, `I` or `R` received during
the wait cancels the queued move. Stopping with `B` *before* repositioning
avoids the wait.

**Errors**: `ERR:P needs positions` (no argument) · `ERR:no positions` (all
fields empty) · `ERR:no '?' fields in P query`.

**Related**: `PR` (relative), `PA` (all motors, one value), `V` (speed used),
`AD` (ramps used).

### `PR` — relative position move

**Shape B.** Same syntax, launch sequence, deferral behaviour and errors as
`P`, but every value is a **delta from the motor's current position**.

| Send | Effect | Reply |
|---|---|---|
| `PR:1000` | motor 1 forward 1000 steps | `OK` |
| `PR:-1000` | motor 1 back 1000 steps | `OK` |
| `PR:500,,-500` | M1 +500, skip M2, M3 −500, synchronised | `OK` |
| `PR:?` | query, identical to `P:?` | `M1=…` |

The offset is applied **inside the drive** (control-word bit 6), so there is no
read-modify-write race even if the motor is still moving when `PR` arrives.
The error texts say `P` (`ERR:P needs positions`) because the two commands
share a handler.

### `PA` — all motors to the same position

**Shape D.** One target for every configured motor, launched by broadcast.

| Send | Effect | Reply |
|---|---|---|
| `PA:5000` | every motor → step 5000, simultaneous start | `OK` |
| `PA:0` | everyone back to zero | `OK` |
| `PA:?` | positions of **all** motors | `M1=…,M2=…,…,M6=…` |

**What it triggers**: a broadcast prepare (target + motor 1's stored
velocity), broadcast enable, broadcast start — three slave-0 frames, no
per-motor stagger even with 32 drives. Because the prepare is broadcast, all
motors run at **motor 1's `V` speed**, not their own. The velocity-mode
deferral rule of `P` applies.

**Errors**: `ERR:PA needs position`.

### `PRA` — all motors by the same relative distance

**Shape D.** The relative twin of `PA`: every motor moves by the same delta
from where it is, launched by broadcast.

| Send | Effect | Reply |
|---|---|---|
| `PRA:1000` | every motor forward 1000 steps | `OK` |
| `PRA:-250` | every motor back 250 steps | `OK` |
| `PRA:?` | positions of all motors (same as `PA:?`) | `M1=…,M2=…` |

Same broadcast behaviour, motor-1 speed and deferral rule as `PA`.
**Errors**: `ERR:PRA needs delta`.

### Position query replies

`P:?`, `PR:?` and `PA:?` all answer with `M<n>=<steps>` pairs for the motors
asked. A motor that did not answer on RS-485 shows `M<n>=ERROR` and the whole
reply is prefixed with `WARN: `:

```
WARN: M1=10000,M2=ERROR
```

---

## 5. Speed and ramp commands

Runtime speed/ramp settings live in the controller's RAM and are **lost on
reboot or power failure**. Values set in the sketch (`setMotionDefaults`,
`setMotorVelocity`, `setMotorAccelDecel`, `setMotorHomingVelocity`) are
re-applied on every boot and whenever a drive comes back online.

### `V` — run velocity per motor (rps)

**Shape B.** Stores the speed each motor will use for its next `P`/`PR`/`PA`
move or `VS` jog. **Nothing is written to the drive until that next move.**

| Send | Effect | Reply |
|---|---|---|
| `V:0.5` | motor 1 = 0.5 rps | `Set Motor 1 V: 0.50` |
| `V:0.5,1,0.3` | M1 0.5, M2 1.0, M3 0.3 rps | `Set V: 0.50,1.00,0.30` |
| `V:,,2` | motor 3 only | `Set V: 2.00` |
| `V:?` | read stored speeds | `V: 0.30,0.30,0.30,0.30,0.30,0.30` |

Default (if never set) is 0.3 rps = 18 RPM. There is no range check on `V`;
the drive clips to what it can do. **Errors**: `ERR:V needs argument`.

### `VA` — velocity for all motors (broadcast)

**Shape D.** Stores the same speed for every motor **and** writes it to all
drives immediately with one broadcast frame, so a running `VS` jog changes
speed at once.

| Send | Effect | Reply |
|---|---|---|
| `VA:1` | all motors 1 rps | `VA: Set all motors velocity to 1.00 rps (BROADCAST)` |
| `VA:?` | same as `V:?` | `V: 1.00,1.00,…` |

**Errors**: `ERR:VA needs velocity` · `ERR:VA range 0-1000 rps`.

### `VS` — start velocity mode (endless rotation)

**Shape C** (positional selectors). The selected motors run continuously,
each at its own stored `V` speed.

| Send | Effect | Reply |
|---|---|---|
| `VS` or `VS:all` | all motors jog, broadcast start | `VS: velocity mode started (stop with B)` |
| `VS:1,,1` | motors 1 and 3 only | `VS: velocity mode started for M1,3 (stop with B)` |
| `VS:,1` | motor 2 only | `VS: velocity mode started for M2 (stop with B)` |

**What it triggers**: cancels any deferred move, writes each selected motor's
own speed and switches it to velocity mode, then launches: one broadcast
enable + start when every motor is selected (all shafts begin together), or a
per-motor enable + start for a subset (broadcast would also start the
others). Direction is the sign of the speed (`V:-0.5` jogs backwards).
Velocity mode consumes no position range, so use it for anything that spins
forever (turntables, conveyors). **Errors**: `ERR:VS no motors selected
(e.g. VS:1,,1)`.

**Leaving velocity mode**: `B` (any type) stops it. `P`/`PR`/`PA`, `Z`, `ZO`
and `H<n>` all brake the motors on the `AD` decel ramp first (see `P`). `Z`,
`ZO` and `H<n>` block the controller while braking, capped at 5 s; `P`-family
commands wait in the background. A 2 rps jog at `AD:0.2` takes 10 s to brake,
so send `B` first when the ramp is slow.

### `AD` — acceleration and deceleration per motor (rps/s)

**Shape B** (positional values). Each slot is `acc` (accel = decel) or
`acc/dec`. Written to the drive immediately (also refreshing the quick-stop
ramp used by `B2`) and remembered for every re-initialisation.

| Send | Effect | Reply |
|---|---|---|
| `AD:0.2` | motor 1: accel = decel = 0.2 rps/s | `AD: M1=0.2/0.2 rps/s (accel/decel)` |
| `AD:0.3/0.1` | motor 1: accel 0.3, decel 0.1 | `AD: M1=0.3/0.1 rps/s (accel/decel)` |
| `AD:0.2,,0.5/0.2` | M1 0.2/0.2, skip M2, M3 0.5/0.2 | `AD: M1=0.2/0.2,M3=0.5/0.2 rps/s (accel/decel)` |
| `AD:?` or `AD:` | read every motor | `AD: M1=0.5/0.5,M2=0.5/0.5,… rps/s (accel/decel)` |

Ramp time = speed ÷ ramp: 0.3 rps at 0.1 rps/s takes 3 s to reach speed. For
geared mechanisms pair a slow ramp with drive parameter **P19 = 7** (speed
smoothness, set on the drive panel). Default 0.5/0.5 rps/s.

**Errors**: `ERR:AD range 0.1..6553.5 rps/s (M2)` (zero, negative or too
large in that slot; nothing was written) · `ERR:AD needs values (…)`.

> **Changed in 0.4.0.** `AD` used to apply to all motors, and `AD:0.3,0.1`
> meant "accel 0.3, decel 0.1 for everyone". It is now positional like every
> other value command; use `ADA` for the all-motors form.

### `ADA` — acceleration and deceleration for all motors

**Shape D.**

| Send | Effect | Reply |
|---|---|---|
| `ADA:0.2` | every motor accel = decel = 0.2 rps/s | `ADA: all motors accel=0.2 decel=0.2 rps/s` |
| `ADA:0.3/0.1` (or `ADA:0.3,0.1`) | every motor accel 0.3, decel 0.1 | `ADA: all motors accel=0.3 decel=0.1 rps/s` |
| `ADA:?` | same as `AD:?` | `AD: M1=…` |

**Errors**: `ERR:ADA range 0.1..6553.5 rps/s`.

`I` and `R` both re-initialise a motor with its current ramps (sketch table
or runtime `AD`/`ADA`), so a runtime value survives them; only a reboot
reverts to the sketch.

---

## 6. Stopping commands

### `B`, `B1`, `B2`, `B3` — stop

**Shape C** (positional selectors). The **stop type is part of the head**,
like `H<method>`; the argument selects motors.

| Head | Stop type | How the drive stops |
|---|---|---|
| `B` (or `B0`) | controlled stop | halt bit: ramps down on the normal `AD` decel ramp; the jog setpoint is zeroed so the motor cannot resume when halt is released |
| `B1` | quick stop, decel ramp | quick-stop code 1: decelerate on the normal ramp, then hold |
| `B2` | fast brake | quick-stop code 2: decelerate on the quick-stop ramp (0x6085, refreshed by `AD`) |
| `B3` | immediate | quick-stop code 3: stop at once, no ramp — the emergency stop |

| Send | Effect | Reply |
|---|---|---|
| `B` | controlled stop, all motors | `All motors stopped with deceleration` |
| `B:,1` | controlled stop, motor 2 | `Motor 2 stopped with deceleration` |
| `B1:` | quick stop on decel ramp, all | `Deceleration brake applied to all motors` |
| `B1:1,,1` | same, motors 1 and 3 | `Deceleration brake applied to motor 1,3` |
| `B2:` | fast brake, all | `Fast brake applied to all motors` |
| `B2:1,,1` | fast brake, motors 1 and 3 | `Fast brake applied to motor 1,3` |
| `B3:` or `B3:all` | immediate stop, all | `Immediate brake applied to all motors` |
| `B3:,1` | immediate stop, motor 2 | `Immediate brake applied to motor 2` |

**What it triggers**: cancels any move waiting for a ramp-down ("stop wins"),
then sends the stop to each selected motor in turn. After a `B` the status
word shows `HALT` (bit 8); after a quick stop the drive sits in the
"Quick stop active" state. The next `P`/`VS` re-enables automatically. `BS`
reports the result.

**Errors**: `ERR:B type is B, B1, B2 or B3` (e.g. `B4`) · `ERR:B no motors
selected (e.g. B2:1,,1)` (argument given but every field empty — note that a
bare `B1:` with nothing after the colon counts as *all*, only a string of bare
commas is "none").

### `L` — loosen brakes (release torque)

**Shape C.** Opens the brake / drops holding torque so the shaft can be turned
by hand.

| Send | Effect | Reply |
|---|---|---|
| `L` or `L:all` | all motors | `All motor brakes loosened` |
| `L:,1` | motor 2 | `Motor 2 brake loosened` |
| `L:1,,1` | motors 1 and 3 | `Motor 1,3 brake loosened` |

**What it triggers**: control word "brake open" on each selected drive, which
also **disables motor torque** — a loosened axis will not hold position. The
next move command re-enables the drive automatically. **Errors**: `ERR:L no
motors selected (e.g. L:1,,1)`.

### `BS` — brake status

**Shape A / C.** Reads the selected motors' status words and reports brake
and motion state. `BS` = all, `BS:1,,1` = motors 1 and 3.

```
Send:  BS
Reply: BRAKE STATUS: M1:FREE/RUN,M2:BRAKE/STOP,M3:ERROR | SUMMARY: MIXED
Send:  BS:,1
Reply: BRAKE STATUS: M2:BRAKE/STOP | SUMMARY: BRAKED
```

| Token | Meaning |
|---|---|
| `BRAKE` | drive is halted (bit 8) **or** not enabled (bit 2 clear) — holding or released, no motion possible |
| `FREE` | enabled and not halted |
| `RUN` / `STOP` | motor running (bit 9) / standing |
| `ERROR` | no RS-485 answer |
| `SUMMARY` | `BRAKED` (all selected braked), `FREE` (all free), `MIXED`, or `ERROR` (none of the selected answered) |

**Errors**: `ERR:BS no motors selected (e.g. BS:1,,1)`.

---

## 7. Zero and homing commands

### `Z` — current position becomes 0 (no movement)

**Shape C.** Redefines the position counter of the selected motors so that
where they stand now reads 0.

| Send | Effect | Reply |
|---|---|---|
| `Z` or `Z:all` | all motors | `All motors set to zero position` |
| `Z:1,1` | motors 1 and 2 | `Motor 1,2 set to zero position` |
| `Z:1,,1` | motors 1 and 3 | `Motor 1,3 set to zero position` |

**What it triggers**: if a selected motor is jogging it is braked to standstill
first (blocking, max 5 s). Then, per motor: homing method 0 "set current
position" is executed, 20 ms pause, and the drive is put back into position
mode. The controller marks the motor as zero-set for `ZS`. The shaft does not
move. **Errors**: `ERR:Z no motors selected (e.g. Z:1,,1)`.

### `ZO` — current position becomes a value (no movement)

**Shape B.** Same as `Z` but with a value per motor.

| Send | Effect | Reply |
|---|---|---|
| `ZO:10000` | where motor 1 is now reads 10000 | `ZO: M1=10000 (current position redefined)` |
| `ZO:10000,1000` | M1 reads 10000, M2 reads 1000 | `ZO: M1=10000,M2=1000 (current position redefined)` |
| `ZO:,,500` | motor 3 only | `ZO: M3=500 (current position redefined)` |

Use it to restore a known coordinate after a power cycle without re-homing
(the drive's counter resets to 0 on power-up). Internally `Z` and `ZO` run
homing method 0 with the value in the drive's home-offset register (0x607C),
the same register `HO` uses — that is why every `H<n>` re-writes the stored
`HO` offset before starting. **Errors**: `ERR:ZO needs values` · `ERR:no
values`.

### `ZOA` — every motor's current position becomes the same value

**Shape D.** `ZOA:0` is the same as `Z`; `ZOA:5000` makes every motor read
5000 where it stands. Reply as `ZO` listing every motor:
`ZO: M1=5000,M2=5000,… (current position redefined)`.

### `ZS` — zero status

**Shape A / C.** `ZS` = all, `ZS:1,,1` = motors 1 and 3.

```
Send:  ZS
Reply: ZERO STATUS: M1:ZERO(0),M2:NOT_SET(1500),M3:ERROR | ZERO_NOT_SET
```

`ZERO` means a `Z` or `ZO` was applied to that motor since the controller
booted; `NOT_SET` means it was not. **Homing does not set this flag** — it
tracks the explicit zero commands only; the homing state is in `LS` and
`SD`. The number in brackets is the live position. Trailer `ALL_ZERO_SET`
only when every selected motor is `ZERO` and answered. **Errors**: `ERR:ZS no
motors selected (e.g. ZS:1,,1)`.

### `HV` — homing velocity (rps)

**Shape B.** Written to the drive immediately (both the switch-search speed
0x6099 and the zero-find speed 0x609B) and remembered for re-initialisation.

| Send | Effect | Reply |
|---|---|---|
| `HV:0.3` | motor 1 homes at 0.3 rps | `Set Motor 1 HV: 0.30` |
| `HV:0.3,0.5` | M1 0.3, M2 0.5 | `Set HV: 0.3,0.5` |
| `HV:,,0.2` | motor 3 only | `Set HV: 0.2` |

Default 0.3 rps. There is no `HV:?` — read it back with `DG` or keep it in
the sketch. **Errors**: `ERR:HV needs argument`.

### `HVA` — homing velocity for all motors

**Shape D.** `HVA:0.2` sets every motor's homing speed to 0.2 rps; reply
`Set HV: 0.2,0.2,…`.

### `HO` — homing offset (steps past the sensor)

**Shape B.** How far the motor continues after the origin sensor trips before
it declares zero (drive register 0x607C). Sign sets the direction.

| Send | Effect | Reply |
|---|---|---|
| `HO:1000` | motor 1 stops 1000 steps past the sensor | `HO: M1=1000 steps after sensor (used at next homing)` |
| `HO:1000,2000` | M1 1000, M2 2000 | `HO: M1=1000,M2=2000 steps after sensor (used at next homing)` |
| `HO:-800` | the other direction | `HO: M1=-800 steps after sensor (used at next homing)` |
| `HO:?` | read stored offsets | `HO: M1=1000,M2=0,M3=0,M4=0,M5=0,M6=0` |

**What it triggers**: stored in the controller, written to the drive now, and
**re-written before every `H<n>` and every drive re-initialisation** (`R`,
`MC`, automatic `EVENT:ONLINE`). **Errors**: `ERR:HO needs steps (HO:1000 or
HO:?)` · `ERR:no offset values`.

### `HOA` — homing offset for all motors

**Shape D.** `HOA:1000` gives every motor a 1000-step offset; reply
`HO: M1=1000,M2=1000,… steps after sensor (used at next homing)`.

### `H1` … `H12` — start homing

**Shape C**, with the homing **method in the head**.

| Send | Effect | Reply |
|---|---|---|
| `H1` | home all motors, method 1 | `Homing method 1 started for all motors` |
| `H3:1,,1` | home motors 1 and 3, method 3 | `Homing method 3 started for M1,3` |
| `H3:,1` | motor 2 only | `Homing method 3 started for M2` |
| `H3:all` | all motors, method 3 | `Homing method 3 started for all motors` |

The drive supports homing methods 0–12 (JMC Modbus manual V2.2, §6.3).
Method 0 ("no movement, current position becomes the origin") is what `Z`
and `ZO` use; `H1`–`H12` are the moving methods. The three inputs involved
are the **CW limit**, the **CCW limit** and the **origin switch** (called
"SW mechanical origin" in the manual, status bit 11). Direction words below
are the manual's: "left" = towards the CW limit, "right" = towards the CCW
limit.

| Method | Uses | Behaviour (per the JMC manual) | Max homing speed |
|---|---|---|---|
| 1 | CW limit | move left, stop at the CW limit | 300 rpm (5 rps) |
| 2 | CCW limit | move right, stop at the CCW limit | 300 rpm |
| 3 | origin, CW limit as fallback | move left to the origin switch; if the CW limit is hit first, reverse and stop at the origin | 300 rpm |
| 4 | origin, CCW limit as fallback | mirror of 3, moving right | 300 rpm |
| 5 | CW limit | fast to the limit, back off slowly, re-approach slowly, stop at the CW limit | 1200 rpm |
| 6 | CCW limit | mirror of 5 | 1200 rpm |
| 7 | origin, CW limit as fallback | fast to the origin (reverse at the CW limit if needed), back off, re-approach slowly, stop at the origin | 1200 rpm |
| 8 | origin, CCW limit as fallback | mirror of 7 | 1200 rpm |
| 9 | CW limit | as 5, and the `HO` offset may be positive or negative | 1200 rpm |
| 10 | CCW limit | as 6, signed offset | 1200 rpm |
| 11 | origin, CW limit as fallback | as 7, signed offset | 1200 rpm |
| 12 | origin, CCW limit as fallback | as 8, signed offset | 1200 rpm |

`HV` sets the search speed (0x6099) and the slow zero-find speed (0x609B)
to the same value; keep it well under the method's maximum. Methods that end
on a limit switch (1, 2, 5, 6, 9, 10) also raise the normal
`EVENT:LIMIT:…` on the way — that is expected.

**What it triggers**, per selected motor: brake to standstill if jogging
(blocking, max 5 s) → re-write the stored `HO` offset → switch to homing mode →
write the method → pulse the start bit → arm the event monitor. The reply
means **homing has started**, not finished. Completion and failure **are
pushed as events**: `EVENT:HOMED:M1 POS=0 ST=0x…` when the drive reports
homing complete (status bit 12 while in homing mode) and
`EVENT:HOMING_ERROR:M1 ST=0x…` when it reports a homing error (bit 13 in
homing mode). You can also poll `LS` (`HOMED=BUSY` → `1` or `ERR`),
`S:n,FLAGS` (`HOMED`) or `SD`. Set `HV` and `HO` *before* homing. The drive
stays in homing mode until the next move command; the next `P`/`PR`/`PA`
switches it back to position mode automatically.

**Errors**: `ERR:homing method H1..H12` · `ERR:H no motors selected (e.g.
H3:1,,1)`.

---

## 8. Status and diagnostic commands

### `S` — status word in six formats

**Shape F.** `S:<motor>[,<FORMAT>]` — motor `0` (or bare `S`) = all motors;
`1..n` = one motor. Format names are case-insensitive.

| Format | Send | Reply | Use when |
|---|---|---|---|
| `RAW` (default) | `S:0` · `S` | `STATUS:M1=0x1437,M2=ERROR,M3=0x1637` | logging / machine parsing |
| `COMPACT` | `S:1,COMPACT` | `M1:RUN\|STP\|TR\|POS=10000\|VEL=0.0` | polling a move until `TR` |
| `STATE` | `S:0,STATE` | `STATES:M1:Device enable,M2:ERROR,M3:Fault alarm` | which drive state each motor is in |
| `BITS` | `S:1,BITS` | `M1:0001 0100 0011 0111` | comparing against the manual's bit table |
| `FLAGS` | `S:1,FLAGS` | `M1:READY\|INIT\|ENABLED\|WORKING\|QSTOP\|REACHED\|SETPOINT_OK` | reading named bits at a glance |
| `DETAIL` | `S:0,DETAIL` | one multi-line packet **per motor** (below) | a quick dump without `SD`'s prose |

`COMPACT` tokens: first `RUN` (enabled) / `OFF` (disabled) / `ERR` (fault
bit set); second `MOV` / `STP`; third `TR` (target reached) / `NR`.

`FLAGS` names bits 10, 12 and 13 **according to the motor's current mode**,
because the drive redefines them per mode (see §12): in position mode
`REACHED` / `SETPOINT_OK` / `POSERR`, in velocity mode `AT_SPEED` /
`VEL_ZERO` / `MAX_ACCEL`, in homing mode `HOME_REACHED` / `HOMED` /
`HOME_ERROR`. The mode is the one the controller last commanded (`P`-family
and `Z`/`ZO` → position, `VS` → velocity, `H<n>` → homing).

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

A motor that does not answer gives `M2: NO REPLY (TIMEOUT)` in `DETAIL`, or
`M2=ERROR` / `M2:ERROR` in the other formats.

**Errors**: `ERR:S motor 0..6` (number out of range) · `ERR:S formats
RAW|COMPACT|DETAIL|STATE|BITS|FLAGS`.

### `SD` — detailed status in plain language

**Shape A / E / F.** One packet per motor.

| Send | Reports |
|---|---|
| `SD` | all motors |
| `SD:?` · `SD:,,?` | positional: motor 1 · motor 3 |
| `SD:2` | motor 2 (plain number also accepted) |

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

Variations you will see:

- `Motor: ENABLED, RUNNING, HALT active, QUICK-STOP active` — extra tokens
  appear when those bits are set.
- `State: Quick stop active (entering initialization)`.
- `Homing:` is mode-aware. In homing mode it reads the drive live:
  `IN PROGRESS`, `complete` or `ERROR`. In the other modes it reports what
  the controller latched: `homed` (a homing run completed since the last
  re-init), `not homed since boot`, or `ERROR (last run failed)`.
- `Limits: CW *** ACTIVE *** | CCW clear` when a limit switch is pressed;
  `Origin switch (SW): ON` while the origin sensor is covered.
- Bit 13 line by mode: `OVER-POSITION ERROR (bit13)` (position),
  `MAX ACCELERATION reached (bit13)` (velocity), `HOMING ERROR (bit13)`
  (homing); plus `WARNING active (bit7)` when set.
- On a fault the last line becomes
  `FAULT: over-current (0x1001=0x2) -> send R:1 to reset` — the fault names
  are `over-current`, `voltage-error`, `over-temperature`,
  `communication-error`, `over-position-protect`, `motor-phase-loss`,
  `generic-error`, or `unknown`. (The hint says `R:1`; the positional form
  for motor 1 is `R:1`, for motor 3 it is `R:,,1`.)
- A dead drive: `=== MOTOR 2 === NOT RESPONDING (TIMEOUT) - check power,
  slave ID, RS485 wiring`.

**Errors**: `ERR:no '?' fields in SD query` · `ERR:SD use SD, SD:? or
SD:1..6`.

### `LS` — limit switches, origin switch and homing state

**Shape A / C.** One line with every switch input and the homing state per
motor, machine-friendly, valid in any mode. `LS` = all motors, `LS:1,,1` =
motors 1 and 3.

```
Send:  LS
Reply: LIMITS: M1:CW=0 CCW=0 SW=1 HOMED=1 MODE=POS,M2:CW=1 CCW=0 SW=0 HOMED=BUSY MODE=HOM,M3:ERROR
```

| Field | Values | Source |
|---|---|---|
| `CW` / `CCW` | `1` = that limit-switch input is active | status bits 14 / 15 |
| `SW` | `1` = the origin (mechanical-origin) switch input is active | status bit 11 |
| `HOMED` | `0` never homed since the drive was last initialised · `BUSY` homing in progress · `1` homing completed · `ERR` the drive reported a homing error | bit 12 / 13 while in homing mode, latched afterwards |
| `MODE` | `POS`, `VEL` or `HOM` — the mode the controller last commanded | controller |

**Errors**: `ERR:LS no motors selected (e.g. LS:1,,1)`.

### `DG` — low-level register diagnosis

**Shape A / F.** `DG:<motor>` reads back what the drive actually holds and
live-tests both Modbus write function codes; `DG`, `DG:0` or `DG:all` does
it for every motor, one packet each. Harmless: the write tests re-write
values already in the drive, so nothing moves.

```
Send:  DG:1
Reply: DG M1: fmt6000=0 st6041=0x1437 mode6061=1 err1001=0x0
       tgt607A=0,10000 vel6081=0,3 acc6083=5 dec6084=5 pos6064=0,10000
       WRITE06=OK WRITE10=OK libScale=10
```

| Field | Register | Meaning |
|---|---|---|
| `fmt6000` | 0x6000 | 32-bit word order (0 = high word first, expected) |
| `st6041` | 0x6041 | status word |
| `mode6061` | 0x6061 | operation mode shown by the drive: 1 position, 3 velocity, 6 homing |
| `err1001` | 0x1001 | error register (see §12) |
| `tgt607A` | 0x607A | target position as `hi,lo` 16-bit words |
| `vel6081` | 0x6081 | profile velocity as `hi,lo`, in drive units (rps × 10) |
| `acc6083` / `dec6084` | 0x6083 / 0x6084 | ramps in drive units (rps/s × 10) |
| `pos6064` | 0x6064 | actual position as `hi,lo` |
| `WRITE06` / `WRITE10` | — | result of a single-register / multi-register write test; `EXCEPTION(exc2)` shows the Modbus exception code |
| `libScale` | — | the library's velocity scale (10 = rps × 10; 1 after `setDriveP45(0)`) |

How to read it when a motor "replies OK but does not move": `tgt607A` not
changing after `P` → writes are not landing (check `WRITE10`); `vel6081=0,0`
→ velocity 0, send `V`; `mode6061` wrong → send `R`; `err1001` non-zero →
fault, send `R`. **Errors**: `ERR:DG usage DG, DG:all or DG:1..6`.

---

## 9. Administration commands

### `I` — detect and initialise motors

**Shape A / C.** Re-runs the full initialisation and reports per motor. `I`
or `I:all` = every configured slot, `I:1,,1` = motors 1 and 3.

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
detected and initialized!` is appended. With many motors the report arrives
in several packets (about three motors per packet); the `Total …` line is in
the last one.

**What it triggers**: cancels any deferred move; per motor: enable sequence,
position mode, that motor's ramps (sketch table or runtime `AD`), homing
speeds, and marks the motor online/offline for the event monitor. The
homed flag is kept. Initialisation does not move the motor or change its
position counter. **Errors**: `ERR:I no motors selected (e.g. I:1,,1)`.

### `R` — alarm reset and re-initialise

**Shape C.** Clears a drive fault and re-runs that motor's initialisation.

| Send | Effect | Reply |
|---|---|---|
| `R` or `R:all` | all motors | `All motors reset complete` |
| `R:1` | motor 1 | `Motor 1 reset complete` |
| `R:,,1` | motor 3 | `Motor 3 reset complete` |
| `R:1,,1` | motors 1 and 3 | `Motor 1,3 reset complete` |

A motor that fails to come back is tagged `(FAIL)`: `Motor 2(FAIL) reset
complete`. **What it triggers**: cancels any deferred move; per motor: alarm
reset control word, then full re-init with that motor's current ramps, homing
speed and `HO` offset; clears the fault flag in the monitor so a persisting
fault will be reported again as a new `EVENT:FAULT`. **Errors**: `ERR:R no
motors selected (e.g. R:1,,1)`.

### `MC` — motor count at runtime

**Shape D / E.** How many slave IDs (1..n) the controller manages.

| Send | Reply |
|---|---|
| `MC:?` or `MC:` | `MC: 6 motors (slave IDs 1..6)` |
| `MC:12` | `MC: 12 motors, 7 online (not saved - set in sketch with setMotorCount)` |

Growing the count probes and initialises the new slots at once; shrinking it
simply stops managing the higher IDs. Not persistent — the sketch's
`setMotorCount()` applies after reboot. Monitor latency scales with the count
(§10). **Errors**: `ERR:MC range 1..32`.

### `X` — reboot the controller

**Shape A.** Replies `System restart initiated...`, waits 50 ms and performs
a soft reset of the Controllino. After re-initialisation a new `Motor ready`
arrives at `PC_IP:PC_PORT`. All runtime settings (`V`, `AD`, `HV`, `HO`, `MC`)
revert to the sketch values; drive position counters are **not** affected
(the drives are not power-cycled).

---

## 10. Events: what the controller sends on its own

A background monitor reads **one motor's status word every 8 ms**, cycling
through the configured motors, and pushes a datagram to `PC_IP:PC_PORT`
whenever something **changes**. Events are edge-triggered: you get one message
when a condition appears and one when it clears, never a repeat while it
persists.

| Trigger (what happened on the drive) | Event pushed | What to do |
|---|---|---|
| CW limit input goes active (status bit 14 rises) | `EVENT:LIMIT:M2:CW POS=48211 ST=0x4677` | the drive has already stopped that direction; move away (`PR`) or re-home |
| CW limit input released | `EVENT:LIMIT:M2:CW_CLEARED` | — |
| CCW limit input goes active (bit 15) | `EVENT:LIMIT:M2:CCW POS=-120 ST=0x8677` | as above |
| CCW limit released | `EVENT:LIMIT:M2:CCW_CLEARED` | — |
| Origin switch input goes active (bit 11 rises) | `EVENT:ORIGIN:M2:ON POS=1012 ST=0x1c37` | informational; homing methods 3/4/7/8/11/12 stop here |
| Origin switch input released | `EVENT:ORIGIN:M2:OFF` | — |
| Homing run completes (bit 12 rises while in homing mode, after `H<n>`) | `EVENT:HOMED:M2 POS=0 ST=0x1437` | the counter now reads the `HO` offset relative to the switch; safe to position |
| Homing run fails (bit 13 while in homing mode) | `EVENT:HOMING_ERROR:M2 ST=0x3437 (check limit/origin wiring, HV, HO)` | check the switches, `HV`, `HO`; `R:…` then retry |
| Fault bit (bit 3) rises | `EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT` | read `SD`, fix the cause, `R:…` |
| Fault bit clears | `EVENT:FAULT_CLEARED:M3` | — |
| Two consecutive status reads fail | `EVENT:OFFLINE:M4` | check power / RS-485 / slave ID; the controller retries every 3 s |
| An offline drive answers again | `EVENT:ONLINE:M4 ST=0x1437 (re-initialised)` | the drive was **re-initialised automatically** (ramps, homing speed, `HO`), but if it was power-cycled its **position counter is now 0** — re-home or `ZO` before trusting coordinates |

`ERR=` in a fault event is the raw 0x1001 register, followed by the decoded
names: `CURRENT`, `VOLTAGE`, `TEMP`, `COMM`, `OVERPOS`, `PHASE`, or `GENERIC`
when no specific bit is set.

**Latency**: a change is noticed within about *count × 13 ms* (6 motors →
under 100 ms; 32 motors → under 450 ms). Offline drives are probed only every
3 s with a 12 ms timeout so a dead drive cannot slow the sweep for the live
ones.

**What is *not* an event**: target reached and the end of a `B` ramp are not
pushed. Poll `S:n,COMPACT` (`TR`) for those.

---

## 11. Trigger and response matrix

One row per command: what it touches, what it sends to the drives, and what
comes back.

| Command | Writes to drives? | Stored in controller? | Cancels a pending move? | Brakes a jog first? | Reply |
|---|---|---|---|---|---|
| `P` / `PR` | yes: target + velocity, enable, start | mode = position | supersedes it | yes, non-blocking, `OK` deferred | `OK` |
| `PA` / `PRA` | yes, broadcast | mode = position | supersedes it | yes, non-blocking | `OK` |
| `P:?` `PR:?` `PA:?` | reads position | — | no | no | `M1=…` / `WARN: …` |
| `V` | **no** (used at next move) | yes | no | no | `Set … V: …` |
| `VA` | yes, broadcast velocity | yes | no | no | `VA: …` |
| `VS` | yes: mode, velocity, start (broadcast or per motor) | mode = velocity | yes | — | `VS: velocity mode started … (stop with B)` |
| `AD` / `ADA` | yes, selected / all drives (+ quick-stop ramp) | yes | no | no | `AD: M1=… rps/s (accel/decel)` / `ADA: all motors …` |
| `B` `B1` `B2` `B3` | yes: halt / quick-stop | — | yes | — | `… stopped with deceleration` / `… brake applied to …` |
| `L` | yes: brake open (torque off) | — | no | no | `… brake(s) loosened` |
| `BS` | reads status | — | no | no | `BRAKE STATUS: …` |
| `Z` / `ZOA:0` | yes: homing method 0 + mode | zero flag, mode = position | no | yes, blocking ≤ 5 s | `… set to zero position` |
| `ZO` / `ZOA` | yes: home offset + method 0 + mode | zero flag, mode = position | no | yes, blocking ≤ 5 s | `ZO: … (current position redefined)` |
| `ZS` | reads position | — | no | no | `ZERO STATUS: …` |
| `LS` | reads status | — | no | no | `LIMITS: …` |
| `HV` / `HVA` | yes: 0x6099 + 0x609B | yes | no | no | `Set … HV: …` |
| `HO` / `HOA` | yes: 0x607C | yes | no | no | `HO: … (used at next homing)` |
| `H<n>` | yes: `HO`, homing mode, method, start | mode = homing, homing armed | no | yes, blocking ≤ 5 s | `Homing method n started for …` then `EVENT:HOMED` / `EVENT:HOMING_ERROR` |
| `S` | reads status (+ position/velocity for COMPACT/DETAIL) | — | no | no | per format |
| `SD` | reads status, position, velocity, error reg | — | no | no | one packet per motor |
| `DG` | reads 9 registers, 2 harmless writes | — | no | no | `DG M<n>: …` (one packet per motor) |
| `I` | yes: full init, selected slots | online flags, mode = position | yes | — | `MOTOR INITIALIZATION: …` |
| `R` | yes: alarm reset + init | online/fault flags, mode = position | yes | — | `… reset complete` |
| `MC` | yes, for newly added slots | yes (RAM) | no | no | `MC: …` |
| `X` | no | all RAM cleared | yes (reboot) | — | `System restart initiated...` then `Motor ready` |

---

## 12. Status word, drive states and error register

### Status word 0x6041 (what `S`, `SD`, `LS`, events and `BS` decode)

Bits 0–9, 11, 14 and 15 mean the same in every mode. **Bits 10, 12 and 13
are redefined by the drive per operation mode** (manual, 0x6041 table), which
is why the controller tracks the mode it last commanded and names them
accordingly.

| Bit | Mask | Flag name (`S:n,FLAGS`) | Meaning when 1 |
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
origin switch · `0x1438`/`0x…F` fault.

### Drive state (low nibble, shown by `S:…,STATE` and `SD`)

| Low 4 bits | State name |
|---|---|
| 0x0 | Uninitialized |
| 0x1 | Initialized |
| 0x3 | Power on |
| 0x7 | Device enable (bit 5 set) · Quick stop active (bit 5 clear) |
| 0x8 | Error state |
| 0xF | Fault alarm |
| other | Unknown |

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

Every `ERR:` string the controller can send, and what to fix.

| Message | Cause |
|---|---|
| `INVALID COMMAND` | unknown head — commands are upper-case |
| `ERR:P needs positions` | `P:` / `PR:` with no argument |
| `ERR:no positions` | every field empty, e.g. `P:,,` |
| `ERR:no '?' fields in P query` | a `?` was in the argument but not in a valid slot |
| `ERR:PA needs position` · `ERR:PRA needs delta` | `PA:` / `PRA:` with no argument |
| `ERR:V needs argument` | `V:` with no argument |
| `ERR:VA needs velocity` | `VA:` with no argument |
| `ERR:VA range 0-1000 rps` | negative or absurd speed |
| `ERR:VS no motors selected (e.g. VS:1,,1)` | selector fields all empty |
| `ERR:AD range 0.1..6553.5 rps/s (Mn)` | zero, negative or too large ramp in slot n |
| `ERR:AD needs values (…)` | `AD:` with only empty slots |
| `ERR:ADA range 0.1..6553.5 rps/s` | zero, negative or too large ramp |
| `ERR:B type is B, B1, B2 or B3` | `B4`, `B9`… |
| `ERR:B no motors selected (e.g. B2:1,,1)` | selector fields all empty |
| `ERR:L no motors selected (e.g. L:1,,1)` | as above |
| `ERR:BS no motors selected (e.g. BS:1,,1)` · `ERR:ZS …` · `ERR:LS …` · `ERR:I …` | as above |
| `ERR:Z no motors selected (e.g. Z:1,,1)` | as above |
| `ERR:ZO needs values` · `ERR:no values` | `ZO:` empty / all fields empty |
| `ERR:HV needs argument` | `HV:` empty |
| `ERR:HO needs steps (HO:1000 or HO:?)` · `ERR:no offset values` | `HO:` empty / all fields empty |
| `ERR:homing method H1..H12` | `H0`, `H13`… |
| `ERR:H no motors selected (e.g. H3:1,,1)` | selector fields all empty |
| `ERR:S motor 0..n` | motor number above the configured count |
| `ERR:S formats RAW\|COMPACT\|DETAIL\|STATE\|BITS\|FLAGS` | misspelt format |
| `ERR:no '?' fields in SD query` · `ERR:SD use SD, SD:? or SD:1..n` | bad `SD` argument |
| `ERR:DG usage DG, DG:all or DG:1..n` | out-of-range motor number |
| `ERR:R no motors selected (e.g. R:1,,1)` | selector fields all empty |
| `ERR:MC range 1..32` | count out of range |

`WARN:` is not an error: the command succeeded for the motors listed with
values; those marked `ERROR` did not answer on the bus.

---

## 14. Recipes and sequences

### First contact after flashing

```
                      ← Motor ready            (arrives on PC_PORT at boot)
I                     → MOTOR INITIALIZATION: … Total motors detected: 6 of 6
P:?                   → M1=0
S:0,COMPACT           → M1:RUN|STP|TR|POS=0|VEL=0.0,M2:…
```

### A typical positioning cycle

```
V:0.5,0.5,0.5         → Set V: 0.50,0.50,0.50       speed for the next move
ADA:0.3               → ADA: all motors accel=0.3 decel=0.3 rps/s
P:4000,8000,12000     → OK                            launched, synchronised
S:0,COMPACT           → …|MOV|NR|…                    still moving
S:0,COMPACT           → …|STP|TR|…                    arrived
PA:0                  → OK                            everyone home
```

### Jog, then position

```
V:1                   → Set Motor 1 V: 1.00
VS                    → VS: velocity mode started (stop with B)
B                     → All motors stopped with deceleration     (avoids the deferred wait)
P:0                   → OK
```

### Commission a homing axis

```
HV:0.2                → Set Motor 1 HV: 0.20          slow search speed
HO:1000               → HO: M1=1000 steps after sensor (used at next homing)
H3:1                  → Homing method 3 started for M1
                      ← EVENT:ORIGIN:M1:ON POS=-3120 ST=0x1e37   (switch found)
                      ← EVENT:HOMED:M1 POS=0 ST=0x1437           (done)
LS:1                  → LIMITS: M1:CW=0 CCW=0 SW=0 HOMED=1 MODE=HOM
P:?                   → M1=0
```

If `EVENT:HOMING_ERROR:M1 …` arrives instead, check the switch wiring and
polarity (drive P42), lower `HV`, then `R:1` and retry.

### Home every axis at once, then set per-axis ramps

```
HVA:0.2               → Set HV: 0.2,0.2,0.2,0.2,0.2,0.2
HOA:500               → HO: M1=500,…,M6=500 steps after sensor (used at next homing)
H3                    → Homing method 3 started for all motors
                      ← EVENT:HOMED:M1 … EVENT:HOMED:M6 (one per motor, any order)
LS                    → LIMITS: M1:… HOMED=1 …,M6:… HOMED=1 …
AD:0.2,0.2,0.5/0.2,0.5/0.2,0.1,0.1   → AD: M1=0.2/0.2,…,M6=0.1/0.1 rps/s (accel/decel)
```

### Recover from a fault

```
                      ← EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT
SD:,,?                → === MOTOR 3 … FAULT: over-current (0x1001=0x2) -> send R:… to reset
   (fix the cause: jam, wiring, load)
R:,,1                 → Motor 3 reset complete
                      ← EVENT:FAULT_CLEARED:M3
```

### Handle a limit switch

```
                      ← EVENT:LIMIT:M2:CW POS=48211 ST=0x4677
PR:,-2000             → OK                            back off the switch
                      ← EVENT:LIMIT:M2:CW_CLEARED
```

### Drive power-cycled while the controller kept running

```
                      ← EVENT:OFFLINE:M4
                      ← EVENT:ONLINE:M4 ST=0x1437 (re-initialised)
ZS                    → ZERO STATUS: …M4:ZERO(0)…      counter is 0 now
H3:,,,1               → Homing method 3 started for M4  (or ZO:,,,<known> if you know where it is)
```

### Emergency

```
B3:                   → Immediate brake applied to all motors
L                     → All motor brakes loosened      (only if the shafts must be free)
R                     → All motors reset complete      when safe to resume
```

### Speed-scale verification (once per new drive type)

```
V:1                   1 rps commanded
VS                    run, count shaft revolutions for 60 s (expect exactly 60)
B                     stop
```

Ten times too fast → add `jmc.setDriveP45(0)` in the sketch (see
[HARDWARE.md](HARDWARE.md)).
