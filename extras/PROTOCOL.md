# JMCMotors — Command Reference (v0.6)

The complete reference for the text protocol implemented by this library: the
one grammar every command follows, every command with its arguments, what each
one **triggers** inside the controller and on the RS-485 bus, every reply it can
produce, the unsolicited **events** the controller pushes on its own, and the
status/error words you will see in those replies.

Library: https://github.com/ailagari/JMCMotors
Author: LAGARI A <lagariscience@gmail.com> · License: MIT

Companion documents: [HARDWARE.md](HARDWARE.md) (drive setup, wiring, bring-up)
· [README](../README.md) (install and quick start).

**Boards.** The protocol is the same on a CONTROLLINO, an ESP32, a Mega 2560
and an Arduino Uno/Nano. The Uno leaves out `PS`/`PRS`, `SAVE`/`LOAD`, `SD`,
`DG`, `BS`, `ZS`, `MC:n` (`MC:?` works), `X`, `HVZ`/`HACC` (they follow
`HV`/`ACC`), the `S` formats `STATE`/`BITS`/`FLAGS`/`DETAIL`, and the
explanatory text after errors and in the `WD` / `I` replies and fault names
(the raw `ERR=` code stays); it manages at most 6 motors. Those commands
answer `ERR:<CMD> not available on this board`. Sections marked
*(not on Uno)* below are affected.

---

## Contents

1. [Transport and addressing](#1-transport-and-addressing)
2. [The grammar: one rule for every command](#2-the-grammar-one-rule-for-every-command)
3. [Reply types and multi-packet markers](#3-reply-types-and-multi-packet-markers)
4. [Motion commands](#4-motion-commands) — `P` `PR` `PS` `PRS` `SL`
5. [Speed and ramp commands](#5-speed-and-ramp-commands) — `V` `VS` `ACC` `DEC`
6. [Stopping commands](#6-stopping-commands) — `B` `B1` `B2` `B3` `L` `BS`
7. [Zero and homing commands](#7-zero-and-homing-commands) — `Z` `ZO` `ZS` `HV` `HVZ` `HACC` `HO` `H1`…`H12`
8. [Status and diagnostic commands](#8-status-and-diagnostic-commands) — `S` `SD` `LS` `DG`
9. [Controller commands](#9-controller-commands) — `I` `R` `MC` `WD` `SAVE` `LOAD` `VER` `PING` `X`
10. [Events: what the controller sends on its own](#10-events-what-the-controller-sends-on-its-own)
11. [Trigger and response matrix](#11-trigger-and-response-matrix)
12. [Status word, drive states and error register](#12-status-word-drive-states-and-error-register)
13. [Error message catalogue](#13-error-message-catalogue)
14. [Recipes and sequences](#14-recipes-and-sequences)
15. [Changes from 0.5 and 0.4](#15-changes-from-05-and-04)

---

## 1. Transport and addressing

Two link types carry the same text; the sketch picks one.

| Item | UDP (`JMCLinkUDP`: CONTROLLINO, ESP32 WiFi, Ethernet shields) | Serial (`JMCLinkSerial`: Uno USB, RS-232 COM port, any Stream) |
|---|---|---|
| Framing | one command per datagram, no terminator needed | one command per line, ended by CR, LF or CR LF |
| Controller listens on | the board's IP and port (`pcLink.begin(port, pcIp, pcPort)`) | the serial port (Uno example: 57600 baud) |
| Command replies go to | the **sender** of the command (its IP and source port) **and** `PC_IP:PC_PORT` (once if those are the same) | the serial port; each message ends with CR LF |
| Events and `Motor ready` go to | `PC_IP:PC_PORT` **only** | the serial port |
| Longest command | 511 characters (enough for 32 positions); 95 on an Uno | same |
| Sender filter | `pcLink.allowSender(ip)` (up to 4); others are dropped and counted | — (physical link) |

| Item | Value |
|---|---|
| Whitespace | leading/trailing whitespace and line endings are trimmed |
| Case | command heads may be upper or lower case (`p:1000` works); values are case-insensitive too |
| Authentication | none beyond the UDP sender filter — keep the control network separate |
| Unknown command | reply `ERR:unknown command <HEAD>` |
| Too long | `ERR:command too long` — the line is dropped, never truncated |

### Static addressing, no pairing

There is no discovery, beacon or auto-adopt. Both addresses are fixed in the
sketch:

```cpp
Ethernet.begin(mac, boardIp);                   // or WiFi.config(...) + WiFi.begin(...)
pcLink.begin(boardPort, pcIp, pcPort);          // listen here; events go to pcIp:pcPort
pcLink.allowSender(pcIp);                       // optional: only the PC may command
```

- **Controller side** — the board's own static IP/port; each board needs a
  unique IP (and, on Ethernet, MAC).
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

- Numbers are checked **strictly**: the whole field must be a number
  (`-1000`, `+5`, `0.25`, `.5`). `P:12abc`, `V:fast` or `P:1e3` are refused
  with `ERR:P bad value '12abc' (M1)` and nothing moves. Integers must fit in
  32 bits.
- **Tags:** any command may start with `#<digits> ` (up to 9 digits and a
  space): `#17 P:1000`. Every reply to that command (including deferred `OK`s
  and all `:MORE` packets) starts with the same `#17 `; events never carry a
  tag. Use it to match replies to requests when several are in flight.
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
| synchronised arrival | `PS:1000,,3000` · `PRS:M2=-500,M5=800` | `PSA:5000` · `PRSA:-500` |
| soft limits | `SL:-1000/50000,,0/8000` · `SL:M2=OFF` · `SL:?` | `SLA:0/100000` · `SLA:OFF` · `SLA:?` |
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
| **Drive did not take it** | `ERR:P M3 not accepted (TIMEOUT) - nothing started` | a drive did not answer, or answered with a Modbus exception, while the command was being sent (see below) |
| **Unknown** | `ERR:unknown command PX` | head not recognised |
| **Deferred** | `OK` arrives later | `P`/`PR` sent while motors are in velocity mode (see §4) |
| **No further reply** | — | `X` answers once, then reboots |

**`OK` means every drive accepted it.** Every frame the controller sends to
a drive is checked (CRC, timeout, exception). If one fails, the reply names
the first motor and the reason — `TIMEOUT`, `CRC`, `EXCEPTION`, `BADFRAME`
— and says what happened:

| Command | When a drive fails | Reply |
|---|---|---|
| `P` `PR` `PS` `PRS` (+A) | while the targets are sent: **nothing is started** (a partial coordinated move is worse than none) | `ERR:P M3 not accepted (TIMEOUT) - nothing started` |
| same | at the start pulse (rare): the others may be moving | `ERR:P M3 not accepted (TIMEOUT) to the start - check it with S` |
| `VS` | jog not started; motors already prepared are halted again | `ERR:VS M3 not accepted (TIMEOUT) - nothing started` |
| `B` `B1` `B2` `B3` | the others **were** stopped | `ERR:B M3 not accepted (TIMEOUT) - the others were stopped` |
| `Z` `ZO` | that motor's position was not set | `ERR:Z M3 not accepted (TIMEOUT) - position not set` |
| `H<n>` | homing may not have started on it | `ERR:H M3 not accepted (TIMEOUT) - homing may not have started` |
| `ACC` `DEC` `HV` `HVZ` `HACC` `HO` | stored in the controller, written again at the next `I` / `R` / re-init | `ERR:ACC M3 not accepted (TIMEOUT) - stored, written again at the next I / R` |
| `L` | | `ERR:L M3 not accepted (TIMEOUT)` |
| `I` `R` | per motor in the report | `NOT DETECTED (TIMEOUT)` / `Motor 3(FAIL) reset complete` |

A drive that fails is also polled by the monitor; after two failed reads it
is reported `EVENT:OFFLINE:Mn`, and from then on motion commands that select
it are refused (`ERR:P M3 offline …`) and stops skip it (it still receives the
broadcast stop) until `EVENT:ONLINE`.

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
| `PA:5000` | every motor → 5000, each at its own `V`, broadcast start | `OK` |
| `P:?` · `P:,,?` · `P:?,?` · `P:M3` | **read** position of M1 · M3 · M1+M2 · M3 | `M1=10000` · `M3=200` · `M1=10000,M2=2000` |
| `PA:?` | read every motor | `M1=…,M2=…,…` |

**What it triggers**

1. For each addressed motor the controller writes the target (0x607A) and the
   motor's stored velocity (`V`) into the drive — *prepare*.
2. Then it launches. `PA`, and a `P` that addresses every motor, launch with
   **broadcast** frames (slave ID 0) so all drives start in the same instant;
   a subset gets its start edges back to back, one frame apart (about 2 ms
   per motor at 115200 baud, 6 ms at 38400). Every motor runs at **its own**
   `V` (send `VA` first for one common speed).
3. The new target takes effect **immediately** (control-word bit 5): a `P`
   sent mid-move abandons the move in flight and heads to the new target.
   The start is the field-tested sequence: prepare (control word 0x2F),
   bit 4 rising (0x3F), bit 4 low (0x2F).
4. **Soft limits** (`SL`) are checked first: a target outside the motor's
   limits refuses the whole command (`ERR:P M2 target 60000 outside soft
   limit 0..50000`) and nothing moves.
5. `OK` means "launched", not "arrived". Watch `S:n:COMPACT` (`TR` = target
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
motors (e.g. PA:1000)` · `ERR:P cannot mix values and ?` · `ERR:P bad value 'x' (M2)`
· `ERR:P M2 offline (check power / RS-485, or send I)` (a selected motor is
offline: nothing moves; `PA` skips offline motors) · `ERR:P no motor online`
· `ERR:P M2 target … outside soft limit …`. On the Uno the `(e.g. …)` hints
are left out.

### `PR` — relative position move · `PRA` — all motors

Same syntax, launch sequence, deferral behaviour and errors as `P` (with
`PR` in the messages), but every value is a **delta from the motor's current
position**: `PR:1000` forward 1000, `PR:-1000` back, `PR:500,,-500`
synchronised, `PRA:-250` every motor back 250. `PR:?` and `PRA:?` read
positions, identical to `P:?` / `PA:?`.

**How it is sent (0.6).** The controller reads each motor's actual position
and sends an absolute target, *position + delta*. It does not use the drive's
relative bit (control word bit 6): per the manual that adds the delta to the
drive's internal previous target, which cannot be read back (0x607A returns
the last value written) and need not match the shaft after `Z`, `ZO` or
homing — `P:5000`, `ZO:0`, `PR:100` could otherwise travel to 5100. Sent
mid-move, `PR` is relative to where the shaft is at that moment. Soft limits
are checked against the exact target; a target beyond the 32-bit range is
held at ±2,147,483,647 (and then refused by any soft limit). A motor that
does not answer the position read refuses the command: `ERR:PR M2 not
responding`.

### `PS` — synchronised-arrival move · `PSA`, `PRS` · `PRSA` *(not on Uno)*

Same slots as `P` / `PR` (`PS:1000,,3000`, `PS:M2=500,M5=-800`, `PSA:0`,
`PRS:-500,,500`), but every selected motor **starts and arrives together**,
moving in a straight line in joint space.

**What it triggers**

1. Reads every selected motor's actual position (a `PRS` delta is taken from
   the actual position, because the scaling needs exact distances). A motor
   that does not answer refuses the command: `ERR:PS M2 not responding`.
2. Checks soft limits, like `P`.
3. Works out one common profile: with *dᵢ* = each motor's distance and *k* =
   *dᵢ / dmax*, the speed, acceleration and deceleration are the largest that
   keep **every** axis at or below its own `V`, `ACC` and `DEC` once scaled by
   *k*. The longest axis runs its profile; the others run proportionally
   slower profiles of the same duration.
4. Writes each motor's scaled speed and ramps, prepares the targets and starts
   them together (broadcast when every motor takes part). The motors' own
   `ACC`/`DEC` are written back before their next normal move or jog.
5. Replies `OK`, or `OK (sync within 20% - drive resolution is 0.1 rps and
   0.1 rps/s; raise V, ACC, DEC for tighter sync)` when rounding to the
   drive's 0.1 rps / 0.1 rps/s registers makes the axes' speeds or ramps
   differ from each other by more than 2 %. The number is the largest such
   difference (an upper bound: a ramp mismatch only matters while ramping).
   Slow axes are the ones affected — e.g. an axis scaled to 0.25 rps runs at
   0.3. Axes that all round the same way stay in step and get a plain `OK`.

Motors that are jogging are braked first and the move starts when they are at
rest, exactly like `P`. `PS:?` is refused (`ERR:PS takes values; read
positions with P:?`).

### `SL` — software position limits · `SLA`

| Send | Effect | Reply |
|---|---|---|
| `SL:-1000/50000` | motor 1 may only be commanded within −1000 … 50000 steps | `SL: M1=-1000/50000` |
| `SL:0/8000,,0/12000` · `SL:M3=0/12000` | per motor | `SL: M1=0/8000,M3=0/12000` |
| `SL:M2=OFF` | remove motor 2's limits | `SL: M2=OFF` |
| `SLA:0/100000` · `SLA:OFF` | every motor | `SLA: all motors = 0/100000` |
| `SL:?` · `SL:M2` · `SLA:?` | read | `SL: M1=-1000/50000,M2=OFF` |

`P`, `PR`, `PS`, `PRS` outside the limits are refused before anything is
sent. A jog (`VS`) is checked by the monitor while it runs and stopped at the
limit in its direction of travel: `EVENT:SOFTLIMIT:M1 POS=50012 LIMIT=50000`
(the motor overshoots by its deceleration distance). Soft limits are set in
the sketch with `setMotorSoftLimits(id, min, max)` and are kept by `SAVE`.
They complement, and do not replace, the drives' hardware limit switches.
**Errors**: `ERR:SL value must be min/max with min < max, or OFF (M1)`.

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
| `VA:1` | every motor; also **sent at once to every motor that is jogging**, so a running jog changes speed at once | `VA: all motors = 1.00 rps (BROADCAST)` |
| `V:?` · `V:,,?` · `V:M3` · `VA:?` | read | `V: M1=0.50 rps` · `V: M1=0.30,M2=0.30,… rps` |

Default 0.3 rps = 18 RPM. Range 0.01–1000 rps in magnitude. **Errors**:
`ERR:V needs values (…)` · `ERR:V range 0.01..1000.0 rps (M2)` · `ERR:V cannot
mix values and ?`.

### `VS` — start velocity mode (endless rotation) · `VSA`

| Send | Effect | Reply |
|---|---|---|
| `VSA` | all motors jog at their `V` speeds, broadcast start | `VS: velocity mode started (stop with B)` |
| `VS:1,,1` · `VS:M1,M3` | motors 1 and 3 only | `VS: velocity mode started for M1,3 (stop with B)` |

**What it triggers**: cancels any deferred move; per selected motor, following
the manual's velocity-mode routine: halt (0x010F), select velocity mode, write
the motor's own speed, so nothing turns yet; then one broadcast release
(0x000F) when every motor is selected, so all shafts start in the same
instant, or a per-motor release for a subset.
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
0.1–3276.7 (register 32767: the manual types 0x6083 / 0x6084 both as UNSIGNED16 and as INTEGER16, so the controller stays inside both). **Errors**: `ERR:ACC range 0.10..3276.7 rps/s (M2)` and the
generic ones above.

---

## 6. Stopping commands

### `B`, `B1`, `B2`, `B3` — stop

The **stop type is the digit in the head**; the argument selects motors.

| Head | Stop type | How the drive stops |
|---|---|---|
| `B` (= `B0`) | controlled stop | halt bit: ramps down on the normal `DEC` ramp (the controller sets the drive's halt code 0x605D to 1 = "halt with the current deceleration" at every init); the jog setpoint is zeroed so the motor cannot resume when halt is released |
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

**What it triggers**: cancels any move waiting for a ramp-down ("stop wins").
A stop of **every** motor goes out first as **one broadcast frame** (slave 0,
allowed for writes by the manual): all drives stop in the same instant, and a
drive that does not answer cannot delay the others. Then each selected online
motor gets the stop again individually, which confirms it heard it; one that
does not answer gives `ERR:B M3 not accepted (TIMEOUT) - the others were
stopped`. A stop of selected motors (`B:,1`) is sent to those motors only. The quick
stops `B1`…`B3` write the code to 0x605A, then control word **0x000B**: bit 2
= 0 with bit 1 = 1 is "quick stop" in the manual's state table, and bits 0 and
3 stay 1, so there is no "motor enable 1→0" edge (bit table) that could let
the motor coast instead of braking. After a halt (`B`), the next move keeps
the halt on until its own start pulse, so the drive cannot resume its
previous target for a moment. After a `B` the status word shows
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
max 5 s); then, per motor, homing method 0 "set current position" (manual
p.20), a wait for status bit 12 (complete, at most 200 ms), a **read-back of
the position**, and back to position mode. The shaft does not move. The
controller marks the motor as zero-set for `ZS`.

### `ZO` — current position becomes a value · `ZOA`

| Send | Effect | Reply |
|---|---|---|
| `ZO:10000` | where motor 1 is now reads 10000 | `ZO: M1=10000 (current position redefined)` |
| `ZO:10000,,500` · `ZO:M1=10000,M3=500` | M1 reads 10000, M3 reads 500 | `ZO: M1=10000,M3=500 (current position redefined)` |
| `ZOA:5000` | every motor reads 5000 | `ZO: M1=5000,M2=5000,… (current position redefined)` |

Use it to restore a known coordinate after a drive power cycle without
re-homing. Internally `Z` and `ZO` run homing method 0 with the value in the
drive's home-offset register — the same register `HO` uses — which is why
every `H<n>` re-writes the stored `HO` first.

**Verified, not assumed.** The manual does not say which sign the offset has
for method 0, so the controller reads the position back. A drive that applies
it the other way round (reads −value) is set again with the sign flipped and
checked again; a drive that ignores it is reported: `ERR:ZO M1 reads 0 after
setting 5000`. **Errors**: `ERR:ZO needs values (…)` · `ERR:ZO takes values,
not ? (read positions with P:?)` · `ERR:ZO M1 reads <x> after setting <v>`.

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

Default 0.3 rps. The drive's methods allow at most 300 rpm (methods 1–4) or
1200 rpm (5–12), so `HV` and `HVZ` accept 0.01–20 rps; keep methods 1–4
under 5 rps.

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
| `HO:1000,,-800` · `HO:M3=-800` | per motor; negative = the other direction — **methods 9–12 only** (manual §6.3; `H1`…`H8` refuse it) | `HO: M1=1000,M3=-800 steps after sensor (used at next homing)` |
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

**Which way is which.** The manual's homing descriptions (§6.3) put the **CW
limit at the "left" end** (method 1 moves left to it) and the CCW limit at the
"right" end; with the usual CiA 402 numbering that makes CW the end reached
moving towards **lower (negative) positions** and CCW the positive end. The
drive refuses motion *into* an active limit and allows motion away from it. So
if negative moves do nothing while positive ones work, check `LS:Mn`: `CW=1`
with nothing pressed means the input polarity (drive P42) or the wiring is
wrong. Confirm the directions once on your machine: jog slowly negative
(`V:-0.1`, `VS:Mn`) and press each switch by hand.

**Checked before anything moves (0.6)**, from the same section of the
manual: methods 1–4 home at most at 300 r/min, so `HV` and `HVZ` above 5 rps
are refused for them (`ERR:H3 M1 HV/HVZ above 5 rps (H1..H4: max 300 r/min)`);
only methods 9–12 accept a negative home offset, so a negative `HO` with
methods 1–8 is refused (`ERR:H3 M1 negative HO needs H9..H12`). The manual's
own homing example uses 10 rps with method 1; the controller follows the
stated 300 r/min limit.

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
· `ERR:H3 M1 HV/HVZ above 5 rps (H1..H4: max 300 r/min)` · `ERR:H3 M1 negative
HO needs H9..H12`
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
`R` and `I` run the manual's power-up routine (0x0001 → 0x0003 → 0x000F),
which briefly switches the motor off: send them with the motor stopped. The
jog speed is set to 0 before the motor is enabled again, so a drive that was
left in velocity mode cannot start turning.
Clears a drive fault, re-runs init with the motor's ramps and homing
parameters, and keeps the homed flag. If the fault is gone afterwards, it
sends `EVENT:FAULT_CLEARED:Mn` right away; if it persists, it is reported again
as a new `EVENT:FAULT`.

### `MC` — motor count

| Send | Reply |
|---|---|
| `MC:?` or `MC` | `MC: 6 motors (slave IDs 1..6)` |
| `MC:12` | `MC: 12 motors, 7 online (SAVE to keep across reboot)` |

Growing the count probes and initialises the new slots at once. Kept across
reboot only by `SAVE`; otherwise the sketch's `setMotorCount()` applies.
**Errors**: `ERR:MC range 1..32`.

### `WD` — communication watchdog

| Send | Effect | Reply |
|---|---|---|
| `WD:500` | if no command arrives for 500 ms, every motor stops | `WD: 500 ms - send any command (PING) more often than that` |
| `WD:0` | off (default) | `WD: off` |
| `WD:?` · `WD` | read | `WD: off` · `WD: 500 ms - all motors stop when no command arrives for that long` |

When it expires: any queued move is cancelled, every online motor gets a
controlled stop (`B`), and one event goes out:
`EVENT:WATCHDOG:STOP no command for 500 ms - all motors stopped`. It re-arms
with the next command. Any command feeds it, including `PING` and rejected
ones; datagrams dropped by the sender filter do not. Range 50 … 65000 ms; set
in the sketch with `setCommandWatchdog(ms)`; kept by `SAVE`.
**Errors**: `ERR:WD range 0 (off) or 50..65000 ms`.

The separate **processor watchdog** (`enableHardwareWatchdog(ms)` in the
sketch) reboots the board if the firmware itself hangs.

### `SAVE` · `LOAD` — settings in EEPROM *(not on Uno)*

**Policy.** The sketch's values are the admin's configuration: edit, flash,
done. `SAVE` exists for changes made over UDP on a *deployed* unit, so nobody
has to plug in a USB cable. The saved block carries a fingerprint of the
sketch values it was saved under, and is applied at boot **only while that
fingerprint still matches**. The moment the admin re-flashes the sketch with
different values, the sketch wins again and the block is ignored (reported
on the Serial Monitor and by `SAVE:?`). An old `SAVE` can therefore never
override a new flash.

| Send | Effect | Reply |
|---|---|---|
| `SAVE` | store `V`, `ACC`, `DEC`, `HV`, `HVZ`, `HACC`, `HO`, `SL` of every motor, the `WD` time and the motor count (AVR: EEPROM, ESP32: flash) | `SAVE: 6 motors stored (V, ACC, DEC, HV, HVZ, HACC, HO, SL, WD, MC)` |
| `SAVE:?` | is anything stored, and does it apply? | `SAVE: nothing stored - sketch values in use` · `SAVE: settings stored and applied at boot (SAVE:CLEAR to forget)` · `SAVE: settings stored but IGNORED - sketch was re-flashed with other values (LOAD applies them, SAVE re-arms)` |
| `SAVE:CLEAR` | forget the stored block | `SAVE: cleared - sketch values apply at next boot` |
| `LOAD` | apply the stored block now, even if the sketch changed since | `LOAD: 6 motors loaded and applied` |

The block is CRC-protected; a corrupt or absent block is ignored
(`ERR:LOAD nothing stored (or checksum failed)` on `LOAD`). Re-flashing the
same sketch values keeps a saved block active; re-flashing different ones
sidelines it until the next `SAVE`. **Errors**: `ERR:SAVE use SAVE, SAVE:?
or SAVE:CLEAR` · `ERR:SAVE EEPROM too small`.

### `VER` · `PING` · `X`

| Send | Reply | Notes |
|---|---|---|
| `VER` | `JMCMotors 0.6.0 (max 32 motors)` | firmware version and this board's motor limit (6 on an Uno) |
| `PING` | `PONG` | cheap link check; no bus traffic |
| `X` | `System restart initiated...` then `Motor ready` on PC_PORT | soft reset (AVR: jump to 0, ESP32: `ESP.restart()`); runtime settings revert to the sketch (or the `SAVE`d block); drive counters untouched |

---

## 10. Events: what the controller sends on its own

A background monitor reads **one motor's status word every 8 ms**, cycling
through the configured motors, and pushes a datagram to `PC_IP:PC_PORT`
whenever something **changes**: one message when a condition appears, one
when it clears, never a repeat while it persists.

| Trigger (what happened on the drive) | Event | What to do |
|---|---|---|
| CW limit input goes active (bit 14 rises) | `EVENT:LIMIT:M2:CW POS=-48211 ST=0x4677` | the drive has stopped moving that way; CW is the negative ("left") end per the manual, so back off with a positive `PR` (see "Which way is which" in §7), or re-home |
| CW limit input released | `EVENT:LIMIT:M2:CW_CLEARED` | — |
| CCW limit input goes active (bit 15) / released | `EVENT:LIMIT:M2:CCW POS=-120 ST=0x8677` / `EVENT:LIMIT:M2:CCW_CLEARED` | as above |
| Origin switch input goes active (bit 11) / released | `EVENT:ORIGIN:M2:ON POS=1012 ST=0x1c37` / `EVENT:ORIGIN:M2:OFF` | informational; methods 3/4/7/8/11/12 stop here |
| Homing run completes (bit 12 in homing mode, after `H<n>`) | `EVENT:HOMED:M2 POS=0 ST=0x1437` | position is valid; safe to move |
| Homing run fails (bit 13 in homing mode) | `EVENT:HOMING_ERROR:M2 ST=0x3437 (check limit/origin wiring, HV, HO)` | check the switches and P42 polarity, lower `HV`, `R:…`, retry |
| Fault bit (bit 3) rises / clears | `EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT` / `EVENT:FAULT_CLEARED:M3` | `SD` for the cause, fix it, `R:…` |
| Two consecutive status reads fail | `EVENT:OFFLINE:M4` | check power / RS-485 / slave ID; retried every 3 s |
| A jog (`VS`) reaches a soft limit (`SL`) | `EVENT:SOFTLIMIT:M1 POS=50012 LIMIT=50000` | the motor was stopped; jog the other way or reposition |
| No command for the `WD` time | `EVENT:WATCHDOG:STOP no command for 500 ms - all motors stopped` | the PC or network stalled; check it, then resume |
| An offline drive answers again | `EVENT:ONLINE:M4 ST=0x1437 (re-initialised)` | re-initialised automatically (ramps, homing parameters, `HO`); a power-cycled drive is back at **position 0** — re-home or `ZO` |

`ERR=` in a fault event is the raw 0x1001 register followed by the decoded
names `CURRENT`, `VOLTAGE`, `TEMP`, `COMM`, `OVERPOS`, `PHASE`, or `GENERIC`.
**Latency**: about *count × 13 ms* (6 motors → under 100 ms; 32 → under
450 ms). **Not an event**: target reached and the end of a `B` ramp — poll
`S:n:COMPACT` for `TR`.

**What polling cannot see.** The monitor reads the status word, one motor
at a time; an input that is active for less than one sweep can fall between
two reads. Limit switches are not affected in practice — the drive stops
*on* an active limit, so it stays active until the motor backs off. An
origin switch crossed at speed (or a limit touched only while a homing run
reverses) can be missed; the drive itself still uses it, so homing works —
only the `EVENT:ORIGIN` for that brief pass may not arrive. Use `LS` while
commissioning, and do not rely on `EVENT:ORIGIN` as a position trigger.

---

## 11. Trigger and response matrix

| Command | Writes to drives? | Stored in controller? | Cancels a pending move? | Brakes a jog first? | Reply |
|---|---|---|---|---|---|
| `P` / `PR` / `PA` / `PRA` | yes: target + velocity, enable, start (broadcast for all) | mode = position | supersedes it | yes, non-blocking, `OK` deferred | `OK` |
| `PS` / `PRS` (+A) | yes: scaled speed/ramps + target, start | mode = position | supersedes it | yes, non-blocking, `OK` deferred | `OK` / `OK (sync within …)` |
| `P:?` `PA:?` | reads position | — | no | no | `M1=…` / `WARN: …` |
| `SL` / `SLA` | no | yes | no | no | `SL: …` / `SLA: …` |
| `WD` | no | yes | no | no | `WD: …` |
| `V` | **no** (used at next move) | yes | no | no | `Set V: M1=…` |
| `VA` | yes, to jogging motors | yes | no | no | `VA: all motors = …` |
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
| `VER` / `PING` | no | — | no | no | `JMCMotors 0.6.0 (max 32 motors)` / `PONG` |
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
| `ERR:V range 0.01..1000.0 rps (M2)` · `ERR:HV range 0.01..20.0 rps (M2)` (same for `HVZ`) · `ERR:ACC range 0.10..3276.7 rps/s (M2)` (same for `DEC`, `HACC`) | value out of range in that slot; nothing written |
| `ERR:ZO takes values, not ? (read positions with P:?)` | `ZO:?` |
| `ERR:B type is B, B1, B2 or B3` | `B4`, `B9`… |
| `ERR:homing method H1..H12` | `H0`, `H13`… |
| `ERR:S formats RAW\|COMPACT\|DETAIL\|STATE\|BITS\|FLAGS` | misspelt format |
| `ERR:MC range 1..32` | count out of range |
| `ERR:SAVE use SAVE, SAVE:? or SAVE:CLEAR` · `ERR:SAVE EEPROM too small` | bad `SAVE` argument / no room |
| `ERR:LOAD nothing stored (or checksum failed)` | `LOAD` with no valid block |
| `ERR:<CMD> bad value '<text>' (M<n>)` | a field is not a valid number |
| `ERR:<CMD> M<n> not accepted (<reason>)<what happened>` | a drive did not answer or rejected a frame while the command was sent — see "OK means every drive accepted it" in §3 |
| `ERR:command too long` | the line / datagram is longer than the command buffer (511, Uno 95) |
| `ERR:<CMD> M<n> offline (check power / RS-485, or send I)` · `ERR:<CMD> no motor online` | a motion command selected a motor the monitor has marked offline |
| `ERR:<CMD> M<n> target <t> outside soft limit <min>..<max>` | `SL` refused the move |
| `ERR:PS M<n> not responding` · `ERR:PR M<n> not responding` | `PS`/`PRS`/`PR` could not read a motor's position |
| `ERR:H<m> M<n> HV/HVZ above 5 rps (H1..H4: max 300 r/min)` | homing speed above the manual's limit for methods 1–4 |
| `ERR:H<m> M<n> negative HO needs H9..H12` | negative home offset with methods 1–8 |
| `ERR:PS takes values; read positions with P:?` | `PS:?` |
| `ERR:SL value must be min/max with min < max, or OFF (M<n>)` | bad `SL` field |
| `ERR:WD range 0 (off) or 50..65000 ms` | bad `WD` value |
| `ERR:<CMD> not available on this board` | Uno: `PS`, `PRS`, `SAVE`, `LOAD`, `SD`, `DG`, `BS`, `ZS`, `MC:n`, `X`, `HVZ`, `HACC` |
| `ERR:S formats on this board: RAW\|COMPACT` | Uno: other `S` formats |

`WARN:` is not an error: the command succeeded for the motors listed with
values; those marked `ERROR` did not answer on the bus.

---

## 14. Recipes and sequences

### First contact after flashing

```
                      ← Motor ready            (arrives on PC_PORT at boot)
VER                   → JMCMotors 0.6.0 (max 32 motors)
IA                    → MOTOR INITIALIZATION: … Total motors detected: 6 of 6
PA:?                  → M1=0,M2=0,…
SA:COMPACT            → M1:RUN|STP|TR|POS=0|VEL=0.0,M2:…
```

### A typical positioning cycle

```
VA:0.5                → VA: all motors = 0.50 rps
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
SAVE                  → SAVE: 6 motors stored (V, ACC, DEC, HV, HVZ, HACC, HO, SL, WD, MC)
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

### Coordinated move that arrives together

```
V:2,2,1               → Set V: M1=2.00,M2=2.00,M3=1.00 rps    per-axis maxima
PS:40000,10000,-5000  → OK                       all three arrive at the same moment
```

### Watchdog-protected session

```
WD:500                → WD: 500 ms - send any command (PING) more often than that
#1 PA:0               → #1 OK
PING   (every 200 ms) → PONG
                      ← EVENT:WATCHDOG:STOP no command for 500 ms - all motors stopped   (if the PC stalls)
```

### Emergency

```
B3                    → Immediate brake applied to all motors
LA                    → All motor brakes loosened      (only if the shafts must be free)
RA                    → All motors reset complete      when safe to resume
```

---

## 15. Changes from 0.5 and 0.4

**0.5 → 0.6** — the commands are unchanged; these are additions and the
behaviour changes worth knowing:

| 0.5 | 0.6 | Why |
|---|---|---|
| CONTROLLINO only, UDP only | CONTROLLINO, ESP32, Uno/Nano, Mega 2560; UDP or serial link | the hardware in use |
| `PA` ran every motor at motor 1's `V` | each motor at its own `V` | per-motor speeds are respected everywhere |
| a non-number parsed as 0 (`P:abc` → step 0) | `ERR:P bad value 'abc' (M1)` | a typo must never move a motor |
| over-long packet truncated | `ERR:command too long` | same |
| `PR` used the drive's relative bit 6 (adds to its internal previous target) | `PR` = actual position + delta, sent as an absolute target | exact after `Z`/`ZO`/homing; exact soft-limit checks |
| `OK` even if a drive missed a frame | `ERR:<CMD> Mn not accepted (<reason>) …`; coordinated moves abort before starting | `OK` is a guarantee |
| stops sent drive by drive | stop of all motors: one broadcast frame first, then confirmed per drive | simultaneous, and a dead drive cannot delay the others |
| homing speeds and offsets not checked | `H1`–`H4` refuse `HV`/`HVZ` > 5 rps; `H1`–`H8` refuse a negative `HO` | manual §6.3 |
| move waited up to 100 ms for status bit 12 (0.5.1) | removed; start sequence as in 0.1–0.5.0 | bit 12 is the set-point acknowledge in the manual's timing diagram; the wait only added latency |
| — | `PS`, `PRS`, `PSA`, `PRSA` | synchronised arrival |
| — | `SL`, `SLA`, `EVENT:SOFTLIMIT` | software travel limits |
| — | `WD`, `EVENT:WATCHDOG` | stop motors when the PC goes quiet |
| — | `#<n>` reply tags | match replies to requests |
| `SAVE` kept V … HO, MC | also `SL` and `WD` | |
| `VER` → `JMCMotors 0.5.1` | `JMCMotors 0.6.0 (max 32 motors)` | shows the board's motor limit |

**0.4 → 0.5**

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
