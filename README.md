# JMCMotors

**Control up to 32 integrated servo/stepper drives from a Controllino with
plain UDP text commands.**

Works with JMC integrated servo/stepper drives (IHSS / IHT "-RC" series) over
the Controllino's built-in RS-485, verified against the JMC Modbus-RTU manual
V2.2 and the IHSS60-R/RC datasheet, and bench-tested on real hardware.

---

## 1. Install

**Arduino IDE Library Manager:** Tools → Manage Libraries… → search
**JMCMotors** → Install (the CONTROLLINO and Ethernet libraries are pulled in
as dependencies). Also install the **CONTROLLINO** board package via Boards
Manager.

**From this repository:** Sketch → Include Library → Add .ZIP Library… →
select the downloaded ZIP, or copy the folder into
`Documents/Arduino/libraries/`.

**Examples** (File → Examples → JMCMotors):

| Example | What it is |
|---|---|
| `JMCMotors_6Motor` | the deployable controller: admin settings block, per-motor table, full UDP command system |
| `JMCMotors_Minimal` | the same command system in six lines, library defaults |
| `JMCMotors_DirectAPI` | moving a motor from C++ (`JMCBus` + `JMCMotor`), no Ethernet |

## 2. Set up the motors

Four steps per installation. The full version, with wiring diagram,
commissioning sequence and a troubleshooting table, is
**[extras/HARDWARE.md](extras/HARDWARE.md)**.

**2.1 Wire it**

- Motor power to each drive from the motor supply; the Controllino on its own
  logic supply, with a common ground.
- RS-485 **A/B daisy-chain** from the Controllino's built-in RS-485 port
  through every drive, **120 Ω termination across A/B at the last drive**.
- Limit / origin switches go to the **drive's** inputs, not the Controllino.
- Ethernet from the Controllino to the PC's switch (static IP, no DHCP).

**2.2 Set the drive parameters** (front panel, once per drive; restart the
drive afterwards)

| Parameter | Value | Why |
|---|---|---|
| **P43** | **0** | RS-485/Modbus mode — **factory default is 1 = CANopen and the drive won't answer!** |
| **P40** | 1, 2, 3… | unique slave ID = the motor number in every command, consecutive from 1 |
| **P41** | **7** | 115200 baud |
| **P42** | 0 / 1 | sensor polarity: 0 = PNP, 1 = NPN (default) — wrong polarity shows as a permanently active limit |
| **P17** | e.g. 10 (= 4000) | steps per revolution; positions in commands are raw steps, so `P:4000` is one turn only when P17 = 4000 |
| **P19** | 7 for geared loads | speed smoothness (S-curve); default 2, sometimes found at 0 |

**2.3 Configure the sketch** — motor count, network, and the speed/ramp table
(section 3 below). Sketch values are re-applied on every boot and whenever a
drive comes back online; values set over UDP last until reboot.

**2.4 Commission each axis** with the commands in section 5: `IA` (all
detected?) → `PR:4000` (one turn, right direction?) → `V:1` + `VS:M1` for 60 s
(exactly 60 revs?) → `ACC`/`DEC` until the start/stop is smooth → `Z` or `H<n>` to
set the zero → press each limit switch and watch for `EVENT:LIMIT`.

## 3. Flash the example

Open **File → Examples → JMCMotors → JMCMotors_6Motor**. Networking is
fixed/static — no pairing, no auto-discovery. Edit the block near the top:

```cpp
IPAddress CONTROLLINO_IP  (192, 168, 1, 101);   // this board's own address
uint16_t  CONTROLLINO_PORT = 8001;

IPAddress PC_IP   (192, 168, 1, 100);           // where replies/events go
uint16_t  PC_PORT  = 8000;

byte CONTROLLINO_MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
```

PC and Controllino must be able to reach each other directly (same subnet).
Flashing more Controllinos? Give each a unique IP and MAC.

Per-motor velocity/accel/decel/homing speed have their own editable table
right below the network block — one row per motor.

## 4. Connect from your PC

Any UDP tool (SocketTest, ncat, your own app) pointed at
`CONTROLLINO_IP:CONTROLLINO_PORT` works. One command per UDP packet, plain
text, no terminator needed. Command replies come straight back to the
sender (the IP and port your tool sent from) **and** to the configured
`PC_IP:PC_PORT`; events go only to `PC_IP:PC_PORT`. On boot, once the drives
are initialised, the controller sends `Motor ready` to `PC_IP:PC_PORT`.

**No replies at all?** Check that `PC_IP` is your PC's actual IP on the
Controllino's subnet, and allow your UDP tool through Windows Firewall
(inbound UDP is blocked on networks marked "Public"). Resetting the
Controllino while listening on `PC_PORT` should show `Motor ready`.

## 5. Command set

One command per UDP packet, plain text, arguments after the colon. Units:
speed **rps**, accel/decel **rps/s**, position **encoder steps** (signed
32-bit). Every motor command follows **one rule**:

```
CMD:<slots>      individual motors   P:1000,,3000   L:,1   P:M17=1000   V:M3
CMDA[:value]     all motors          PA:5000        LA     VA:0.5       BA
```

Slots are positional (field N = motor N, empty = skip, a single value with no
comma = motor 1), or motor names (`M3=500`, `M1,M3`) in any order. `?` reads
instead of writes. The stop type and homing method are the digit in the head
(`B2:1,,1`, `H3A`); the status format goes after a second colon
(`S:1,,1:COMPACT`, `SA:FLAGS`). A bare head without slots is an error, except
`B`, `B1`, `B2`, `B3`, which always stop every motor. The motor count is one
variable (`setMotorCount(n)`, `MC:n`, or `SAVE`d; 1–32) and every command
follows it.

**Motion**

| Command | Action |
|---|---|
| `P:1000,,3000` · `P:M17=1000` | absolute move — M1→1000, skip M2, M3→3000 (synchronised) · motor 17 |
| `PR:-1000,,500` | relative move — delta from each motor's current position |
| `PA:5000` · `PRA:-500` | all motors → 5000 · all motors back 500, broadcast simultaneous start |
| `P:?` · `P:,,?` · `P:M3` · `PA:?` | read position — M1 · M3 · M3 · all motors |

**Speed & ramps** (runtime; `SAVE` to keep, or hardcode in the sketch)

| Command | Action |
|---|---|
| `V:0.5,,0.3` · `VA:1` · `V:?` · `VA:?` | run speed per motor (rps) · all · read M1 · read all |
| `VS:1,,1` · `VSA` | velocity mode (endless rotation) — motors 1 and 3 · all; stop with `B` |
| `ACC:0.2,,0.5` · `ACCA:0.3` · `ACC:?` | acceleration per motor (rps/s) · all · read |
| `DEC:0.2,,0.1` · `DECA:0.1` · `DEC:?` | deceleration per motor (rps/s) · all · read |

**Stopping**

| Command | Action |
|---|---|
| `B` · `BA` · `B:,1` | controlled stop on the decel ramp — all · all · motor 2 |
| `B1` `B2` `B3` (+`A`, +`:1,,1`) | quick-stop on decel ramp · fast brake · immediate — all or selected |
| `L:,1` · `LA` | loosen brakes (releases torque) — motor 2 · all |
| `BS:,1` · `BSA` | brake status — motor 2 · all |

**Zero & homing**

| Command | Action |
|---|---|
| `Z:1,,1` · `ZA` | set current position = 0 (no move) — motors 1 and 3 · all |
| `ZO:10000,,500` · `ZOA:5000` | set current position = value (no move) |
| `ZS:,1` · `ZSA` | zero status |
| `HV` · `HVZ` · `HACC` · `HO` (+`A`, `?`) | homing search speed (rps), zero-find speed (rps), homing ramp (rps/s), offset past the sensor (steps) — each per motor, all, or read |
| `H3:1,,1` · `H3A` | start homing, methods 1–12 (limits CW/CCW, origin switch, fast variants); completion arrives as `EVENT:HOMED` |

**Status & diagnostics**

| Command | Action |
|---|---|
| `SA` · `S:M1:COMPACT` | status word (formats: RAW COMPACT STATE BITS FLAGS DETAIL; flags are mode-aware) |
| `SDA` · `SD:M3` | detailed human-readable status — all · motor 3 |
| `LSA` · `LS:1,,1` | limit switches CW/CCW, origin switch SW, homing state and mode per motor |
| `DGA` · `DG:M1` | low-level register dump ("replies OK but won't move") |

**Controller**

| Command | Action |
|---|---|
| `IA` · `I:1,,1` | detect + initialise — all · motors 1 and 3 |
| `RA` · `R:M3` | alarm reset + re-init — all · motor 3 |
| `MC:?` · `MC:8` | read · set active motor count (1–32) at runtime |
| `SAVE` · `SAVE:?` · `SAVE:CLEAR` · `LOAD` | keep V, ACC, DEC, HV, HVZ, HACC, HO and MC in EEPROM (applied at every boot) |
| `VER` · `PING` · `X` | firmware version · link check (`PONG`) · reboot the controller |

**What comes back.** `OK` for a launched move; a confirmation sentence for
settings and stops; a data line for reads (`M1=10000,M3=200`); multi-line
packets for `I`, `SD` and `DG`; `WARN: …` when a motor did not answer on
RS-485; `ERR:<reason>` when the arguments are wrong (nothing was sent to the
drives); `ERR:unknown command <HEAD>` for an unknown head. When one command
answers with several packets, each ends with `:MORE` and the last with
`:END`. A `P`/`PR` sent while motors are jogging answers `OK` only once the
jog has braked and the move has actually started.

**Events — what the controller sends without being asked.** A background
monitor reads one motor's status every 8 ms and pushes a message to
`PC_IP:PC_PORT` on every *change* (one message when a condition appears, one
when it clears):

| Trigger on the drive | Event | What to do |
|---|---|---|
| limit switch pressed / released | `EVENT:LIMIT:M2:CW POS=48211 ST=0x4677` · `EVENT:LIMIT:M2:CW_CLEARED` (same for `CCW`) | back off with `PR`, or re-home |
| origin switch (SW) covered / uncovered | `EVENT:ORIGIN:M2:ON POS=1012 ST=0x1c37` · `EVENT:ORIGIN:M2:OFF` | informational |
| homing finished / failed | `EVENT:HOMED:M2 POS=0 ST=0x1437` · `EVENT:HOMING_ERROR:M2 ST=0x3437 (…)` | position now valid · check switches, `HV`, `HO`, then `R:M2` and retry |
| drive fault raised / cleared | `EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT` · `EVENT:FAULT_CLEARED:M3` | `SD` for the decoded cause, fix it, `R:M3` |
| drive stops answering / answers again | `EVENT:OFFLINE:M4` · `EVENT:ONLINE:M4 ST=0x1437 (re-initialised)` | after `ONLINE` the drive was re-initialised automatically, but a power-cycled drive is back at position 0 — re-home or `ZO` |

Target reached is **not** an event: poll `S:M1:COMPACT` (`TR`).

**Complete reference** — every command with its arguments, what it triggers
on the bus, every reply and error string, the six `S` formats, status-bit and
error-register tables, step-by-step recipes and the 0.4 → 0.5 migration
table: **[extras/PROTOCOL.md](extras/PROTOCOL.md)**.


## Multiple units

Each Controllino needs its own static IP + MAC (edit the network block per
board). There's no pairing/discovery layer, so double-check the addresses
you set are correct and reachable before relying on replies.

## How settings work

1. **The admin configures the sketch** — the "EDIT ME" blocks in
   `JMCMotors_6Motor` (network, motor count, speeds, ramps, homing) — and
   flashes the Controllino. Those values are re-applied on every boot and
   whenever a drive comes back online, so they survive power failures.
2. **On a deployed unit, settings can be changed over UDP** (`V`, `ACC`,
   `DEC`, `HV`, `HVZ`, `HACC`, `HO`, `MC`). They last until the next reboot
   unless the operator sends **`SAVE`**, which stores them in the
   Controllino's EEPROM — no USB cable, no re-flash. `SAVE:?` shows what is
   stored, `SAVE:CLEAR` forgets it, `LOAD` re-applies it.
3. **The sketch always wins after a re-flash.** The saved block remembers
   which sketch values it was made under and is ignored as soon as the admin
   flashes different ones, so an old `SAVE` can never override a new flash.

## Changes in 0.5.1

Cross-checked against the JMC Modbus-RTU manual V2.2:

- The drive's halt code (0x605D) is now set to 1 at every init, so the
  controlled stop `B` is defined as "halt on the current deceleration" rather
  than relying on the drive's undocumented default.
- Entering velocity mode now follows the manual's routine (halt, select mode,
  write speed, release), so a jog cannot start at the previous profile speed
  before `VS` has written the jog speed, and all motors really start on the
  broadcast release.
- A position move waits (up to 100 ms) for status bit 12, the manual's "can
  set new position" condition, before pulsing bit 4.
- `HV` / `HVZ` are capped at 20 rps, the drive's 1200 rpm homing ceiling.

## Changes in 0.5.0

- **One grammar for every command**: `CMD:<slots>` for individual motors,
  `CMDA` for all motors (`PA`, `VA`, `BA`, `LA`, `ZA`, `RA`, `IA`, `SA`, `SDA`,
  `LSA`, `DGA`, `VSA`, `H3A`, …). Bare selector commands are refused, except
  the stop commands `B`…`B3`, which always stop every motor. The `all` keyword
  is gone.
- **Motor names** `M3=500` / `M1,M3` work in every slot command, so the PC
  never counts commas: `P:M17=1000`, `L:M2`, `V:M3` (read).
- **`ACC` / `DEC`** replace `AD`: acceleration and deceleration are separate
  settings that behave exactly like `V` (per motor, `A` for all, `?` to read).
- **`HVZ`** (zero-find speed) and **`HACC`** (homing ramp) added; `HV:?` added.
- **`SAVE` / `LOAD`**: V, ACC, DEC, HV, HVZ, HACC, HO and the motor count can be
  stored in EEPROM and are applied at every boot; `SAVE:CLEAR` returns to the
  sketch values.
- `S` and `DG` use slots and names like every other command (`S:M2:COMPACT`,
  `DG:M1`, `SA`, `DGA`); the `S` format goes after a second colon.
- `V:?`, `HO:?` and friends read per slot (`V:?` = motor 1, `VA:?` = all);
  set/read replies are labelled `M1=…`.
- `VER`, `PING`, lower-case heads, `ERR:unknown command <HEAD>`, and
  `:MORE` / `:END` markers on multi-packet replies.


## Changes in 0.4.0

- Every command now has a per-motor and an all-motors form: `VS:1,,1`,
  `I:1,,1`, `BS`/`ZS` selections, and the new all-motors twins `PRA`, `ADA`,
  `ZOA`, `HOA`, `HVA`.
- **`AD` is now per motor** (`AD:0.2,,0.5/0.2`); the old "all motors" form is
  `ADA:0.2` / `ADA:0.3/0.1`.
- New `LS` command: CW/CCW limits, origin switch, homing state and mode per
  motor on one line.
- New events `EVENT:ORIGIN`, `EVENT:HOMED`, `EVENT:HOMING_ERROR`.
- Status bits 10/12/13 are decoded according to the drive's mode, so `HOMED`
  really means homing complete; `SD` reports `Mode:` and a mode-aware
  `Homing:` line.
- Long per-motor replies are split into packets of about 300 characters, and
  string constants moved to flash: RAM use dropped from 67 % to 38 % on the
  Controllino MAXI, which is what makes 32 motors workable.
- Example sketch: a settings table with fewer rows than motors falls back to
  the global values instead of giving those motors zero speed.

## Going deeper

**Interactive reference:** <https://ailagari.github.io/JMCMotors/> — every
command with its replies, a command builder, a firmware simulator you can
type into, a status-word decoder, events, recipes and the 0.4 → 0.5
migration table. (Source: [docs/index.html](docs/index.html), a single
self-contained page.)

| Document | Contents |
|---|---|
| [extras/PROTOCOL.md](extras/PROTOCOL.md) | command grammar and the six command shapes, every command with triggers and replies, events, trigger/response matrix, status bits, error catalogue, recipes |
| [extras/HARDWARE.md](extras/HARDWARE.md) | parts list, wiring, drive parameters (P17/P19/P40–P43…), slave IDs, sketch configuration, first power-up, per-axis commissioning, 32-motor scaling, position limits, troubleshooting |
| [src/JMCMotors.h](src/JMCMotors.h) | the three layers (bus / motor / controller) and the direct C++ API for advanced sketches |

## Author / license / status

**Author:** LAGARI A — lagariscience@gmail.com · **License:** MIT ·
**Repo:** https://github.com/ailagari/JMCMotors

**Status:** stable, MIT-licensed. Verified against the JMC Modbus-RTU manual
V2.2 and the IHSS60-R/RC datasheet, and bench-tested on real hardware. Issues
and pull requests welcome.
