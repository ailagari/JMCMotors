# JMCMotors

**Control up to 32 integrated servo/stepper drives from a Controllino with
plain UDP text commands.**

Works with JMC integrated servo/stepper drives (IHSS / IHT "-RC" series) over
the Controllino's built-in RS-485, verified against the JMC Modbus-RTU manual
V2.2 and the IHSS60-R/RC datasheet, and bench-tested on real hardware.

---

## 1. Install

**Arduino IDE:** Sketch → Include Library → Add .ZIP Library… → select the
downloaded ZIP of this repository. (Or copy the folder into
`Documents/Arduino/libraries/`.) Requires the **CONTROLLINO** board package +
library and the built-in **Ethernet** library.

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

**2.4 Commission each axis** with the commands in section 5: `I` (all
detected?) → `PR:4000` (one turn, right direction?) → `V:1` + `VS` for 60 s
(exactly 60 revs?) → `AD` until the start/stop is smooth → `Z` or `H<n>` to
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

One command per UDP packet, plain text, **upper-case head**, arguments after
the colon. Units: speed **rps**, accel/decel **rps/s**, position **encoder
steps** (signed 32-bit).

**How commands are shaped.** Every command is one of six shapes; once you
know the shape you know how its arguments are read.

| Shape | Arguments | Commands |
|---|---|---|
| Bare | none — acts on all motors or the controller | `I` `X` `VS` `BS` `ZS` `SD` `B` `L` `Z` `R` |
| Positional **values** | field N = a value for motor N; empty field = skip; a single value with no comma = motor 1 | `P` `PR` `ZO` `HO` `V` `HV` |
| Positional **selectors** | field N = a flag for motor N (any text selects, empty skips); empty or `all` = every motor | `B` `B1` `B2` `B3` `L` `Z` `R` `H1`…`H12` |
| Whole-bus value | one value for all motors | `PA` `VA` `AD` `MC` |
| Query | `?` in a field reads instead of writes | `P:?` `PA:?` `SD:?` `V:?` `HO:?` `AD:?` `MC:?` |
| Motor number | the argument is a motor **number** | `S:<n>` `SD:<n>` `DG:<n>` |

So `P:1000,,3000` moves motors 1 and 3, `L:1,,1` loosens motors 1 and 3, and
`L:,1` (not `L:2`) loosens motor 2.

**What comes back.** `OK` for a launched move; a confirmation sentence for
settings and stops (`Fast brake applied to motor 1,3`); a data line for
queries (`M1=10000,M3=200`); multi-line packets for `I`, `SD` and `DG`;
`WARN: …` when a motor did not answer on RS-485; `ERR:<reason>` when the
arguments are wrong (nothing was sent to the drives); `INVALID COMMAND` for
an unknown head. A `P`/`PR`/`PA` sent while motors are jogging (`VS`) answers
`OK` only once the jog has braked and the move has actually started.

**Motion**

| Command | Action |
|---|---|
| `P:1000,,3000` | absolute move — M1→1000, skip M2, M3→3000 (synchronised) |
| `PR:-1000,,500` | relative move — delta from each motor's current position |
| `PA:5000` | all motors → 5000, broadcast simultaneous start |
| `P:?` · `P:,,?` · `PA:?` | read position — M1 · M3 · all motors |

**Speed & ramps** (runtime; lost on power-off — hardcode in the sketch to keep)

| Command | Action |
|---|---|
| `V:0.5,,0.3` | run speed per motor (rps) · `V:?` reads stored speeds |
| `VA:1` | set every motor's speed at once (broadcast) |
| `VS` | start velocity mode (endless rotation); stop with `B` |
| `AD:0.2` · `AD:0.3,0.1` | accel=decel · accel,decel (rps/s), all motors · `AD:?` reads |

**Stopping**

| Command | Action |
|---|---|
| `B` (or `B0`) | controlled stop on the decel ramp, all motors |
| `B1:` · `B2:` · `B3:` | decel quick-stop · fast brake · immediate, all motors |
| `B2:1,,1` | same types, selected motors — here fast-brake M1 and M3 |
| `L` · `L:1,,1` | loosen brakes (releases torque) — all · motors 1 and 3 |
| `BS` | brake status per motor |

**Zero & homing**

| Command | Action |
|---|---|
| `Z` · `Z:1,,1` | set current position = 0 (no move) — all · motors 1 and 3 |
| `ZO:10000,,500` | set current position = value (no move) — M1=10000, M3=500 |
| `ZS` | zero status |
| `HV:0.3` · `HV:0.3,0.5` | homing speed (rps) — all · per motor |
| `HO:1000,,-800` | steps to travel past the origin sensor (sign = direction) · `HO:?` reads |
| `H3` · `H3:1,,1` | start homing, method 1–12 — all · motors 1 and 3 |

**Status & diagnostics**

| Command | Action |
|---|---|
| `S:0` · `S:1,COMPACT` | status word (formats: RAW COMPACT STATE BITS FLAGS DETAIL) |
| `SD` · `SD:,,?` | detailed human-readable status — all · motor 3 |
| `DG:1` | low-level register dump for one motor ("replies OK but won't move") |

**Administration**

| Command | Action |
|---|---|
| `I` | detect + initialise all motors |
| `R` · `R:1,,1` | alarm reset + re-init — all · motors 1 and 3 |
| `MC:?` · `MC:8` | read · set active motor count (1–32) at runtime |
| `X` | reboot the controller |

On boot the controller sends `Motor ready`. Replies go to the sender and to
`PC_IP:PC_PORT`; events and `Motor ready` go to `PC_IP:PC_PORT` only.

**Events — what the controller sends without being asked.** A background
monitor reads one motor's status every 8 ms and pushes a message to
`PC_IP:PC_PORT` on every *change* (one message when a condition appears, one
when it clears):

| Trigger on the drive | Event | What to do |
|---|---|---|
| limit switch pressed / released | `EVENT:LIMIT:M2:CW POS=48211 ST=0x4677` · `EVENT:LIMIT:M2:CW_CLEARED` (same for `CCW`) | back off with `PR`, or re-home |
| drive fault raised / cleared | `EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT` · `EVENT:FAULT_CLEARED:M3` | `SD` for the decoded cause, fix it, `R:…` |
| drive stops answering / answers again | `EVENT:OFFLINE:M4` · `EVENT:ONLINE:M4 ST=0x1437 (re-initialised)` | after `ONLINE` the drive was re-initialised automatically, but a power-cycled drive is back at position 0 — re-home or `ZO` |

Target reached and homing complete are **not** events: poll `S:n,COMPACT`
(`TR`) or `S:n,FLAGS` (`HOMED`).

**Complete reference** — every command with its arguments, what it triggers
on the bus, every reply and error string, the six `S` formats, status-bit and
error-register tables, and step-by-step recipes:
**[extras/PROTOCOL.md](extras/PROTOCOL.md)**.

## Multiple units

Each Controllino needs its own static IP + MAC (edit the network block per
board). There's no pairing/discovery layer, so double-check the addresses
you set are correct and reachable before relying on replies.

Settings sent over UDP (`V`, `HV`, `HO`, `MC`, `AD`) are lost on power
failure; hardcode keepers in the sketch (the per-motor table, or
`setMotorVelocity(...)` etc. directly) — they re-apply on every boot and
whenever a drive comes back online.

## Going deeper

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
