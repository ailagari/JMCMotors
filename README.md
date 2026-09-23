# LagariMotors

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

## 2. Set up the drives (once, on each drive's panel)

| Parameter | Value | Why |
|---|---|---|
| **P43** | **0** | RS-485/Modbus mode — **factory default is CANopen and the drive won't answer!** |
| **P40** | 1, 2, 3… | unique slave ID per drive |
| **P41** | **7** | 115200 baud |

Wire RS-485 A/B as a daisy-chain to all drives (120 Ω termination at the
last one), Ethernet to the same network as your PC. Full drive-parameter
guide (smoothness, steps/rev, sensor polarity…): **[extras/HARDWARE.md](extras/HARDWARE.md)**.

## 3. Flash the example

Open **File → Examples → LagariMotors → LagariMotors_6Motor**. Networking is
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
text, no terminator needed. Replies and events come back to whatever
`PC_IP:PC_PORT` you configured in the sketch. On boot, once the drives are
initialised, the controller sends `Motor ready` to that address.

## 5. Command set

One command per UDP packet, plain text; arguments follow the colon. **Every
motor-addressing command uses the same positional slots** (like `P`): field N =
motor N, an empty field skips that motor, an empty argument (or `all`) = every
motor — so `1,,1` = motors 1 and 3. For `P`/`PR`/`V`/`ZO`/`HO` the field is a
per-motor **value**; for `B`/`H`/`L`/`Z`/`R` it is just a **selector** (so
`L:,1` = motor 2, not `L:2`). Units: speed **rps**, accel/decel **rps/s**,
position **encoder steps** (signed 32-bit).

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

On boot the controller sends `Motor ready`; all replies and events go to the
configured `PC_IP:PC_PORT`.

The controller also **pushes events without being asked** — limit switches,
drive faults (decoded), drives going offline/online:

```
EVENT:LIMIT:M2:CW POS=48211 ST=0x4677
EVENT:FAULT:M3 ST=0x678 ERR=0x2 CURRENT
EVENT:ONLINE:M4 / EVENT:OFFLINE:M4
```

**Complete reference** — every command's exact syntax, all reply strings,
formats, status-bit tables, recipes: **[extras/PROTOCOL.md](extras/PROTOCOL.md)**.

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
| [extras/PROTOCOL.md](extras/PROTOCOL.md) | full command/response reference, events, status bits, recipes |
| [extras/HARDWARE.md](extras/HARDWARE.md) | drive parameters (P17/P19/P42…), wiring, speed-scale verification, 32-motor scaling, position limits |
| [src/LagariMotors.h](src/LagariMotors.h) | the three layers (bus / motor / controller) and the direct C++ API for advanced sketches |

## Author / license / status

**Author:** LAGARI A — lagariscience@gmail.com · **License:** MIT ·
**Repo:** https://github.com/ailagari/LagariMotors

**Status:** stable, MIT-licensed. Verified against the JMC Modbus-RTU manual
V2.2 and the IHSS60-R/RC datasheet, and bench-tested on real hardware. Issues
and pull requests welcome.
