# JMCMotors

**Control up to 32 JMC integrated servo/stepper drives from a CONTROLLINO,
an ESP32 or an Arduino Uno with plain text commands.**

Works with JMC integrated servo/stepper drives (IHSS / IHT "-RC" series) over
RS-485 (Modbus-RTU), cross-checked against the JMC Modbus-RTU manual V2.2 and
the IHSS60-R/RC datasheet ([how it is verified](extras/VERIFICATION.md)).

| Board (`BOARD` in the sketch) | PC link (`PC_LINK`) | Drives (RS-485) | Motors |
|---|---|---|---|
| **CONTROLLINO MAXI / MEGA** (`BOARD_CONTROLLINO`) | Ethernet UDP, or USB | built-in RS-485 — nothing to wire to the board | 1–32 |
| **ESP32**, any Arduino-ESP32 2.x / 3.x board (`BOARD_ESP32`) | WiFi UDP, or USB | TTL-to-RS-485 module, pins set in the sketch | 1–32 |
| **Arduino Uno / Nano** (`BOARD_UNO`) | USB (or RS-232 COM port) | TTL-to-RS-485 module, pins set in the sketch | 1–6 |
| Arduino Mega 2560 (`BOARD_MEGA`) | Ethernet shield, or USB | TTL-to-RS-485 module on Serial1/2/3 | 1–32 |

One example, **`JMCMotors_Controller`**, runs on all of them: you pick the board
at the top of the sketch and it compiles what that board needs.

> **RS-485, not RS-232.** The JMC "-RC" drives have an RS-485 port only. Between
> an Uno or ESP32 and the drives you need a **TTL-to-RS-485** module (MAX485,
> MAX3485 / SP3485 for 3.3 V), not a TTL-to-RS-232 (MAX3232) one. A MAX3232
> module is fine on the *PC* side of an Uno, to reach a PC's COM port.

The same commands, replies and events work on every board; the Uno leaves out
a few extras to fit its 32 KB of flash (see [Board differences](#board-differences)).

---

## 1. Install

**Arduino IDE Library Manager:** Tools → Manage Libraries… → search
**JMCMotors** → Install (the CONTROLLINO and Ethernet libraries come along as
dependencies; they are only used on boards that need them). Install the board
package for your hardware via Boards Manager (**CONTROLLINO_Boards**,
**esp32** by Espressif, or the built-in Arduino AVR boards).

**From this repository:** Sketch → Include Library → Add .ZIP Library… →
select the downloaded ZIP, or copy the folder into
`Documents/Arduino/libraries/`.

**Examples** (File → Examples → JMCMotors):

| Example | What it is |
|---|---|
| `JMCMotors_Controller` | the deployable controller for **every board**: select the board, set the RS-485 module pins (ESP32 / Uno / Mega), network, motors, per-motor table, soft limits, watchdogs |
| `JMCMotors_Minimal` | the CONTROLLINO command system in a dozen lines, library defaults |
| `JMCMotors_DirectAPI` | moving a motor from C++ (`JMCBus` + `JMCMotor`), no command layer, any board |

## 2. Set up the motors

Four steps per installation. The full version, with wiring diagrams for every
board, commissioning sequence and a troubleshooting table, is
**[extras/HARDWARE.md](extras/HARDWARE.md)**.

**2.1 Wire it**

- Motor power to each drive from the motor supply; the controller board on its
  own logic supply, with a **common ground**.
- RS-485 **A/B daisy-chain** from the board (CONTROLLINO: built-in port;
  Uno/ESP32/Mega: the TTL-to-RS-485 module) through every drive, **120 Ω
  termination across A/B at the last drive**.

  | Module pin | Uno | ESP32 | Mega 2560 |
  |---|---|---|---|
  | RO (receiver out) | 8 | GPIO 16 (RX2) | 19 (RX1) |
  | DI (driver in) | 9 | GPIO 17 (TX2) | 18 (TX1) |
  | DE + RE (tied) | 7 | GPIO 4 | 7 |

  Auto-direction modules (no DE/RE pins) work too: pass `-1` as the DE pin.
  On an ESP32 use a 3.3 V module, or level-shift RO.
- Limit / origin switches go to the **drive's** inputs, not the controller.
- Network boards: static IP, PC on the same subnet.

**2.2 Set the drive parameters** (front panel, once per drive; restart the
drive afterwards)

| Parameter | Value | Why |
|---|---|---|
| **P43** | **0** | RS-485/Modbus mode — **factory default is 1 = CANopen and the drive won't answer!** |
| **P40** | 1, 2, 3… | unique slave ID = the motor number in every command, consecutive from 1 |
| **P41** | **7** = 115200 baud (CONTROLLINO, ESP32, Mega) · **38400** on an Uno (see the manual for its code) | must match the sketch |
| **P42** | 0 / 1 | sensor polarity: 0 = PNP, 1 = NPN (default) — wrong polarity shows as a permanently active limit |
| **P17** | e.g. 10 (= 4000) | steps per revolution; positions in commands are raw steps, so `P:4000` is one turn only when P17 = 4000 |
| **P19** | 7 for geared loads | speed smoothness (S-curve); default 2, sometimes found at 0 |

**2.3 Configure the sketch** — the "EDIT ME" blocks: network or pins, motor
count, speeds and ramps, soft limits, watchdogs. Sketch values are re-applied
on every boot and whenever a drive comes back online.

**2.4 Commission each axis** with the commands in section 5: `IA` (all
detected?) → `PR:4000` (one turn, right direction?) → `V:1` + `VS:M1` for 60 s
(exactly 60 revs?) → `ACC`/`DEC` until the start/stop is smooth → `Z` or `H<n>` to
set the zero → press each limit switch and watch for `EVENT:LIMIT` → set `SL`
soft limits inside the travel.

## 3. Flash the example

Open **File → Examples → JMCMotors → JMCMotors_Controller** and work through
the five **EDIT ME** steps at the top.

**Step 1 — select the board** (and pick the same one in Tools → Board):

```cpp
#define BOARD    BOARD_ESP32       // BOARD_CONTROLLINO, BOARD_ESP32, BOARD_UNO or BOARD_MEGA
#define PC_LINK  LINK_NETWORK      // LINK_NETWORK (Ethernet / WiFi) or LINK_USB
```

**Step 2 — the RS-485 module pins**, only for the ESP32, Uno and Mega (the
CONTROLLINO's RS-485 is built in). Each board has its own block; edit yours:

```cpp
#define ESP32_RS485_RX     16      // GPIO wired to the module's RO
#define ESP32_RS485_TX     17      // GPIO wired to the module's DI
#define ESP32_RS485_DE     4       // GPIO wired to DE + RE, or -1 (auto-direction module)
#define ESP32_DRIVE_BAUD   115200  // must match drive parameter P41

#define UNO_RS485_RX       8       // SoftwareSerial: the Uno's hardware port is the USB link
#define UNO_RS485_TX       9
#define UNO_RS485_DE       7
#define UNO_DRIVE_BAUD     38400

#define MEGA_RS485_PORT    Serial1 // Serial1 = 19/18, Serial2 = 17/16, Serial3 = 15/14
#define MEGA_RS485_DE      7
```

**Step 3 — network** (board IP and port, PC IP and port, MAC for Ethernet,
SSID and password for WiFi, `ONLY_ACCEPT_PC`), **step 4 — motors** (count,
global speeds and ramps, per-motor table with homing and soft limits),
**step 5 — safety** (command and processor watchdogs).

If step 1 and Tools → Board disagree, the sketch stops with a message such as
`STEP 1 says BOARD_ESP32, but Tools > Board is not an ESP32 board`, before
anything is uploaded. Networking is fixed/static — no pairing, no
auto-discovery. Flashing more boards? Give each a unique IP and MAC.

Behind the steps the sketch builds three objects — the PC link, the drive
bus and the controller. Writing your own sketch, that is all it takes:

```cpp
EthernetUDP   udp;    JMCLinkUDP    pcLink(udp);       // or WiFiUDP, or JMCLinkSerial pcLink(Serial)
JMCBus        bus(Serial3);                           // the RS-485 serial port
JMCController jmc(bus, pcLink);
// setup(): start the network / serial ports, bus.begin(baud, dePin), jmc.begin()
// loop():  jmc.run();
```

## 4. Connect from your PC

**UDP boards** (CONTROLLINO, ESP32, Mega + shield): any UDP tool (SocketTest,
ncat, your own app) pointed at the board's IP and port. One command per UDP
packet, plain text, no terminator needed. Replies go back to the sender
**and** to `PC_IP:PC_PORT`; events go only to `PC_IP:PC_PORT`. On boot the
board sends `Motor ready` to `PC_IP:PC_PORT`.

**Uno:** open the Serial Monitor (or your program) at **57600** baud. One
command per line (CR, LF or CR LF); every reply and event ends with CR LF.

**No replies at all?** Check that `PC_IP` is your PC's actual IP on the board's
subnet, and allow your UDP tool through Windows Firewall (inbound UDP is
blocked on networks marked "Public"). Resetting the board while listening on
`PC_PORT` should show `Motor ready`.

**Resolume, MadMapper, lighting consoles (DMX):** the
**[JMC DMX Bridge](extras/dmx-bridge/README.md)** runs on the same PC, listens
for Art-Net and sACN, and turns each motor's DMX value (16-bit or 8-bit) into
live position commands, with min/max mapping per motor, a STOP button and the
watchdog as a safety net. The guide there walks through Resolume Arena step by
step.

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
| `P:1000,,3000` · `P:M17=1000` | absolute move — M1→1000, skip M2, M3→3000, all started together, each at its own `V` · motor 17 |
| `PR:-1000,,500` | relative move — delta from each motor's current position |
| `PA:5000` · `PRA:-500` | all motors → 5000 · all motors back 500, one broadcast start |
| `PS:1000,,3000` · `PRS:…` · `PSA:…` | **synchronised arrival** — speeds and ramps scaled per axis so every motor starts *and arrives* together (straight line), never above any axis's own `V`/`ACC`/`DEC` |
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
| `SAVE` · `SAVE:?` · `SAVE:CLEAR` · `LOAD` | keep V, ACC, DEC, HV, HVZ, HACC, HO, SL, WD and MC in EEPROM (applied at every boot) |
| `VER` · `PING` · `X` | firmware version · link check (`PONG`) · reboot the controller |

**Safety**

| Command | Action |
|---|---|
| `SL:-1000/50000,,0/8000` · `SL:M2=OFF` · `SLA:0/100000` · `SL:?` | software position limits per motor (steps): `P`/`PR`/`PS` beyond them are refused, a jog (`VS`) stops at them with `EVENT:SOFTLIMIT` |
| `WD:500` · `WD:0` · `WD:?` | communication watchdog: no command for 500 ms → every motor stops, `EVENT:WATCHDOG:STOP`. The PC sends `PING` (or anything) more often |
| `#17 P:1000` | optional tag: any command may start with `#<number> `; every reply to it starts with `#17 `, so the PC can match replies to requests (not on Uno) |

**What comes back.** `OK` for a launched move; a confirmation sentence for
settings and stops; a data line for reads (`M1=10000,M3=200`); multi-line
packets for `I`, `SD` and `DG`; `WARN: …` when a motor did not answer on
RS-485; `ERR:<reason>` when the arguments are wrong (nothing was sent to the
drives); `ERR:unknown command <HEAD>` for an unknown head. When one command
answers with several packets, each ends with `:MORE` and the last with
`:END`. A `P`/`PR` sent while motors are jogging answers `OK` only once the
jog has braked and the move has actually started. Numbers are checked
strictly: `P:12abc` or `V:fast` answer `ERR:P bad value '12abc' (M1)`
instead of moving to a partial number; a command line longer than the
buffer answers `ERR:command too long` and is not run.

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
| jog reaches a soft limit (`SL`) | `EVENT:SOFTLIMIT:M1 POS=50012 LIMIT=50000` | the motor was stopped |
| no command for the `WD` time | `EVENT:WATCHDOG:STOP no command for 500 ms - all motors stopped` | check the PC / network, then resume |
| drive stops answering / answers again | `EVENT:OFFLINE:M4` · `EVENT:ONLINE:M4 ST=0x1437 (re-initialised)` | after `ONLINE` the drive was re-initialised automatically, but a power-cycled drive is back at position 0 — re-home or `ZO` |

Target reached is **not** an event: poll `S:M1:COMPACT` (`TR`).

**Complete reference** — every command with its arguments, what it triggers
on the bus, every reply and error string, the six `S` formats, status-bit and
error-register tables, step-by-step recipes and the 0.4 → 0.5 migration
table: **[extras/PROTOCOL.md](extras/PROTOCOL.md)**.


## Board differences

Everything above works on every board, with these exceptions on the
**Uno / Nano** (32 KB flash, 2 KB RAM — the Uno example uses 98 % of the flash).
Every motion, homing, limit, soft-limit, watchdog and failure check is the
same; what is left out is convenience:

| Left out on the Uno | Use instead |
|---|---|
| `PS` / `PRS` synchronised arrival | `P` / `PA` (all motors still start together) |
| `SAVE` / `LOAD` | change the sketch and re-flash (the Uno is on a USB cable anyway) |
| `SD`, `DG`, `S` formats `STATE`/`BITS`/`FLAGS`/`DETAIL` | `S:..:COMPACT`, `S` (raw word), `LS` |
| `BS`, `ZS` | `S:..:COMPACT`, `P:?` |
| `MC:n` (count at runtime; `MC:?` works) | `setMotorCount()` in the sketch |
| `X` (soft restart) | the reset button |
| `HVZ`, `HACC` as separate settings | they follow `HV` and `ACC` (the default anyway) |
| automatic sign correction in `ZO` | the read-back check stays: a drive that disagrees is reported |
| `#<n>` reply tags | one command at a time: replies come back in command order |
| explanations after errors (`… - nothing started`, example hints), the `WD` / `I` hint sentences, the explanation after `EVENT:WATCHDOG:STOP`, fault names in `EVENT:FAULT` (the raw `ERR=0x..` code stays) | the `ERR:<CMD> …` start is the same; codes are listed in PROTOCOL.md |
| motors 7–32 | at most 6 motors |

Those commands answer `ERR:<CMD> not available on this board`.

**Nano:** the Arduino IDE limits every Nano entry to 30 KB, and the controller
needs 31 KB. A Nano with the current bootloader runs the same Optiboot as an
Uno, so upload with **Tools → Board → "Arduino Uno"** (32 KB). A clone with the
old bootloader (uploads only as "ATmega328P (Old Bootloader)") needs the new
one first: Tools → Burn Bootloader with an ISP programmer — or use an Uno.

Build-flag users (PlatformIO) can switch each part with `-DJMC_FEATURE_SYNC=0/1`,
`JMC_FEATURE_SAVE`, `JMC_FEATURE_EXTRA_STATUS`, `JMC_FEATURE_EXTRAS`, `JMC_FEATURE_HINTS`,
`JMC_FULL_REPORTS`, `JMC_MAX_MOTORS` (see [src/JMCConfig.h](src/JMCConfig.h)).

## Safety and reliability

- **Communication watchdog** (`WD`, `setCommandWatchdog(ms)`): every motor
  stops when the PC goes quiet. On by default in the ESP32 example (WiFi can
  drop); recommended wherever a PC drives motion.
- **Processor watchdog** (`enableHardwareWatchdog(4000)`): the board reboots
  if the firmware ever hangs; long operations feed it. Test it once on your
  board first — some old Mega 2560 bootloaders reboot-loop after a watchdog
  reset.
- **Soft limits** (`SL`, `setMotorSoftLimits(id, min, max)`): moves outside
  are refused before anything is sent; jogs stop at the limit.
- **Sender filter** (`pcLink.allowSender(ip)`): UDP commands from any other
  address are dropped.
- **No heap, no `String`**: fixed buffers only, sized per board, so memory use
  is what the IDE reports at compile time and cannot fragment over weeks.
- **Strict parsing**: malformed numbers and over-long lines are refused, never
  truncated into a different command.
- **`OK` means every drive accepted it**: if a drive misses or rejects a frame
  while a command is sent, the reply says which motor and why
  (`ERR:P M3 not accepted (TIMEOUT) - nothing started`), and a coordinated
  move is aborted before any motor starts.
- **Stops**: a stop of all motors is one broadcast frame first (every drive at
  the same instant, nothing can delay it), then confirmed per drive.
- Every bus reply is CRC-checked, with timeouts, retries and exception
  decoding; a drive that stops answering is reported (`EVENT:OFFLINE`) and
  re-initialised when it returns.
- Limit and origin switches are wired to the drives and act there, without
  the controller. Per the manual the **CW limit is the negative end** and CCW
  the positive end: if negative moves do nothing, `LS` showing `CW=1` with
  nothing pressed points at the sensor polarity (P42) or wiring. For people-safety, wire a hardware emergency stop that cuts
  motor power; software stops are not a substitute.

## Multiple units

Each Controllino needs its own static IP + MAC (edit the network block per
board). There's no pairing/discovery layer, so double-check the addresses
you set are correct and reachable before relying on replies.

## How settings work

1. **The admin configures the sketch** — the "EDIT ME" blocks in the
   example (network, motor count, speeds, ramps, homing, soft limits,
   watchdogs) — and flashes the board. Those values are re-applied on every boot and
   whenever a drive comes back online, so they survive power failures.
2. **On a deployed unit, settings can be changed over the link** (`V`, `ACC`,
   `DEC`, `HV`, `HVZ`, `HACC`, `HO`, `SL`, `WD`, `MC`). They last until the next
   reboot unless the operator sends **`SAVE`**, which stores them in EEPROM
   (on an ESP32: in its flash) — no USB cable, no re-flash. (Not on the Uno.) `SAVE:?` shows what is
   stored, `SAVE:CLEAR` forgets it, `LOAD` re-applies it.
3. **The sketch always wins after a re-flash.** The saved block remembers
   which sketch values it was made under and is ignored as soon as the admin
   flashes different ones, so an old `SAVE` can never override a new flash.

## Changes in 0.6.0

**Boards:** CONTROLLINO, **ESP32** and **Arduino Uno/Nano** (plus Mega 2560
with an Ethernet shield). The PC link is now a separate object: UDP over
Ethernet or WiFi (`JMCLinkUDP`), or a serial line (`JMCLinkSerial`). The
RS-485 bus takes any serial port, with a DE/RE pin or an auto-direction
module.

**New commands and features**

- `PS` / `PRS` / `PSA` / `PRSA`: synchronised-arrival moves.
- `SL` soft position limits, `WD` communication watchdog,
  `enableHardwareWatchdog()`, UDP sender allow-list, `#<n>` reply tags.
- `EVENT:SOFTLIMIT`, `EVENT:WATCHDOG`.
- Strict number parsing; `ERR:command too long`.
- No heap use and no `String` anywhere (motors live in fixed storage).
- Streaming: registers that already hold the right value are not written
  again, so a position update with an unchanged speed is 3 frames per drive
  (all motors: one target frame each + one broadcast start).
- [JMC DMX Bridge](extras/dmx-bridge/README.md): Art-Net / sACN from
  Resolume, MadMapper or a console to live motor positions.

**Behaviour changes**

- `PA` / `PRA` now move each motor at **its own** `V` (0.5 used one speed
  for all). Send `VA` first for one common speed.
- `PR` is sent as an absolute target (actual position + delta) instead of the
  drive's relative bit, which adds to the drive's internal previous target —
  after `Z`, `ZO` or homing that could travel the wrong distance. Soft limits
  on `PR` are now exact.
- Homing follows the manual's limits: `H1`–`H4` refuse `HV`/`HVZ` above 5 rps
  (300 r/min), `H1`–`H8` refuse a negative `HO`.
- The 0.5.1 wait for status bit 12 before each move is removed again: the
  manual's timing diagram marks bit 12 as the set-point acknowledge, which
  made every move wait its full 100 ms per motor. The start sequence is the
  field-tested one from 0.1–0.5.0 (prepare, bit 4 rising, bit 4 low).
- `X` also restarts an ESP32.

**Migrating a 0.5 sketch** (the commands themselves are unchanged):

| 0.5 | 0.6 |
|---|---|
| `JMCController jmc;` | `EthernetUDP udp; JMCLinkUDP pcLink(udp); JMCBus bus(Serial3); JMCController jmc(bus, pcLink);` |
| `jmc.setNetwork(mac, ip, port, pcIp, pcPort)` · `setStationId(n)` | `Ethernet.begin(mac, ip); pcLink.begin(port, pcIp, pcPort);` in `setup()` |
| bus started inside `jmc.begin()` | `bus.beginControllino(115200);` before `jmc.begin()` |

Or start from the new `JMCMotors_Controller` example (set `BOARD_CONTROLLINO`) and copy your values in.

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
  set new position" condition, before pulsing bit 4 *(removed again in 0.6.0)*.
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

## Tests

How everything is verified — compile matrix, host tests, the cross-check
against the JMC manual and a bench checklist — is in
**[extras/VERIFICATION.md](extras/VERIFICATION.md)**.

`extras/tests/` builds the library on a PC against simulated JMC drives that
answer real Modbus-RTU frames (CRC-checked), and runs 956 checks in four
programs. The **frame check** compares every command's register writes with
the manual's routines (~4000 frames). The **motion check** drives a simulated
axis with CW / CCW / origin switches: forward and reverse moves and jogs, one
and several motors, all 12 homing methods, limits, status formats, resets.
The command suite covers the full profile and the Uno profile: exact bus
frames compared byte-for-byte with the examples in the JMC manual (power-up,
modes, `0x607A` in both word orders, speeds, ramps, homing parameters),
negative moves and jogs, strict parsing, soft limits, watchdog, synchronised
arrival, homing, faults, offline/online, `SAVE`/`LOAD`, multi-packet replies
the serial link, drives that go silent or reject frames mid-command, and a
`millis()` rollover (49.7 days). Needs any C++ compiler (`g++`, `clang++`, or
`pip install ziglang`):

```bash
bash extras/tests/run_tests.sh
```

The DMX bridge has its own test (Python, no hardware): `python extras/dmx-bridge/test_bridge.py`.

## Going deeper

**Interactive reference:** <https://ailagari.github.io/JMCMotors/> — every
command with its replies, a command builder, a firmware simulator you can
type into, a status-word decoder, events, recipes and migration tables. (Source: [docs/index.html](docs/index.html), a single
self-contained page.)

| Document | Contents |
|---|---|
| [extras/VERIFICATION.md](extras/VERIFICATION.md) | how the library is verified: every requirement of the JMC manual mapped to the code and a test, where the manual is unclear and what was chosen, and a bench checklist for new installations |
| [extras/PROTOCOL.md](extras/PROTOCOL.md) | command grammar and the six command shapes, every command with triggers and replies, events, trigger/response matrix, status bits, error catalogue, recipes |
| [extras/HARDWARE.md](extras/HARDWARE.md) | parts list, wiring for each board, drive parameters (P17/P19/P40–P43…), slave IDs, sketch configuration, first power-up, per-axis commissioning, 32-motor scaling, position limits, troubleshooting |
| [extras/dmx-bridge/README.md](extras/dmx-bridge/README.md) | the DMX bridge: Art-Net / sACN to live motor positions; step-by-step Resolume Arena and MadMapper setup |
| [src/JMCMotors.h](src/JMCMotors.h) | the three layers (bus / motor / controller) and the direct C++ API for advanced sketches |

## Author / license / status

**Author:** LAGARI A — lagariscience@gmail.com · **License:** MIT ·
**Repo:** https://github.com/ailagari/JMCMotors

**Status:** 0.6.0 compiles for CONTROLLINO MAXI / MEGA, Uno (Nano as "Uno"),
Mega 2560, ESP32, ESP32-S3 and ESP32-C3, and passes 956 host-test checks
against simulated drives; cross-checked three ways against the JMC Modbus-RTU
manual V2.2 ([extras/VERIFICATION.md](extras/VERIFICATION.md)). Earlier
versions ran on real drives; 0.6.0 has not been run on drives yet — the bench
checklist in VERIFICATION.md is the next step. MIT-licensed. Issues
and pull requests welcome.
