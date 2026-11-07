# JMCMotors — How it is verified

What has been checked, against what, and what only a real machine can
confirm. Library version 0.6.0; reference: *JMC Modbus-RTU communication
manual V2.2* for the IHSS / IHT "-RC" drives ("the manual"; § = section,
p. = page of the PDF), read page by page including its tables, timing
diagrams, homing diagrams and register list.

Four layers:

| Layer | What it proves | How to repeat it |
|---|---|---|
| **Compile matrix** | every example builds for every supported board, and a wrong board selection stops with a clear message | `arduino-cli compile` for CONTROLLINO MAXI / MEGA, Uno, Mega 2560, ESP32 / S3 / C3 (15 board × link combinations of `JMCMotors_Controller`; a Nano is uploaded as "Arduino Uno", see the README) |
| **Host tests** | 956 checks in four programs run the real library sources against simulated drives that answer real, CRC-checked Modbus-RTU frames: the command suite (full and Uno profile), a **frame check** (every command's exact register writes against the manual's routines, ~4000 frames, plus the manual's general rules over all of them) and a **motion check** (a simulated axis with CW / CCW / origin switches: forward and reverse moves and jogs, one and several motors, all 12 homing methods following §6.3, homing errors, limits, status formats, resets, faults, power cycles) | `bash extras/tests/run_tests.sh` |
| **Independent audit** | a separate review that read the whole PDF and the source without seeing these conclusions; its findings are listed below with what was done | — |
| **Manual cross-check** | each requirement of the manual, mapped to the library and to a test (table below) | this file |
| **Bench** | timing and behaviour that only real drives show | the checklist at the end |

## Manual cross-check

| # | The manual says | §   | The library does | Test |
|---|---|---|---|---|
| 1 | RS-485 two-wire half duplex, slaves 1–32, 120 Ω at the last drive | 3 | any serial port + DE/RE pin (or auto-direction module), slave IDs 1..32 | — |
| 2 | Function codes 0x03, 0x06, 0x10 only; at most 63 words per read | 3, 7 | uses only those three; reads 1–2 words | every frame of the tests |
| 3 | Slave 0 = broadcast, writes only, no reply | 4 | broadcast for simultaneous starts, `VA` and stops; never reads from 0, never waits for a reply | "B starts with one broadcast halt" |
| 4 | CRC-16 (Modbus), low byte first | 4, 7 | same | the CRCs printed in the manual (`29 96`, `7E 11`, `67 C9`, …) recomputed and matched |
| 5 | 32-bit registers are written with 0x10 only; word order set by 0x6000 (read-only) | 5 | 32-bit values always with 0x10; reads 0x6000 at every init and handles both orders | `P:200000` = `01 10 60 7A 00 02 04 00 03 0D 40 29 96` (0x6000 = 0) and `… 0D 40 00 03 9F 8F` (0x6000 = 1); negative readback in both orders |
| 6 | Power-up: control word 0x0001 → 0x0003 → 0x000F | 6.1 | the same three writes in `begin()`, `I`, `R` and on re-connection | frames `57 DE`, `D6 1F`, `D6 1A` |
| 7 | Position mode: 0x6060 = 1, then 0x607A / 0x6081 / 0x6083 / 0x6084, then bit 4 0→1 | 6.1 | the same; bit 4 rising (0x3F) then low again (0x2F), bit 5 = new target at once | frames `56 14`, `12 06`, `29 96`; the 0x3F / 0x2F pulse |
| 7a | Bit 4 is sampled only while status bit 12 is 1; the timing diagrams (p. 12–13) show bit 12 staying **1 throughout** when control-word bit 5 = 1, and dropping to 0 only in the buffered mode (bit 5 = 0) while a second set-point waits | 6.1 | always sends bit 5 = 1, so a new target is accepted at any time — also mid-move — without a bit-12 handshake | simulated drive follows p. 13; bench check 4 |
| 8 | Bit 6 = relative: "the sum of the two positions" (the drive's previous target) | 6.1 | **not used by `PR`**: the controller sends *actual position + delta* as an absolute target, because the drive's previous target cannot be read back (0x607A returns the last value written) and differs from the shaft after `Z` / `ZO` / homing | "`P:5000`, `ZO:0`, `PR:100` ends at 100, not 5100"; "PR never sets bit 6" |
| 9 | Velocity mode: 0x010F (pause), 0x6060 = 3, 0x6081 (signed), 0x000F starts, 0x010F stops | 6.2 | `VS` follows that routine; the sign of `V` is the direction | frames `D7 8A`, `D7 D5`, `D6 1A`; a negative jog reads `VEL=-1.0` |
| 10 | Homing: 0x6060 = 6, 0x6098 method, 0x6099 / 0x609B speeds, 0x609A ramp, 0x607C offset, bit 4 starts | 6.3 | `H1`…`H12` write all five parameters again, then mode, method, bit 4 | frames `17 D6`, `D7 E5`, `93 40`, `12 99`, `B7 5B`, `5C 62`, `D7 D6` |
| 11 | Methods 1–4: at most 300 r/min; 5–12: at most 1200 r/min; 0x6099 / 0x609B in 0..100000, 0 makes homing fail | 6.3, 5 | `H1`–`H4` refuse `HV` / `HVZ` above 5 rps; `HV` / `HVZ` limited to 0.01–20 rps everywhere | "`H1` refused at 10 rps, `H5` accepted" |
| 12 | Only methods 9–12 accept a negative home offset | 6.3 | `H1`–`H8` refuse a negative `HO` | "`H3` / `H8` refused, `H9` ends at −500" |
| 13 | Method 0: no movement, the current position becomes the origin (offset 0x607C) | 6.3 | `Z` (= 0) and `ZO:<value>`; `HO` is written again before every `H<n>` | `ZO` to ±values, then homing uses `HO` again |
| 14 | 0x6081, 0x606C, 0x6099, 0x609B are ×10 rps; 0x6083 / 0x6084 / 0x6085 / 0x609A are ×10 rps/s | 5 | converted in one place (`speedToReg`, `accelToReg`), never 0 for a non-zero request | `V:1` → 10, `ACC:10` → 100, `HACC:100` → 1000, `HV:10` → 100 |
| 15 | Halt code 0x605D (1 = current decel); quick-stop code 0x605A (1 / 2 / 3); 0x6085 quick-stop ramp | 5 | halt code 1 at every init; `B1`–`B3` write the code then 0x000B (bit 2 = 0, bit 1 = 1, enable bits kept); 0x6085 kept equal to `DEC` | frame check `B1`/`B2`/`B3` |
| 16 | Status word: bits 10 / 12 / 13 change meaning per mode; 11 = origin (SW), 14 = CW limit, 15 = CCW limit | 5 | decoded per the mode last commanded (`S:…:FLAGS`, `SD`, `LS`, events) | limit / origin / homing events |
| 17 | Error register 0x1001: bit 0 generic, 1 current, 2 voltage, 3 temperature, 4 communication, 5 over-position, 7 phase loss | 5 | names in `EVENT:FAULT` and `SD` | 0x22 → `CURRENT OVERPOS` |
| 18 | Exception replies, codes 1–11 | 4 | detected (5-byte frame, CRC-checked), reported as `EXCEPTION`, code shown by `DG` | "a drive that rejects a write: `ERR:P M1 not accepted (EXCEPTION) - nothing started`" |
| 19 | Homing method 1 moves "left" to the CW limit, method 2 "right" to the CCW limit; every homing diagram (p. 20–26) draws CW at the left end of the axis, CCW at the right, the SW origin between | 6.3 | documented: CW = the negative end, CCW = the positive end; the drive refuses motion into an active limit | "CW limit blocks negative moves, positive ones still run" |
| 20 | Register list (§8, p. 32): 0x6040 and 0x6060 are **write-only** (reading them = exception 7) | 8 | never reads them; reads only 0x1001, 0x6000, 0x6041, 0x6061, 0x6064, 0x606C, 0x607A, 0x6081, 0x6083, 0x6084 | code review |
| 21 | 0x6083 / 0x6084: UNSIGNED16 in §5 (p. 9), INTEGER16 in the register list (p. 32) | 5, 8 | ramps limited to 3276.7 rps/s = register 32767, valid under both | "`ACC:3276.8` refused, `ACC:3276.7` → 32767" |
| 22 | 0x6081 target velocity: INT32 (p. 14, p. 32; "INSIGNED32" on p. 9 is a typo) | 5 | signed: the sign of `V` is the jog direction; position moves send the magnitude | negative jog test |
| 23 | 0x10 writes of several registers must follow the register-list order (p. 32 tip); the 9-register example on p. 14 relies on that | 8 | never writes more than one register per 0x10 frame (a 32-bit value = its two words) | every frame of the tests |
| 24 | Exception reply example: `01 86 0B 03 A7` (function 06, code 11 "register does not exist") | 7.5 | parsed, code kept in `lastException()`, reported as `EXCEPTION` | CRC `03 A7` recomputed; "code 11 recorded" |

## Where the manual is unclear, and what the library chose

| Topic | The manual | The library |
|---|---|---|
| Status bit 5 | bit table: "1 = quick stop"; state table: 1 in "Device enable", 0 in "Quick stop active" | follows the state table (and CiA 402): 1 = quick stop **not** active |
| Error reset bit | bit table: bit 7; state table header: bit 8 | bit 7 (0x0080), as in the bit table |
| Homing speed for method 1 | text: max 300 r/min; the homing code example sets 10 rps (600 r/min) with method 1 | follows the stated limit (refuses > 5 rps for H1–H4) |
| Status bit 12 in position mode | "1 = can set new position" and "bit 4 is only sampled while bit 12 is 1"; the diagrams call it "set-point ack" | resolved by the p. 13 diagram: with bit 5 = 1 (what the library sends) bit 12 stays 1, so no handshake is needed; bench check 4 confirms it on your drive |
| Quick stop | control-word bit table: "bit 2 0→1 quick stop"; state table: quick stop = bit 2 **0** with bit 1 = 1 | follows the state table and CiA 402: writes 0x0002 after the quick-stop code |
| 0x6000 access | RO on p. 4, RW in the register list | only reads it |
| Ramp register type | 0x6083 / 0x6084 UNSIGNED16 (p. 9) vs INTEGER16 (p. 32) | stays within 0..32767 |
| Typos | halt-code table headed "605Ah" (p. 7), the deceleration example on p. 15 printed with "60 7A 00 03" | uses 0x605D and the frame on p. 15's first line (`01 06 60 84 00 64 D6 08`, which the tests match) |
| Velocity routine order | p. 19: mode 3, then pause 0x010F, then speed | pause first, then mode 3, then speed — both put halt on before the speed is written |
| "Left" / "right" | not tied to position sign in the text | CW = negative end, from the method numbering (CiA 402 method 1 = negative limit switch); see bench check 2 |

## Independent audit — findings and actions

| Finding | Action |
|---|---|
| After a quick stop (`B1`–`B3`) of a jog, the next re-enable (`R`, `I`, a move) wrote 0x000F to a drive still in velocity mode with a jog speed set — the motor could start turning | **fixed**: quick stops clear the jog speed; the power-up routine writes speed 0 before 0x000F (also found independently by the frame check) |
| After `B` mid-move, the next command released the halt (0x000F) before the new target was loaded — the drive could resume the old target for a moment (p.18) | **fixed**: the halt stays on until the new move's start pulse; a test proves the old behaviour lurched and the new one does not |
| Quick stop as 0x0002 clears bit 3, which the bit table (p.5) calls "motor enable 1→0" | **changed** to 0x000B: quick stop under the state table, enable bits kept |
| `ZO` relies on an unstated offset sign for homing method 0 | **made self-verifying**: read back, sign corrected once, otherwise reported |
| `getDetailedStatus().quickStopActive` read bit 5 the wrong way round | **fixed** (state table p.7) |
| Exceptions 6 (busy) and 10 (CRC error, "requesting retransmission", p.3) were not retried | **fixed**: retried within the normal retries |
| `VA` broadcast a signed speed also into drives in position mode | **fixed**: sent only to jogging motors |
| `DG` re-wrote a default value when its read failed | **fixed**: write test skipped |
| The broadcast start pulse was ~0.65 ms wide; the manual gives no processing time | **widened** to 2 ms |
| A broadcast reaches every drive on the bus, also ones not configured (`MC`) | documented: `MC` must match the drives on the bus; the controller uses broadcast starts only when all configured motors move |
| `R` / `I` briefly switch the motor off (power-up routine) | documented: send with the motor stopped |
| Homing "complete" fallback (at rest, bit 12 set, 1 s after start) could in theory report a run that never started | kept (needed when homing starts on the switch); bench check 6 |

## What the library guarantees on top of the manual

- `OK` means every drive accepted every frame of the command; otherwise
  `ERR:<CMD> Mn not accepted (<reason>) …`, and coordinated moves abort before
  any motor starts.
- A stop of all motors is one broadcast frame first (simultaneous; a dead
  drive cannot delay it), then confirmed per drive.
- Motion commands refuse a motor known to be offline; `PA` and friends skip it.
- Soft limits, a communication watchdog, a processor watchdog, strict number
  parsing, no heap and no `String`, `millis()` rollover handled (tested across
  the 49.7-day wrap).

## Bench checklist (real drives)

The simulated drives move at constant speed without ramps and cannot show
real timing. Before relying on a new installation:

1. **Every drive answers:** `VER`, then `IA` → `Total motors detected: N of N`.
2. **Directions and limits:** `LS:M1` with nothing pressed must show `CW=0
   CCW=0 SW=0`. Jog slowly negative (`V:-0.1`, `VS:M1`) and press the CW
   switch: the motor must stop and `EVENT:LIMIT:M1:CW` arrive. Repeat positive
   with CCW. If negative moves do nothing at all, `CW=1` (polarity P42 or
   wiring) is the usual cause.
3. **Steps:** `Z:M1`, `PR:4000` → one revolution at P17 = 4000; `P:?` →
   `M1=4000`; `PR:-8000` → `M1=-4000`; `P:0` → `M1=0`.
4. **Mid-move retarget (status bit 12):** `V:0.5`, `P:40000`, and within a
   second `P:-4000`. Per the manual's p. 13 diagram the motor turns round and
   ends at `M1=-4000`. If it finishes the first move instead, report it with
   the drive model and firmware.
5. **Synchronised arrival:** two axes, `PS:20000,10000` — both stop together.
6. **Homing:** `HV:0.5`, `H3:M1` → `EVENT:HOMED:M1`; `LS:M1` → `HOMED=1`.
   Also start once with the axis already on the origin switch, and once with
   the switch unplugged: `EVENT:HOMING_ERROR` must arrive, not `HOMED`.
6a. **Position set:** `ZO:M1=5000`, `P:?` → `M1=5000` (the controller checks
   this itself and reports a drive that disagrees).
6b. **Quick stop:** jog, `B3` → the motor stops at once and holds torque;
   `S:M1:STATE` → `Quick stop active`; `R:M1` → it stays still.
7. **Faults:** block a shaft (gently) or pull a motor plug → `EVENT:FAULT`;
   fix, `R:M1` → `EVENT:FAULT_CLEARED`.
8. **Watchdog:** `WD:500`, stop sending → `EVENT:WATCHDOG:STOP`, motors stop.
9. **Power-cycle a drive** while running → `EVENT:OFFLINE`, then `EVENT:ONLINE
   … (re-initialised)`; its position is 0 until re-homed.

Results from real installations (drive model, firmware, P-settings, which
checks passed) are welcome as GitHub issues — they make the library more
trustworthy for everyone.
