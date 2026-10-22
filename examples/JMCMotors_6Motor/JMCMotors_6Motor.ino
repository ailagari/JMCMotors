/*
  JMCMotors_6Motor - deployable multi-motor controller over UDP.
  Part of JMCMotors (MIT) by LAGARI A <lagariscience@gmail.com>
  https://github.com/ailagari/JMCMotors

  How to use + command list: README.md
  Interactive reference:     https://ailagari.github.io/JMCMotors/
                             (every command, replies, a builder and a simulator)
  Full protocol reference:   extras/PROTOCOL.md
  Drive setup & bring-up:    extras/HARDWARE.md

  HOW SETTINGS WORK
  -----------------
  1. The ADMIN edits the blocks marked "EDIT ME" below, flashes the
     Controllino, done. These values are re-applied on every boot and
     whenever a drive comes back online, so they survive power failures.
  2. On a deployed unit, settings can be changed over UDP (V, ACC, DEC, HV,
     HVZ, HACC, HO, MC). Those changes last until the next reboot - unless
     the operator sends SAVE, which stores them in the Controllino's EEPROM
     so no USB cable is needed. SAVE:? shows whether a block is stored,
     SAVE:CLEAR forgets it, LOAD re-applies it.
  3. The sketch always wins after a re-flash: the saved block remembers which
     sketch values it was made under, and is ignored as soon as the admin
     flashes different ones. So an old SAVE can never override a new flash.

  NETWORK: fixed/static - no pairing, no auto-discovery. Replies go back to
  whoever sent the command AND to PC_IP:PC_PORT; events (limit switches,
  origin switch, homing done, faults, drives online/offline) go only to
  PC_IP:PC_PORT. On boot, once the drives are initialised, the controller
  sends "Motor ready" to that address.
*/
#include <JMCMotors.h>

//==============================================================================
//  EDIT ME (1/3) - network addresses
//==============================================================================
// Controllino (this board): a free, static address on the same subnet as the
// PC. Flashing more Controllinos? Give each one a unique IP and MAC.
IPAddress CONTROLLINO_IP  (192, 168, 1, 101);
uint16_t  CONTROLLINO_PORT = 8001;

// PC that will send commands and receive replies/events.
IPAddress PC_IP   (192, 168, 1, 100);
uint16_t  PC_PORT  = 8000;

byte CONTROLLINO_MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };

//==============================================================================
//  EDIT ME (2/3) - number of motors on the bus
//==============================================================================
// Drives with Modbus slave IDs 1..MOTOR_COUNT (set P40 on each drive), up to
// 32. Everything adapts to this one number: detection, the event monitor,
// every command's motor slots. Set it to the number actually installed - a
// missing drive costs a probe on every monitor sweep. MC:n changes it live.
const uint8_t MOTOR_COUNT = 6;

//==============================================================================
//  EDIT ME (3/3) - speeds, ramps, homing
//==============================================================================
// Pick ONE mode:
//   false = GLOBAL    - every motor uses the GLOBAL_* values below.
//   true  = PER-MOTOR - each motor uses its own row in the MOTOR[] table.
const bool PER_MOTOR_SETTINGS = false;

// ---- GLOBAL values (always the fallback) -----------------------------------
float GLOBAL_VELOCITY   = 0.3;   // run speed, rps        (0.3 rps = 18 RPM)
float GLOBAL_ACCEL      = 0.5;   // acceleration, rps/s
float GLOBAL_DECEL      = 0.5;   // deceleration, rps/s
float GLOBAL_HOMING_VEL = 0.3;   // homing search speed, rps

// ---- PER-MOTOR table (used when PER_MOTOR_SETTINGS = true) ------------------
// One row per motor, in order (row 0 = motor 1). A 0 in any column means
// "use the global / default for that one"; motors beyond the last row use
// the globals. Columns:
//   velocity    run speed, rps
//   accel       acceleration, rps/s
//   decel       deceleration, rps/s
//   homingVel   homing search speed, rps            (HV)
//   homingZero  slow zero-find speed, rps           (HVZ; 0 = same as HV)
//   homingAccel homing ramp, rps/s                  (HACC; 0 = same as accel)
//   homingOff   steps to travel past the origin sensor before zero (HO; sign = direction)
struct MotorSettings { float velocity, accel, decel, homingVel, homingZero, homingAccel; long homingOff; };

MotorSettings MOTOR[] = {
  // velocity, accel, decel, homingVel, homingZero, homingAccel, homingOff
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0 },   // motor 1
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0 },   // motor 2
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0 },   // motor 3
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0 },   // motor 4
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0 },   // motor 5
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0 },   // motor 6
};
const uint8_t MOTOR_ROWS = sizeof(MOTOR) / sizeof(MOTOR[0]);

// Only if a motor measurably runs 10x faster than commanded (drive P45 in
// whole-rps units): uncomment. See extras/HARDWARE.md, speed-scale check.
// #define DRIVE_P45_WHOLE_RPS
//==============================================================================

JMCController jmc;                     // uses Serial3 (Controllino RS-485)

/*  UDP COMMANDS (v0.5) - one per packet to CONTROLLINO_IP:CONTROLLINO_PORT.
    One rule for every motor command:
      CMD:<slots>     individual motors - positional slots (field N = motor N,
                      empty = skip: "1000,,3000" = M1 and M3) or motor names
                      ("M3=500,M17=1000" / "M1,M3"); '?' reads
      CMDA[:value]    all motors (the A suffix); e.g. PA:5000, VA:0.5, BA, LA
      P:1000,,3000   absolute move ;  PA:5000 all ;  P:? / PA:? read
      PR:-500,,200   relative move ;  PRA:-500 all
      V:0.5,,0.3     run speed (rps) ;  VA:0.5 ;  V:? / VA:? read
      ACC / DEC      acceleration / deceleration (rps/s), same forms as V
      VS:1,,1 / VSA  jog endlessly ;  stop with B
      B / B3         controlled / immediate stop, ALL (a bare stop always = all)
      B2:1,,1 / B2A  fast brake M1+M3 / all ;  L:,1 / LA loosen ;  BS / BSA
      Z:1,,1 / ZA    current position -> 0 ;  ZO:5000 / ZOA:5000 -> value
      HV HVZ HACC HO homing speed, zero-find speed, ramp, offset (per motor, A, ?)
      H3:1,,1 / H3A  start homing, methods 1..12
      S:M1:COMPACT / SA:FLAGS   status ;  SD / SDA ;  LS / LSA limits+origin+homing
      R:,,1 / RA     alarm reset ;  I:1,,1 / IA re-detect ;  MC:? / MC:8 count
      SAVE / SAVE:? / SAVE:CLEAR / LOAD   keep runtime settings without USB
      VER / PING / X             version / link check / reboot
    EVENTS to PC_IP:PC_PORT: LIMIT (CW/CCW), ORIGIN (SW), HOMED, HOMING_ERROR,
    FAULT, FAULT_CLEARED, ONLINE, OFFLINE.  Full reference: extras/PROTOCOL.md */

void setup() {
  Serial.begin(115200);

  jmc.setNetwork(CONTROLLINO_MAC,
                 CONTROLLINO_IP, CONTROLLINO_PORT,
                 PC_IP,          PC_PORT);

  jmc.setMotorCount(MOTOR_COUNT);

  // Globals first (the fallback for every motor), then the per-motor rows.
  jmc.setMotionDefaults(GLOBAL_ACCEL, GLOBAL_DECEL, GLOBAL_VELOCITY, GLOBAL_HOMING_VEL);
  if (PER_MOTOR_SETTINGS) {
    for (uint8_t i = 0; i < MOTOR_COUNT && i < MOTOR_ROWS; i++) {
      uint8_t id = i + 1;              // motor IDs are 1-based
      jmc.setMotorVelocity(id, MOTOR[i].velocity);
      jmc.setMotorAccelDecel(id, MOTOR[i].accel, MOTOR[i].decel);
      jmc.setMotorHomingVelocity(id, MOTOR[i].homingVel);
      jmc.setMotorHomingZeroVelocity(id, MOTOR[i].homingZero);
      jmc.setMotorHomingAccel(id, MOTOR[i].homingAccel);
      jmc.setMotorHomingOffset(id, MOTOR[i].homingOff);
    }
  }
#ifdef DRIVE_P45_WHOLE_RPS
  jmc.setDriveP45(0);
#endif

  // begin(): RS-485 + Ethernet + motor detection. Applies a SAVEd block on
  // top of the values above only if the sketch is unchanged since the SAVE
  // (it prints which on the Serial Monitor), then sends "Motor ready".
  uint8_t found = jmc.begin();
  Serial.print(found);
  Serial.println(F(" motor(s) detected"));
}

void loop() {
  jmc.run();                           // non-blocking: commands + events
}
