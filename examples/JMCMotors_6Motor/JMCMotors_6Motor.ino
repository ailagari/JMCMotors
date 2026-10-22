/*
  JMCMotors_6Motor - 6-motor servo/stepper controller over UDP.
  Part of JMCMotors (MIT) by LAGARI A <lagariscience@gmail.com>
  https://github.com/ailagari/JMCMotors

  How to use + command list: README.md
  Full protocol reference:   extras/PROTOCOL.md
  Drive setup & bring-up:    extras/HARDWARE.md

  Networking is fixed/static - no pairing, no auto-discovery. Edit the
  block below for your network; every reply and event this controller
  sends goes to PC_IP:PC_PORT, permanently (command replies also go back
  to whoever sent the command). On boot, once the drives are
  initialised, it sends "Motor ready" to that address.
*/
#include <JMCMotors.h>

//==============================================================================
//  EDIT ME - network addresses
//==============================================================================
// Controllino (this board) - must be a free, static address on the same
// subnet as the PC (UDP here is unicast to fixed addresses, no broadcast
// discovery, so PC and Controllino must be able to reach each other directly).
IPAddress CONTROLLINO_IP  (192, 168, 1, 101);
uint16_t  CONTROLLINO_PORT = 8001;

// PC that will send commands and receive replies/events.
IPAddress PC_IP   (192, 168, 1, 100);
uint16_t  PC_PORT  = 8000;

// MAC must be unique on the subnet if you run more than one Controllino.
byte CONTROLLINO_MAC[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
//==============================================================================

//==============================================================================
//  EDIT ME - velocity / accel / decel / homing speed
//==============================================================================
// Pick ONE mode:
//   false = GLOBAL   - every motor uses the same GLOBAL_* values below.
//   true  = PER-MOTOR - each motor uses its own row in the MOTOR[] table.
const bool PER_MOTOR_SETTINGS = false;

// ---- GLOBAL mode (used when PER_MOTOR_SETTINGS = false) --------------------
float GLOBAL_ACCEL      = 0.5;   // rps/s
float GLOBAL_DECEL      = 0.5;   // rps/s
float GLOBAL_VELOCITY   = 0.3;   // rps
float GLOBAL_HOMING_VEL = 0.3;   // rps

// ---- Number of motors on the bus (slave IDs 1..MOTOR_COUNT, up to 32) ------
// Everything adapts to this one number: detection, the event monitor, every
// command's motor slots. Set it to the number of drives actually installed
// (a missing drive costs a probe on every monitor sweep). It can also be
// changed live with the MC command (until the next reboot).
const uint8_t MOTOR_COUNT = 6;

// ---- PER-MOTOR mode (used when PER_MOTOR_SETTINGS = true) ------------------
// One row per motor, in order (row 0 = motor 1, row 1 = motor 2, ...).
// velocity/homingVel in rps (decimals OK), accel/decel in rps/s. These are
// hardcoded values: they're re-applied on every boot and whenever a drive
// comes back online, so they survive a power failure (unlike the runtime
// V / AD / HV commands, which only last until the next reboot).
// Motors beyond the last row (or a row with a 0) use the GLOBAL_* values.
struct MotorSettings { float velocity, accel, decel, homingVel; };

MotorSettings MOTOR[] = {
  // velocity, accel, decel, homingVel
  { 0.3, 0.5, 0.5, 0.3 },   // motor 1
  { 0.3, 0.5, 0.5, 0.3 },   // motor 2
  { 0.3, 0.5, 0.5, 0.3 },   // motor 3
  { 0.3, 0.5, 0.5, 0.3 },   // motor 4
  { 0.3, 0.5, 0.5, 0.3 },   // motor 5
  { 0.3, 0.5, 0.5, 0.3 },   // motor 6
};
const uint8_t MOTOR_ROWS = sizeof(MOTOR) / sizeof(MOTOR[0]);
//==============================================================================

JMCController jmc;                     // uses Serial3 (Controllino RS-485)

/*  UDP COMMANDS (v0.5) - one per packet to CONTROLLINO_IP:CONTROLLINO_PORT;
    replies return to the sender + PC_IP:PC_PORT; events to PC_IP:PC_PORT.
    One rule for every motor command:
      CMD:<slots>     individual motors - positional slots (field N = motor N,
                      empty = skip: "1000,,3000" = M1 and M3) or motor names
                      ("M3=500,M17=1000" / "M1,M3"); '?' reads
      CMDA[:value]    all motors (the A suffix); e.g. PA:5000, VA:0.5, BA, LA
    Full reference: extras/PROTOCOL.md.
      P:1000,,3000   absolute move (synchronised) ;  PA:5000 all ;  P:? / PA:? read
      PR:-500,,200   relative move ;  PRA:-500 all
      V:0.5,,0.3     run speed (rps) ;  VA:0.5 all ;  V:? / VA:? read
      ACC:0.2,,0.5   acceleration (rps/s) ;  ACCA:0.3 ;  DEC / DECA deceleration
      VS:1,,1 / VSA  jog endlessly: M1+M3 / all ;  stop with B
      B / B3         controlled / immediate stop, all (bare stop = all, always)
      B2:1,,1 / B2A  fast brake M1+M3 / all ;  L:,1 / LA loosen brake ;  BS / BSA
      Z:1,,1 / ZA    current position -> 0 ;  ZO:5000 / ZOA:5000 -> value ;  ZS / ZSA
      HV HVZ HACC HO homing speed, zero-find speed, ramp, offset (each: per motor,
                     A = all, ? = read) ;  H3:1,,1 / H3A start homing, methods 1..12
      S:1,,1:COMPACT / SA:FLAGS   status word ;  SD / SDA detailed ;  LS / LSA
                     limits+origin+homing ;  DG / DGA register dump
      R:,,1 / RA     alarm reset ;  I:1,,1 / IA re-detect ;  MC:? / MC:8 count
      SAVE / LOAD / SAVE:CLEAR   keep V ACC DEC HV HVZ HACC HO MC in EEPROM
      VER / PING / X             version / link check / reboot
    EVENTS pushed to PC_IP:PC_PORT: LIMIT (CW/CCW), ORIGIN (SW), HOMED,
    HOMING_ERROR, FAULT, FAULT_CLEARED, ONLINE, OFFLINE.                        */

void setup() {
  Serial.begin(115200);

  jmc.setNetwork(CONTROLLINO_MAC,
                 CONTROLLINO_IP, CONTROLLINO_PORT,
                 PC_IP,          PC_PORT);

  jmc.setMotorCount(MOTOR_COUNT);      // drives with slave IDs 1..MOTOR_COUNT (up to 32)

  // Globals always apply as the fallback (motors without a table row, or a
  // row holding 0); per-motor rows override them when enabled.
  jmc.setMotionDefaults(GLOBAL_ACCEL, GLOBAL_DECEL, GLOBAL_VELOCITY, GLOBAL_HOMING_VEL);
  if (PER_MOTOR_SETTINGS) {
    for (uint8_t i = 0; i < MOTOR_COUNT && i < MOTOR_ROWS; i++) {
      uint8_t id = i + 1;              // motor IDs are 1-based
      jmc.setMotorVelocity(id, MOTOR[i].velocity);
      jmc.setMotorAccelDecel(id, MOTOR[i].accel, MOTOR[i].decel);
      jmc.setMotorHomingVelocity(id, MOTOR[i].homingVel);
    }
  }

  // jmc.setMotorHomingOffset(1, 1000);       // optional: steps past sensor, per motor
  // jmc.setMotorHomingZeroVelocity(1, 0.05); // optional: slow final approach (HVZ)
  // jmc.setMotorHomingAccel(1, 0.2);         // optional: homing ramp (HACC)
  // jmc.setDriveP45(0);                      // only if a motor runs 10x faster than commanded
  // Note: a block stored with the SAVE command overrides all of the above at
  // boot - send SAVE:CLEAR to return to the sketch values.

  uint8_t found = jmc.begin();
  Serial.print(found);
  Serial.println(F(" motor(s) detected"));
}

void loop() {
  jmc.run();                           // non-blocking: commands + events
}
