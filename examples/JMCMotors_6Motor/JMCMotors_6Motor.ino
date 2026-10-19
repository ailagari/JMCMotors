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

// ---- PER-MOTOR mode (used when PER_MOTOR_SETTINGS = true) ------------------
// One row per motor, in order (row 0 = motor 1, row 1 = motor 2, ...).
// velocity/homingVel in rps (decimals OK), accel/decel in rps/s. These are
// hardcoded values: they're re-applied on every boot and whenever a drive
// comes back online, so they survive a power failure (unlike the runtime
// V / AD / HV commands, which only last until the next reboot).
const uint8_t MOTOR_COUNT = 6;
struct MotorSettings { float velocity, accel, decel, homingVel; };

MotorSettings MOTOR[MOTOR_COUNT] = {
  // velocity, accel, decel, homingVel
  { 0.3, 0.5, 0.5, 0.3 },   // motor 1
  { 0.3, 0.5, 0.5, 0.3 },   // motor 2
  { 0.3, 0.5, 0.5, 0.3 },   // motor 3
  { 0.3, 0.5, 0.5, 0.3 },   // motor 4
  { 0.3, 0.5, 0.5, 0.3 },   // motor 5
  { 0.3, 0.5, 0.5, 0.3 },   // motor 6
};
//==============================================================================

JMCController jmc;                     // uses Serial3 (Controllino RS-485)

/*  UDP COMMANDS - send one per packet to CONTROLLINO_IP:CONTROLLINO_PORT;
    replies return to the sender + PC_IP:PC_PORT; events to PC_IP:PC_PORT. Full reference: extras/PROTOCOL.md.
      P:1000,,3000   absolute move: M1->1000, skip M2, M3->3000 (synchronised)
      PR:-1000       relative move ;  PA:5000 all motors ;  P:? / PA:? read pos
      V:0.5          speed (rps) ;  AD:0.2 accel+decel (rps/s)
      VS             run continuously (endless) ;  stop with B
      B              controlled stop, all ;  B3: immediate stop, all
      B2:1,,1        fast-brake M1 and M3 (positional slots, like P: empty=skip)
      L              loosen brakes ;  BS brake status
      Z / ZO:5000    current position -> 0 / -> 5000 (no movement)
      HV:0.3 HO:1000 H3   homing: speed, offset, home ;  H3:1,,1 = M1 and M3
      S / SD         status (raw / human-readable)
      R / I / X      alarm reset / re-detect motors / reboot controller
      MC:? / MC:8    read / set active motor count at runtime                   */

void setup() {
  Serial.begin(115200);

  jmc.setNetwork(CONTROLLINO_MAC,
                 CONTROLLINO_IP, CONTROLLINO_PORT,
                 PC_IP,          PC_PORT);

  jmc.setMotorCount(MOTOR_COUNT);      // drives with slave IDs 1..MOTOR_COUNT (up to 32)

  if (PER_MOTOR_SETTINGS) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      uint8_t id = i + 1;              // motor IDs are 1-based
      jmc.setMotorVelocity(id, MOTOR[i].velocity);
      jmc.setMotorAccelDecel(id, MOTOR[i].accel, MOTOR[i].decel);
      jmc.setMotorHomingVelocity(id, MOTOR[i].homingVel);
    }
  } else {
    jmc.setMotionDefaults(GLOBAL_ACCEL, GLOBAL_DECEL, GLOBAL_VELOCITY, GLOBAL_HOMING_VEL);
  }

  // jmc.setMotorHomingOffset(1, 1000); // optional: steps past sensor, per motor
  // jmc.setDriveP45(0);                // only if a motor runs 10x faster than commanded

  uint8_t found = jmc.begin();
  Serial.print(found);
  Serial.println(F(" motor(s) detected"));
}

void loop() {
  jmc.run();                           // non-blocking: commands + events
}
