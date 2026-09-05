/*
  LagariMotors_6Motor - 6-motor servo/stepper controller over UDP.
  Part of LagariMotors (MIT) by LAGARI A <lagariscience@gmail.com>
  https://github.com/ailagari/LagariMotors

  How to use + command list: README.md
  Full protocol reference:   extras/PROTOCOL.md
  Drive setup & bring-up:    extras/HARDWARE.md

  Networking is fixed/static - no pairing, no auto-discovery. Edit the
  block below for your network; every reply and event this controller
  sends goes to PC_IP:PC_PORT, permanently.
*/
#include <LagariMotors.h>

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
//  EDIT ME - per-motor velocity / accel / decel / homing speed
//==============================================================================
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

void setup() {
  Serial.begin(115200);

  jmc.setNetwork(CONTROLLINO_MAC,
                 CONTROLLINO_IP, CONTROLLINO_PORT,
                 PC_IP,          PC_PORT);

  jmc.setMotorCount(MOTOR_COUNT);      // drives with slave IDs 1..MOTOR_COUNT (up to 32)

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    uint8_t id = i + 1;                // motor IDs are 1-based
    jmc.setMotorVelocity(id, MOTOR[i].velocity);
    jmc.setMotorAccelDecel(id, MOTOR[i].accel, MOTOR[i].decel);
    jmc.setMotorHomingVelocity(id, MOTOR[i].homingVel);
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
