/*
  JMCMotors_Minimal - the smallest possible sketch: the whole UDP command
  system with the library's defaults (0.3 rps, 0.5 rps/s ramps).
  Part of JMCMotors (MIT) by LAGARI A - https://github.com/ailagari/JMCMotors
  Interactive command reference: https://ailagari.github.io/JMCMotors/

  Station ID 1 gives this board MAC DE:AD:BE:EF:00:01 and IP 192.168.1.101,
  listening on UDP port 8001. Replies/events go to 192.168.1.100:8000 (the
  default PC address) - change it with setNetwork() as in JMCMotors_6Motor.
  Send "VER", then "IA" from any UDP tool to check the link and the drives.
*/
#include <JMCMotors.h>

JMCController jmc;                     // Controllino built-in RS-485 (Serial3)

void setup() {
  jmc.setStationId(1);                 // derives MAC + IP (192.168.1.101)
  jmc.setMotorCount(2);                // drives with slave IDs 1..2
  jmc.begin();                         // bus + Ethernet + motors, sends "Motor ready"
}

void loop() {
  jmc.run();                           // non-blocking: commands + events
}
