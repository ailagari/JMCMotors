/*
  JMCMotors_Minimal - the smallest complete sketch (CONTROLLINO MAXI / MEGA)
  Part of JMCMotors (MIT) by LAGARI A - https://github.com/ailagari/JMCMotors
  Interactive command reference: https://ailagari.github.io/JMCMotors/

  Drives on the built-in RS-485, commands over Ethernet UDP to
  192.168.1.101:8001, events to 192.168.1.100:8000. Library defaults
  (0.3 rps, 0.5 rps/s ramps). Send "VER", then "IA" from any UDP tool.
  Any other board (ESP32, Uno, Mega): JMCMotors_Controller, set BOARD.
*/
#include <Ethernet.h>
#include <JMCMotors.h>
#include <JMCLinkUDP.h>

EthernetUDP   udp;
JMCLinkUDP    pcLink(udp);
JMCBus        bus(Serial3);
JMCController jmc(bus, pcLink);

void setup() {
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
  Ethernet.begin(mac, IPAddress(192, 168, 1, 101));
  pcLink.begin(8001, IPAddress(192, 168, 1, 100), 8000);
  bus.beginControllino(115200);
  jmc.setMotorCount(2);                  // drives with slave IDs 1..2
  jmc.begin();
}

void loop() {
  jmc.run();
}
