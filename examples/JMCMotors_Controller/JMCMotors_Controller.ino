/*
  JMCMotors_Controller - the deployable multi-motor controller, for every board
  Part of JMCMotors (MIT) by LAGARI A <lagariscience@gmail.com>
  https://github.com/ailagari/JMCMotors
  Interactive reference:     https://ailagari.github.io/JMCMotors/
                             (every command, replies, a builder and a simulator)
  Full protocol reference:   extras/PROTOCOL.md
  Drive setup & bring-up:    extras/HARDWARE.md

  HOW TO USE
  ----------
  1. STEP 1: select your board (and pick the same one in Tools > Board).
  2. STEP 2: only for ESP32 / Uno / Mega - the pins of the RS-485 module.
  3. STEP 3: network (or USB), STEP 4: motors, STEP 5: safety.
  4. Upload. The sketch compiles only what your board needs, and stops with
     a clear message if STEP 1 and Tools > Board do not match.

  WHAT EACH BOARD USES
  --------------------
  Board              Drives (RS-485)                     PC link
  CONTROLLINO        built-in RS-485 port, nothing to set Ethernet UDP, or USB
  ESP32              TTL-to-RS-485 module on UART2 *      WiFi UDP, or USB
  Uno / Nano         TTL-to-RS-485 module, SoftwareSerial USB cable (57600)
  Mega 2560          TTL-to-RS-485 module on Serial1/2/3  Ethernet shield, or USB
  * UART1 on the ESP32-C3 / C6, which have only two UARTs.

  UNO PROFILE (32 KB flash): up to 6 motors; every motion, homing, limit and
  safety function works, but PS/PRS, SAVE/LOAD, SD/DG, BS/ZS, MC:n, X, HVZ/HACC
  and the explanatory text in replies are left out (see the README).

  RS-485, NOT RS-232: the JMC "-RC" drives speak RS-485 only. Between the
  board and the drives use a TTL-to-RS-485 module (MAX485; MAX3485 / SP3485
  for the 3.3 V ESP32). A TTL-to-RS-232 (MAX3232) module will not talk to
  the drives.

  HOW SETTINGS WORK
  -----------------
  The values in this file are the admin's configuration: re-applied on
  every boot and whenever a drive comes back online. Changed at runtime
  (V, ACC, DEC, HV, HVZ, HACC, HO, SL, WD, MC) they last until reboot -
  unless the operator sends SAVE (not on the Uno). A re-flash with other
  values always wins over a SAVEd block.
*/

//==============================================================================
//  STEP 1 - SELECT YOUR BOARD           (EDIT ME: set BOARD to one of these)
//==============================================================================
#define BOARD_CONTROLLINO  1   // CONTROLLINO MAXI / MEGA (built-in RS-485)
#define BOARD_ESP32        2   // any ESP32 board (ESP32, S2, S3, C3, C6 ...)
#define BOARD_UNO          3   // Arduino Uno / Nano / Pro Mini (ATmega328P). Nano: upload with
                               // Tools > Board > "Arduino Uno" (the IDE's Nano entries allow only
                               // 30 KB; new-bootloader Nanos run the Uno's Optiboot)
#define BOARD_MEGA         4   // Arduino Mega 2560

#define BOARD  BOARD_CONTROLLINO

// How the PC sends commands (EDIT ME):
#define LINK_NETWORK  1        // UDP: Ethernet (CONTROLLINO, Mega + shield) or WiFi (ESP32)
#define LINK_USB      2        // the USB cable (Serial Monitor or a COM-port program)

#define PC_LINK  LINK_NETWORK  // the Uno always uses LINK_USB (no room for a network stack)

//==============================================================================
//  STEP 2 - RS-485 MODULE PINS          (EDIT ME - only the block of your board)
//==============================================================================
//  Module wiring:  RO -> board RX pin,  DI <- board TX pin,
//                  DE + RE (tied together) <- board DE pin,  A / B -> drives.
//  Module without DE/RE pins (automatic direction): set the DE pin to -1.

// ---- ESP32 ----------------------------------------------------------------
#define ESP32_RS485_RX     16      // GPIO wired to the module's RO
#define ESP32_RS485_TX     17      // GPIO wired to the module's DI
#define ESP32_RS485_DE     4       // GPIO wired to DE + RE, or -1
#define ESP32_DRIVE_BAUD   115200  // drive parameter P41 = 7

// ---- Uno / Nano (SoftwareSerial: the hardware port is the USB link) -------
#define UNO_RS485_RX       8       // pin wired to RO (needs pin-change interrupts: 8-13, A0-A5 ok)
#define UNO_RS485_TX       9       // pin wired to DI
#define UNO_RS485_DE       7       // pin wired to DE + RE, or -1
#define UNO_DRIVE_BAUD     38400   // SoftwareSerial limit: set every drive's P41 to 38400

// ---- Mega 2560 --------------------------------------------------------------
#define MEGA_RS485_PORT    Serial1 // Serial1 = RX1 19 / TX1 18, Serial2 = 17 / 16, Serial3 = 15 / 14
#define MEGA_RS485_DE      7       // pin wired to DE + RE, or -1 (not 10 / 50-53: Ethernet shield)
#define MEGA_DRIVE_BAUD    115200  // drive parameter P41 = 7

// ---- CONTROLLINO: nothing to set (built-in RS-485, 115200 baud, P41 = 7) ---

//==============================================================================
//  STEP 3 - NETWORK                     (EDIT ME - used with PC_LINK LINK_NETWORK)
//==============================================================================
// This board, static address (no DHCP), on the PC's subnet:
#define BOARD_IP_ADDR      192, 168, 1, 101
#define BOARD_UDP_PORT     8001
// The PC: receives events and a copy of every reply.
#define PC_IP_ADDR         192, 168, 1, 100
#define PC_UDP_PORT        8000
// Ethernet (CONTROLLINO, Mega + shield): unique MAC per board.
#define BOARD_MAC          0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01
// WiFi (ESP32):
#define WIFI_SSID          "your-network"
#define WIFI_PASSWORD      "your-password"
#define WIFI_GATEWAY       192, 168, 1, 1
#define WIFI_SUBNET        255, 255, 255, 0
// Accept commands only from the PC (recommended in a plant). false = anyone.
const bool ONLY_ACCEPT_PC = false;

// USB link (PC_LINK LINK_USB): one command per line, replies end with CR LF.
#define USB_BAUD           ((BOARD == BOARD_UNO) ? 57600 : 115200)

//==============================================================================
//  STEP 4 - MOTORS                      (EDIT ME)
//==============================================================================
// Drives with Modbus slave IDs 1..MOTOR_COUNT (drive parameter P40).
// Up to 32 (Uno: 6). MC:n changes it live.
const uint8_t MOTOR_COUNT = 6;

const bool PER_MOTOR_SETTINGS = false;   // false = GLOBAL_* for every motor

const float GLOBAL_VELOCITY   = 0.3;     // rps (0.3 rps = 18 RPM)
const float GLOBAL_ACCEL      = 0.5;     // rps/s
const float GLOBAL_DECEL      = 0.5;     // rps/s
const float GLOBAL_HOMING_VEL = 0.3;     // rps

// One row per motor (row 0 = motor 1). A 0 means "use the global/default";
// motors beyond the last row use the globals. softMin / softMax: position
// limits in steps (moves outside are refused, jogs stop there); 0/0 = none.
// Kept in flash (PROGMEM), so a long table costs no RAM on the Uno.
struct MotorSettings {
  float velocity, accel, decel, homingVel, homingZero, homingAccel;
  long  homingOff, softMin, softMax;
};
const MotorSettings MOTOR[] PROGMEM = {
  // vel  acc  dec  hVel hZero hAcc  hOff  softMin softMax
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0,      0,      0 },   // motor 1
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0,      0,      0 },   // motor 2
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0,      0,      0 },   // motor 3
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0,      0,      0 },   // motor 4
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0,      0,      0 },   // motor 5
  { 0.3, 0.5, 0.5, 0.3, 0.0, 0.0,    0,      0,      0 },   // motor 6
};
const uint8_t MOTOR_ROWS = sizeof(MOTOR) / sizeof(MOTOR[0]);

//==============================================================================
//  STEP 5 - SAFETY                      (EDIT ME)
//==============================================================================
// Communication watchdog: stop every motor when no command arrives for this
// long (the PC then sends PING more often). 0 = off. On by default for the
// ESP32, because WiFi can drop.
const uint16_t COMMAND_WATCHDOG_MS = (BOARD == BOARD_ESP32) ? 1000 : 0;
// Processor watchdog: reboot the board if the firmware ever hangs. Test it
// once first - some old Mega 2560 bootloaders reboot-loop after a reset.
const bool HARDWARE_WATCHDOG = false;
// Only if a motor measurably runs 10x faster than commanded (drive P45):
// #define DRIVE_P45_WHOLE_RPS
//==============================================================================
//  Nothing to edit below this line.
//==============================================================================

// ---- does STEP 1 match Tools > Board? ------------------------------------------
#if BOARD == BOARD_CONTROLLINO
  #if !defined(CONTROLLINO_MAXI) && !defined(CONTROLLINO_MEGA)
    #error "STEP 1 says BOARD_CONTROLLINO, but Tools > Board is not a CONTROLLINO MAXI or MEGA. Select it there (CONTROLLINO_Boards package), or change BOARD."
  #endif
#elif BOARD == BOARD_ESP32
  #if !defined(ARDUINO_ARCH_ESP32)
    #error "STEP 1 says BOARD_ESP32, but Tools > Board is not an ESP32 board. Select it there (esp32 package), or change BOARD."
  #endif
#elif BOARD == BOARD_UNO
  #if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega328__) && !defined(__AVR_ATmega168__)
    #error "STEP 1 says BOARD_UNO, but Tools > Board is not an Uno / Nano / Pro Mini (ATmega328P). Select it there, or change BOARD."
  #endif
  #if PC_LINK != LINK_USB
    #error "The Uno uses the USB link: set PC_LINK to LINK_USB (a network stack does not fit in its 32 KB)."
  #endif
#elif BOARD == BOARD_MEGA
  #if !defined(__AVR_ATmega2560__)
    #error "STEP 1 says BOARD_MEGA, but Tools > Board is not an Arduino Mega 2560. Select it there, or change BOARD."
  #endif
  #if defined(CONTROLLINO_MAXI) || defined(CONTROLLINO_MEGA)
    #error "Tools > Board is a CONTROLLINO: set BOARD to BOARD_CONTROLLINO (its RS-485 port is built in)."
  #endif
#else
  #error "STEP 1: set BOARD to BOARD_CONTROLLINO, BOARD_ESP32, BOARD_UNO or BOARD_MEGA."
#endif
#if PC_LINK != LINK_NETWORK && PC_LINK != LINK_USB
  #error "STEP 1: set PC_LINK to LINK_NETWORK or LINK_USB."
#endif

// ---- libraries for this board -------------------------------------------------------
#include <JMCMotors.h>
#if PC_LINK == LINK_NETWORK
  #if BOARD == BOARD_ESP32
    #include <WiFi.h>
    #include <WiFiUdp.h>
  #else
    #include <Ethernet.h>
  #endif
  #include <JMCLinkUDP.h>
#else
  #include <JMCLinkSerial.h>
#endif
#if BOARD == BOARD_UNO
  #include <SoftwareSerial.h>
#endif

// ---- the drive bus ---------------------------------------------------------------------
#if BOARD == BOARD_CONTROLLINO
  JMCBus bus(Serial3);                                // built-in RS-485
  void startDrives() { bus.beginControllino(115200); }
#elif BOARD == BOARD_ESP32
  #if SOC_UART_NUM > 2
    #define ESP32_RS485_PORT Serial2
  #else
    #define ESP32_RS485_PORT Serial1                  // ESP32-C3 / C6: two UARTs
  #endif
  JMCBus bus(ESP32_RS485_PORT);
  void startDrives() {
    ESP32_RS485_PORT.begin(ESP32_DRIVE_BAUD, SERIAL_8N1, ESP32_RS485_RX, ESP32_RS485_TX);
    bus.begin(ESP32_DRIVE_BAUD, ESP32_RS485_DE);
  }
#elif BOARD == BOARD_UNO
  SoftwareSerial rs485(UNO_RS485_RX, UNO_RS485_TX);
  JMCBus bus(rs485);
  void startDrives() { rs485.begin(UNO_DRIVE_BAUD); bus.begin(UNO_DRIVE_BAUD, UNO_RS485_DE); }
#elif BOARD == BOARD_MEGA
  JMCBus bus(MEGA_RS485_PORT);
  void startDrives() { MEGA_RS485_PORT.begin(MEGA_DRIVE_BAUD); bus.begin(MEGA_DRIVE_BAUD, MEGA_RS485_DE); }
#endif

// ---- the PC link ---------------------------------------------------------------------------
#if PC_LINK == LINK_NETWORK
  #if BOARD == BOARD_ESP32
    WiFiUDP udp;
  #else
    EthernetUDP udp;
  #endif
  JMCLinkUDP pcLink(udp);
  #define DEBUG_PRINT(x)  Serial.print(x)               // USB is free for diagnostics
  #define DEBUG_PRINTLN(x) Serial.println(x)
  void startLink() {
    Serial.begin(115200);
    IPAddress boardIp(BOARD_IP_ADDR), pcIp(PC_IP_ADDR);
  #if BOARD == BOARD_ESP32
    WiFi.mode(WIFI_STA);
    WiFi.config(boardIp, IPAddress(WIFI_GATEWAY), IPAddress(WIFI_SUBNET));
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    for (int t = 0; t < 40 && WiFi.status() != WL_CONNECTED; t++) delay(250);
    Serial.println(WiFi.status() == WL_CONNECTED ? F("WiFi connected")
                                                 : F("WiFi not connected yet (retrying in the background)"));
  #else
    byte mac[6] = { BOARD_MAC };
    Ethernet.begin(mac, boardIp);
  #endif
    pcLink.begin(BOARD_UDP_PORT, pcIp, PC_UDP_PORT);
    if (ONLY_ACCEPT_PC) pcLink.allowSender(pcIp);
  }
#else
  JMCLinkSerial pcLink(Serial);
  #define DEBUG_PRINT(x)                                // USB carries the commands
  #define DEBUG_PRINTLN(x)
  void startLink() { Serial.begin(USB_BAUD); }
#endif

JMCController jmc(bus, pcLink);

void setup() {
  startLink();
  startDrives();

  jmc.setMotorCount(MOTOR_COUNT);
  jmc.setMotionDefaults(GLOBAL_ACCEL, GLOBAL_DECEL, GLOBAL_VELOCITY, GLOBAL_HOMING_VEL);
  if (PER_MOTOR_SETTINGS) {
    for (uint8_t i = 0; i < MOTOR_COUNT && i < MOTOR_ROWS; i++) {
      MotorSettings m;
      memcpy_P(&m, &MOTOR[i], sizeof(m));
      uint8_t id = i + 1;
      jmc.setMotorVelocity(id, m.velocity);
      jmc.setMotorAccelDecel(id, m.accel, m.decel);
      jmc.setMotorHomingVelocity(id, m.homingVel);
      jmc.setMotorHomingZeroVelocity(id, m.homingZero);
      jmc.setMotorHomingAccel(id, m.homingAccel);
      jmc.setMotorHomingOffset(id, m.homingOff);
      if (m.softMin < m.softMax) jmc.setMotorSoftLimits(id, m.softMin, m.softMax);
    }
  }
  jmc.setCommandWatchdog(COMMAND_WATCHDOG_MS);
#ifdef DRIVE_P45_WHOLE_RPS
  jmc.setDriveP45(0);
#endif

  uint8_t found = jmc.begin();             // detects motors, sends "Motor ready"
  DEBUG_PRINT(found);
  DEBUG_PRINTLN(F(" motor(s) detected"));
  (void)found;

  if (HARDWARE_WATCHDOG) jmc.enableHardwareWatchdog(4000);
}

void loop() {
  jmc.run();                               // non-blocking: commands, events, watchdogs
}
