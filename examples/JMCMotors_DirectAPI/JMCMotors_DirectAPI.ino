/*
  JMCMotors_DirectAPI - drive a motor from C++ without the command layer
  Part of JMCMotors (MIT) by LAGARI A - https://github.com/ailagari/JMCMotors
  Interactive command reference: https://ailagari.github.io/JMCMotors/

  Uses the two lower layers directly: JMCBus (Modbus-RTU over RS-485) and
  JMCMotor (one drive). No network, no command parser - for a stand-alone
  machine sequence. Compiles on CONTROLLINO, Mega, Uno and ESP32; the bus
  setup below picks the right serial port for the board.

  Every call returns a JMCResult, so a comms failure is never silent.
  Drive parameters: extras/HARDWARE.md (P43 = 0, P40 = 1, P41 = baud).
*/
#include <JMCMotors.h>

#if defined(CONTROLLINO_MAXI) || defined(CONTROLLINO_MEGA)
  JMCBus bus(Serial3);                         // built-in RS-485
  #define DEBUG_PORT Serial
  void startBus() { bus.beginControllino(115200); }
#elif defined(ARDUINO_ARCH_ESP32)
  #if SOC_UART_NUM > 2
    #define RS485_PORT Serial2                 // module on GPIO16/17, DE on GPIO4
  #else
    #define RS485_PORT Serial1                 // ESP32-C3 / C6: pick free GPIOs
  #endif
  JMCBus bus(RS485_PORT);
  #define DEBUG_PORT Serial
  void startBus() { RS485_PORT.begin(115200, SERIAL_8N1, 16, 17); bus.begin(115200, 4); }
#elif defined(__AVR_ATmega2560__)
  JMCBus bus(Serial1);                         // Mega: module on pins 19/18, DE on 7
  #define DEBUG_PORT Serial
  void startBus() { Serial1.begin(115200); bus.begin(115200, 7); }
#else
  #include <SoftwareSerial.h>                  // Uno: module on pins 8/9, DE on 7
  SoftwareSerial rs485(8, 9);
  JMCBus bus(rs485);
  #define DEBUG_PORT Serial
  void startBus() { rs485.begin(38400); bus.begin(38400, 7); }   // drives set to 38400
#endif

JMCMotor motor1(bus, 1);                       // drive with Modbus slave ID 1

const float RUN_SPEED = 0.3;                   // rps (0.3 rps = 18 RPM)
const long  TRAVEL    = 4000;                  // steps (one turn at drive P17 = 4000)

static void report(const __FlashStringHelper* what, JMCResult r) {
  DEBUG_PORT.print(what);
  DEBUG_PORT.print(F(" -> "));
  DEBUG_PORT.println(JMCBus::resultName(r));
}

// Wait until the drive reports "target reached" (status bit 10) or timeout.
static bool waitUntilReached(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  delay(50);                                   // let the drive leave "reached"
  while (millis() - t0 < timeout_ms) {
    if (motor1.isTargetReached()) return true;
    delay(20);
  }
  return false;
}

void setup() {
  DEBUG_PORT.begin(115200);
  startBus();
  report(F("begin"), motor1.begin(0.5, 0.5)); // enable sequence, position mode, ramps
  report(F("zero here"), motor1.setZeroPosition());
}

void loop() {
  report(F("move +"), motor1.moveToPosition(TRAVEL, RUN_SPEED));
  if (!waitUntilReached(10000)) DEBUG_PORT.println(F("timeout waiting for target"));
  DEBUG_PORT.print(F("position = ")); DEBUG_PORT.println(motor1.getPosition());
  delay(1000);

  report(F("move -"), motor1.moveToPosition(-TRAVEL, RUN_SPEED));
  if (!waitUntilReached(20000)) DEBUG_PORT.println(F("timeout waiting for target"));
  DEBUG_PORT.print(F("position = ")); DEBUG_PORT.println(motor1.getPosition());

  JMCMotor::MotorStatus st = motor1.getDetailedStatus();
  if (st.valid && st.errorStatus) {
    JMCMotor::ErrorFlags e = motor1.getErrorFlags();
    DEBUG_PORT.print(F("FAULT 0x1001=0x")); DEBUG_PORT.println(e.raw, HEX);
    report(F("alarm reset"), motor1.alarmReset());
    report(F("re-begin"), motor1.begin(0.5, 0.5));
  }
  if (st.valid && (st.cwLimit || st.ccwLimit)) DEBUG_PORT.println(F("a limit switch is active"));
  delay(1000);
}
