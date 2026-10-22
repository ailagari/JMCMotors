/*
  JMCMotors_DirectAPI - drive a motor from C++ without the UDP layer.
  Part of JMCMotors (MIT) by LAGARI A - https://github.com/ailagari/JMCMotors

  Uses the two lower layers directly: JMCBus (Modbus-RTU over the Controllino
  RS-485 port) and JMCMotor (one drive). No Ethernet, no command parser -
  useful for a stand-alone machine sequence, or for a sketch that keeps the
  UDP controller but needs its own logic too (JMCController::motor(i) hands
  out the same JMCMotor objects).

  Every call returns a JMCResult so a comms failure is never silent.
  Wiring and drive parameters: extras/HARDWARE.md (P43 = 0, P40 = 1, P41 = 7).
*/
#include <JMCMotors.h>

JMCBus   bus(Serial3);                 // Controllino built-in RS-485
JMCMotor motor1(bus, 1);               // drive with Modbus slave ID 1

const float RUN_SPEED = 0.3;           // rps (0.3 rps = 18 RPM)
const long  TRAVEL    = 4000;          // steps (= one turn at drive P17 = 4000)

static void report(const char* what, JMCResult r) {
  Serial.print(what);
  Serial.print(F(" -> "));
  Serial.println(JMCBus::resultName(r));
}

// Block until the drive reports "target reached" (status bit 10) or timeout.
static bool waitUntilReached(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (motor1.isTargetReached()) return true;
    delay(20);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  bus.beginControllino(115200);                  // 115200 baud = drive P41 = 7

  // Enable sequence, position mode, 0.5 rps/s ramps. Fails with TIMEOUT if the
  // drive is not answering (check P43 = 0, slave ID, A/B wiring).
  report("begin", motor1.begin(0.5, 0.5));

  report("zero here", motor1.setZeroPosition());   // current position = 0
}

void loop() {
  // Forward one turn, wait, back to zero, wait, repeat.
  report("move +", motor1.moveToPosition(TRAVEL, RUN_SPEED));
  if (!waitUntilReached(10000)) Serial.println(F("timeout waiting for target"));
  Serial.print(F("position = ")); Serial.println(motor1.getPosition());
  delay(1000);

  report("move 0", motor1.moveToPosition(0, RUN_SPEED));
  if (!waitUntilReached(10000)) Serial.println(F("timeout waiting for target"));
  Serial.print(F("position = ")); Serial.println(motor1.getPosition());

  // Decode the status word once per cycle.
  JMCMotor::MotorStatus st = motor1.getDetailedStatus();
  if (st.valid && st.errorStatus) {
    JMCMotor::ErrorFlags e = motor1.getErrorFlags();
    Serial.print(F("FAULT 0x1001=0x")); Serial.println(e.raw, HEX);
    report("alarm reset", motor1.alarmReset());
    report("re-begin", motor1.begin(0.5, 0.5));
  }
  if (st.valid && (st.cwLimit || st.ccwLimit))
    Serial.println(F("a limit switch is active"));

  delay(1000);
}
