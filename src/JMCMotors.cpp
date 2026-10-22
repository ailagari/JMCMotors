/*==============================================================================
  JMCMotors.cpp  -  implementation (JMCBus + JMCMotor + JMCController)
  Author: LAGARI A <lagariscience@gmail.com>  |  License: MIT
==============================================================================*/
#include "JMCMotors.h"
#include <Controllino.h>
#include <EEPROM.h>
#include <math.h>

//==============================================================================
//  JMCBus
//==============================================================================

// Largest frame we ever build/receive with these drives (write-multiple of
// 4 x 32-bit regs = 25 bytes). 32 gives comfortable head-room.
static const uint8_t JMC_MAX_FRAME = 32;

JMCBus::JMCBus(HardwareSerial& serial) : _serial(serial) {}

void JMCBus::beginControllino(uint32_t baud) {
  _useControllino = true;
  _serial.begin(baud, SERIAL_8N1);       // JMC default: 8 data, 1 stop, no parity
  Controllino_RS485Init();
  Controllino_RS485RxEnable();
  _lastActivityUs = micros();
}

void JMCBus::begin(uint32_t baud, int dePin, int rePin) {
  _useControllino = false;
  _dePin = dePin;
  _rePin = rePin;
  if (_dePin >= 0) pinMode(_dePin, OUTPUT);
  if (_rePin >= 0) pinMode(_rePin, OUTPUT);
  _serial.begin(baud, SERIAL_8N1);
  rxEnable();
  _lastActivityUs = micros();
}

void JMCBus::txEnable() {
  if (_useControllino) { Controllino_RS485TxEnable(); return; }
  if (_rePin >= 0) digitalWrite(_rePin, HIGH);  // disable receiver (RE active-low)
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);  // enable driver
}

void JMCBus::rxEnable() {
  if (_useControllino) { Controllino_RS485RxEnable(); return; }
  if (_dePin >= 0) digitalWrite(_dePin, LOW);
  if (_rePin >= 0) digitalWrite(_rePin, LOW);
}

void JMCBus::interFrameGuard() {
  // Guarantee >= 3.5 char times of bus idle before a new frame.
  uint32_t elapsed = micros() - _lastActivityUs;
  if (elapsed < _gapUs) delayMicroseconds(_gapUs - elapsed);
}

void JMCBus::drainRx() {
  while (_serial.available()) _serial.read();
}

uint16_t JMCBus::crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;   // transmit low byte first, then high byte
}

JMCResult JMCBus::transact(uint8_t* req, uint8_t reqLen,
                           uint8_t* resp, uint8_t expectedRespLen,
                           uint8_t slave, uint8_t funcCode) {
  uint16_t crc = crc16(req, reqLen);
  req[reqLen]     = (uint8_t)(crc & 0xFF);
  req[reqLen + 1] = (uint8_t)(crc >> 8);
  uint8_t totalReq = reqLen + 2;

  uint8_t attempts = _retries + 1;
  JMCResult lastErr = JMC_ERR_TIMEOUT;

  for (uint8_t a = 0; a < attempts; a++) {
    interFrameGuard();
    drainRx();

    txEnable();
    _serial.write(req, totalReq);
    _serial.flush();                 // block until the last bit is on the wire
    delayMicroseconds(50);           // transceiver settle before releasing DE
    rxEnable();
    _lastActivityUs = micros();

    if (slave == 0) {                // broadcast: no response expected
      delayMicroseconds(_gapUs);
      _lastActivityUs = micros();
      return JMC_OK;
    }

    uint8_t idx = 0;
    uint32_t start = millis();
    while (idx < expectedRespLen) {
      if (_serial.available()) {
        resp[idx++] = (uint8_t)_serial.read();
      } else if ((uint16_t)(millis() - start) > _timeoutMs) {
        break;
      }
    }
    _lastActivityUs = micros();

    if (idx < 5)          { lastErr = JMC_ERR_TIMEOUT;  continue; }
    if (resp[0] != slave) { lastErr = JMC_ERR_BADFRAME; continue; }

    if (resp[1] == (funcCode | 0x80)) {          // exception frame
      uint16_t rc = crc16(resp, 3);
      if (((rc & 0xFF) == resp[3]) && ((rc >> 8) == resp[4])) {
        _lastException = resp[2];
        return JMC_ERR_EXCEPTION;
      }
      lastErr = JMC_ERR_CRC;
      continue;
    }

    if (resp[1] != funcCode || idx < expectedRespLen) { lastErr = JMC_ERR_BADFRAME; continue; }

    uint16_t rc = crc16(resp, expectedRespLen - 2);
    if (((rc & 0xFF) != resp[expectedRespLen - 2]) ||
        ((rc >> 8)  != resp[expectedRespLen - 1])) { lastErr = JMC_ERR_CRC; continue; }

    _lastException = 0;
    return JMC_OK;
  }
  return lastErr;
}

JMCResult JMCBus::writeSingle(uint8_t slave, uint16_t reg, uint16_t value) {
  uint8_t req[JMC_MAX_FRAME];
  req[0] = slave;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint8_t resp[JMC_MAX_FRAME];
  return transact(req, 6, resp, (slave == 0 ? 0 : 8), slave, 0x06);
}

JMCResult JMCBus::writeMultiple(uint8_t slave, uint16_t reg,
                                const uint16_t* words, uint8_t count) {
  if (count == 0 || (7 + count * 2 + 2) > JMC_MAX_FRAME) return JMC_ERR_PARAM;
  uint8_t req[JMC_MAX_FRAME];
  req[0] = slave;
  req[1] = 0x10;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = 0x00;
  req[5] = count;
  req[6] = count * 2;
  uint8_t p = 7;
  for (uint8_t i = 0; i < count; i++) {
    req[p++] = (uint8_t)(words[i] >> 8);
    req[p++] = (uint8_t)(words[i] & 0xFF);
  }
  uint8_t resp[JMC_MAX_FRAME];
  return transact(req, p, resp, (slave == 0 ? 0 : 8), slave, 0x10);
}

JMCResult JMCBus::readHolding(uint8_t slave, uint16_t reg,
                              uint16_t* out, uint8_t count) {
  if (slave == 0) return JMC_ERR_PARAM;            // broadcast reads illegal
  if (count == 0 || (3 + count * 2 + 2) > JMC_MAX_FRAME) return JMC_ERR_PARAM;
  uint8_t req[JMC_MAX_FRAME];
  req[0] = slave;
  req[1] = 0x03;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = 0x00;
  req[5] = count;
  uint8_t resp[JMC_MAX_FRAME];
  uint8_t expected = 3 + count * 2 + 2;
  JMCResult r = transact(req, 6, resp, expected, slave, 0x03);
  if (r != JMC_OK) return r;
  if (resp[2] != count * 2) return JMC_ERR_BADFRAME;
  for (uint8_t i = 0; i < count; i++)
    out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
  return JMC_OK;
}

const char* JMCBus::resultName(JMCResult r) {
  switch (r) {
    case JMC_OK:            return "OK";
    case JMC_ERR_TIMEOUT:   return "TIMEOUT";
    case JMC_ERR_CRC:       return "CRC";
    case JMC_ERR_EXCEPTION: return "EXCEPTION";
    case JMC_ERR_BADFRAME:  return "BADFRAME";
    case JMC_ERR_PARAM:     return "PARAM";
    default:                return "?";
  }
}

//==============================================================================
//  JMCMotor
//==============================================================================

// Control-word bit patterns (manual section "0x6040 control word").
static const uint16_t CW_INIT       = 0x0001; // parameter init, brake shut
static const uint16_t CW_BRAKE_OPEN = 0x0003; // drive power, brake open
static const uint16_t CW_ENABLE     = 0x000F; // operation enabled
static const uint16_t CW_SAMPLE_BIT = 0x0010; // bit4: sample position / start homing
static const uint16_t CW_IMMEDIATE  = 0x0020; // bit5: new target takes effect now (position mode) -
                                               // abandons an in-flight move instead of finishing it first
static const uint16_t CW_RELATIVE   = 0x0040; // bit6: relative positioning
static const uint16_t CW_HALT       = 0x010F; // bit8: halt (controlled stop)
static const uint16_t CW_QUICKSTOP  = 0x0002; // bit2: quick stop
static const uint16_t CW_RESET      = 0x0080; // bit7: reset & clear error

JMCMotor::JMCMotor(JMCBus& bus, uint8_t slaveId) : _bus(bus), _slave(slaveId) {}

JMCResult JMCMotor::ctrl(uint16_t value) {
  _last = _bus.writeSingle(_slave, JMCReg::CONTROL, value);
  return _last;
}

// rps -> speed register counts. Default scale 10 per the JMC Modbus manual
// ("setting value is 10 times the actual value"): 0.1 rps resolution, so
// decimal speeds like 0.3 rps are exact (register 3). See setSpeedScale().
int32_t JMCMotor::spdReg(float rps) const {
  int32_t v = (int32_t)lroundf(rps * _speedScale);
  // Guard against a silent no-move: a nonzero requested speed must never
  // round down to 0 counts (target velocity 0 = motor accepts the move but
  // never turns).
  if (v == 0 && rps >  0.001f) v =  1;
  if (v == 0 && rps < -0.001f) v = -1;
  return v;
}

uint16_t JMCMotor::toAccelReg(float rps_s) {
  long v = lroundf(rps_s * 10.0f);        // manual: register = rps/s x 10
  if (v == 0 && rps_s > 0.001f) v = 1;    // never 0 (= instant): min 0.1 rps/s
  if (v < 0)     v = 0;
  if (v > 65535) v = 65535;
  return (uint16_t)v;
}

JMCResult JMCMotor::write32(uint8_t slave, uint16_t reg, int32_t value) {
  uint16_t hi = (uint16_t)((uint32_t)value >> 16);
  uint16_t lo = (uint16_t)((uint32_t)value & 0xFFFF);
  uint16_t words[2];
  if (_formatMSBFirst) { words[0] = hi; words[1] = lo; }   // 0x6000 = 0
  else                 { words[0] = lo; words[1] = hi; }   // 0x6000 = 1
  _last = _bus.writeMultiple(slave, reg, words, 2);
  return _last;
}

JMCResult JMCMotor::read32(uint16_t reg, int32_t& out) {
  uint16_t words[2] = {0, 0};
  _last = _bus.readHolding(_slave, reg, words, 2);
  if (_last != JMC_OK) { out = 0; return _last; }
  uint32_t v;
  if (_formatMSBFirst) v = ((uint32_t)words[0] << 16) | words[1];
  else                 v = ((uint32_t)words[1] << 16) | words[0];
  out = (int32_t)v;
  return JMC_OK;
}

JMCResult JMCMotor::begin(float acceleration, float deceleration) {
  _accel = acceleration;
  _decel = deceleration;

  // Detect 32-bit word order (default 0 = MSB first).
  uint16_t fmt = 0;
  if (_bus.readHolding(_slave, JMCReg::FORMAT, &fmt, 1) == JMC_OK)
    _formatMSBFirst = (fmt == 0);

  ctrl(CW_INIT);
  ctrl(CW_BRAKE_OPEN);
  ctrl(CW_ENABLE);
  setOperationMode(MODE_POSITION);
  setAccelDecel(_accel, _decel);
  // Halt code (0x605D): how control-word bit8 stops the motor. The drive's
  // default 0 is undefined in the manual; 1 = halt on the current
  // deceleration ramp, which is what the controlled stop (B) promises.
  _bus.writeSingle(_slave, JMCReg::HALTCODE, 1);

  // Safe, slow homing defaults (zero speeds make the drive reject homing).
  setHomingAccel(_accel > 0.0f ? _accel : 5.0f);
  write32(_slave, JMCReg::HOME_SPEED,   spdReg(0.3f));
  write32(_slave, JMCReg::HOME_ZEROSPD, spdReg(0.3f));

  getStatusWord();          // presence probe -> sets _last
  return _last;
}

bool JMCMotor::isPresent() {
  getStatusWord();
  return _last == JMC_OK;
}

JMCResult JMCMotor::setAccelDecel(float accel_rps_s, float decel_rps_s) {
  _accel = accel_rps_s;
  _decel = decel_rps_s;
  JMCResult r = _bus.writeSingle(_slave, JMCReg::ACCEL, toAccelReg(accel_rps_s));
  if (r != JMC_OK) { _last = r; return r; }
  _last = _bus.writeSingle(_slave, JMCReg::DECEL, toAccelReg(decel_rps_s));
  // Quick-stop ramp (0x6085) defaults to 0 in the drive, which would make a
  // "fast brake" (quick-stop code 2) undefined - keep it at the normal decel
  // so B:2 always has a sane ramp. Override via bus if a harder stop is wanted.
  _bus.writeSingle(_slave, JMCReg::QS_DECEL, toAccelReg(decel_rps_s));
  return _last;
}

JMCResult JMCMotor::setOperationMode(OperationMode mode) {
  // Switching 0x6060 while the shaft is still turning under velocity mode is
  // the dead-stop case: the drive drops the velocity setpoint the instant the
  // mode changes. Brake on the decel ramp first, then switch at standstill.
  if (mode != MODE_VELOCITY) stopVelocityMode();
  enable();
  _last = _bus.writeSingle(_slave, JMCReg::MODE, (uint16_t)mode);
  return _last;
}

JMCResult JMCMotor::loosenBrake() { return ctrl(CW_BRAKE_OPEN); }
JMCResult JMCMotor::enable()      { return ctrl(CW_ENABLE); }
JMCResult JMCMotor::alarmReset()  { return ctrl(CW_RESET); }

JMCResult JMCMotor::moveToPosition(long steps, float velocity_rps, bool absolute) {
  JMCResult r = preparePositionMove(steps, velocity_rps, absolute);
  if (r != JMC_OK) return r;
  r = enableMovement();
  if (r != JMC_OK) return r;
  return startMovement();
}

JMCResult JMCMotor::preparePositionMove(long steps, float velocity_rps, bool absolute) {
  setOperationMode(MODE_POSITION);
  _moveBase = (absolute ? CW_ENABLE : (CW_ENABLE | CW_RELATIVE)) | CW_IMMEDIATE;
  // Relative moves use drive-side bit6 only - adding the current position in
  // software as well was the classic double-count bug.
  write32(_slave, JMCReg::TARGET_VEL, spdReg(fabs(velocity_rps)));
  write32(_slave, JMCReg::TARGET_POS, (int32_t)steps);
  return _last;
}

// Manual, position mode: the new target is acquired on the bit4 0->1 edge
// only while status bit12 ("can set new position") is 1. An idle drive has
// it set; one that is still latching the previous set-point may not, so wait
// briefly for it rather than losing the move. Bounded at ~100 ms.
JMCResult JMCMotor::enableMovement() {
  for (uint8_t k = 0; k < 20; k++) {
    uint16_t s = getStatusWord();
    if (_last != JMC_OK || (s & 0x1000)) break;
    delay(5);
  }
  return ctrl(_moveBase | CW_SAMPLE_BIT);
}
JMCResult JMCMotor::startMovement()  { return ctrl(_moveBase); }

long JMCMotor::getPosition() {
  int32_t v = 0;
  read32(JMCReg::POS_ACTUAL, v);
  return (long)v;
}

JMCResult JMCMotor::setVelocity(float velocity_rps) {
  setOperationMode(MODE_VELOCITY);
  setTargetVelocity(velocity_rps);
  _velRunning = true;
  return enable();
}

JMCResult JMCMotor::setTargetVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::TARGET_VEL, spdReg(velocity_rps));
}

// Manual, velocity mode routine: halt first (0x010F), then select the mode
// and write the speed, then release halt (0x000F) to start. Entering velocity
// mode while enabled would start the shaft at whatever 0x6081 still holds
// (the last profile speed) before the jog speed is written - and it would
// start at the mode switch instead of at the synchronised launch.
JMCResult JMCMotor::prepareVelocityMode(float velocity_rps) {
  ctrl(CW_HALT);                                            // hold while we set up
  _last = _bus.writeSingle(_slave, JMCReg::MODE, (uint16_t)MODE_VELOCITY);
  setTargetVelocity(velocity_rps);
  _moveBase = CW_ENABLE;                                    // start = release halt
  _velRunning = true;
  return _last;
}

// ---- velocity -> position/homing handoff ------------------------------------
// Bit8 (halt) brakes on the normal decel ramp and holds the shaft; the drive
// stays in velocity mode meanwhile, so no setpoint is dropped mid-spin. Once
// the shaft is at rest the mode switch is harmless.
JMCResult JMCMotor::beginVelocityRampDown() {
  float v = fabs(getVelocity());
  if (_last != JMC_OK) v = 0.0f;                // no reply: don't wait on it
  float d = (_decel > 0.01f) ? _decel : 1.0f;   // rps/s
  long ms = lroundf(v / d * 1000.0f) + 250;     // ramp time + drive reaction margin
  if (ms < 150)   ms = 150;
  if (ms > 20000) ms = 20000;                   // 2 rps at AD:0.1 is already 20 s
  _rampMs = (uint16_t)ms;
  setTargetVelocity(0.0f);                      // nothing to resume into
  return ctrl(CW_HALT);
}

bool JMCMotor::isSpinning(float eps_rps) {
  float v = getVelocity();
  if (_last != JMC_OK) return false;            // unreachable drive: stop waiting
  return fabs(v) > eps_rps;
}

// Halt must be released again or the position move that follows never starts.
JMCResult JMCMotor::endVelocityRampDown() {
  _velRunning = false;
  _moveBase   = CW_ENABLE;
  return ctrl(CW_ENABLE);
}

// Blocking form. The caller's loop is stuck here for the whole brake, so the
// wait is capped well below the ramp estimate: a slow AD decel from a fast jog
// would otherwise freeze the controller for many seconds. Stop with B before
// homing or re-zeroing a fast jog if the ramp is longer than that.
JMCResult JMCMotor::stopVelocityMode(uint16_t timeout_ms) {
  if (!_velRunning) return _last;
  beginVelocityRampDown();
  uint16_t limit = timeout_ms ? timeout_ms : _rampMs;
  if (!timeout_ms && limit > BLOCKING_RAMP_CAP_MS) limit = BLOCKING_RAMP_CAP_MS;
  uint32_t t0 = millis();
  while ((millis() - t0) < limit) {
    if (!isSpinning()) break;
    delay(5);
  }
  return endVelocityRampDown();
}

float JMCMotor::getVelocity() {
  int32_t v = 0;
  read32(JMCReg::VEL_ACTUAL, v);
  return v / _speedScale;         // same unit scale as the target registers
}

JMCResult JMCMotor::setHomingVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::HOME_SPEED, spdReg(fabs(velocity_rps)));
}

JMCResult JMCMotor::setHomingZeroVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::HOME_ZEROSPD, spdReg(fabs(velocity_rps)));
}

JMCResult JMCMotor::setHomingAccel(float accel_rps_s) {
  _last = _bus.writeSingle(_slave, JMCReg::HOME_ACCEL, toAccelReg(accel_rps_s));
  return _last;
}

JMCResult JMCMotor::setHomingOffset(long steps) {
  return write32(_slave, JMCReg::HOME_OFFSET, (int32_t)steps);
}

JMCResult JMCMotor::performHoming(uint8_t method) {
  if (method > 12) { _last = JMC_ERR_PARAM; return _last; }
  setOperationMode(MODE_HOMING);
  _last = _bus.writeSingle(_slave, JMCReg::HOME_METHOD, method);
  if (_last != JMC_OK) return _last;
  ctrl(CW_ENABLE);                          // ensure enabled, bit4 low
  return ctrl(CW_ENABLE | CW_SAMPLE_BIT);   // bit4 0->1 starts homing
}

JMCResult JMCMotor::setZeroPosition() { return setCurrentPosition(0); }

// Redefine the position counter without any movement. Homing method 0 declares
// the current residence the origin, and the home offset (0x607C) shifts the
// readout, so afterwards the current physical position reads <value>.
JMCResult JMCMotor::setCurrentPosition(long value) {
  setOperationMode(MODE_HOMING);
  _bus.writeSingle(_slave, JMCReg::HOME_METHOD, 0);   // method 0: no movement
  write32(_slave, JMCReg::HOME_OFFSET, (int32_t)value);
  ctrl(CW_ENABLE);
  return ctrl(CW_ENABLE | CW_SAMPLE_BIT);             // bit4 0->1 executes it
}

JMCResult JMCMotor::stop() {
  // Braked -> a later mode switch is safe. Zero the jog setpoint too, so a
  // stopped jog cannot pick its old speed back up when halt is released.
  if (_velRunning) { _velRunning = false; setTargetVelocity(0.0f); }
  return ctrl(CW_HALT);
}

JMCResult JMCMotor::quickStop(QuickStopMode mode) {
  _velRunning = false;
  _bus.writeSingle(_slave, JMCReg::QUICKSTOP, (uint16_t)mode);  // how to stop
  return ctrl(CW_QUICKSTOP);                                    // bit2: do it
}

JMCResult JMCMotor::eStop() { return quickStop(QUICK_STOP_IMMEDIATE); }

uint16_t JMCMotor::getStatusWord() {
  uint16_t s = 0;
  _last = _bus.readHolding(_slave, JMCReg::STATUS, &s, 1);
  return (_last == JMC_OK) ? s : 0;
}

JMCMotor::MotorStatus JMCMotor::getDetailedStatus() {
  MotorStatus st;
  uint16_t s = getStatusWord();
  st.valid = (_last == JMC_OK);
  st.readyToInit        = s & 0x0001;
  st.initComplete       = s & 0x0002;
  st.motorEnabled       = s & 0x0004;
  st.errorStatus        = s & 0x0008;
  st.driveWorking       = s & 0x0010;
  st.quickStopActive    = s & 0x0020;
  st.initState          = s & 0x0040;
  st.warning            = s & 0x0080;
  st.motorHalt          = s & 0x0100;
  st.motorRunning       = s & 0x0200;
  st.targetReached      = s & 0x0400;
  st.swMechanicalOrigin = s & 0x0800;
  st.homingComplete     = s & 0x1000;
  st.overPositionError  = s & 0x2000;
  st.cwLimit            = s & 0x4000;
  st.ccwLimit           = s & 0x8000;
  return st;
}

JMCMotor::ErrorFlags JMCMotor::getErrorFlags() {
  ErrorFlags e;
  uint16_t r = 0;
  _last = _bus.readHolding(_slave, JMCReg::ERROR, &r, 1);
  e.valid         = (_last == JMC_OK);
  e.raw           = r;
  e.generic       = r & 0x0001;
  e.current       = r & 0x0002;
  e.voltage       = r & 0x0004;
  e.temperature   = r & 0x0008;
  e.communication = r & 0x0010;
  e.overPosition  = r & 0x0020;
  e.phaseLoss     = r & 0x0080;
  return e;
}

bool JMCMotor::isRunning()       { uint16_t s = getStatusWord(); return (_last == JMC_OK) && (s & 0x0200); }
bool JMCMotor::isTargetReached() { uint16_t s = getStatusWord(); return (_last == JMC_OK) && (s & 0x0400); }
bool JMCMotor::hasFault()        { uint16_t s = getStatusWord(); return (_last == JMC_OK) && (s & 0x0008); }

JMCResult JMCMotor::broadcastPrepareMove(long steps, float velocity_rps, bool absolute) {
  _bus.writeSingle(0, JMCReg::MODE, MODE_POSITION);
  _moveBase = (absolute ? CW_ENABLE : (CW_ENABLE | CW_RELATIVE)) | CW_IMMEDIATE;
  write32(0, JMCReg::TARGET_VEL, spdReg(fabs(velocity_rps)));
  write32(0, JMCReg::TARGET_POS, (int32_t)steps);
  return _last;
}

JMCResult JMCMotor::broadcastSetVelocity(float velocity_rps) {
  return write32(0, JMCReg::TARGET_VEL, spdReg(velocity_rps));
}

JMCResult JMCMotor::broadcastEnableMovement() {
  _last = _bus.writeSingle(0, JMCReg::CONTROL, _moveBase | CW_SAMPLE_BIT);
  return _last;
}

JMCResult JMCMotor::broadcastStartMovement() {
  _last = _bus.writeSingle(0, JMCReg::CONTROL, _moveBase);
  return _last;
}

JMCResult JMCMotor::broadcastStop() {
  _last = _bus.writeSingle(0, JMCReg::CONTROL, CW_HALT);
  return _last;
}

//==============================================================================
//  JMCController - status decode helpers (file-local)
//==============================================================================

// Drive state name per the manual's status-word state table.
static const char* driveStateName(uint16_t s) {
  switch (s & 0x000F) {
    case 0x0: return "Uninitialized";
    case 0x1: return "Initialized";
    case 0x3: return "Power on";
    case 0x7: return (s & 0x0020) ? "Device enable" : "Quick stop active";
    case 0xF: return "Fault alarm";
    case 0x8: return "Error state";
    default:  return "Unknown";
  }
}

// Status-word flag names. Bits 10, 12 and 13 depend on the operation mode
// (manual, 0x6041 table): in position mode bit12 = "can set new position",
// in velocity mode = "velocity is 0", in homing mode = "homing complete";
// bit13 = over-position error / max acceleration / homing error.
static String flagsString(uint16_t s, uint8_t mode) {
  const char* names[16] = {
    "READY","INIT","ENABLED","FAULT","WORKING","QSTOP","INITSTATE","WARN",
    "HALT","RUN","REACHED","SWORIGIN","SETPOINT_OK","POSERR","CWLIM","CCWLIM"
  };
  if (mode == JMCMotor::MODE_VELOCITY) {
    names[10] = "AT_SPEED"; names[12] = "VEL_ZERO"; names[13] = "MAX_ACCEL";
  } else if (mode == JMCMotor::MODE_HOMING) {
    names[10] = "HOME_REACHED"; names[12] = "HOMED"; names[13] = "HOME_ERROR";
  }
  String out = "";
  for (int b = 0; b < 16; b++)
    if (s & (1 << b)) { if (out.length()) out += "|"; out += names[b]; }
  return out.length() ? out : String("NONE");
}

static const char* modeName(uint8_t mode) {
  switch (mode) {
    case JMCMotor::MODE_POSITION: return "position";
    case JMCMotor::MODE_VELOCITY: return "velocity";
    case JMCMotor::MODE_HOMING:   return "homing";
    default:                      return "unknown";
  }
}

static String bitsString(uint16_t s) {
  String out = "";
  for (int b = 15; b >= 0; b--) {
    out += (s & (1 << b)) ? "1" : "0";
    if (b == 12 || b == 8 || b == 4) out += " ";
  }
  return out;
}

//==============================================================================
//  JMCController - lifecycle
//==============================================================================

JMCController::JMCController(HardwareSerial& rs485Serial) : _bus(rs485Serial) {
  // V3 network defaults; override with setNetwork().
  const byte defMac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  memcpy(_mac, defMac, 6);
  _localIp = IPAddress(192, 168, 1, 101);
  _pcIp    = IPAddress(192, 168, 1, 100);
  for (uint8_t i = 0; i < JMC_MAX_MOTORS; i++) {
    _m[i] = nullptr;
    _storedVel[i]     = NAN;   // NAN = "not hardcoded" -> default applied at init
    _homingVel[i]     = NAN;
    _homingZeroVel[i] = NAN;   // NAN = follow HV
    _homingAccel[i]   = NAN;   // NAN = follow accel
    _accelSet[i]      = NAN;
    _decelSet[i]      = NAN;
    _homingOffset[i]  = 0;
    _offlineTry[i]    = 0;
    memset(&_cache[i], 0, sizeof(MotorCache));
    _cache[i].mode = JMCMotor::MODE_POSITION;
  }
}

void JMCController::setNetwork(const byte* mac,
                               IPAddress localIp, uint16_t localPort,
                               IPAddress pcIp,    uint16_t pcPort) {
  memcpy(_mac, mac, 6);
  _localIp   = localIp;
  _localPort = localPort;
  _pcIp      = pcIp;
  _pcPort    = pcPort;
}

void JMCController::setStationId(uint8_t id, IPAddress ipBase) {
  _stationId = id;
  // One number -> unique MAC + IP: flashing a fleet means changing one digit
  // per board, and duplicate addresses cannot happen by mistake.
  const byte mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, id };
  memcpy(_mac, mac, 6);
  _localIp = IPAddress(ipBase[0], ipBase[1], ipBase[2],
                       (uint8_t)(ipBase[3] + id));
}

void JMCController::setMotorCount(uint8_t count) {
  if (count >= 1 && count <= JMC_MAX_MOTORS) _count = count;
}

void JMCController::setMotionDefaults(float accel_rps_s, float decel_rps_s,
                                      float velocity_rps, float homingVel_rps) {
  _defAccel     = accel_rps_s;
  _defDecel     = decel_rps_s;
  _defVel       = velocity_rps;
  _defHomingVel = homingVel_rps;
}

void JMCController::setDriveP45(uint8_t p45) {
  // Default (1): register = rps x 10 per the Modbus manual - decimals work.
  // setDriveP45(0): whole-rps units (scale 1) - use only if a motor
  // measurably runs 10x faster than commanded.
  _speedScale = (p45 == 0) ? 1.0f : 10.0f;
  for (uint8_t i = 0; i < JMC_MAX_MOTORS; i++)
    if (_m[i]) _m[i]->setSpeedScale(_speedScale);
}

// ---- Per-motor hardcoded settings (motor IDs are 1-based) -------------------
// A value <= 0 (or NAN) means "not set": before begin() the global default
// is applied at init (NAN sentinel); after begin() it is applied right away.
void JMCController::setMotorVelocity(uint8_t motorId, float rps) {
  if (motorId < 1 || motorId > JMC_MAX_MOTORS) return;
  uint8_t i = motorId - 1;
  _storedVel[i] = (rps > 0.0f) ? rps : (_m[i] ? _defVel : NAN);
}

void JMCController::setMotorHomingVelocity(uint8_t motorId, float rps) {
  if (motorId < 1 || motorId > JMC_MAX_MOTORS) return;
  uint8_t i = motorId - 1;
  if (!(rps > 0.0f)) { if (!_m[i]) { _homingVel[i] = NAN; return; } rps = _defHomingVel; }
  _homingVel[i] = rps;
  if (_m[i] && _cache[i].online) applyHomingParams(i);
}

// Slow zero-find speed (0x609B). 0 / NAN = same as HV (the default).
void JMCController::setMotorHomingZeroVelocity(uint8_t motorId, float rps) {
  if (motorId < 1 || motorId > JMC_MAX_MOTORS) return;
  uint8_t i = motorId - 1;
  _homingZeroVel[i] = (rps > 0.0f) ? rps : NAN;
  if (_m[i] && _cache[i].online) applyHomingParams(i);
}

// Homing ramp (0x609A). 0 / NAN = same as the motor's acceleration (default).
void JMCController::setMotorHomingAccel(uint8_t motorId, float rps_s) {
  if (motorId < 1 || motorId > JMC_MAX_MOTORS) return;
  uint8_t i = motorId - 1;
  _homingAccel[i] = (rps_s > 0.0f) ? rps_s : NAN;
  if (_m[i] && _cache[i].online) applyHomingParams(i);
}

void JMCController::setMotorHomingOffset(uint8_t motorId, long steps) {
  if (motorId < 1 || motorId > JMC_MAX_MOTORS) return;
  uint8_t i = motorId - 1;
  _homingOffset[i] = steps;
  if (_m[i] && _cache[i].online) _m[i]->setHomingOffset(steps);
}

void JMCController::setMotorAccelDecel(uint8_t motorId, float accel_rps_s, float decel_rps_s) {
  if (motorId < 1 || motorId > JMC_MAX_MOTORS) return;
  uint8_t i = motorId - 1;
  if (!(accel_rps_s > 0.0f) || !(decel_rps_s > 0.0f)) {   // 0 = instant ramp: refuse
    if (!_m[i]) { _accelSet[i] = NAN; _decelSet[i] = NAN; return; }
    accel_rps_s = _defAccel; decel_rps_s = _defDecel;
  }
  _accelSet[i] = accel_rps_s;
  _decelSet[i] = decel_rps_s;
  if (_m[i] && _cache[i].online) _m[i]->setAccelDecel(accel_rps_s, decel_rps_s);
}

// Write motor i's homing parameters to its drive: search speed (HV), zero-find
// speed (HVZ, follows HV when unset), homing ramp (HACC, follows accel when
// unset) and the offset (HO). Called at init, re-init and before every H<n>.
void JMCController::applyHomingParams(uint8_t i) {
  if (!_m[i]) return;
  float hv   = isnan(_homingVel[i])     ? _defHomingVel : _homingVel[i];
  float hvz  = isnan(_homingZeroVel[i]) ? hv            : _homingZeroVel[i];
  float hacc = isnan(_homingAccel[i])   ? (isnan(_accelSet[i]) ? _defAccel : _accelSet[i])
                                        : _homingAccel[i];
  _m[i]->setHomingVelocity(hv);
  _m[i]->setHomingZeroVelocity(hvz);
  _m[i]->setHomingAccel(hacc);
  _m[i]->setHomingOffset(_homingOffset[i]);
}

//==============================================================================
//  JMCController - saved settings (EEPROM)
//==============================================================================
// Layout at EEPROM address 0:
//   SavedHeader { magic 'JMC5', layout version, motor count }
//   SavedMotor x count { vel, acc, dec, hv, hvz, hacc, hoff }   (NAN = "not set")
//   uint16 CRC-16 (Modbus polynomial) over header + motors
// 32 motors need 6 + 32*28 + 2 = 904 bytes; the ATmega2560 has 4096.

// Policy: the sketch's values are the admin's configuration. A SAVEd block is
// for changes made over UDP on a deployed unit (no USB needed) and carries a
// fingerprint of the sketch values that were in force when it was saved. At
// boot the block is applied only while that fingerprint still matches - the
// moment the admin re-flashes with different values, the sketch wins again
// and the block is ignored (SAVE:? says so; LOAD applies it anyway).
namespace {
  struct SavedHeader { uint32_t magic; uint8_t version; uint8_t count; uint16_t sketchCrc; };
  struct SavedMotor  { float vel, acc, dec, hv, hvz, hacc; long hoff; };
  const uint32_t SAVE_MAGIC   = 0x4A4D4335UL;   // 'JMC5'
  const uint8_t  SAVE_VERSION = 2;
  const int      SAVE_BASE    = 0;

  uint16_t crcStep(uint16_t crc, uint8_t b) {   // Modbus CRC-16, one byte
    crc ^= b;
    for (uint8_t k = 0; k < 8; k++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    return crc;
  }
  template <typename T> uint16_t crcPut(int& addr, const T& v, uint16_t crc) {
    const uint8_t* p = (const uint8_t*)&v;
    for (size_t k = 0; k < sizeof(T); k++) { EEPROM.update(addr++, p[k]); crc = crcStep(crc, p[k]); }
    return crc;
  }
  template <typename T> uint16_t crcGet(int& addr, T& v, uint16_t crc) {
    uint8_t* p = (uint8_t*)&v;
    for (size_t k = 0; k < sizeof(T); k++) { p[k] = EEPROM.read(addr++); crc = crcStep(crc, p[k]); }
    return crc;
  }
}

// CRC over everything the sketch configured (count, defaults, per-motor
// values) - computed once in begin(), before any saved block is applied.
uint16_t JMCController::sketchFingerprint() {
  uint16_t crc = 0xFFFF;
  const uint8_t* p;
  auto feed = [&](const void* v, size_t n) {
    p = (const uint8_t*)v;
    for (size_t k = 0; k < n; k++) crc = crcStep(crc, p[k]);
  };
  feed(&_count, 1);
  feed(&_defAccel, sizeof(float)); feed(&_defDecel, sizeof(float));
  feed(&_defVel, sizeof(float));   feed(&_defHomingVel, sizeof(float));
  feed(&_speedScale, sizeof(float));
  for (uint8_t i = 0; i < _count; i++) {
    feed(&_storedVel[i], sizeof(float));     feed(&_accelSet[i], sizeof(float));
    feed(&_decelSet[i], sizeof(float));      feed(&_homingVel[i], sizeof(float));
    feed(&_homingZeroVel[i], sizeof(float)); feed(&_homingAccel[i], sizeof(float));
    feed(&_homingOffset[i], sizeof(long));
  }
  return crc;
}

bool JMCController::saveSettings() {
  int needed = (int)sizeof(SavedHeader) + _count * (int)sizeof(SavedMotor) + 2;
  if ((int)EEPROM.length() < SAVE_BASE + needed) return false;
  int addr = SAVE_BASE;
  uint16_t crc = 0xFFFF;
  SavedHeader h = { SAVE_MAGIC, SAVE_VERSION, _count, _sketchCrc };
  crc = crcPut(addr, h, crc);
  for (uint8_t i = 0; i < _count; i++) {
    SavedMotor m = { _storedVel[i], _accelSet[i], _decelSet[i], _homingVel[i],
                     _homingZeroVel[i], _homingAccel[i], _homingOffset[i] };
    crc = crcPut(addr, m, crc);
  }
  uint16_t stored = crc;
  crcPut(addr, stored, 0);
  return true;
}

bool JMCController::hasSaved() {
  SavedHeader h; int addr = SAVE_BASE;
  crcGet(addr, h, 0xFFFF);
  return h.magic == SAVE_MAGIC && h.version == SAVE_VERSION &&
         h.count >= 1 && h.count <= JMC_MAX_MOTORS;
}

// True when the stored block was saved under the sketch values now flashed.
bool JMCController::savedMatchesSketch() {
  SavedHeader h; int addr = SAVE_BASE;
  crcGet(addr, h, 0xFFFF);
  return hasSaved() && h.sketchCrc == _sketchCrc;
}

bool JMCController::loadSettings(bool force) {
  SavedHeader h; int addr = SAVE_BASE;
  uint16_t crc = crcGet(addr, h, 0xFFFF);
  if (h.magic != SAVE_MAGIC || h.version != SAVE_VERSION ||
      h.count < 1 || h.count > JMC_MAX_MOTORS) return false;
  if (!force && h.sketchCrc != _sketchCrc) return false;   // re-flashed: sketch wins
  SavedMotor m[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < h.count; i++) crc = crcGet(addr, m[i], crc);
  uint16_t stored; crcGet(addr, stored, 0);
  if (stored != crc) return false;

  _count = h.count;
  if (_monIdx >= _count) _monIdx = -1;
  for (uint8_t i = 0; i < h.count; i++) {
    _storedVel[i]     = m[i].vel;
    _accelSet[i]      = m[i].acc;
    _decelSet[i]      = m[i].dec;
    _homingVel[i]     = m[i].hv;
    _homingZeroVel[i] = m[i].hvz;
    _homingAccel[i]   = m[i].hacc;
    _homingOffset[i]  = m[i].hoff;
    if (_m[i] && _cache[i].online) {           // running: push to the drives now
      if (isnan(_storedVel[i])) _storedVel[i] = _defVel;
      if (isnan(_homingVel[i])) _homingVel[i] = _defHomingVel;
      if (isnan(_accelSet[i]))  _accelSet[i]  = _defAccel;
      if (isnan(_decelSet[i]))  _decelSet[i]  = _defDecel;
      _m[i]->setAccelDecel(_accelSet[i], _decelSet[i]);
      applyHomingParams(i);
    }
  }
  return true;
}

void JMCController::clearSaved() {
  for (size_t k = 0; k < sizeof(SavedHeader); k++) EEPROM.update(SAVE_BASE + k, 0xFF);
}

uint8_t JMCController::begin(uint32_t baud) {
  _bus.beginControllino(baud);
  // Short per-attempt timeouts so a dead drive can't stall the loop long.
  _bus.setRetries(1);
  _bus.setResponseTimeout(30);

  Ethernet.begin(_mac, _localIp);
  _udp.begin(_localPort);

  // Sketch values are the admin's configuration. A block SAVEd over UDP on
  // the deployed unit is applied on top - but only while the sketch values
  // it was saved under are still the ones flashed (see sketchFingerprint).
  _sketchCrc = sketchFingerprint();
  if (Serial) {
    if (loadSettings(false)) Serial.println(F("JMC: SAVEd settings applied (sketch unchanged since SAVE)"));
    else if (hasSaved())     Serial.println(F("JMC: SAVEd block ignored - sketch values changed; using sketch (LOAD applies it)"));
    else                     Serial.println(F("JMC: sketch settings in use (nothing SAVEd)"));
  } else {
    loadSettings(false);
  }

  uint8_t found = 0;
  for (uint8_t i = 0; i < _count; i++) {
    bool ok = initMotorSlot(i);
    if (ok) found++;
    if (Serial) {
      Serial.print(F("JMC motor "));
      Serial.print(i + 1);
      Serial.print(F(" -> "));
      Serial.println(JMCBus::resultName(_m[i]->lastResult()));
    }
  }
  // Tell the configured PC initialisation is done and the motors are up.
  reply(F("Motor ready"));
  return found;
}

// Allocate (if needed), fast-probe and initialise one motor slot.
// The fast presence probe keeps boot quick even with many empty slots.
bool JMCController::initMotorSlot(uint8_t i) {
  if (!_m[i]) _m[i] = new JMCMotor(_bus, i + 1);   // slave IDs 1..count
  // Resolve "not hardcoded" sentinels to the global defaults.
  if (isnan(_storedVel[i])) _storedVel[i] = _defVel;
  if (isnan(_homingVel[i])) _homingVel[i] = _defHomingVel;
  if (isnan(_accelSet[i]))  _accelSet[i]  = _defAccel;
  if (isnan(_decelSet[i]))  _decelSet[i]  = _defDecel;
  _m[i]->setSpeedScale(_speedScale);               // match drive P45 unit

  _bus.setResponseTimeout(PROBE_TIMEOUT_MS);
  _bus.setRetries(0);
  bool present = _m[i]->isPresent();
  _bus.setResponseTimeout(30);
  _bus.setRetries(1);

  JMCResult r = JMC_ERR_TIMEOUT;
  if (present) {
    r = _m[i]->begin(_accelSet[i], _decelSet[i]);  // per-motor ramps
    applyHomingParams(i);                          // HV / HVZ / HACC / HO
  }
  _cache[i].online  = (r == JMC_OK);
  _cache[i].failCnt = 0;
  // Fresh init: position mode, no homing run completed yet.
  _cache[i].mode        = JMCMotor::MODE_POSITION;
  _cache[i].homed       = false;
  _cache[i].homingArmed = false;
  _cache[i].homeErr     = false;
  if (r == JMC_OK) {
    uint16_t s = _m[i]->getStatusWord();
    _cache[i].status = s;
    _cache[i].cw     = s & 0x4000;
    _cache[i].ccw    = s & 0x8000;
    _cache[i].sw     = s & 0x0800;
    _cache[i].fault  = s & 0x0008;
  }
  return r == JMC_OK;
}

// ---- mode bookkeeping --------------------------------------------------------
void JMCController::noteModeOne(uint8_t i, uint8_t mode) {
  _cache[i].mode = mode;
  if (mode != JMCMotor::MODE_HOMING) _cache[i].homingArmed = false;
}

void JMCController::noteMode(const bool* sel, uint8_t mode) {
  for (uint8_t i = 0; i < _count; i++)
    if (sel[i]) noteModeOne(i, mode);
}

void JMCController::run() {
  handleUdp();          // always first: commands take priority
  servicePendingMove(); // launch a move once the jog it interrupted has braked
  serviceMonitor();     // one cheap status read per tick + event push
}

JMCMotor* JMCController::motor(uint8_t index) {
  return (index < _count) ? _m[index] : nullptr;
}

//==============================================================================
//  JMCController - comms & parsing helpers
//==============================================================================

void JMCController::sendTo(IPAddress ip, uint16_t port, const String& msg) {
  _udp.beginPacket(ip, port);
  _udp.write(msg.c_str());
  _udp.endPacket();
}

// Command replies go to the fixed PC and, if different, to the sender of the
// last command - so a tool sending from a random port still gets its answer.
void JMCController::reply(const String& msg) {
  sendTo(_pcIp, _pcPort, msg);
  if (_senderPort && (!(_senderIp == _pcIp) || _senderPort != _pcPort))
    sendTo(_senderIp, _senderPort, msg);
}

// Multi-packet replies: every packet but the last ends with ":MORE", the last
// with ":END", so the PC knows when a list is complete. A single-packet reply
// carries no marker.
void JMCController::replyMulti(const String& packet, bool last) {
  reply(packet + (last ? F(":END") : F(":MORE")));
}

// Send 'out' now if it has grown past REPLY_CHUNK_CHARS and restart it with
// the header, so per-motor lists stay bounded whatever the motor count.
bool JMCController::flushIfLong(String& out, const String& header) {
  if (out.length() < REPLY_CHUNK_CHARS) return false;
  reply(out + F(":MORE"));
  _chunked = true;
  out = header;
  return true;
}

String JMCController::replyChunkEnd(const String& out) {
  if (!_chunked) return out;
  _chunked = false;
  return out + F(":END");
}

// ---- argument grammar (v0.5) --------------------------------------------------
// Individual form   CMD:<fields>
//     positional slots: field N = motor N, an empty field skips that motor
//     ("1000,,3000" = M1 and M3); a single value with no comma = motor 1;
//     or motor names: "M3,M17" (selectors) / "M3=500,M17=1000" (values),
//     "M3" or "M3=?" in a value command = read motor 3.
// All-motors form   CMDA[:value]  (the A suffix) - one value for every motor.
// '?' in a value field reads that motor instead of writing.

// "M3" -> 2 ; anything else -> -1
static int motorIndexOf(const String& name, uint8_t count) {
  if (name.length() < 2 || (name[0] != 'M' && name[0] != 'm')) return -1;
  for (unsigned k = 1; k < name.length(); k++) if (!isDigit(name[k])) return -1;
  int n = name.substring(1).toInt();
  return (n >= 1 && n <= count) ? n - 1 : -1;
}

static void setField(JMCController::Fields& Fv, uint8_t i, const String& v) {
  Fv.active[i] = true; Fv.n++;
  if (v == "?") { Fv.query[i] = true; Fv.nq++; Fv.l[i] = 0; Fv.f[i] = 0; }
  else { Fv.query[i] = false; Fv.l[i] = atol(v.c_str()); Fv.f[i] = v.toFloat(); }
}

// Returns the number of addressed motors, 0 = nothing given, -1 = bad syntax.
int JMCController::parseValues(const String& csv, Fields& Fv, bool all) {
  memset(&Fv, 0, sizeof(Fv));
  String s = csv; s.trim();
  if (!s.length()) return 0;

  if (all) {                                   // CMDA:value - one value for everyone
    if (s.indexOf(',') >= 0) return -1;
    for (uint8_t i = 0; i < _count; i++) setField(Fv, i, s);
    return _count;
  }
  if (s[0] == 'M' || s[0] == 'm') {            // M-pairs: M3=500,M17=1000 / M3=?
    String work = s + F(",");
    int start = 0;
    while (start < (int)work.length()) {
      int comma = work.indexOf(',', start);
      String part = work.substring(start, comma); part.trim();
      start = comma + 1;
      if (!part.length()) continue;
      int eq = part.indexOf('=');
      String name = (eq < 0) ? part : part.substring(0, eq); name.trim();
      String val  = (eq < 0) ? String(F("?")) : part.substring(eq + 1); val.trim();
      int idx = motorIndexOf(name, _count);
      if (idx < 0 || !val.length()) return -1;
      if (!Fv.active[idx]) setField(Fv, (uint8_t)idx, val);
    }
    return Fv.n;
  }
  if (s.indexOf(',') < 0) { setField(Fv, 0, s); return 1; }   // single value = motor 1
  String work = s + F(",");                     // positional slots
  int start = 0;
  for (uint8_t i = 0; i < _count; i++) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    start = comma + 1;
    if (part.length()) setField(Fv, i, part);
  }
  return Fv.n;
}

// Selectors: "1,,1" (slots) or "M1,M3" (names); all = every motor.
// Returns the count selected, 0 = nothing given, -1 = bad motor name.
int JMCController::parseSelection(const String& csv, bool* sel, bool all) {
  for (uint8_t i = 0; i < _count; i++) sel[i] = false;
  if (all) { for (uint8_t i = 0; i < _count; i++) sel[i] = true; return _count; }
  String s = csv; s.trim();
  if (!s.length()) return 0;
  int n = 0;
  String work = s + F(",");
  int start = 0;
  if (s[0] == 'M' || s[0] == 'm') {
    while (start < (int)work.length()) {
      int comma = work.indexOf(',', start);
      String part = work.substring(start, comma); part.trim();
      start = comma + 1;
      if (!part.length()) continue;
      int eq = part.indexOf('=');
      if (eq >= 0) part = part.substring(0, eq);
      int idx = motorIndexOf(part, _count);
      if (idx < 0) return -1;
      if (!sel[idx]) { sel[idx] = true; n++; }
    }
    return n;
  }
  for (uint8_t i = 0; i < _count; i++) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    start = comma + 1;
    if (part.length()) { sel[i] = true; n++; }
  }
  return n;
}

bool JMCController::selectionError(const __FlashStringHelper* cmd, int n) {
  if (n > 0) return false;
  String c(cmd);
  if (n < 0) reply(String(F("ERR:")) + c + F(" motor must be M1..M") + String(_count));
  else       reply(String(F("ERR:")) + c + F(" needs motor slots (e.g. ") + c + F(":1,,1 or ") +
                   c + F(":M1,M3) - ") + c + F("A = all motors"));
  return true;
}

bool JMCController::valuesError(const __FlashStringHelper* cmd, int n, bool all) {
  if (n > 0) return false;
  String c(cmd);
  if (all)       reply(String(F("ERR:")) + c + F("A takes one value for all motors (e.g. ") + c + F("A:1000)"));
  else if (n < 0) reply(String(F("ERR:")) + c + F(" motor must be M1..M") + String(_count) + F(" (e.g. ") + c + F(":M3=1000)"));
  else            reply(String(F("ERR:")) + c + F(" needs values (e.g. ") + c + F(":1000,,3000 or ") +
                        c + F(":M3=1000) - ") + c + F("A:<value> = all motors"));
  return true;
}

// Heads that take motor slots and therefore have an A-suffixed all-motors form.
bool JMCController::isMotorCommand(const String& b) {
  if (b == "P" || b == "PR" || b == "V" || b == "VS" || b == "ACC" || b == "DEC" ||
      b == "L" || b == "BS" || b == "Z" || b == "ZO" || b == "ZS" || b == "HV" ||
      b == "HVZ" || b == "HACC" || b == "HO" || b == "S" || b == "SD" || b == "LS" ||
      b == "DG" || b == "I" || b == "R" || b == "B") return true;
  if (b.length() >= 2 && (b[0] == 'B' || b[0] == 'H')) {       // B0..B3, H1..H12
    for (unsigned k = 1; k < b.length(); k++) if (!isDigit(b[k])) return false;
    return true;
  }
  return false;
}

//==============================================================================
//  JMCController - UDP command dispatch
//==============================================================================

void JMCController::handleUdp() {
  int packetSize = _udp.parsePacket();
  if (!packetSize) return;

  memset(_cmdBuf, 0, CMD_BUFFER_SIZE);
  _udp.read(_cmdBuf, CMD_BUFFER_SIZE - 1);

  String in = String(_cmdBuf);
  in.trim();
  if (!in.length()) return;

  int colon = in.indexOf(':');
  String head = (colon < 0) ? in : in.substring(0, colon);
  String args = (colon < 0) ? String() : in.substring(colon + 1);
  head.trim(); head.toUpperCase();            // lower-case heads are accepted
  args.trim();

  // No pairing/auto-adopt: events always go to the fixed PC address set via
  // setNetwork()/setStationId(). Replies also go back to this sender.
  _senderIp   = _udp.remoteIP();
  _senderPort = _udp.remotePort();
  _chunked    = false;

  // ---- controller-level commands (no motor slots, no A form) ----
  if (head == "MC")   { handleMotorCount(args); return; }
  if (head == "VER")  { reply(String(F("JMCMotors ")) + F(JMCMOTORS_VERSION)); return; }
  if (head == "PING") { reply(F("PONG")); return; }
  if (head == "SAVE") { handleSave(args); return; }
  if (head == "LOAD") { handleLoad(); return; }
  if (head == "X") {
    reply(F("System restart initiated..."));
    delay(50);
#if defined(__AVR__)
    asm volatile("jmp 0");            // soft reboot (AVR / Controllino)
#endif
    return;
  }

  // ---- motor commands: CMD:<slots> individual, CMDA[:value] all motors ----
  bool all = false;
  if (head.length() >= 2 && head.endsWith("A")) {
    String base = head.substring(0, head.length() - 1);
    if (isMotorCommand(base)) { head = base; all = true; }
  }

  if      (head == "P")    handlePosition(args, true,  all);
  else if (head == "PR")   handlePosition(args, false, all);
  else if (head == "V")    handleFloatSetting(SET_V,    args, all);
  else if (head == "ACC")  handleFloatSetting(SET_ACC,  args, all);
  else if (head == "DEC")  handleFloatSetting(SET_DEC,  args, all);
  else if (head == "HV")   handleFloatSetting(SET_HV,   args, all);
  else if (head == "HVZ")  handleFloatSetting(SET_HVZ,  args, all);
  else if (head == "HACC") handleFloatSetting(SET_HACC, args, all);
  else if (head == "VS")   handleVelocityStart(args, all);
  else if (head == "L")    handleLoosen(args, all);
  else if (head == "BS")   handleBrakeStatus(args, all);
  else if (head == "Z")    handleZero(args, all);
  else if (head == "ZO")   handleZeroOffset(args, all);
  else if (head == "ZS")   handleZeroStatus(args, all);
  else if (head == "HO")   handleHomingOffset(args, all);
  else if (head == "S")    handleStatus(args, all);
  else if (head == "SD")   handleStatusDetail(args, all);
  else if (head == "LS")   handleLimitStatus(args, all);
  else if (head == "DG")   handleDiagnose(args, all);
  else if (head == "I")    handleInit(args, all);
  else if (head == "R")    handleReset(args, all);
  else if (head == "B")    handleBrake(0, args, all);
  else if (head[0] == 'B' && isMotorCommand(head))        // B0..B3
    handleBrake(head.substring(1).toInt(), args, all);
  else if (head[0] == 'H' && isMotorCommand(head))        // H1..H12
    handleHoming(head.substring(1).toInt(), args, all);
  else reply(String(F("ERR:unknown command ")) + head);
}

//==============================================================================
//  JMCController - background monitor (non-blocking, one motor per tick)
//==============================================================================

void JMCController::serviceMonitor() {
  uint32_t now = millis();
  if (now - _lastTick < MONITOR_TICK_MS) return;
  _lastTick = now;

  // Pick the next motor: online ones every sweep, offline ones only every
  // OFFLINE_RETRY_MS (so a dead drive can't eat bus time every cycle).
  int chosen = -1;
  for (uint8_t scan = 0; scan < _count; scan++) {
    _monIdx = (_monIdx + 1) % _count;
    if (_cache[_monIdx].online) { chosen = _monIdx; break; }
    if (now - _offlineTry[_monIdx] >= OFFLINE_RETRY_MS) { chosen = _monIdx; break; }
  }
  if (chosen < 0) return;

  MotorCache& c = _cache[chosen];

  uint16_t s;
  if (!c.online) {
    // Dead-drive probe: short timeout, no retries, so it can't stall the
    // sweep and delay limit detection on the live motors.
    _offlineTry[chosen] = now;
    _bus.setResponseTimeout(PROBE_TIMEOUT_MS);
    _bus.setRetries(0);
    s = _m[chosen]->getStatusWord();
    _bus.setResponseTimeout(30);
    _bus.setRetries(1);
  } else {
    s = _m[chosen]->getStatusWord();
  }
  bool ok = (_m[chosen]->lastResult() == JMC_OK);
  String mn = String(F("M")) + String(chosen + 1);

  if (!ok) {
    if (c.online && ++c.failCnt >= 2) {
      c.online = false;
      c.failCnt = 0;
      pushEvent(String(F("EVENT:OFFLINE:")) + mn);
    }
    return;
  }

  c.failCnt = 0;
  if (!c.online) {
    c.online = true;
    // Drive (re)appeared - it may have been power-cycled, so re-initialise it
    // fully (enable sequence, accel/decel, homing parameters, HO offset).
    _m[chosen]->begin(_accelSet[chosen], _decelSet[chosen]);
    applyHomingParams(chosen);
    s = _m[chosen]->getStatusWord();
    c.mode = JMCMotor::MODE_POSITION;       // begin() put it in position mode
    c.homed = false;                        // counter is 0 again: not homed
    c.homingArmed = false;
    c.homeErr = false;
    c.sw = s & 0x0800;
    pushEvent(String(F("EVENT:ONLINE:")) + mn + F(" ST=0x") + String(s, HEX) + F(" (re-initialised)"));
  }

  bool cw    = s & 0x4000;
  bool ccw   = s & 0x8000;
  bool sw    = s & 0x0800;
  bool fault = s & 0x0008;

  // Limit-switch edges: push immediately with position context.
  if (cw && !c.cw)
    pushEvent(String(F("EVENT:LIMIT:")) + mn + F(":CW POS=") + String(_m[chosen]->getPosition()) +
              String(F(" ST=0x")) + String(s, HEX));
  if (!cw && c.cw)
    pushEvent(String(F("EVENT:LIMIT:")) + mn + F(":CW_CLEARED"));
  if (ccw && !c.ccw)
    pushEvent(String(F("EVENT:LIMIT:")) + mn + F(":CCW POS=") + String(_m[chosen]->getPosition()) +
              String(F(" ST=0x")) + String(s, HEX));
  if (!ccw && c.ccw)
    pushEvent(String(F("EVENT:LIMIT:")) + mn + F(":CCW_CLEARED"));

  // Origin (SW) switch edges - bit11 is the mechanical-origin input in every mode.
  if (sw && !c.sw)
    pushEvent(String(F("EVENT:ORIGIN:")) + mn + F(":ON POS=") + String(_m[chosen]->getPosition()) +
              String(F(" ST=0x")) + String(s, HEX));
  if (!sw && c.sw)
    pushEvent(String(F("EVENT:ORIGIN:")) + mn + F(":OFF"));

  // Homing progress - only meaningful while the drive is in homing mode:
  // bit12 = homing complete, bit13 = homing error. bit12 may still be set
  // from a previous run for a moment after the start pulse, so completion is
  // accepted only after it has been seen low once, or once the motor is
  // demonstrably at rest well after the start.
  if (c.homingArmed && c.mode == JMCMotor::MODE_HOMING) {
    bool done = s & 0x1000;
    bool err  = s & 0x2000;
    bool run  = s & 0x0200;
    if (err && !c.homeErr) {
      c.homeErr = true;
      c.homingArmed = false;
      pushEvent(String(F("EVENT:HOMING_ERROR:")) + mn + F(" ST=0x") + String(s, HEX) +
                F(" (check limit/origin wiring, HV, HO)"));
    } else if (!done) {
      c.homingBusySeen = true;
    } else if (c.homingBusySeen || (!run && (now - c.homingStart) > 1000)) {
      c.homed = true;
      c.homingArmed = false;
      pushEvent(String(F("EVENT:HOMED:")) + mn + F(" POS=") + String(_m[chosen]->getPosition()) +
                String(F(" ST=0x")) + String(s, HEX));
    }
  }

  // Fault edges: decode the 0x1001 error register for the event.
  if (fault && !c.fault) {
    JMCMotor::ErrorFlags e = _m[chosen]->getErrorFlags();
    String detail = "";
    if (e.valid) {
      if (e.current)       detail += F(" CURRENT");
      if (e.voltage)       detail += F(" VOLTAGE");
      if (e.temperature)   detail += F(" TEMP");
      if (e.communication) detail += F(" COMM");
      if (e.overPosition)  detail += F(" OVERPOS");
      if (e.phaseLoss)     detail += F(" PHASE");
      if (e.generic && !detail.length()) detail = F(" GENERIC");
    }
    pushEvent(String(F("EVENT:FAULT:")) + mn + F(" ST=0x") + String(s, HEX) +
              String(F(" ERR=0x")) + String(e.raw, HEX) + detail);
  }
  if (!fault && c.fault)
    pushEvent(String(F("EVENT:FAULT_CLEARED:")) + mn);

  c.cw = cw; c.ccw = ccw; c.sw = sw; c.fault = fault; c.status = s;
}

// Velocity -> position/homing handoff for a group of motors.
// A drive whose 0x6060 mode register changes while the shaft still turns drops
// the velocity setpoint instantly - the motor stops dead, then runs the new
// move. So halt everyone first (bit8 = decel ramp), wait once for the whole
// group to come to rest, and only then let the callers switch mode. One shared
// wait keeps a multi-motor move synchronised and bounds the stall to the
// longest ramp actually needed.
// Start the brake on every selected motor that is jogging. Returns false when
// none of them is - then the caller can act immediately. deadline is when to
// give up waiting: the longest expected ramp, so a drive that never reports
// standstill (mechanically stalled, unplugged mid-ramp) cannot pin the move.
bool JMCController::rampDownBegin(const bool* sel, bool* braking, uint32_t& deadline) {
  cancelPendingMove();       // a queued move is superseded; free its brake set
  uint16_t limit = 0;
  bool any = false;
  for (uint8_t i = 0; i < _count; i++) {
    braking[i] = sel[i] && _m[i]->velocityModeRunning();
    if (!braking[i]) continue;                  // idle, or already in position mode
    _m[i]->beginVelocityRampDown();
    if (_m[i]->rampDownTime_ms() > limit) limit = _m[i]->rampDownTime_ms();
    any = true;
  }
  deadline = millis() + limit;
  return any;
}

bool JMCController::rampDownDone(const bool* braking) {
  for (uint8_t i = 0; i < _count; i++)
    if (braking[i] && _m[i]->isSpinning()) return false;
  return true;
}

// Releases halt - without this the position move that follows never starts.
void JMCController::rampDownEnd(const bool* braking) {
  for (uint8_t i = 0; i < _count; i++)
    if (braking[i]) _m[i]->endVelocityRampDown();
}

// Blocking form, for the commands that redefine the position counter or start
// homing (Z / ZO / H). They are setup commands, not motion timing, so the few
// hundred ms of brake is spent here rather than deferred.
void JMCController::rampDownSelected(const bool* sel) {
  bool braking[JMC_MAX_MOTORS];
  uint32_t deadline;
  if (!rampDownBegin(sel, braking, deadline)) return;
  uint32_t cap = millis() + JMCMotor::BLOCKING_RAMP_CAP_MS;
  if ((int32_t)(deadline - cap) > 0) deadline = cap;   // never freeze for a full slow ramp
  while ((int32_t)(millis() - deadline) < 0) {
    if (rampDownDone(braking)) break;
    delay(5);
  }
  rampDownEnd(braking);
}

// Launch a prepared-and-braked move. Split out of handlePosition so the
// deferred path (servicePendingMove) runs exactly the same sequence.
void JMCController::launchMove(const bool* sel, const long* target,
                               bool absolute, bool broadcast) {
  noteMode(sel, JMCMotor::MODE_POSITION);
  if (broadcast) {
    _m[0]->broadcastPrepareMove(target[0], _storedVel[0], absolute);
    _m[0]->broadcastEnableMovement();
    delayMicroseconds(500);
    _m[0]->broadcastStartMovement();
    return;
  }

  uint8_t n = 0;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->preparePositionMove(target[i], _storedVel[i], absolute);
    n++;
  }
  if (n == _count) {
    // Every motor is moving: launch with TWO broadcast frames (slave 0) so
    // all drives start in the same instant - no per-motor stagger even at 32.
    _m[0]->broadcastEnableMovement();
    delayMicroseconds(300);
    _m[0]->broadcastStartMovement();
  } else {
    // Partial selection: per-motor launch (broadcast would also retrigger
    // the motors that were NOT commanded).
    for (uint8_t i = 0; i < _count; i++)
      if (sel[i]) _m[i]->enableMovement();
    delayMicroseconds(200);
    for (uint8_t i = 0; i < _count; i++)
      if (sel[i]) _m[i]->startMovement();
  }
}

// Called from run(): the motors that were jogging have braked, so the command
// is simply re-run. rampDownEnd() has cleared their velocity-mode flag, so the
// second pass finds nothing to wait for and launches straight away.
void JMCController::servicePendingMove() {
  if (!_pending.active) return;

  uint32_t now = millis();
  bool expired = (int32_t)(now - _pending.deadline) >= 0;
  if (!expired) {
    if (now - _pending.lastPoll < RAMP_POLL_MS) return;   // don't flood the bus
    _pending.lastPoll = now;
    if (!rampDownDone(_pending.braking)) return;
  }

  rampDownEnd(_pending.braking);
  _pending.active = false;
  String a = _pending.args;
  _pending.args = "";
  handlePosition(a, _pending.absolute, _pending.broadcast);   // PA/PRA re-run as 'all'
}

// B / VS / I / R: whatever was queued is no longer what the operator wants.
// Releases halt first (the motors are at rest, so nothing moves) - the caller
// then applies its own stop, or starts a new jog.
void JMCController::cancelPendingMove() {
  if (!_pending.active) return;
  _pending.active = false;
  _pending.args = "";
  rampDownEnd(_pending.braking);
}

//==============================================================================
//  JMCController - command handlers (v0.5)
//==============================================================================

// ---- P / PR : position moves and '?' reads ------------------------------------
// P = absolute target, PR = relative delta (drive-side bit6 does the math;
// adding the current position in software was the classic double-count bug).
//   P:1000,,3000      M1 -> 1000, M3 -> 3000 (synchronised)
//   P:M17=1000        motor 17 only
//   P:?  P:,,?  P:M3  reads
//   PA:5000 / PRA:-500   every motor, broadcast; PA:? reads all
void JMCController::handlePosition(const String& args, bool absolute, bool all) {
  const __FlashStringHelper* cmd = absolute ? F("P") : F("PR");
  Fields Fv;
  int n = parseValues(args, Fv, all);
  if (valuesError(cmd, n, all)) return;

  if (Fv.nq) {                                  // reads
    if (Fv.nq != Fv.n) { reply(String(F("ERR:")) + cmd + F(" cannot mix values and ?")); return; }
    replyPositions(Fv.query);
    return;
  }

  // Coming out of velocity mode (VS): the drives must reach standstill before
  // their 0x6060 mode register may change - switching mode on a turning shaft
  // stops it dead, with no ramp. Brake the whole selection on its decel ramp
  // and hand the move to servicePendingMove(), which launches it (all motors
  // together, still synchronised) as soon as they are at rest.
  if (rampDownBegin(Fv.active, _pending.braking, _pending.deadline)) {
    _pending.args      = args;      // re-run verbatim once they are at rest
    _pending.absolute  = absolute;
    _pending.broadcast = all;
    _pending.lastPoll  = millis();
    _pending.active    = true;
    return;                         // "OK" is sent when the move really starts
  }

  // PA / PRA: one broadcast prepare (motor 1's speed) + broadcast launch.
  launchMove(Fv.active, Fv.l, absolute, all);
  reply(F("OK"));
}

// Report positions of the selected motors: "M1=1500,M3=200" (+ WARN: on errors).
void JMCController::replyPositions(const bool* sel) {
  String out = "";
  bool anyError = false, first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    if (!first) out += F(",");
    long p = _m[i]->getPosition();
    if (_m[i]->lastResult() != JMC_OK) { out += String(F("M")) + String(i + 1) + F("=ERROR"); anyError = true; }
    else                                out += String(F("M")) + String(i + 1) + F("=") + String(p);
    first = false;
    if (out.length() >= REPLY_CHUNK_CHARS) {        // many motors: several packets
      reply((anyError ? String(F("WARN: ")) + out : out) + F(":MORE"));
      _chunked = true;
      out = ""; anyError = false; first = true;
    }
  }
  reply(replyChunkEnd(anyError ? String(F("WARN: ")) + out : out));
}

// ---- V / ACC / DEC / HV / HVZ / HACC : one per-motor float each ----------------
//   V:0.5,,0.3   V:M3=0.3   V:?   V:M3     set / read individual motors
//   VA:1         ACCA:0.3   HVA:?          all motors (VA also broadcasts now)
void JMCController::handleFloatSetting(FloatSetting which, const String& args, bool all) {
  const __FlashStringHelper* cmd;
  const __FlashStringHelper* unit;
  float lo, hi;
  switch (which) {
    case SET_V:    cmd = F("V");    unit = F(" rps");   lo = 0.01f;    hi = 1000.0f; break;   // sign = direction
    case SET_ACC:  cmd = F("ACC");  unit = F(" rps/s"); lo = 0.1f;     hi = 6553.5f; break;
    case SET_DEC:  cmd = F("DEC");  unit = F(" rps/s"); lo = 0.1f;     hi = 6553.5f; break;
    // Manual: homing methods allow at most 300 rpm (1-4) or 1200 rpm (5-12) = 20 rps.
    case SET_HV:   cmd = F("HV");   unit = F(" rps");   lo = 0.01f;    hi = 20.0f;   break;
    case SET_HVZ:  cmd = F("HVZ");  unit = F(" rps");   lo = 0.01f;    hi = 20.0f;   break;
    default:       cmd = F("HACC"); unit = F(" rps/s"); lo = 0.1f;     hi = 6553.5f; break;
  }
  Fields Fv;
  int n = parseValues(args, Fv, all);
  if (valuesError(cmd, n, all)) return;

  if (Fv.nq) {                                  // reads: "V: M1=0.50,M3=0.30"
    if (Fv.nq != Fv.n) { reply(String(F("ERR:")) + cmd + F(" cannot mix values and ?")); return; }
    String out = String(cmd) + F(": ");
    bool first = true;
    for (uint8_t i = 0; i < _count; i++) {
      if (!Fv.query[i]) continue;
      if (!first) out += F(",");
      first = false;
      float v;
      switch (which) {
        case SET_V:    v = _storedVel[i]; break;
        case SET_ACC:  v = _accelSet[i];  break;
        case SET_DEC:  v = _decelSet[i];  break;
        case SET_HV:   v = _homingVel[i]; break;
        case SET_HVZ:  v = isnan(_homingZeroVel[i]) ? _homingVel[i] : _homingZeroVel[i]; break;
        default:       v = isnan(_homingAccel[i]) ? _accelSet[i] : _homingAccel[i]; break;
      }
      out += String(F("M")) + String(i + 1) + F("=") + String(v, 2);
      if (flushIfLong(out, String(cmd) + F(": "))) first = true;
    }
    reply(replyChunkEnd(out + unit));
    return;
  }

  // Range check every value before touching anything (V may be negative:
  // the sign is the jog direction, the magnitude must be in range).
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    float v = (which == SET_V) ? fabs(Fv.f[i]) : Fv.f[i];
    if (v < lo || v > hi) {
      reply(String(F("ERR:")) + cmd + F(" range ") + String(lo, 2) + F("..") + String(hi, 1) + unit +
            String(F(" (M")) + String(i + 1) + F(")"));
      return;
    }
  }

  String out = (all ? String(cmd) + F("A: all motors = ") : String(F("Set ")) + cmd + F(": "));
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    float v = Fv.f[i];
    switch (which) {
      case SET_V:    _storedVel[i] = v; break;                 // used by the next move / VS
      case SET_ACC:  _accelSet[i] = v; _m[i]->setAccelDecel(v, _decelSet[i]); break;
      case SET_DEC:  _decelSet[i] = v; _m[i]->setAccelDecel(_accelSet[i], v); break;
      case SET_HV:   _homingVel[i] = v;     applyHomingParams(i); break;
      case SET_HVZ:  _homingZeroVel[i] = v; applyHomingParams(i); break;
      default:       _homingAccel[i] = v;   applyHomingParams(i); break;
    }
    if (!all) {
      if (!first) out += F(",");
      first = false;
      out += String(F("M")) + String(i + 1) + F("=") + String(v, 2);
      if (flushIfLong(out, String(F("Set ")) + cmd + F(": "))) first = true;
    }
  }
  if (all) {
    if (which == SET_V) _m[0]->broadcastSetVelocity(Fv.f[0]);   // a running jog follows at once
    out += String(Fv.f[0], 2) + unit;
    if (which == SET_V) out += F(" (BROADCAST)");
    reply(out);
  } else {
    reply(replyChunkEnd(out + unit));
  }
}

// ---- VS : velocity mode (endless jog) -------------------------------------------
// VS:1,,1 / VS:M1,M3 -> those motors; VSA -> every motor (broadcast start).
// Each motor jogs at its own stored V speed (sign = direction).
void JMCController::handleVelocityStart(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("VS"), n)) return;

  cancelPendingMove();          // back to jogging: a queued move is stale
  String list = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->prepareVelocityMode(_storedVel[i]);
    if (list.length()) list += F(",");
    list += String(i + 1);
  }
  noteMode(sel, JMCMotor::MODE_VELOCITY);

  if (n == (int)_count) {
    // Every motor: one broadcast launch so all shafts start together.
    _m[0]->broadcastEnableMovement();
    delayMicroseconds(300);
    _m[0]->broadcastStartMovement();
    reply(F("VS: velocity mode started (stop with B)"));
  } else {
    // Subset: per-motor launch (broadcast would also start the others).
    for (uint8_t i = 0; i < _count; i++) if (sel[i]) _m[i]->enableMovement();
    delayMicroseconds(200);
    for (uint8_t i = 0; i < _count; i++) if (sel[i]) _m[i]->startMovement();
    reply(String(F("VS: velocity mode started for M")) + list + F(" (stop with B)"));
  }
}

// ---- S : status word in six formats ---------------------------------------------
//   S:1,,1  S:M2         selected motors, RAW
//   S:1,,1:COMPACT       format after a second colon (RAW COMPACT STATE BITS FLAGS DETAIL)
//   SA  SA:FLAGS         all motors
void JMCController::handleStatus(const String& args, bool all) {
  String selPart = args, fmt = F("RAW");
  if (all) { fmt = args; selPart = ""; }
  else {
    int c2 = args.indexOf(':');
    if (c2 >= 0) { fmt = args.substring(c2 + 1); selPart = args.substring(0, c2); }
  }
  fmt.trim(); fmt.toUpperCase(); selPart.trim();
  if (!fmt.length()) fmt = F("RAW");

  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(selPart, sel, all);
  if (selectionError(F("S"), n)) return;

  bool first = true;
  if (fmt == "RAW") {
    String out = F("STATUS:");
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      if (!first) out += F(",");
      first = false;
      uint16_t s = _m[i]->getStatusWord();
      if (_m[i]->lastResult() != JMC_OK) out += String(F("M")) + String(i + 1) + F("=ERROR");
      else out += String(F("M")) + String(i + 1) + F("=0x") + String(s, HEX);
      if (flushIfLong(out, F("STATUS:"))) first = true;
    }
    reply(replyChunkEnd(out));
  }
  else if (fmt == "COMPACT") {
    String out = "";
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      if (!first) out += F(",");
      first = false;
      out += compactStatus(i);
      if (flushIfLong(out, F(""))) first = true;
    }
    reply(replyChunkEnd(out));
  }
  else if (fmt == "STATE") {
    String out = F("STATES:");
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      if (!first) out += F(",");
      first = false;
      uint16_t s = _m[i]->getStatusWord();
      out += String(F("M")) + String(i + 1) + F(":");
      out += (_m[i]->lastResult() != JMC_OK) ? String(F("ERROR")) : driveStateName(s);
      if (flushIfLong(out, F("STATES:"))) first = true;
    }
    reply(replyChunkEnd(out));
  }
  else if (fmt == "BITS" || fmt == "FLAGS") {
    bool bits = (fmt == "BITS");
    String out = "";
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      uint16_t s = _m[i]->getStatusWord();
      if (!first) out += F("\n");
      first = false;
      out += String(F("M")) + String(i + 1) + F(":");
      if (_m[i]->lastResult() != JMC_OK) out += F("ERROR");
      else out += bits ? bitsString(s) : flagsString(s, _cache[i].mode);
      if (flushIfLong(out, F(""))) first = true;
    }
    reply(replyChunkEnd(out));
  }
  else if (fmt == "DETAIL") {                   // one packet per motor
    int left = n;
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      left--;
      if (n == 1) reply(detailStatus(i)); else replyMulti(detailStatus(i), left == 0);
    }
  }
  else reply(F("ERR:S formats RAW|COMPACT|DETAIL|STATE|BITS|FLAGS"));
}

String JMCController::compactStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  if (_m[i]->lastResult() != JMC_OK) return String(F("M")) + String(i + 1) + F(":ERROR");
  String t1 = (s & 0x0008) ? String(F("ERR")) : ((s & 0x0004) ? String(F("RUN")) : String(F("OFF")));
  String t2 = (s & 0x0200) ? String(F("MOV")) : String(F("STP"));
  String t3 = (s & 0x0400) ? String(F("TR"))  : String(F("NR"));
  return String(F("M")) + String(i + 1) + F(":") + t1 + F("|") + t2 + F("|") + t3 +
         String(F("|POS=")) + String(_m[i]->getPosition()) +
         String(F("|VEL=")) + String(_m[i]->getVelocity(), 1);
}

String JMCController::detailStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  String mn = String(F("M")) + String(i + 1);
  if (_m[i]->lastResult() != JMC_OK)
    return mn + F(": NO REPLY (") + String(JMCBus::resultName(_m[i]->lastResult())) + F(")");
  String out = mn + F(" (ID ") + String(_m[i]->slaveId()) + F(")\n");
  out += String(F("  Status: 0x")) + String(s, HEX) + F(" (") + driveStateName(s) + F(")\n");
  out += String(F("  Mode:   ")) + String(modeName(_cache[i].mode)) + F("\n");
  out += String(F("  Flags:  ")) + flagsString(s, _cache[i].mode) + F("\n");
  out += String(F("  Bits:   ")) + bitsString(s) + F("\n");
  out += String(F("  Pos:    ")) + String(_m[i]->getPosition()) + F("\n");
  out += String(F("  Vel:    ")) + String(_m[i]->getVelocity(), 1) + F(" rps");
  if (s & 0x0008) {
    JMCMotor::ErrorFlags e = _m[i]->getErrorFlags();
    out += String(F("\n  Error reg 0x1001 = 0x")) + String(e.raw, HEX);
  }
  return out;
}

// ---- SD : detailed status in plain language (one packet per motor) ---------------
String JMCController::humanStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  String mn = String(F("MOTOR ")) + String(i + 1);

  if (_m[i]->lastResult() != JMC_OK)
    return String(F("=== ")) + mn + F(" === NOT RESPONDING (") +
           String(JMCBus::resultName(_m[i]->lastResult())) +
           F(") - check power, slave ID, RS485 wiring");

  long  pos = _m[i]->getPosition();
  float vel = _m[i]->getVelocity();

  String out = String(F("=== ")) + mn + F(" (Slave ID ") + String(_m[i]->slaveId()) +
               String(F(") Status 0x")) + String(s, HEX) + F(" ===\n");

  out += String(F("State: ")) + String(driveStateName(s));
  if (s & 0x0040) out += F(" (entering initialization)");
  out += F("\n");

  out += F("Motor: ");
  out += (s & 0x0004) ? String(F("ENABLED")) : String(F("DISABLED"));
  if (s & 0x0200)      out += F(", RUNNING");
  else                 out += F(", stopped");
  if (s & 0x0100)      out += F(", HALT active");
  if (s & 0x0020)      out += F(", QUICK-STOP active");
  out += F("\n");

  uint8_t mode = _cache[i].mode;
  out += String(F("Mode: ")) + String(modeName(mode)) + F("\n");

  out += String(F("Position: ")) + String(pos) + F(" steps (target ");
  out += (s & 0x0400) ? String(F("REACHED")) : String(F("NOT reached"));
  out += F(")\n");
  out += String(F("Velocity: ")) + String(vel, 1) + F(" rps\n");

  // Homing - bit12/13 mean "complete"/"error" only in homing mode; in the
  // other modes report what the controller latched from the last run.
  out += F("Homing: ");
  if (mode == JMCMotor::MODE_HOMING) {
    if      (s & 0x2000) out += F("ERROR");
    else if (s & 0x1000) out += F("complete");
    else if (_cache[i].homingArmed) out += F("IN PROGRESS");
    else out += F("not done");
  } else {
    if      (_cache[i].homeErr) out += F("ERROR (last run failed)");
    else if (_cache[i].homed)   out += F("homed");
    else                        out += F("not homed since boot");
  }
  out += F(" | Origin switch (SW): ");
  out += (s & 0x0800) ? String(F("ON")) : String(F("off"));
  out += F("\n");

  out += F("Limits: CW ");
  out += (s & 0x4000) ? String(F("*** ACTIVE ***")) : String(F("clear"));
  out += F(" | CCW ");
  out += (s & 0x8000) ? String(F("*** ACTIVE ***")) : String(F("clear"));
  out += F("\n");

  if (s & 0x2000) {
    if      (mode == JMCMotor::MODE_POSITION) out += F("OVER-POSITION ERROR (bit13)\n");
    else if (mode == JMCMotor::MODE_VELOCITY) out += F("MAX ACCELERATION reached (bit13)\n");
    else                                      out += F("HOMING ERROR (bit13)\n");
  }
  if (s & 0x0080) out += F("WARNING active (bit7)\n");

  if (s & 0x0008) {
    JMCMotor::ErrorFlags e = _m[i]->getErrorFlags();
    out += F("FAULT:");
    bool named = false;
    if (e.valid) {
      if (e.current)       { out += F(" over-current");          named = true; }
      if (e.voltage)       { out += F(" voltage-error");         named = true; }
      if (e.temperature)   { out += F(" over-temperature");      named = true; }
      if (e.communication) { out += F(" communication-error");   named = true; }
      if (e.overPosition)  { out += F(" over-position-protect"); named = true; }
      if (e.phaseLoss)     { out += F(" motor-phase-loss");      named = true; }
      if (e.generic && !named) { out += F(" generic-error");     named = true; }
    }
    if (!named) out += F(" unknown");
    if (e.valid) out += String(F(" (0x1001=0x")) + String(e.raw, HEX) + F(")");
    out += String(F(" -> send R:M")) + String(i + 1) + F(" to reset\n");
  } else {
    out += F("Health: OK, no faults\n");
  }
  return out;
}

void JMCController::handleStatusDetail(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("SD"), n)) return;
  int left = n;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    left--;
    if (n == 1) reply(humanStatus(i)); else replyMulti(humanStatus(i), left == 0);
  }
}

// ---- B / B1 / B2 / B3 : stop --------------------------------------------------------
// The type is the digit in the head: B (= B0) controlled stop on the decel ramp,
// B1 quick stop on the decel ramp, B2 fast brake (quick-stop ramp), B3 immediate.
//   B2:1,,1  B2:M1,M3   selected motors        B2A   every motor
//   B / B1 / B2 / B3 with nothing after = every motor (emergency alias: a stop
//   is never refused for a missing selector).
void JMCController::handleBrake(int type, const String& args, bool all) {
  if (type < 0 || type > 3) { reply(F("ERR:B type is B, B1, B2 or B3")); return; }
  String a = args; a.trim();
  if (!all && !a.length()) all = true;        // bare stop = all motors, always

  bool sel[JMC_MAX_MOTORS];
  int  n = parseSelection(a, sel, all);
  if (selectionError(F("B"), n)) return;

  // Stop wins over a move that is still waiting for the jog to brake: drop it
  // before halting, or it would launch a moment later.
  cancelPendingMove();

  String list = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    if      (type == 1) _m[i]->quickStop(JMCMotor::QUICK_STOP_DECEL);
    else if (type == 2) _m[i]->quickStop(JMCMotor::QUICK_STOP_FAST);
    else if (type == 3) _m[i]->quickStop(JMCMotor::QUICK_STOP_IMMEDIATE);
    else                _m[i]->stop();
    if (list.length()) list += F(",");
    list += String(i + 1);
  }

  bool every = (n == (int)_count);
  String tgt = every ? String(F("all motors")) : (String(F("motor ")) + list);
  if      (type == 1) reply(String(F("Deceleration brake applied to ")) + tgt);
  else if (type == 2) reply(String(F("Fast brake applied to ")) + tgt);
  else if (type == 3) reply(String(F("Immediate brake applied to ")) + tgt);
  else if (every)     reply(F("All motors stopped with deceleration"));
  else                reply(String(F("Motor ")) + list + F(" stopped with deceleration"));
}

// ---- L : loosen brakes -------------------------------------------------------------
void JMCController::handleLoosen(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("L"), n)) return;

  String done = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->loosenBrake();
    if (done.length()) done += F(",");
    done += String(i + 1);
  }
  reply((n == (int)_count) ? String(F("All motor brakes loosened"))
                           : String(F("Motor ")) + done + F(" brake loosened"));
}

// ---- BS : brake status --------------------------------------------------------------
void JMCController::handleBrakeStatus(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("BS"), n)) return;

  String out = F("BRAKE STATUS: ");
  int braked = 0, freed = 0, errors = 0;
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    if (!first) out += F(",");
    first = false;
    uint16_t s = _m[i]->getStatusWord();
    if (_m[i]->lastResult() != JMC_OK) { out += String(F("M")) + String(i + 1) + F(":ERROR"); errors++; continue; }
    bool brakeEngaged = (s & 0x0100) || !(s & 0x0004);   // halted or not enabled
    bool running      = s & 0x0200;
    out += String(F("M")) + String(i + 1) + F(":") + (brakeEngaged ? String(F("BRAKE")) : String(F("FREE"))) + F("/") +
           (running ? String(F("RUN")) : String(F("STOP")));
    if (brakeEngaged) braked++; else freed++;
    if (flushIfLong(out, F("BRAKE STATUS: "))) first = true;
  }
  out += F(" | SUMMARY: ");
  if (errors == n)            out += F("ERROR");
  else if (braked && !freed)  out += F("BRAKED");
  else if (freed && !braked)  out += F("FREE");
  else                        out += F("MIXED");
  reply(replyChunkEnd(out));
}

// ---- LS : limit & sensor status -------------------------------------------------------
// One line, machine-friendly: every switch input and the homing state per
// motor, independent of which mode the drive is in.
//   LIMITS: M1:CW=0 CCW=0 SW=1 HOMED=1 MODE=POS,M2:CW=1 CCW=0 SW=0 HOMED=0 MODE=HOM
void JMCController::handleLimitStatus(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("LS"), n)) return;

  String out = F("LIMITS: ");
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    if (!first) out += F(",");
    first = false;
    uint16_t s = _m[i]->getStatusWord();
    out += String(F("M")) + String(i + 1) + F(":");
    if (_m[i]->lastResult() != JMC_OK) { out += F("ERROR"); continue; }
    MotorCache& c = _cache[i];
    bool homed = (c.mode == JMCMotor::MODE_HOMING) ? (s & 0x1000) : c.homed;
    bool herr  = (c.mode == JMCMotor::MODE_HOMING) ? (s & 0x2000) : c.homeErr;
    out += F("CW=");    out += (s & 0x4000) ? String(F("1")) : String(F("0"));
    out += F(" CCW=");  out += (s & 0x8000) ? String(F("1")) : String(F("0"));
    out += F(" SW=");   out += (s & 0x0800) ? String(F("1")) : String(F("0"));
    out += F(" HOMED="); out += herr ? String(F("ERR")) : (homed ? String(F("1")) : (c.homingArmed ? String(F("BUSY")) : String(F("0"))));
    out += F(" MODE=");
    out += (c.mode == JMCMotor::MODE_HOMING) ? String(F("HOM")) :
           (c.mode == JMCMotor::MODE_VELOCITY) ? String(F("VEL")) : String(F("POS"));
    if (flushIfLong(out, F("LIMITS: "))) first = true;
  }
  reply(replyChunkEnd(out));
}

// ---- Z : current position becomes 0 ---------------------------------------------------
void JMCController::handleZero(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("Z"), n)) return;

  rampDownSelected(sel);
  String done = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->setZeroPosition();
    delay(20);
    _m[i]->setOperationMode(JMCMotor::MODE_POSITION);
    noteModeOne(i, JMCMotor::MODE_POSITION);
    _cache[i].zeroSet = true;
    if (done.length()) done += F(",");
    done += String(i + 1);
  }
  reply((n == (int)_count) ? String(F("All motors set to zero position"))
                           : String(F("Motor ")) + done + F(" set to zero position"));
}

// ---- ZO : redefine current position (no movement) --------------------------------------
// ZO:10000,,500 -> motor 1's CURRENT position now reads 10000, motor 3's 500.
// ZOA:5000 -> every motor reads 5000 where it stands.
void JMCController::handleZeroOffset(const String& args, bool all) {
  Fields Fv;
  int n = parseValues(args, Fv, all);
  if (valuesError(F("ZO"), n, all)) return;
  if (Fv.nq) { reply(F("ERR:ZO takes values, not ? (read positions with P:?)")); return; }

  rampDownSelected(Fv.active);
  String out = F("ZO: ");
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    _m[i]->setCurrentPosition(Fv.l[i]);
    delay(20);                              // let homing method 0 latch
    _m[i]->setOperationMode(JMCMotor::MODE_POSITION);
    noteModeOne(i, JMCMotor::MODE_POSITION);
    _cache[i].zeroSet = true;
    if (!first) out += F(",");
    out += String(F("M")) + String(i + 1) + F("=") + String(Fv.l[i]);
    first = false;
    if (flushIfLong(out, F("ZO: "))) first = true;
  }
  reply(replyChunkEnd(out + F(" (current position redefined)")));
}

// ---- ZS : zero status ---------------------------------------------------------------------
void JMCController::handleZeroStatus(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("ZS"), n)) return;

  String out = F("ZERO STATUS: ");
  bool allSet = true, first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    if (!first) out += F(",");
    first = false;
    long p = _m[i]->getPosition();
    bool ok = (_m[i]->lastResult() == JMC_OK);
    if (!ok) { out += String(F("M")) + String(i + 1) + F(":ERROR"); allSet = false; continue; }
    out += String(F("M")) + String(i + 1) + F(":") + (_cache[i].zeroSet ? String(F("ZERO")) : String(F("NOT_SET"))) +
           String(F("(")) + String(p) + F(")");
    if (!_cache[i].zeroSet) allSet = false;
    if (flushIfLong(out, F("ZERO STATUS: "))) first = true;
  }
  out += allSet ? String(F(" | ALL_ZERO_SET")) : String(F(" | ZERO_NOT_SET"));
  reply(replyChunkEnd(out));
}

// ---- R : alarm reset + re-initialise --------------------------------------------------------
void JMCController::handleReset(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("R"), n)) return;

  cancelPendingMove();
  String done = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->alarmReset();
    JMCResult r = _m[i]->begin(_accelSet[i], _decelSet[i]);
    applyHomingParams(i);
    _cache[i].online = (r == JMC_OK);
    _cache[i].fault  = false;
    noteModeOne(i, JMCMotor::MODE_POSITION);   // begin() leaves it in position mode
    _cache[i].homeErr = false;                 // alarm cleared; 'homed' is kept
    if (done.length()) done += F(",");
    done += String(i + 1) + (r == JMC_OK ? String() : String(F("(FAIL)")));
  }
  reply((n == (int)_count) ? String(F("All motors reset complete"))
                           : String(F("Motor ")) + done + F(" reset complete"));
}

// ---- I : detect + initialise ---------------------------------------------------------------
void JMCController::handleInit(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("I"), n)) return;

  cancelPendingMove();
  String out = F("MOTOR INITIALIZATION:\n");
  int found = 0;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    out += String(F("Motor ")) + String(i + 1) + F(" (Slave ID ") + String(_m[i]->slaveId()) + F("): ");
    // Full init with this motor's own ramps and homing parameters.
    JMCResult r = _m[i]->begin(_accelSet[i], _decelSet[i]);
    applyHomingParams(i);
    _cache[i].online = (r == JMC_OK);
    noteModeOne(i, JMCMotor::MODE_POSITION);
    if (r == JMC_OK) {
      found++;
      uint16_t s = _m[i]->getStatusWord();
      out += F("DETECTED - Initialized in Position mode\n");
      out += String(F("  Status: 0x")) + String(s, HEX) + F(" (") + driveStateName(s) + F(")\n");
      out += String(F("  Position: ")) + String(_m[i]->getPosition()) + F("\n");
    } else {
      out += String(F("NOT DETECTED (")) + String(JMCBus::resultName(r)) + F(")\n");
      out += F("  Check: Power, Address, RS485 wiring\n");
    }
    flushIfLong(out, F(""));              // many motors: continues in a new packet
  }
  out += String(F("Total motors detected: ")) + String(found) + F(" of ") + String(n);
  if (found == n) out += F("\nSUCCESS: All expected motors detected and initialized!");
  reply(replyChunkEnd(out));
}

// ---- MC : motor count (runtime) --------------------------------------------
//   MC:?    read the active motor count
//   MC:12   set count to 12 (1..32); newly added slots are probed+initialised.
// Kept across reboot only via SAVE; otherwise the sketch's setMotorCount() applies.
void JMCController::handleMotorCount(const String& args) {
  String a = args; a.trim();
  if (!a.length() || a.indexOf('?') >= 0) {
    reply(String(F("MC: ")) + String(_count) + F(" motors (slave IDs 1..") + String(_count) + F(")"));
    return;
  }
  int n = a.toInt();
  if (n < 1 || n > JMC_MAX_MOTORS) {
    reply(String(F("ERR:MC range 1..")) + String(JMC_MAX_MOTORS));
    return;
  }

  uint8_t oldCount = _count;
  _count = (uint8_t)n;
  if (_monIdx >= _count) _monIdx = -1;

  int online = 0;
  for (uint8_t i = 0; i < _count; i++) {
    if (i < oldCount && _m[i]) { if (_cache[i].online) online++; continue; }
    if (initMotorSlot(i)) online++;    // probe + init slots added just now
  }
  reply(String(F("MC: ")) + String(_count) + F(" motors, ") + String(online) +
        F(" online (SAVE to keep across reboot)"));
}

// ---- SAVE / LOAD : settings in EEPROM ----------------------------------------
//   SAVE         store V, ACC, DEC, HV, HVZ, HACC, HO and MC; applied at every boot
//   SAVE:?       is anything stored?
//   SAVE:CLEAR   forget the stored block (the sketch values apply at next boot)
//   LOAD         re-apply the stored block now
void JMCController::handleSave(const String& args) {
  String a = args; a.trim(); a.toUpperCase();
  if (a == "?") {
    if (!hasSaved())              reply(F("SAVE: nothing stored - sketch values in use"));
    else if (savedMatchesSketch()) reply(F("SAVE: settings stored and applied at boot (SAVE:CLEAR to forget)"));
    else                          reply(F("SAVE: settings stored but IGNORED - sketch was re-flashed with other values (LOAD applies them, SAVE re-arms)"));
    return;
  }
  if (a == "CLEAR") { clearSaved(); reply(F("SAVE: cleared - sketch values apply at next boot")); return; }
  if (a.length())   { reply(F("ERR:SAVE use SAVE, SAVE:? or SAVE:CLEAR")); return; }
  if (!saveSettings()) { reply(F("ERR:SAVE EEPROM too small")); return; }
  reply(String(F("SAVE: ")) + String(_count) + F(" motors stored (V, ACC, DEC, HV, HVZ, HACC, HO, MC) - applied at every boot until the sketch is re-flashed with other values"));
}

void JMCController::handleLoad() {
  if (!loadSettings(true)) { reply(F("ERR:LOAD nothing stored (or checksum failed)")); return; }
  reply(String(F("LOAD: ")) + String(_count) + F(" motors loaded and applied"));
}

// ---- DG : low-level register diagnosis ---------------------------------------
// Reads back what the drive ACTUALLY holds (mode, targets, ramps) and tests
// both Modbus write types, so a "replies OK but doesn't move" case can be
// pinpointed remotely. 32-bit registers are shown as raw hi,lo words.
//   DG:,,1  DG:M3   selected motors (one packet each)     DGA   every motor
void JMCController::handleDiagnose(const String& args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("DG"), n)) return;
  int left = n;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    left--;
    if (n == 1) reply(diagnose(i)); else replyMulti(diagnose(i), left == 0);
  }
}

String JMCController::diagnose(uint8_t i) {
  int id = i + 1;
  uint8_t slave = _m[i]->slaveId();
  uint16_t w[2];
  JMCResult r;
  String out = String(F("DG M")) + String(id) + F(": ");

  r = _bus.readHolding(slave, JMCReg::FORMAT, w, 1);
  out += F("fmt6000=");
  out += (r == JMC_OK) ? String(w[0]) : String(JMCBus::resultName(r));

  r = _bus.readHolding(slave, JMCReg::STATUS, w, 1);
  out += F(" st6041=");
  out += (r == JMC_OK) ? (String(F("0x")) + String(w[0], HEX)) : String(JMCBus::resultName(r));

  r = _bus.readHolding(slave, JMCReg::MODE_DISP, w, 1);
  out += F(" mode6061=");
  out += (r == JMC_OK) ? String((int16_t)w[0]) : String(JMCBus::resultName(r));

  r = _bus.readHolding(slave, JMCReg::ERROR, w, 1);
  out += F(" err1001=");
  out += (r == JMC_OK) ? (String(F("0x")) + String(w[0], HEX)) : String(JMCBus::resultName(r));

  r = _bus.readHolding(slave, JMCReg::TARGET_POS, w, 2);
  out += F("\ntgt607A=");
  out += (r == JMC_OK) ? (String(w[0]) + F(",") + String(w[1])) : String(JMCBus::resultName(r));

  uint16_t tv[2] = {0, 0};
  r = _bus.readHolding(slave, JMCReg::TARGET_VEL, tv, 2);
  out += F(" vel6081=");
  out += (r == JMC_OK) ? (String(tv[0]) + F(",") + String(tv[1])) : String(JMCBus::resultName(r));

  r = _bus.readHolding(slave, JMCReg::ACCEL, w, 1);
  uint16_t accVal = (r == JMC_OK) ? w[0] : 5;
  out += F(" acc6083=");
  out += (r == JMC_OK) ? String(w[0]) : String(JMCBus::resultName(r));

  r = _bus.readHolding(slave, JMCReg::DECEL, w, 1);
  out += F(" dec6084=");
  out += (r == JMC_OK) ? String(w[0]) : String(JMCBus::resultName(r));

  r = _bus.readHolding(slave, JMCReg::POS_ACTUAL, w, 2);
  out += F(" pos6064=");
  out += (r == JMC_OK) ? (String(w[0]) + F(",") + String(w[1])) : String(JMCBus::resultName(r));

  // Write tests: rewrite values already in the drive (harmless, no motion).
  r = _bus.writeSingle(slave, JMCReg::ACCEL, accVal);
  out += F("\nWRITE06=");
  out += String(JMCBus::resultName(r));
  if (r == JMC_ERR_EXCEPTION) out += String(F("(exc")) + String(_bus.lastException()) + F(")");

  r = _bus.writeMultiple(slave, JMCReg::TARGET_VEL, tv, 2);
  out += F(" WRITE10=");
  out += String(JMCBus::resultName(r));
  if (r == JMC_ERR_EXCEPTION) out += String(F("(exc")) + String(_bus.lastException()) + F(")");

  out += String(F(" libScale=")) + String(_speedScale, 0);
  return out;
}

// ---- HO : homing offset --------------------------------------------------------
// Steps the motor moves AFTER the origin sensor triggers, before declaring
// zero (drive register 0x607C). Sign sets the direction.
//   HO:1000,,-800   HO:M3=500   HO:?   HO:M3        individual
//   HOA:500         HOA:?                            all motors
// Stored AND written to the drive, re-applied at every homing and re-init.
void JMCController::handleHomingOffset(const String& args, bool all) {
  Fields Fv;
  int n = parseValues(args, Fv, all);
  if (valuesError(F("HO"), n, all)) return;

  String out = F("HO: ");
  bool first = true;
  if (Fv.nq) {                                  // reads
    if (Fv.nq != Fv.n) { reply(F("ERR:HO cannot mix values and ?")); return; }
    for (uint8_t i = 0; i < _count; i++) {
      if (!Fv.query[i]) continue;
      if (!first) out += F(",");
      first = false;
      out += String(F("M")) + String(i + 1) + F("=") + String(_homingOffset[i]);
      if (flushIfLong(out, F("HO: "))) first = true;
    }
    reply(replyChunkEnd(out));
    return;
  }
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    _homingOffset[i] = Fv.l[i];
    _m[i]->setHomingOffset(Fv.l[i]);
    if (!first) out += F(",");
    first = false;
    out += String(F("M")) + String(i + 1) + F("=") + String(Fv.l[i]);
    if (flushIfLong(out, F("HO: "))) first = true;
  }
  reply(replyChunkEnd(out + F(" steps after sensor (used at next homing)")));
}

// ---- H1..H12 : start homing ------------------------------------------------------
//   H3:1,,1  H3:M1,M3   selected motors      H3A   every motor
// Each selected motor gets its homing parameters re-applied (HV/HVZ/HACC/HO),
// a running jog is braked to standstill first, and the monitor is armed to
// push EVENT:HOMED / EVENT:HOMING_ERROR.
void JMCController::handleHoming(int method, const String& args, bool all) {
  if (method < 1 || method > 12) { reply(F("ERR:homing method H1..H12")); return; }

  bool sel[JMC_MAX_MOTORS];
  int  n = parseSelection(args, sel, all);
  if (selectionError(F("H"), n)) return;

  rampDownSelected(sel);                          // jog running? brake, don't slam
  String list = "";
  uint32_t now = millis();
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    applyHomingParams(i);
    _m[i]->performHoming(method);
    MotorCache& c = _cache[i];
    c.mode = JMCMotor::MODE_HOMING;
    c.homingArmed = true;
    c.homingBusySeen = false;
    c.homeErr = false;
    c.homed = false;
    c.homingStart = now;
    if (list.length()) list += F(",");
    list += String(i + 1);
  }
  if (n == (int)_count) reply(String(F("Homing method ")) + String(method) + F(" started for all motors"));
  else                  reply(String(F("Homing method ")) + String(method) + F(" started for M") + list);
}
