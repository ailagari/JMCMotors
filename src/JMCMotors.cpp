/*==============================================================================
  JMCMotors.cpp  -  implementation (JMCBus + JMCMotor + JMCController)
  Author: LAGARI A <lagariscience@gmail.com>  |  License: MIT
==============================================================================*/
#include "JMCMotors.h"
#include <Controllino.h>
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

JMCResult JMCMotor::enableMovement() { return ctrl(_moveBase | CW_SAMPLE_BIT); }
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

JMCResult JMCMotor::prepareVelocityMode(float velocity_rps) {
  setOperationMode(MODE_VELOCITY);
  setTargetVelocity(velocity_rps);
  _moveBase = CW_ENABLE;
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
    _storedVel[i]    = NAN;   // NAN = "not hardcoded" -> default applied at init
    _homingVel[i]    = NAN;
    _accelSet[i]     = NAN;
    _decelSet[i]     = NAN;
    _homingOffset[i] = 0;
    _offlineTry[i]   = 0;
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
  if (_m[i] && _cache[i].online) {
    _m[i]->setHomingVelocity(rps);
    _m[i]->setHomingZeroVelocity(rps);
  }
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

uint8_t JMCController::begin(uint32_t baud) {
  _bus.beginControllino(baud);
  // Short per-attempt timeouts so a dead drive can't stall the loop long.
  _bus.setRetries(1);
  _bus.setResponseTimeout(30);

  Ethernet.begin(_mac, _localIp);
  _udp.begin(_localPort);

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
    _m[i]->setHomingVelocity(_homingVel[i]);       // hardcoded or default
    _m[i]->setHomingZeroVelocity(_homingVel[i]);
    _m[i]->setHomingOffset(_homingOffset[i]);
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

// "v,v,v,..." with one field per configured motor - lets the all-motors
// forms (ZOA / HOA / HVA) reuse the positional handlers unchanged.
String JMCController::sameValueForAll(const String& value) {
  String v = value; v.trim();
  String out = "";
  for (uint8_t i = 0; i < _count; i++) { if (i) out += F(","); out += v; }
  return out;
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

// Send 'out' now if it has grown past REPLY_CHUNK_CHARS and restart it with
// the header, so per-motor lists stay bounded whatever the motor count.
bool JMCController::flushIfLong(String& out, const __FlashStringHelper* header) {
  if (out.length() < REPLY_CHUNK_CHARS) return false;
  reply(out);
  out = header;
  return true;
}

// Parse positional CSV ("1000,,2000"): value+active per motor slot.
int JMCController::parseFields(const String& csv, long* value, bool* active) {
  int count = 0;
  String work = csv + F(",");
  int start = 0;
  for (uint8_t i = 0; i < _count; i++) {
    active[i] = false; value[i] = 0;
    int comma = work.indexOf(',', start);
    if (comma < 0) continue;
    String part = work.substring(start, comma); part.trim();
    if (part.length() > 0) { value[i] = atol(part.c_str()); active[i] = true; count++; }
    start = comma + 1;
  }
  return count;
}

// Positional motor selection shared by B, H, L, Z, R - the same convention as P:
// field N = motor N, a non-empty field selects it, an empty field skips it.
// Empty argument (or "all") = every motor. Returns the count selected (0 = none).
// The field value itself is ignored (it is a flag, not a number), so "1,,1"
// picks motors 1 and 3 exactly like P:1000,,3000.
int JMCController::parseSelection(const String& csv, bool* sel) {
  String s = csv; s.trim();
  if (s.length() == 0 || s.equalsIgnoreCase("all")) {
    for (uint8_t i = 0; i < _count; i++) sel[i] = true;
    return _count;
  }
  long v[JMC_MAX_MOTORS];
  return parseFields(s, v, sel);          // positional flags -> active motors
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

  String args;
  int colon = in.indexOf(':');
  String head = (colon < 0) ? in : in.substring(0, colon);
  args = (colon < 0) ? String() : in.substring(colon + 1);
  head.trim();

  // No pairing/auto-adopt: events always go to the fixed PC address set via
  // setNetwork()/setStationId(). Replies also go back to this sender.
  _senderIp   = _udp.remoteIP();
  _senderPort = _udp.remotePort();

  // Exact-match multi-letter commands first, then single letters.
  // Every motor command has a per-motor form (positional slots) and an
  // all-motors form: either an empty/'all' selector, or the 'A' suffix for
  // the value commands (PA, PRA, VA, ADA, ZOA, HOA, HVA).
  if      (head == "PA")  handlePositionAll(args, true);
  else if (head == "PRA") handlePositionAll(args, false);
  else if (head == "PR")  handlePosition(args, false);   // relative move
  else if (head == "VA")  handleVelocityAll(args);
  else if (head == "VS")  handleVelocityStart(args);
  else if (head == "HVA") handleHomingVelocity(sameValueForAll(args));
  else if (head == "HV")  handleHomingVelocity(args);
  else if (head == "HOA") handleHomingOffset(sameValueForAll(args));
  else if (head == "HO")  handleHomingOffset(args);
  else if (head == "SD")  handleStatusDetail(args);
  else if (head == "LS")  handleLimitStatus(args);
  else if (head == "BS")  handleBrakeStatus(args);
  else if (head == "ZS")  handleZeroStatus(args);
  else if (head == "ZOA") handleZeroOffset(sameValueForAll(args));
  else if (head == "ZO")  handleZeroOffset(args);
  else if (head == "MC")  handleMotorCount(args);
  else if (head == "ADA") handleAccelDecelAll(args);
  else if (head == "AD")  handleAccelDecel(args);
  else if (head == "DG")  handleDiagnose(args);
  else if (head == "I")   handleInit(args);
  else if (head == "P")  handlePosition(args, true);
  else if (head == "V")  handleVelocity(args);
  else if (head == "S")  handleStatus(args);
  else if (head == "B")  handleBrake(F("B"), args);              // bare B = controlled stop
  else if (head.startsWith("B") && head.length() > 1 && isDigit(head[1]))
                         handleBrake(head, args);             // B1/B2/B3 (positional flags after ':')
  else if (head == "L")  handleLoosen(args);
  else if (head == "Z")  handleZero(args);
  else if (head == "R")  handleReset(args);
  else if (head == "X") {
    reply(F("System restart initiated..."));
    delay(50);
#if defined(__AVR__)
    asm volatile("jmp 0");            // soft reboot (AVR / Controllino)
#endif
  }
  else if (head.startsWith("H") && head.length() > 1 && isDigit(head[1]))
    handleHoming(head, args);
  else reply(F("INVALID COMMAND"));
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
    // fully (enable sequence, accel/decel, homing speeds, stored HO offset).
    _m[chosen]->begin(_accelSet[chosen], _decelSet[chosen]);
    _m[chosen]->setHomingVelocity(_homingVel[chosen]);
    _m[chosen]->setHomingZeroVelocity(_homingVel[chosen]);
    _m[chosen]->setHomingOffset(_homingOffset[chosen]);
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
  if (_pending.broadcast) handlePositionAll(a, _pending.absolute);
  else                    handlePosition(a, _pending.absolute);
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
//  JMCController - command handlers (JMC_CONTROLLINO_V3)
//==============================================================================

// ---- P : absolute positions (reply "OK") ------------------------------------
// P = absolute target, PR = relative delta (drive-side bit6 does the math;
// adding the current position in software was the classic double-count bug).
void JMCController::handlePosition(const String& args, bool absolute) {
  if (!args.length()) { reply(F("ERR:P needs positions")); return; }

  // '?' slots -> selective position read (P:? = M1, P:,,? = M3, ...).
  if (args.indexOf('?') >= 0) { handlePositionQuery(args); return; }

  long target[JMC_MAX_MOTORS];
  bool active[JMC_MAX_MOTORS];
  int  n;

  if (args.indexOf(',') < 0) {            // single value => motor 1
    for (uint8_t i = 0; i < _count; i++) active[i] = false;
    target[0] = atol(args.c_str());
    active[0] = true;
    n = 1;
  } else {
    n = parseFields(args, target, active);
  }
  if (n == 0) { reply(F("ERR:no positions")); return; }

  // Coming out of velocity mode (VS): the drives must reach standstill before
  // their 0x6060 mode register may change - switching mode on a turning shaft
  // stops it dead, with no ramp. Brake the whole selection on its decel ramp
  // and hand the move to servicePendingMove(), which launches it (all motors
  // together, still synchronised) as soon as they are at rest.
  if (rampDownBegin(active, _pending.braking, _pending.deadline)) {
    _pending.args      = args;      // re-run verbatim once they are at rest
    _pending.absolute  = absolute;
    _pending.broadcast = false;
    _pending.lastPoll  = millis();
    _pending.active    = true;
    return;                         // "OK" is sent when the move really starts
  }

  launchMove(active, target, absolute, false);
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
      reply(anyError ? String(F("WARN: ")) + out : out);
      out = ""; anyError = false; first = true;
    }
  }
  if (out.length()) reply(anyError ? String(F("WARN: ")) + out : out);
}

// P:? queries are positional - only motors whose slot contains '?' are read:
//   P:?    -> motor 1        P:,,?  -> motor 3        P:?,?  -> motors 1 and 2
void JMCController::handlePositionQuery(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) sel[i] = false;

  String work = args + F(",");
  int start = 0, n = 0;
  for (uint8_t i = 0; i < _count; i++) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    if (part == "?") { sel[i] = true; n++; }
    start = comma + 1;
  }
  if (n == 0) { reply(F("ERR:no '?' fields in P query")); return; }
  replyPositions(sel);
}

// ---- PA / PRA : broadcast move (absolute / relative) / PA:? read-all ---------
void JMCController::handlePositionAll(const String& args, bool absolute) {
  if (!args.length()) { reply(absolute ? String(F("ERR:PA needs position")) : String(F("ERR:PRA needs delta"))); return; }

  if (args.indexOf('?') >= 0) {           // PA:? / PRA:? -> positions of ALL motors
    bool sel[JMC_MAX_MOTORS];
    for (uint8_t i = 0; i < _count; i++) sel[i] = true;
    replyPositions(sel);
    return;
  }

  long pos = atol(args.c_str());
  bool all[JMC_MAX_MOTORS];
  long target[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) { all[i] = true; target[i] = pos; }

  // broadcastPrepareMove writes 0x6060 raw, so the same standstill rule
  // applies here as for P - see handlePosition().
  if (rampDownBegin(all, _pending.braking, _pending.deadline)) {
    _pending.args      = args;
    _pending.absolute  = absolute;
    _pending.broadcast = true;
    _pending.lastPoll  = millis();
    _pending.active    = true;
    return;                         // "OK" is sent when the move really starts
  }

  launchMove(all, target, absolute, true);
  reply(F("OK"));
}

// ---- S : status in many formats ------------------------------------------------
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

void JMCController::sendDetailStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  String mn = String(F("M")) + String(i + 1);
  if (_m[i]->lastResult() != JMC_OK) {
    reply(mn + F(": NO REPLY (") + String(JMCBus::resultName(_m[i]->lastResult())) + F(")"));
    return;
  }
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
  reply(out);
}

void JMCController::handleStatus(const String& args) {
  // "n[,FORMAT]" - n = 0 (all) or 1.._count.
  String a = args; a.trim();
  String fmt = F("RAW");
  int sel = 0;
  int comma = a.indexOf(',');
  if (comma >= 0) {
    fmt = a.substring(comma + 1); fmt.trim(); fmt.toUpperCase();
    a = a.substring(0, comma); a.trim();
  }
  sel = a.toInt();
  if (sel < 0 || sel > _count) { reply(String(F("ERR:S motor 0..")) + String(_count)); return; }

  int from = (sel == 0) ? 0 : sel - 1;
  int to   = (sel == 0) ? _count - 1 : sel - 1;

  // List formats are sent in several packets when the motor count is large.
  bool first = true;
  if (fmt == "RAW" || fmt == "") {
    String out = F("STATUS:");
    for (int i = from; i <= to; i++) {
      if (!first) out += F(",");
      first = false;
      uint16_t s = _m[i]->getStatusWord();
      if (_m[i]->lastResult() != JMC_OK) out += String(F("M")) + String(i + 1) + F("=ERROR");
      else out += String(F("M")) + String(i + 1) + F("=0x") + String(s, HEX);
      if (flushIfLong(out, F("STATUS:"))) first = true;
    }
    reply(out);
  }
  else if (fmt == "COMPACT") {
    String out = "";
    for (int i = from; i <= to; i++) {
      if (!first) out += F(",");
      first = false;
      out += compactStatus(i);
      if (flushIfLong(out, F(""))) first = true;
    }
    reply(out);
  }
  else if (fmt == "STATE") {
    String out = F("STATES:");
    for (int i = from; i <= to; i++) {
      if (!first) out += F(",");
      first = false;
      uint16_t s = _m[i]->getStatusWord();
      out += String(F("M")) + String(i + 1) + F(":");
      out += (_m[i]->lastResult() != JMC_OK) ? String(F("ERROR")) : driveStateName(s);
      if (flushIfLong(out, F("STATES:"))) first = true;
    }
    reply(out);
  }
  else if (fmt == "BITS") {
    String out = "";
    for (int i = from; i <= to; i++) {
      uint16_t s = _m[i]->getStatusWord();
      if (!first) out += F("\n");
      first = false;
      out += String(F("M")) + String(i + 1) + F(":");
      out += (_m[i]->lastResult() != JMC_OK) ? String(F("ERROR")) : bitsString(s);
      if (flushIfLong(out, F(""))) first = true;
    }
    reply(out);
  }
  else if (fmt == "FLAGS") {
    String out = "";
    for (int i = from; i <= to; i++) {
      uint16_t s = _m[i]->getStatusWord();
      if (!first) out += F("\n");
      first = false;
      out += String(F("M")) + String(i + 1) + F(":");
      out += (_m[i]->lastResult() != JMC_OK) ? String(F("ERROR")) : flagsString(s, _cache[i].mode);
      if (flushIfLong(out, F(""))) first = true;
    }
    reply(out);
  }
  else if (fmt == "DETAIL") {
    for (int i = from; i <= to; i++) sendDetailStatus(i);   // one packet per motor
  }
  else reply(F("ERR:S formats RAW|COMPACT|DETAIL|STATE|BITS|FLAGS"));
}

// ---- SD : detailed status in plain language -------------------------------------
void JMCController::sendHumanStatus(int i) {
  uint16_t s = _m[i]->getStatusWord();
  String mn = String(F("MOTOR ")) + String(i + 1);

  if (_m[i]->lastResult() != JMC_OK) {
    reply(String(F("=== ")) + mn + F(" === NOT RESPONDING (") +
          String(JMCBus::resultName(_m[i]->lastResult())) +
          F(") - check power, slave ID, RS485 wiring"));
    return;
  }

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
    out += String(F(" -> send R:")) + String(i + 1) + F(" to reset\n");
  } else {
    out += F("Health: OK, no faults\n");
  }

  reply(out);
}

void JMCController::handleStatusDetail(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  String a = args; a.trim();

  if (!a.length()) {                        // SD -> all motors
    for (uint8_t i = 0; i < _count; i++) sel[i] = true;
  }
  else if (a.indexOf('?') >= 0) {           // SD:? / SD:,,? -> positional slots
    for (uint8_t i = 0; i < _count; i++) sel[i] = false;
    String work = a + F(",");
    int start = 0, n = 0;
    for (uint8_t i = 0; i < _count; i++) {
      int comma = work.indexOf(',', start);
      if (comma < 0) break;
      String part = work.substring(start, comma); part.trim();
      if (part == "?") { sel[i] = true; n++; }
      start = comma + 1;
    }
    if (n == 0) { reply(F("ERR:no '?' fields in SD query")); return; }
  }
  else {                                    // SD:3 -> motor number also accepted
    int id = a.toInt();
    if (id < 1 || id > _count) { reply(String(F("ERR:SD use SD, SD:? or SD:1..")) + String(_count)); return; }
    for (uint8_t i = 0; i < _count; i++) sel[i] = false;
    sel[id - 1] = true;
  }

  for (uint8_t i = 0; i < _count; i++)
    if (sel[i]) sendHumanStatus(i);         // one packet per motor
}

// ---- B : brakes -----------------------------------------------------------------
// B<type>[:flags] - stop motors. The type is attached to the letter, exactly
// like H<method>:  B1 decel quick-stop, B2 fast, B3 immediate; bare B (or B0)
// is the gentle controlled stop on the decel ramp. Motor selection is
// POSITIONAL, like P: field N = motor N, a non-empty field selects it, an empty
// field skips it; empty (or 'all') = every motor.
//   B1:         deceleration brake, ALL motors
//   B2:1,,1     fast brake motors 1 and 3   (slot 2 empty = skip M2)
//   B3:,1       immediate stop, motor 2 only
//   B / B:1,,1  controlled stop: all / motors 1 and 3
void JMCController::handleBrake(const String& head, const String& args) {
  int type = (head.length() > 1) ? head.substring(1).toInt() : 0;
  if (type < 0 || type > 3) { reply(F("ERR:B type is B, B1, B2 or B3")); return; }

  bool sel[JMC_MAX_MOTORS];
  int  n = parseSelection(args, sel);       // positional selection, like P
  if (n == 0) { reply(F("ERR:B no motors selected (e.g. B2:1,,1)")); return; }

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

  bool all = (n == (int)_count);
  String tgt = all ? String(F("all motors")) : (String(F("motor ")) + list);
  if      (type == 1) reply(String(F("Deceleration brake applied to ")) + tgt);
  else if (type == 2) reply(String(F("Fast brake applied to ")) + tgt);
  else if (type == 3) reply(String(F("Immediate brake applied to ")) + tgt);
  else if (all)       reply(F("All motors stopped with deceleration"));
  else                reply(String(F("Motor ")) + list + F(" stopped with deceleration"));
}

// ---- L : loosen brakes -------------------------------------------------------------
void JMCController::handleLoosen(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);        // positional selection, like P
  if (n == 0) { reply(F("ERR:L no motors selected (e.g. L:1,,1)")); return; }

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

// ---- BS : brake status (BS = all, BS:1,,1 = selected) ---------------------------------
void JMCController::handleBrakeStatus(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);
  if (n == 0) { reply(F("ERR:BS no motors selected (e.g. BS:1,,1)")); return; }

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
  reply(out);
}

// ---- LS : limit & sensor status (LS = all, LS:1,,1 = selected) -------------------------
// One line, machine-friendly: every switch input and the homing state per
// motor, independent of which mode the drive is in.
//   LIMITS: M1:CW=0 CCW=0 SW=1 HOMED=1 MODE=POS,M2:CW=1 CCW=0 SW=0 HOMED=0 MODE=HOM
void JMCController::handleLimitStatus(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);
  if (n == 0) { reply(F("ERR:LS no motors selected (e.g. LS:1,,1)")); return; }

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
  reply(out);
}

// ---- Z : current position becomes 0 ---------------------------------------------------
void JMCController::handleZero(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);        // positional selection, like P
  if (n == 0) { reply(F("ERR:Z no motors selected (e.g. Z:1,,1)")); return; }

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
// ZO:10000,1000  ->  motor 1's CURRENT position now reads 10000, motor 2's 1000.
void JMCController::handleZeroOffset(const String& args) {
  if (!args.length()) { reply(F("ERR:ZO needs values")); return; }

  long value[JMC_MAX_MOTORS];
  bool active[JMC_MAX_MOTORS];
  int  n;

  if (args.indexOf(',') < 0) {              // single value => motor 1
    for (uint8_t i = 0; i < _count; i++) active[i] = false;
    value[0] = atol(args.c_str());
    active[0] = true;
    n = 1;
  } else {
    n = parseFields(args, value, active);
  }
  if (n == 0) { reply(F("ERR:no values")); return; }

  rampDownSelected(active);
  String out = F("ZO: ");
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!active[i]) continue;
    _m[i]->setCurrentPosition(value[i]);
    delay(20);                              // let homing method 0 latch
    _m[i]->setOperationMode(JMCMotor::MODE_POSITION);
    noteModeOne(i, JMCMotor::MODE_POSITION);
    _cache[i].zeroSet = true;
    if (!first) out += F(",");
    out += String(F("M")) + String(i + 1) + F("=") + String(value[i]);
    first = false;
  }
  reply(out + F(" (current position redefined)"));
}

// ---- ZS : zero status (ZS = all, ZS:1,,1 = selected) -------------------------------------
void JMCController::handleZeroStatus(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);
  if (n == 0) { reply(F("ERR:ZS no motors selected (e.g. ZS:1,,1)")); return; }

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
  reply(out);
}

// ---- R : reset ------------------------------------------------------------------------------
void JMCController::handleReset(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);        // positional selection, like P
  if (n == 0) { reply(F("ERR:R no motors selected (e.g. R:1,,1)")); return; }

  cancelPendingMove();
  String done = "";
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->alarmReset();
    JMCResult r = _m[i]->begin(_accelSet[i], _decelSet[i]);
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

// ---- I : detect + initialise (I = all, I:1,,1 = selected) ----------------------------------
void JMCController::handleInit(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);
  if (n == 0) { reply(F("ERR:I no motors selected (e.g. I:1,,1)")); return; }

  cancelPendingMove();
  String out = F("MOTOR INITIALIZATION:\n");
  int found = 0;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    out += String(F("Motor ")) + String(i + 1) + F(" (Slave ID ") + String(_m[i]->slaveId()) + F("): ");
    // Full init with this motor's own ramps (sketch table or runtime AD).
    JMCResult r = _m[i]->begin(_accelSet[i], _decelSet[i]);
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
  reply(out);
}

// ---- MC : motor count (runtime) --------------------------------------------
// Different projects run different bus sizes (e.g. 6 here, 12 elsewhere).
//   MC:?    read the active motor count
//   MC:12   set count to 12 (1..32); newly added slots are probed+initialised.
// Not persistent: after reboot the sketch's setMotorCount() value applies.
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
        F(" online (not saved - set in sketch with setMotorCount)"));
}

// ---- AD / ADA : acceleration / deceleration in rps/s ------------------------
// For geared mechanisms that need slow ramps. Resolution 0.1 rps/s.
// AD is positional like P - one slot per motor, each slot "acc" or "acc/dec":
//   AD:?                read per-motor values
//   AD:0.2              motor 1: accel = decel = 0.2 rps/s
//   AD:0.3/0.1          motor 1: accel 0.3, decel 0.1
//   AD:0.2,,0.5/0.2     motor 1 = 0.2/0.2, skip motor 2, motor 3 = 0.5/0.2
// ADA applies one value to every motor:
//   ADA:0.2             all motors accel = decel = 0.2
//   ADA:0.3/0.1         all motors accel 0.3, decel 0.1   (ADA:0.3,0.1 also OK)
// Written to the drives immediately AND remembered for every re-init.
// Runtime values are lost on controller power failure - hardcode final values
// with setMotionDefaults() or per motor with setMotorAccelDecel().
static bool parseAccDec(const String& field, float& acc, float& dec) {
  String f = field; f.trim();
  int slash = f.indexOf('/');
  if (slash < 0) { acc = dec = f.toFloat(); }
  else { acc = f.substring(0, slash).toFloat(); dec = f.substring(slash + 1).toFloat(); }
  return !(acc <= 0.0f || dec <= 0.0f || acc > 6553.5f || dec > 6553.5f);
}

void JMCController::handleAccelDecel(const String& args) {
  String a = args; a.trim();

  if (!a.length() || a.indexOf('?') >= 0) {
    String out = F("AD: ");
    bool first = true;
    for (uint8_t i = 0; i < _count; i++) {
      if (!first) out += F(",");
      first = false;
      out += String(F("M")) + String(i + 1) + F("=") + String(_accelSet[i], 1) + F("/") + String(_decelSet[i], 1);
      if (flushIfLong(out, F("AD: "))) first = true;
    }
    reply(out + F(" rps/s (accel/decel)"));
    return;
  }

  // Positional slots; a single field with no comma is motor 1 (like P / V).
  String work = a + F(",");
  int start = 0, n = 0;
  String out = F("AD: ");
  for (uint8_t i = 0; i < _count; i++) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    start = comma + 1;
    if (!part.length()) continue;
    float acc, dec;
    if (!parseAccDec(part, acc, dec)) { reply(String(F("ERR:AD range 0.1..6553.5 rps/s (M")) + String(i + 1) + F(")")); return; }
    _accelSet[i] = acc;
    _decelSet[i] = dec;
    _m[i]->setAccelDecel(acc, dec);   // also refreshes quick-stop ramp 0x6085
    if (n) out += F(",");
    out += String(F("M")) + String(i + 1) + F("=") + String(acc, 1) + F("/") + String(dec, 1);
    n++;
  }
  if (n == 0) { reply(F("ERR:AD needs values (AD:0.2 or AD:0.3/0.1,,0.2 or ADA:0.2)")); return; }
  reply(out + F(" rps/s (accel/decel)"));
}

void JMCController::handleAccelDecelAll(const String& args) {
  String a = args; a.trim();
  if (!a.length() || a.indexOf('?') >= 0) { handleAccelDecel(F("?")); return; }
  a.replace(',', '/');                      // accept the old "acc,dec" spelling
  float acc, dec;
  if (!parseAccDec(a, acc, dec)) { reply(F("ERR:ADA range 0.1..6553.5 rps/s")); return; }
  for (uint8_t i = 0; i < _count; i++) {
    _accelSet[i] = acc;
    _decelSet[i] = dec;
    _m[i]->setAccelDecel(acc, dec);
  }
  reply(String(F("ADA: all motors accel=")) + String(acc, 1) + F(" decel=") + String(dec, 1) + F(" rps/s"));
}

// ---- DG : low-level register diagnosis (DG:n) --------------------------------
// Reads back what the drive ACTUALLY holds (mode, targets, ramps) and tests
// both Modbus write types, so a "replies OK but doesn't move" case can be
// pinpointed remotely. 32-bit registers are shown as raw hi,lo words.
// DG / DG:0 / DG:all -> one packet per motor; DG:n -> that motor only.
void JMCController::handleDiagnose(const String& args) {
  String a = args; a.trim();
  if (!a.length() || a == "0" || a.equalsIgnoreCase("all")) {
    for (uint8_t i = 0; i < _count; i++) sendDiagnose(i);
    return;
  }
  int id = a.toInt();
  if (id < 1 || id > _count) { reply(String(F("ERR:DG usage DG, DG:all or DG:1..")) + String(_count)); return; }
  sendDiagnose((uint8_t)(id - 1));
}

void JMCController::sendDiagnose(uint8_t i) {
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
  reply(out);
}

//==============================================================================
//  JMCController - extension commands
//==============================================================================

void JMCController::handleVelocity(const String& args) {
  if (!args.length()) { reply(F("ERR:V needs argument")); return; }
  String s = args;

  if (s.indexOf('?') >= 0) {
    String out = F("V: ");
    for (uint8_t i = 0; i < _count; i++) { if (i) out += F(","); out += String(_storedVel[i]); }
    reply(out);
    return;
  }

  if (s.indexOf(',') < 0) {
    _storedVel[0] = s.toFloat();
    reply(String(F("Set Motor 1 V: ")) + String(_storedVel[0]));
    return;
  }

  String work = s + F(","); int start = 0; String out = F("Set V: "); bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    if (part.length()) {
      _storedVel[i] = part.toFloat();
      if (!first) out += F(",");
      out += String(_storedVel[i]);
      first = false;
    }
    start = comma + 1;
  }
  reply(out);
}

void JMCController::handleVelocityAll(const String& args) {
  String s = args; s.trim();
  if (!s.length()) { reply(F("ERR:VA needs velocity")); return; }
  if (s.indexOf('?') >= 0) { handleVelocity(F("?")); return; }
  float v = s.toFloat();
  if (v < 0 || v > 1000.0f) { reply(F("ERR:VA range 0-1000 rps")); return; }
  for (uint8_t i = 0; i < _count; i++) _storedVel[i] = v;
  _m[0]->broadcastSetVelocity(v);
  reply(String(F("VA: Set all motors velocity to ")) + String(v) + F(" rps (BROADCAST)"));
}

// VS / VS:all -> every motor; VS:1,,1 -> motors 1 and 3 (positional, like P).
// Each motor jogs at its own stored V speed (sign = direction).
void JMCController::handleVelocityStart(const String& args) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel);
  if (n == 0) { reply(F("ERR:VS no motors selected (e.g. VS:1,,1)")); return; }

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

void JMCController::handleHomingVelocity(const String& args) {
  if (!args.length()) { reply(F("ERR:HV needs argument")); return; }
  String s = args;
  if (s.indexOf(',') < 0) {
    float v = s.toFloat();
    _homingVel[0] = v;                  // remembered for drive re-init
    _m[0]->setHomingVelocity(v);        // 0x6099 search speed
    _m[0]->setHomingZeroVelocity(v);    // 0x609B zero-find speed
    reply(String(F("Set Motor 1 HV: ")) + String(v));
    return;
  }
  String work = s + F(","); int start = 0; String out = F("Set HV: "); bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    int comma = work.indexOf(',', start);
    if (comma < 0) break;
    String part = work.substring(start, comma); part.trim();
    if (part.length()) {
      float v = part.toFloat();
      _homingVel[i] = v;
      _m[i]->setHomingVelocity(v);
      _m[i]->setHomingZeroVelocity(v);
      if (!first) out += F(",");
      out += part;
      first = false;
    }
    start = comma + 1;
  }
  reply(out);
}

// ---- HO : homing offset --------------------------------------------------------
// Steps the motor moves AFTER the origin sensor triggers, before declaring
// zero (drive register 0x607C). Sign sets the direction. Positional like P:
//   HO:1000        motor 1 -> 1000 steps past the sensor
//   HO:1000,2000   motors 1+2
//   HO:,,500       motor 3 only
//   HO:?           read the stored offsets
// The offset is stored AND written to the drive, and re-applied automatically
// every time an H<m> homing command runs.
void JMCController::handleHomingOffset(const String& args) {
  if (!args.length()) { reply(F("ERR:HO needs steps (HO:1000 or HO:?)")); return; }
  String s = args;

  if (s.indexOf('?') >= 0) {                 // HO:? -> report stored offsets
    String out = F("HO: ");
    bool first = true;
    for (uint8_t i = 0; i < _count; i++) {
      if (!first) out += F(",");
      first = false;
      out += String(F("M")) + String(i + 1) + F("=") + String(_homingOffset[i]);
      if (flushIfLong(out, F("HO: "))) first = true;
    }
    reply(out);
    return;
  }

  long value[JMC_MAX_MOTORS];
  bool active[JMC_MAX_MOTORS];
  int  n;

  if (s.indexOf(',') < 0) {                  // single value => motor 1
    for (uint8_t i = 0; i < _count; i++) active[i] = false;
    value[0] = atol(s.c_str());
    active[0] = true;
    n = 1;
  } else {
    n = parseFields(s, value, active);
  }
  if (n == 0) { reply(F("ERR:no offset values")); return; }

  String out = F("HO: ");
  bool first = true;
  for (uint8_t i = 0; i < _count; i++) {
    if (!active[i]) continue;
    _homingOffset[i] = value[i];
    _m[i]->setHomingOffset(value[i]);
    if (!first) out += F(",");
    out += String(F("M")) + String(i + 1) + F("=") + String(value[i]);
    first = false;
  }
  reply(out + F(" steps after sensor (used at next homing)"));
}

// H<method>[:flags] - start homing on all motors, or a selected subset.
// Selection is POSITIONAL, like P: field N = motor N, a non-empty field selects
// it, an empty field skips it; empty (or 'all') = every motor. So H3:1,,1 homes
// motors 1 and 3. Each selected motor re-applies its stored HO offset, and a
// running jog is braked to standstill first (rampDownSelected).
void JMCController::handleHoming(const String& head, const String& args) {
  int method = head.substring(1).toInt();
  if (method < 1 || method > 12) { reply(F("ERR:homing method H1..H12")); return; }

  bool sel[JMC_MAX_MOTORS];
  int  n = parseSelection(args, sel);             // positional selection, like P
  if (n == 0) { reply(F("ERR:H no motors selected (e.g. H3:1,,1)")); return; }

  rampDownSelected(sel);                          // jog running? brake, don't slam
  String list = "";
  uint32_t now = millis();
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _m[i]->setHomingOffset(_homingOffset[i]);     // re-apply stored HO offset
    _m[i]->performHoming(method);
    // Arm the monitor: it pushes EVENT:HOMED / EVENT:HOMING_ERROR when the
    // drive reports completion (bit12) or error (bit13) in homing mode.
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
