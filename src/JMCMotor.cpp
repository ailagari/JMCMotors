/*==============================================================================
  JMCMotor.cpp  -  one JMC drive (position, velocity, homing, status)
  Author: LAGARI A <lagariscience@gmail.com>  |  License: MIT
==============================================================================*/
#include "JMCMotors.h"
#include <math.h>

// Control-word bit patterns (manual section "0x6040 control word").
static const uint16_t CW_INIT       = 0x0001; // parameter init, brake shut
static const uint16_t CW_BRAKE_OPEN = 0x0003; // drive power, brake open
static const uint16_t CW_ENABLE     = 0x000F; // operation enabled
static const uint16_t CW_SAMPLE_BIT = 0x0010; // bit4: new set-point / start homing (0->1 edge)
static const uint16_t CW_IMMEDIATE  = 0x0020; // bit5: new target takes effect at once
static const uint16_t CW_RELATIVE   = 0x0040; // bit6: relative positioning
static const uint16_t CW_HALT       = 0x010F; // bit8: halt (controlled stop)
static const uint16_t CW_QUICKSTOP  = 0x000B; // quick stop: bit2 = 0, bit1 = 1 (state table p.6);
                                              // bits 0 and 3 stay 1, so no "motor enable 1->0"
                                              // edge (bit table p.5) can make the motor coast
static const uint16_t CW_RESET      = 0x0080; // bit7: reset & clear error

uint8_t JMCMotor::s_epoch = 0;

// A broadcast since this motor last wrote anything: forget what it wrote.
void JMCMotor::syncEpoch() {
  if (_epoch != s_epoch) { invalidateCache(); _epoch = s_epoch; }
}

void JMCMotor::noteBroadcastStart() { syncEpoch(); _halted = false; _enabledSet = true; }

JMCMotor::JMCMotor(JMCBus& bus, uint8_t slaveId) : _bus(bus), _slave(slaveId) {}

JMCResult JMCMotor::ctrl(uint16_t value) {
  syncEpoch();
  _last = _bus.writeSingle(_slave, JMCReg::CONTROL, value);
  bool ok = (_last == JMC_OK);
  _halted = ok && (value & 0x0100) && (value & 0x000F) == 0x000F;      // enabled + halt bit
  _enabledSet = ok && (value & 0x000F) == 0x000F && !(value & 0x0100);  // enabled, running free
  return _last;
}

// rps -> speed register counts (x10 per the manual, 0.1 rps resolution).
// A nonzero request never rounds to 0 counts: target speed 0 = a move the
// drive accepts but never runs.
int32_t JMCMotor::speedToReg(float rps) const {
  int32_t v = (int32_t)lroundf(rps * _speedScale);
  if (v == 0 && rps >  0.001f) v =  1;
  if (v == 0 && rps < -0.001f) v = -1;
  return v;
}

// rps/s -> ramp register (x10 per the manual). Never 0 (= instant ramp).
uint16_t JMCMotor::accelToReg(float rps_s) {
  long v = lroundf(rps_s * 10.0f);
  if (v == 0 && rps_s > 0.001f) v = 1;
  if (v < 0)     v = 0;
  if (v > 32767) v = 32767;   // valid as UNSIGNED16 and INTEGER16 (manual sections 5 / 8)
  return (uint16_t)v;
}

JMCResult JMCMotor::write32(uint8_t slave, uint16_t reg, int32_t value) {
  uint16_t hi = (uint16_t)((uint32_t)value >> 16);
  uint16_t lo = (uint16_t)((uint32_t)value & 0xFFFF);
  uint16_t words[2];
  if (_formatMSBFirst) { words[0] = hi; words[1] = lo; }   // 0x6000 = 0
  else                 { words[0] = lo; words[1] = hi; }   // 0x6000 = 1
  _last = _bus.writeMultiple(slave, reg, words, 2);
  if (slave == 0) forgetAll();                    // a broadcast changed every drive
  else if (reg == JMCReg::TARGET_VEL) {
    syncEpoch();
    _velSet = (_last == JMC_OK);
    _velReg = value;
  }
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
  _velRunning = false;
  syncEpoch();
  invalidateCache();                              // assume nothing about the drive
  _halted = false;

  uint16_t fmt = 0;                               // 32-bit word order
  if (_bus.readHolding(_slave, JMCReg::FORMAT, &fmt, 1) == JMC_OK)
    _formatMSBFirst = (fmt == 0);

  // Manual power-up sequence: init -> brake open -> operation enabled.
  ctrl(CW_INIT);
  ctrl(CW_BRAKE_OPEN);
  // Speed 0 before enabling: a drive left in velocity mode with a jog speed
  // (controller reboot, quick stop) must not start turning on the 0x000F.
  write32(_slave, JMCReg::TARGET_VEL, 0);
  ctrl(CW_ENABLE);
  setOperationMode(MODE_POSITION);
  setAccelDecel(_accel, _decel);
  // Halt code 1 = bit8 halts on the current deceleration (drive default 0
  // is not defined in the manual).
  _bus.writeSingle(_slave, JMCReg::HALTCODE, 1);

  // Safe, slow homing defaults (zero speeds make the drive reject homing).
  setHomingAccel(_accel > 0.0f ? _accel : 5.0f);
  write32(_slave, JMCReg::HOME_SPEED,   speedToReg(0.3f));
  write32(_slave, JMCReg::HOME_ZEROSPD, speedToReg(0.3f));

  getStatusWord();                                // presence probe -> sets _last
  return _last;
}

bool JMCMotor::isPresent() {
  getStatusWord();
  return _last == JMC_OK;
}

JMCResult JMCMotor::setAccelDecel(float accel_rps_s, float decel_rps_s) {
  _accel = accel_rps_s;
  _decel = decel_rps_s;
  _rampsScaled = false;
  JMCResult r = _bus.writeSingle(_slave, JMCReg::ACCEL, accelToReg(accel_rps_s));
  if (r != JMC_OK) { _last = r; return r; }
  _last = _bus.writeSingle(_slave, JMCReg::DECEL, accelToReg(decel_rps_s));
  // Quick-stop ramp (0x6085) defaults to 0; keep it at the normal decel so
  // the fast brake (quick-stop code 2) always has a sane ramp.
  _bus.writeSingle(_slave, JMCReg::QS_DECEL, accelToReg(decel_rps_s));
  return _last;
}

void JMCMotor::restoreRamps() {
  if (_rampsScaled) setAccelDecel(_accel, _decel);
}

JMCResult JMCMotor::setOperationMode(OperationMode mode) {
  // A mode write while the shaft turns in velocity mode stops it dead:
  // brake on the decel ramp first, then switch at standstill.
  if (mode != MODE_VELOCITY) stopVelocityMode();
  syncEpoch();
  // Already in this mode and enabled (or halted, see below): nothing to send.
  if (_modeSet == mode && (_enabledSet || _halted)) { _last = JMC_OK; return _last; }
  // Halted (after B / a ramp-down): keep the halt while the new mode and
  // target are written. Releasing it first (0x000F) would let the drive
  // resume its previous position target for a moment (manual p.18: halt
  // 1->0 resumes motion); the next start pulse releases it instead.
  if (!_halted) enable();
  _last = _bus.writeSingle(_slave, JMCReg::MODE, (uint16_t)mode);
  _modeSet = (_last == JMC_OK) ? (uint8_t)mode : 0;
  return _last;
}

JMCResult JMCMotor::loosenBrake() { return ctrl(CW_BRAKE_OPEN); }
JMCResult JMCMotor::enable()      { return ctrl(CW_ENABLE); }
JMCResult JMCMotor::alarmReset()  { invalidateCache(); return ctrl(CW_RESET); }

JMCResult JMCMotor::moveToPosition(long steps, float velocity_rps, bool absolute) {
  JMCResult r = preparePositionMove(steps, velocity_rps, absolute);
  if (r != JMC_OK) return r;
  r = enableMovement();
  if (r != JMC_OK) return r;
  return startMovement();
}

// Position move. The control word keeps bit5 ("new target takes effect at
// once", as in every field-tested release) so a move sent mid-move replaces
// the one in flight; bit6 selects relative positioning, which the drive
// computes itself from its internal previous target (manual 6.1). The
// controller's PR does not use bit6: it sends actual position + delta as an
// absolute target, so the result does not depend on that hidden target.
JMCResult JMCMotor::preparePositionMove(long steps, float velocity_rps, bool absolute) {
  if (setOperationMode(MODE_POSITION) != JMC_OK) return _last;   // dead drive: stop here
  restoreRamps();
  _moveBase = (absolute ? CW_ENABLE : (CW_ENABLE | CW_RELATIVE)) | CW_IMMEDIATE;
  int32_t v = speedToReg(fabsf(velocity_rps));
  syncEpoch();
  if (!(_velSet && _velReg == v) &&                // same speed as last time: skip
      write32(_slave, JMCReg::TARGET_VEL, v) != JMC_OK) return _last;
  return write32(_slave, JMCReg::TARGET_POS, (int32_t)steps);
}

JMCResult JMCMotor::prepareScaledMove(long steps, int32_t speedReg, uint16_t accelReg,
                                      uint16_t decelReg, bool absolute) {
  if (setOperationMode(MODE_POSITION) != JMC_OK) return _last;
  _moveBase = (absolute ? CW_ENABLE : (CW_ENABLE | CW_RELATIVE)) | CW_IMMEDIATE;
  if (speedReg < 1) speedReg = 1;
  if (accelReg < 1) accelReg = 1;
  if (decelReg < 1) decelReg = 1;
  _bus.writeSingle(_slave, JMCReg::ACCEL, accelReg);
  _bus.writeSingle(_slave, JMCReg::DECEL, decelReg);
  _rampsScaled = true;
  write32(_slave, JMCReg::TARGET_VEL, speedReg);
  write32(_slave, JMCReg::TARGET_POS, (int32_t)steps);
  return _last;
}

// Manual routine: bit4 0->1 samples the new target and starts the move
// (0x1F after 0x0F); bit4 back to 0 readies the next one.
JMCResult JMCMotor::enableMovement() { return ctrl(_moveBase | CW_SAMPLE_BIT); }
JMCResult JMCMotor::startMovement()  { return ctrl(_moveBase); }

long JMCMotor::getPosition() {
  int32_t v = 0;
  read32(JMCReg::POS_ACTUAL, v);
  return (long)v;
}

long JMCMotor::getTargetPosition() {
  int32_t v = 0;
  read32(JMCReg::TARGET_POS, v);
  return (long)v;
}

JMCResult JMCMotor::setVelocity(float velocity_rps) {
  prepareVelocityMode(velocity_rps);
  return enable();
}

JMCResult JMCMotor::setTargetVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::TARGET_VEL, speedToReg(velocity_rps));
}

// Manual velocity routine: halt (0x010F), select the mode, write the speed,
// then release halt (0x000F) to start - nothing turns before the release.
JMCResult JMCMotor::prepareVelocityMode(float velocity_rps) {
  if (ctrl(CW_HALT) != JMC_OK) return _last;
  restoreRamps();
  _last = _bus.writeSingle(_slave, JMCReg::MODE, (uint16_t)MODE_VELOCITY);
  _modeSet = (_last == JMC_OK) ? (uint8_t)MODE_VELOCITY : 0;
  setTargetVelocity(velocity_rps);
  _moveBase = CW_ENABLE;                         // start = release halt
  _velRunning = true;
  return _last;
}

// Bit8 (halt) brakes on the decel ramp and holds the shaft; the drive stays
// in velocity mode meanwhile, so the later mode switch happens at rest.
JMCResult JMCMotor::beginVelocityRampDown() {
  float v = fabsf(getVelocity());
  if (_last != JMC_OK) v = 0.0f;                // no reply: don't wait on it
  float d = (_decel > 0.01f) ? _decel : 1.0f;
  long ms = lroundf(v / d * 1000.0f) + 250;     // ramp + drive reaction margin
  if (ms < 150)   ms = 150;
  if (ms > 20000) ms = 20000;
  _rampMs = (uint16_t)ms;
  setTargetVelocity(0.0f);                      // nothing to resume into
  return ctrl(CW_HALT);
}

bool JMCMotor::isSpinning(float eps_rps) {
  float v = getVelocity();
  if (_last != JMC_OK) return false;            // unreachable drive: stop waiting
  return fabsf(v) > eps_rps;
}

JMCResult JMCMotor::endVelocityRampDown() {
  _velRunning = false;
  _moveBase   = CW_ENABLE;
  return ctrl(CW_ENABLE);                       // release halt
}

JMCResult JMCMotor::stopVelocityMode(uint16_t timeout_ms) {
  if (!_velRunning) return _last;
  beginVelocityRampDown();
  uint16_t limit = timeout_ms ? timeout_ms : _rampMs;
  if (!timeout_ms && limit > BLOCKING_RAMP_CAP_MS) limit = BLOCKING_RAMP_CAP_MS;
  uint32_t t0 = millis();
  while ((millis() - t0) < limit) {
    JMCWatchdog::feed();
    if (!isSpinning()) break;
    delay(5);
  }
  return endVelocityRampDown();
}

float JMCMotor::getVelocity() {
  int32_t v = 0;
  read32(JMCReg::VEL_ACTUAL, v);
  return v / _speedScale;
}

JMCResult JMCMotor::setHomingVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::HOME_SPEED, speedToReg(fabsf(velocity_rps)));
}

JMCResult JMCMotor::setHomingZeroVelocity(float velocity_rps) {
  return write32(_slave, JMCReg::HOME_ZEROSPD, speedToReg(fabsf(velocity_rps)));
}

JMCResult JMCMotor::setHomingAccel(float accel_rps_s) {
  _last = _bus.writeSingle(_slave, JMCReg::HOME_ACCEL, accelToReg(accel_rps_s));
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
  ctrl(CW_ENABLE);                              // bit4 low
  return ctrl(CW_ENABLE | CW_SAMPLE_BIT);       // bit4 0->1 starts homing
}

JMCResult JMCMotor::setZeroPosition() { return setCurrentPosition(0); }

// Homing method 0 declares the current position the origin; the home offset
// (0x607C) shifts the readout, so the current position then reads <value>.
JMCResult JMCMotor::setCurrentPosition(long value) {
  setOperationMode(MODE_HOMING);
  _bus.writeSingle(_slave, JMCReg::HOME_METHOD, 0);
  write32(_slave, JMCReg::HOME_OFFSET, (int32_t)value);
  ctrl(CW_ENABLE);
  return ctrl(CW_ENABLE | CW_SAMPLE_BIT);
}

JMCResult JMCMotor::stop() {
  // Zero the jog setpoint too, so a stopped jog cannot resume on release.
  if (_velRunning) { _velRunning = false; setTargetVelocity(0.0f); }
  return ctrl(CW_HALT);
}

JMCResult JMCMotor::quickStop(QuickStopMode mode) {
  bool wasJogging = _velRunning;
  _velRunning = false;
  _bus.writeSingle(_slave, JMCReg::QUICKSTOP, (uint16_t)mode);  // how to stop
  JMCResult r = ctrl(CW_QUICKSTOP);                             // bit2 low: do it
  // Clear the jog setpoint too (after the stop, which comes first): the next
  // 0x000F (R, I, re-enable) must not set a velocity-mode drive running again.
  if (wasJogging) setTargetVelocity(0.0f);
  return r;
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
  st.quickStopActive    = !(s & 0x0020) && (s & 0x000F) == 0x0007;   // state table p.7
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
  forgetAll();
  _moveBase = (absolute ? CW_ENABLE : (CW_ENABLE | CW_RELATIVE)) | CW_IMMEDIATE;
  write32(0, JMCReg::TARGET_VEL, speedToReg(fabsf(velocity_rps)));
  write32(0, JMCReg::TARGET_POS, (int32_t)steps);
  return _last;
}

JMCResult JMCMotor::broadcastSetVelocity(float velocity_rps) {
  return write32(0, JMCReg::TARGET_VEL, speedToReg(velocity_rps));
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
  forgetAll();
  _last = _bus.writeSingle(0, JMCReg::CONTROL, CW_HALT);
  return _last;
}
