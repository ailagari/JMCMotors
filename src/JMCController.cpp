/*==============================================================================
  JMCController.cpp  -  N drives + text command system + event monitor
  Author: LAGARI A <lagariscience@gmail.com>  |  License: MIT

  No String objects and no heap use at all: replies are built in one
  fixed buffer (JMCOut), commands are parsed in place. Safe for weeks of
  uptime on a 2 KB AVR.
==============================================================================*/
#include "JMCMotors.h"
#if JMC_FEATURE_SAVE
#include <EEPROM.h>
#endif
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#if defined(__has_include)
  #if __has_include(<new>)
    #include <new>
  #else
    #include <new.h>
  #endif
#else
  #include <new>
#endif

#define HEAD_IS(h, lit) (strcmp_P((h), PSTR(lit)) == 0)

//==============================================================================
//  JMCOut - reply builder
//==============================================================================
void JMCOut::begin() {
  _len = 0;
  _first = true;
  if (!_isEvent && _tag[0]) s(_tag);
  if (_header) p(_header);
}

void JMCOut::reply(const __FlashStringHelper* header) {
  _isEvent = false;
  _header = header;
  _continued = false;
  begin();
}

void JMCOut::event() {
  _isEvent = true;
  _header = nullptr;
  _continued = false;
  begin();
}

void JMCOut::setTag(const char* tag) {
  size_t n = 0;
  if (tag) while (tag[n] && n < sizeof(_tag) - 1) { _tag[n] = tag[n]; n++; }
  _tag[n] = '\0';
}

void JMCOut::c(char ch) { if (_len < JMC_REPLY_BUFFER) _buf[_len++] = ch; }

void JMCOut::s(const char* str) { while (*str) c(*str++); }

void JMCOut::p(const __FlashStringHelper* str) {
  if (!str) return;
  const char* q = reinterpret_cast<const char*>(str);
  char ch;
  while ((ch = (char)pgm_read_byte(q++)) != 0) c(ch);
}

void JMCOut::n(long v) {
  char t[12];
  uint8_t i = 0;
  unsigned long u;
  if (v < 0) { c('-'); u = (unsigned long)(-(v + 1)) + 1UL; } else u = (unsigned long)v;
  do { t[i++] = (char)('0' + (u % 10)); u /= 10; } while (u && i < sizeof(t));
  while (i) c(t[--i]);
}

void JMCOut::hx(uint16_t v) {
  char t[5];
  uint8_t i = 0;
  do { uint8_t d = v & 0xF; t[i++] = (char)(d < 10 ? '0' + d : 'a' + d - 10); v >>= 4; } while (v);
  while (i) c(t[--i]);
}

void JMCOut::f(float v, uint8_t decimals) {
  if (isnan(v)) { s("nan"); return; }
  if (v < 0) { c('-'); v = -v; }
  unsigned long scale = 1;
  for (uint8_t i = 0; i < decimals; i++) scale *= 10;
  unsigned long whole = (unsigned long)(v * (float)scale + 0.5f);
  n((long)(whole / scale));
  if (decimals) {
    c('.');
    unsigned long frac = whole % scale;
    for (unsigned long d = scale / 10; d >= 1; d /= 10) { c((char)('0' + (frac / d) % 10)); if (d == 1) break; }
  }
}

void JMCOut::m(uint8_t index) { c('M'); n((long)index + 1); }

void JMCOut::need(uint8_t bytes) {
  size_t base = 0;
  if (!_isEvent && _tag[0]) base += strlen(_tag);
  if (_header) base += strlen_P(reinterpret_cast<const char*>(_header));
  if (_len > base && _len + bytes + 6 > JMC_REPLY_BUFFER) flushMore();
}

void JMCOut::item(uint8_t reserve) {
  need(reserve);
  if (!_first) c(',');
  _first = false;
}

void JMCOut::flushMore() {
  p(F(":MORE"));
  _buf[_len] = '\0';
  _link.reply(_buf, _len);
  _continued = true;
  begin();
}

void JMCOut::send() {
  if (_continued) p(F(":END"));
  _buf[_len] = '\0';
  if (_isEvent) _link.event(_buf, _len); else _link.reply(_buf, _len);
  _continued = false;
  _len = 0;
}

void JMCOut::sendPart(bool last) {
  p(last ? F(":END") : F(":MORE"));
  _buf[_len] = '\0';
  _link.reply(_buf, _len);
  _continued = false;
  _len = 0;
}

//==============================================================================
//  Status decode helpers (strings live in flash)
//==============================================================================
static const __FlashStringHelper* stateName(uint16_t s) {
  switch (s & 0x000F) {
    case 0x0: return F("Uninitialized");
    case 0x1: return F("Initialized");
    case 0x3: return F("Power on");
    case 0x7: return (s & 0x0020) ? F("Device enable") : F("Quick stop active");
    case 0xF: return F("Fault alarm");
    case 0x8: return F("Error state");
    default:  return F("Unknown");
  }
}

#if JMC_FULL_REPORTS
static const __FlashStringHelper* modeName(uint8_t mode) {
  switch (mode) {
    case JMCMotor::MODE_POSITION: return F("position");
    case JMCMotor::MODE_VELOCITY: return F("velocity");
    case JMCMotor::MODE_HOMING:   return F("homing");
    default:                      return F("unknown");
  }
}

// Bits 10, 12 and 13 change meaning with the drive's mode (manual, 0x6041).
static const __FlashStringHelper* flagName(uint8_t bit, uint8_t mode) {
  switch (bit) {
    case 0:  return F("READY");
    case 1:  return F("INIT");
    case 2:  return F("ENABLED");
    case 3:  return F("FAULT");
    case 4:  return F("WORKING");
    case 5:  return F("QSTOP");
    case 6:  return F("INITSTATE");
    case 7:  return F("WARN");
    case 8:  return F("HALT");
    case 9:  return F("RUN");
    case 10: return mode == JMCMotor::MODE_VELOCITY ? F("AT_SPEED")
                  : mode == JMCMotor::MODE_HOMING   ? F("HOME_REACHED") : F("REACHED");
    case 11: return F("SWORIGIN");
    case 12: return mode == JMCMotor::MODE_VELOCITY ? F("VEL_ZERO")
                  : mode == JMCMotor::MODE_HOMING   ? F("HOMED") : F("SETPOINT_OK");
    case 13: return mode == JMCMotor::MODE_VELOCITY ? F("MAX_ACCEL")
                  : mode == JMCMotor::MODE_HOMING   ? F("HOME_ERROR") : F("POSERR");
    case 14: return F("CWLIM");
    default: return F("CCWLIM");
  }
}
#endif

//==============================================================================
//  EEPROM (AVR: real EEPROM; ESP32: flash emulation, needs begin/commit)
//==============================================================================
namespace {
#if JMC_FEATURE_SAVE
#if JMC_ARCH_ESP32
  bool s_eeReady = false;
  void eeBegin()            { if (!s_eeReady) { EEPROM.begin(JMC_EEPROM_SIZE); s_eeReady = true; } }
  int  eeLength()           { return JMC_EEPROM_SIZE; }
  void eeWrite(int a, uint8_t v) { if (EEPROM.read(a) != v) EEPROM.write(a, v); }
  void eeCommit()           { EEPROM.commit(); }
#else
  void eeBegin()            {}
  int  eeLength()           { return (int)EEPROM.length(); }
  void eeWrite(int a, uint8_t v) { EEPROM.update(a, v); }
  void eeCommit()           {}
#endif
  uint8_t eeRead(int a)     { return EEPROM.read(a); }

  struct SavedHeader { uint32_t magic; uint8_t version; uint8_t count; uint16_t sketchCrc; uint16_t wdMs; };
  struct SavedMotor  { float vel, acc, dec, hv, hvz, hacc; int32_t hoff, smin, smax; };
  const uint32_t SAVE_MAGIC   = 0x4A4D4335UL;   // 'JMC5'
  const uint8_t  SAVE_VERSION = 3;
  const int      SAVE_BASE    = 0;

  uint16_t crcStep(uint16_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t k = 0; k < 8; k++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    return crc;
  }
  template <typename T> uint16_t crcPut(int& addr, const T& v, uint16_t crc) {
    const uint8_t* q = (const uint8_t*)&v;
    for (size_t k = 0; k < sizeof(T); k++) { eeWrite(addr++, q[k]); crc = crcStep(crc, q[k]); }
    return crc;
  }
  template <typename T> uint16_t crcGet(int& addr, T& v, uint16_t crc) {
    uint8_t* q = (uint8_t*)&v;
    for (size_t k = 0; k < sizeof(T); k++) { q[k] = eeRead(addr++); crc = crcStep(crc, q[k]); }
    return crc;
  }
  template <typename T> uint16_t crcFeed(uint16_t crc, const T& v) {
    const uint8_t* q = (const uint8_t*)&v;
    for (size_t k = 0; k < sizeof(T); k++) crc = crcStep(crc, q[k]);
    return crc;
  }
#endif // JMC_FEATURE_SAVE

  // Copy the next comma-separated field of p into out (trimmed). Returns the
  // position after the comma, or NULL when p was the last field. A field
  // longer than cap-1 is marked by out[0] = '\x01' (rejected later).
  const char* nextField(const char* p, char* out, uint8_t cap) {
    const char* e = p;
    while (*e && *e != ',') e++;
    const char* a = p;
    const char* b = e;
    while (a < b && isspace((unsigned char)*a)) a++;
    while (b > a && isspace((unsigned char)b[-1])) b--;
    size_t len = (size_t)(b - a);
    if (len > (size_t)cap - 1) { out[0] = '\x01'; out[1] = '\0'; }
    else { memcpy(out, a, len); out[len] = '\0'; }
    return *e ? e + 1 : nullptr;
  }

  bool isOff(const char* t) {
    return (toupper((unsigned char)t[0]) == 'O' && toupper((unsigned char)t[1]) == 'F' &&
            toupper((unsigned char)t[2]) == 'F' && t[3] == '\0');
  }

  // Strict number parsing: the whole field must be a number.
  // [+-]digits, within the 32-bit signed range.
  bool parseLong(const char* t, long& out) {
    bool neg = false;
    if (*t == '+' || *t == '-') neg = (*t++ == '-');
    if (!*t) return false;
    uint32_t v = 0;
    for (; *t; t++) {
      if (!isdigit((unsigned char)*t)) return false;
      uint8_t d = (uint8_t)(*t - '0');
      if (v > (2147483648UL - d) / 10) return false;      // would exceed 2^31
      v = v * 10 + d;
    }
    if (!neg && v > 2147483647UL) return false;
    out = neg ? (long)(0UL - v) : (long)v;
    return true;
  }
  // [+-]digits[.digits] - no exponents, no "inf"/"nan" (and no strtod,
  // which costs about 1.5 KB of flash on AVR).
  bool parseFloat(const char* t, float& out) {
    bool neg = false;
    if (*t == '+' || *t == '-') neg = (*t++ == '-');
    uint32_t mant = 0;
    int8_t   frac = -1;                     // digits after '.', -1 = no point yet
    uint8_t  digits = 0;
    for (; *t; t++) {
      if (*t == '.') { if (frac >= 0) return false; frac = 0; continue; }
      if (!isdigit((unsigned char)*t)) return false;
      if (mant > 99999999UL) return false;  // more than 9 significant digits
      mant = mant * 10 + (uint32_t)(*t - '0');
      digits++;
      if (frac >= 0) frac++;
    }
    if (!digits) return false;
    float v = (float)mant;
    for (int8_t k = 0; k < frac; k++) v /= 10.0f;
    out = neg ? -v : v;
    return true;
  }

  // a + b, held at the 32-bit limits instead of overflowing.
  long satAdd(long a, long b) {
    if (b > 0 && a > 2147483647L - b) return 2147483647L;
    if (b < 0 && a < (-2147483647L - 1) - b) return -2147483647L - 1;
    return a + b;
  }

  // "M3" -> 2, else -1.
  int motorIndexOf(const char* name, uint8_t count) {
    if ((name[0] != 'M' && name[0] != 'm') || !name[1]) return -1;
    long n = 0;
    for (const char* q = name + 1; *q; q++) {
      if (!isdigit((unsigned char)*q)) return -1;
      n = n * 10 + (*q - '0');
      if (n > 255) return -1;
    }
    return (n >= 1 && n <= count) ? (int)(n - 1) : -1;
  }

}

//==============================================================================
//  Lifecycle and configuration
//==============================================================================
JMCController::JMCController(JMCBus& bus, JMCLink& link)
  : _bus(bus), _link(link), _out(link) {
  memset(_m, 0, sizeof(_m));
  memset(_cache, 0, sizeof(_cache));
  memset(_offlineTry, 0, sizeof(_offlineTry));
  memset(_homingOffset, 0, sizeof(_homingOffset));
  for (uint8_t k = 0; k < 6; k++) {                 // per-motor settings: "not set"
    float* a = floatSetting(k);
    for (uint8_t i = 0; i < JMC_MAX_MOTORS; i++) a[i] = NAN;
  }
  for (uint8_t i = 0; i < JMC_MAX_MOTORS; i++) {
    _softMin[i] = 1; _softMax[i] = 0;              // min > max = no limits
    _cache[i].mode = JMCMotor::MODE_POSITION;
  }
  _cmd[0] = '\0';
  _badText[0] = '\0';
  _count = JMC_MAX_MOTORS < 6 ? JMC_MAX_MOTORS : 6;
}

void JMCController::setMotorCount(uint8_t count) {
  if (count >= 1 && count <= JMC_MAX_MOTORS) _count = count;
}

void JMCController::setMotionDefaults(float accel_rps_s, float decel_rps_s,
                                      float velocity_rps, float homingVel_rps) {
  if (accel_rps_s > 0)   _defAccel = accel_rps_s;
  if (decel_rps_s > 0)   _defDecel = decel_rps_s;
  if (velocity_rps > 0)  _defVel = velocity_rps;
  if (homingVel_rps > 0) _defHomingVel = homingVel_rps;
}

void JMCController::setDriveP45(uint8_t p45) {
  _speedScale = (p45 == 0) ? 1.0f : 10.0f;
  for (uint8_t i = 0; i < JMC_MAX_MOTORS; i++)
    if (_m[i]) _m[i]->setSpeedScale(_speedScale);
}

void JMCController::setMotorVelocity(uint8_t id, float rps) {
  if (id < 1 || id > JMC_MAX_MOTORS) return;
  uint8_t i = id - 1;
  _storedVel[i] = (rps > 0.0f || rps < 0.0f) ? rps : (_m[i] ? _defVel : NAN);
}

void JMCController::setMotorAccelDecel(uint8_t id, float accel_rps_s, float decel_rps_s) {
  if (id < 1 || id > JMC_MAX_MOTORS) return;
  uint8_t i = id - 1;
  if (!(accel_rps_s > 0.0f) || !(decel_rps_s > 0.0f)) {   // 0 would be an instant ramp
    if (!_m[i]) { _accelSet[i] = NAN; _decelSet[i] = NAN; return; }
    accel_rps_s = _defAccel; decel_rps_s = _defDecel;
  }
  _accelSet[i] = accel_rps_s;
  _decelSet[i] = decel_rps_s;
  if (_m[i] && _cache[i].online) _m[i]->setAccelDecel(accel_rps_s, decel_rps_s);
}

void JMCController::setMotorHomingVelocity(uint8_t id, float rps) {
  if (id < 1 || id > JMC_MAX_MOTORS) return;
  uint8_t i = id - 1;
  if (!(rps > 0.0f)) { if (!_m[i]) { _homingVel[i] = NAN; return; } rps = _defHomingVel; }
  _homingVel[i] = rps;
  if (_m[i] && _cache[i].online) applyHomingParams(i);
}

void JMCController::setMotorHomingZeroVelocity(uint8_t id, float rps) {
  if (id < 1 || id > JMC_MAX_MOTORS) return;
  uint8_t i = id - 1;
  _homingZeroVel[i] = (rps > 0.0f) ? rps : NAN;
  if (_m[i] && _cache[i].online) applyHomingParams(i);
}

void JMCController::setMotorHomingAccel(uint8_t id, float rps_s) {
  if (id < 1 || id > JMC_MAX_MOTORS) return;
  uint8_t i = id - 1;
  _homingAccel[i] = (rps_s > 0.0f) ? rps_s : NAN;
  if (_m[i] && _cache[i].online) applyHomingParams(i);
}

void JMCController::setMotorHomingOffset(uint8_t id, long steps) {
  if (id < 1 || id > JMC_MAX_MOTORS) return;
  uint8_t i = id - 1;
  _homingOffset[i] = steps;
  if (_m[i] && _cache[i].online) _m[i]->setHomingOffset(steps);
}

void JMCController::setMotorSoftLimits(uint8_t id, long minSteps, long maxSteps) {
  if (id < 1 || id > JMC_MAX_MOTORS || minSteps >= maxSteps) return;
  _softMin[id - 1] = minSteps;
  _softMax[id - 1] = maxSteps;
}

void JMCController::clearMotorSoftLimits(uint8_t id) {
  if (id < 1 || id > JMC_MAX_MOTORS) return;
  _softMin[id - 1] = 1;
  _softMax[id - 1] = 0;
}

void JMCController::setCommandWatchdog(uint16_t timeoutMs) {
  _wdMs = timeoutMs;
  _lastCmdMs = millis();
  _wdTripped = false;
}

void JMCController::resolveDefaults(uint8_t i) {
  if (isnan(_storedVel[i])) _storedVel[i] = _defVel;
  if (isnan(_homingVel[i])) _homingVel[i] = _defHomingVel;
  if (isnan(_accelSet[i]))  _accelSet[i]  = _defAccel;
  if (isnan(_decelSet[i]))  _decelSet[i]  = _defDecel;
}

// Write HV / HVZ / HACC / HO of motor i to its drive. HVZ follows HV and
// HACC follows the motor's accel until they are set explicitly.
void JMCController::applyHomingParams(uint8_t i) {
  if (!_m[i]) return;
  float hv   = isnan(_homingVel[i])     ? _defHomingVel : _homingVel[i];
  float hvz  = isnan(_homingZeroVel[i]) ? hv            : _homingZeroVel[i];
  float hacc = isnan(_homingAccel[i])   ? (isnan(_accelSet[i]) ? _defAccel : _accelSet[i])
                                        : _homingAccel[i];
  // The order of the manual's homing routine (p.27): 0x609A, 0x6099, 0x609B, 0x607C.
  _m[i]->setHomingAccel(hacc);
  _m[i]->setHomingVelocity(hv);
  _m[i]->setHomingZeroVelocity(hvz);
  _m[i]->setHomingOffset(_homingOffset[i]);
}

uint8_t JMCController::begin() {
  // Short per-attempt timeouts so a dead drive cannot stall the loop for long.
  _bus.setRetries(1);
  _bus.setResponseTimeout(30 + _bus.frameTimeMs(16));

  // The sketch's values are the admin's configuration; a SAVEd block made
  // under exactly these values is applied on top of them.
  _sketchCrc = sketchFingerprint();
  loadSettings(false);

  uint8_t found = 0;
  for (uint8_t i = 0; i < _count; i++) {
    JMCWatchdog::feed();
    if (initMotorSlot(i)) found++;
  }
  _out.event();
  _out.p(F("Motor ready"));
  _out.send();
  _lastCmdMs = millis();
  return found;
}

bool JMCController::initMotorSlot(uint8_t i) {
  slot(i);
  resolveDefaults(i);
  _m[i]->setSpeedScale(_speedScale);

  // Fast presence probe keeps boot quick with many empty slots.
  probeStatus(_m[i]);
  bool present = _m[i]->lastResult() == JMC_OK;

  JMCResult r = JMC_ERR_TIMEOUT;
  if (present) {
    r = _m[i]->begin(_accelSet[i], _decelSet[i]);
    applyHomingParams(i);
  }
  MotorCache& c = _cache[i];
  c.online = (r == JMC_OK);
  c.failCnt = 0;
  c.mode = JMCMotor::MODE_POSITION;
  c.homed = c.homingArmed = c.homeErr = false;
  if (r == JMC_OK) {
    uint16_t s = _m[i]->getStatusWord();
    c.status = s;
    c.cw = s & 0x4000;
    c.ccw = s & 0x8000;
    c.sw = s & 0x0800;
    c.fault = s & 0x0008;
  }
  return r == JMC_OK;
}

// One status read with a short timeout and no retry (absent / dead drives).
uint16_t JMCController::probeStatus(JMCMotor* mt) {
  uint16_t normal = _bus.responseTimeout();
  uint8_t  tries  = _bus.retries();
  _bus.setResponseTimeout(12 + _bus.frameTimeMs(16));
  _bus.setRetries(0);
  uint16_t s = mt->getStatusWord();
  _bus.setResponseTimeout(normal);
  _bus.setRetries(tries);
  return s;
}

// The first motor in fail[] whose bus traffic failed: answer
// "ERR:<cmd> Mn not accepted (<reason>)<note>" and return true.
bool JMCController::busFailed(const __FlashStringHelper* cmd, const bool* fail,
                              const __FlashStringHelper* note) {
  for (uint8_t i = 0; i < _count; i++) {
    if (!fail[i]) continue;
    errHead(cmd); _out.c(' '); _out.m(i);
    _out.p(F(" not accepted (")); _out.p(JMCBus::resultName(_bus.lastFailure())); _out.c(')');
#if JMC_FEATURE_HINTS
    _out.p(note);
#else
    (void)note;                              // Uno: the reason only, for space
#endif
    _out.send();
    return true;
  }
  return false;
}

// Enable sequence, position mode, ramps and homing parameters of motor i.
// Any write that failed on the way makes the result a failure too.
JMCResult JMCController::reinitMotor(uint8_t i) {
  JMCWatchdog::feed();
  uint16_t f0 = _bus.failures();
  JMCResult r = _m[i]->begin(_accelSet[i], _decelSet[i]);
  applyHomingParams(i);
  if (r == JMC_OK && _bus.failures() != f0) r = _bus.lastFailure();
  _cache[i].online = (r == JMC_OK);
  noteModeOne(i, JMCMotor::MODE_POSITION);
  return r;
}

// Slave IDs are 1..count: motor i is drive i + 1.
JMCMotor* JMCController::slot(uint8_t i) {
  if (!_m[i]) _m[i] = new (_pool[i]) JMCMotor(_bus, i + 1);
  return _m[i];
}

void JMCController::noteModeOne(uint8_t i, uint8_t mode) {
  _cache[i].mode = mode;
  if (mode != JMCMotor::MODE_HOMING) _cache[i].homingArmed = false;
}

void JMCController::noteMode(const bool* sel, uint8_t mode) {
  for (uint8_t i = 0; i < _count; i++) if (sel[i]) noteModeOne(i, mode);
}

void JMCController::idleDelay(uint16_t ms) {
  uint32_t t0 = millis();
  while ((uint16_t)(millis() - t0) < ms) { JMCWatchdog::feed(); delay(1); }
}

JMCMotor* JMCController::motor(uint8_t index) {
  return (index < _count) ? _m[index] : nullptr;
}

void JMCController::run() {
  JMCWatchdog::feed();
  int n = _link.receive(_cmd, sizeof(_cmd));
  if (n < 0) {
    _lastCmdMs = millis();
    _wdTripped = false;
    _tag[0] = '\0';
    _out.setTag(_tag);
    say(F("ERR:command too long"));
  } else if (n > 0) {
    handleCommand(_cmd);
  }
  servicePendingMove();
  serviceMonitor();
  serviceWatchdog();
}

//==============================================================================
//  Saved settings (EEPROM)
//==============================================================================
#if JMC_FEATURE_SAVE
uint16_t JMCController::sketchFingerprint() {
  uint16_t crc = 0xFFFF;
  crc = crcFeed(crc, _count);
  crc = crcFeed(crc, _defAccel); crc = crcFeed(crc, _defDecel);
  crc = crcFeed(crc, _defVel);   crc = crcFeed(crc, _defHomingVel);
  crc = crcFeed(crc, _speedScale);
  crc = crcFeed(crc, _wdMs);
  for (uint8_t i = 0; i < _count; i++) {
    crc = crcFeed(crc, _storedVel[i]);     crc = crcFeed(crc, _accelSet[i]);
    crc = crcFeed(crc, _decelSet[i]);      crc = crcFeed(crc, _homingVel[i]);
    crc = crcFeed(crc, _homingZeroVel[i]); crc = crcFeed(crc, _homingAccel[i]);
    crc = crcFeed(crc, _homingOffset[i]);
    crc = crcFeed(crc, _softMin[i]);       crc = crcFeed(crc, _softMax[i]);
  }
  return crc;
}

bool JMCController::saveSettings() {
  eeBegin();
  int needed = (int)sizeof(SavedHeader) + _count * (int)sizeof(SavedMotor) + 2;
  if (eeLength() < SAVE_BASE + needed) return false;
  int addr = SAVE_BASE;
  uint16_t crc = 0xFFFF;
  SavedHeader h;
  memset(&h, 0, sizeof(h));                  // padding bytes too: they are CRC'd
  h.magic = SAVE_MAGIC; h.version = SAVE_VERSION; h.count = _count;
  h.sketchCrc = _sketchCrc; h.wdMs = _wdMs;
  crc = crcPut(addr, h, crc);
  for (uint8_t i = 0; i < _count; i++) {
    SavedMotor m = { _storedVel[i], _accelSet[i], _decelSet[i], _homingVel[i],
                     _homingZeroVel[i], _homingAccel[i], (int32_t)_homingOffset[i],
                     (int32_t)_softMin[i], (int32_t)_softMax[i] };
    crc = crcPut(addr, m, crc);
  }
  uint16_t stored = crc;
  crcPut(addr, stored, 0);
  eeCommit();
  return true;
}

bool JMCController::hasSaved() {
  eeBegin();
  SavedHeader h; int addr = SAVE_BASE;
  crcGet(addr, h, 0xFFFF);
  return h.magic == SAVE_MAGIC && h.version == SAVE_VERSION &&
         h.count >= 1 && h.count <= JMC_MAX_MOTORS;
}

bool JMCController::savedMatchesSketch() {
  if (!hasSaved()) return false;
  SavedHeader h; int addr = SAVE_BASE;
  crcGet(addr, h, 0xFFFF);
  return h.sketchCrc == _sketchCrc;
}

bool JMCController::loadSettings(bool force) {
  eeBegin();
  SavedHeader h; int addr = SAVE_BASE;
  uint16_t crc = crcGet(addr, h, 0xFFFF);
  if (h.magic != SAVE_MAGIC || h.version != SAVE_VERSION ||
      h.count < 1 || h.count > JMC_MAX_MOTORS) return false;
  if (!force && h.sketchCrc != _sketchCrc) return false;     // re-flashed: sketch wins
  // Verify the whole block before applying any of it.
  int a2 = addr;
  SavedMotor tmp;
  for (uint8_t i = 0; i < h.count; i++) crc = crcGet(a2, tmp, crc);
  uint16_t stored; crcGet(a2, stored, 0);
  if (stored != crc) return false;

  uint8_t oldCount = _count;
  _count = h.count;
  if (_monIdx >= _count) _monIdx = -1;
  _wdMs = h.wdMs;
  for (uint8_t i = 0; i < h.count; i++) {
    SavedMotor m;
    crcGet(addr, m, 0);
    _storedVel[i] = m.vel; _accelSet[i] = m.acc; _decelSet[i] = m.dec;
    _homingVel[i] = m.hv;  _homingZeroVel[i] = m.hvz; _homingAccel[i] = m.hacc;
    _homingOffset[i] = m.hoff; _softMin[i] = m.smin; _softMax[i] = m.smax;
    if (_m[i] && _cache[i].online) {                    // running: apply now
      resolveDefaults(i);
      _m[i]->setAccelDecel(_accelSet[i], _decelSet[i]);
      applyHomingParams(i);
    }
  }
  for (uint8_t i = oldCount; i < _count; i++) if (_m[0]) initMotorSlot(i);  // LOAD grew the bus
  return true;
}

void JMCController::clearSaved() {
  eeBegin();
  for (size_t k = 0; k < sizeof(SavedHeader); k++) eeWrite(SAVE_BASE + (int)k, 0xFF);
  eeCommit();
}
#else
// Built without SAVE / LOAD (JMC_FEATURE_SAVE 0): nothing is ever stored.
uint16_t JMCController::sketchFingerprint() { return 0; }
bool JMCController::saveSettings()         { return false; }
bool JMCController::hasSaved()             { return false; }
bool JMCController::savedMatchesSketch()   { return false; }
bool JMCController::loadSettings(bool)     { return false; }
void JMCController::clearSaved()           {}
#endif

//==============================================================================
//  Parsing
//==============================================================================
// Values: positional slots ("1000,,3000", a single value = motor 1), motor
// names ("M3=500,M17=1000", "M3" = read), or one value for every motor
// (all). Returns the number of fields given, 0 = nothing, -1 = bad motor
// name / syntax, -2 = a value is not a valid number (_badText/_badMotor).
int JMCController::parseValues(const char* csv, Fields& Fv, bool all, uint8_t flags) {
  memset(&Fv, 0, sizeof(Fv));
  while (*csv && isspace((unsigned char)*csv)) csv++;
  if (!*csv) return 0;

  char tok[32];                              // fits "M12=-2147483648/2147483647"
  auto setField = [&](uint8_t i, const char* v) -> bool {
    Fv.active[i] = true; Fv.n++;
    if (v[0] == '?' && v[1] == '\0') { Fv.query[i] = true; Fv.nq++; return true; }
    if ((flags & PV_OFF) && isOff(v)) { Fv.off[i] = true; return true; }
    const char* slash = (flags & PV_PAIR) ? strchr(v, '/') : nullptr;
    if (slash) {
      char a[16], b[16];
      size_t la = (size_t)(slash - v);
      if (la == 0 || la >= sizeof(a) || strlen(slash + 1) >= sizeof(b)) goto bad;
      memcpy(a, v, la); a[la] = '\0';
      strcpy(b, slash + 1);
      if (!parseLong(a, Fv.l[i]) || !parseLong(b, Fv.l2[i])) goto bad;
      Fv.f[i] = (float)Fv.l[i];
      Fv.pair[i] = true;
      return true;
    }
    if (flags & PV_INT) {
      if (!parseLong(v, Fv.l[i])) goto bad;
      Fv.f[i] = (float)Fv.l[i];
    } else {
      if (!parseFloat(v, Fv.f[i])) goto bad;
      Fv.l[i] = (long)Fv.f[i];
    }
    return true;
  bad:
    strncpy(_badText, v[0] == '\x01' ? "(too long)" : v, sizeof(_badText) - 1);
    _badText[sizeof(_badText) - 1] = '\0';
    _badMotor = i;
    return false;
  };

  if (all) {
    if (strchr(csv, ',')) return -1;
    nextField(csv, tok, sizeof(tok));
    for (uint8_t i = 0; i < _count; i++) if (!setField(i, tok)) return -2;
    return _count;
  }

  if (csv[0] == 'M' || csv[0] == 'm') {                 // motor names
    const char* q = csv;
    while (q) {
      q = nextField(q, tok, sizeof(tok));
      if (!tok[0]) continue;
      char* eq = strchr(tok, '=');
      const char* val = "?";
      if (eq) { *eq = '\0'; val = eq + 1; while (*val == ' ') val++; }
      char* nameEnd = tok + strlen(tok);
      while (nameEnd > tok && nameEnd[-1] == ' ') *--nameEnd = '\0';
      int idx = motorIndexOf(tok, _count);
      if (idx < 0 || !*val) return -1;
      if (!Fv.active[idx] && !setField((uint8_t)idx, val)) return -2;
    }
    return Fv.n;
  }

  if (!strchr(csv, ',')) {                              // single value = motor 1
    nextField(csv, tok, sizeof(tok));
    return setField(0, tok) ? 1 : -2;
  }

  const char* q = csv;                                  // positional slots
  for (uint8_t i = 0; i < _count && q; i++) {
    q = nextField(q, tok, sizeof(tok));
    if (tok[0] && !setField(i, tok)) return -2;
  }
  return Fv.n;
}

// Selection: positional flags ("1,,1": any text selects) or motor names
// ("M1,M3"); all = every motor. Returns count, 0 = nothing, -1 = bad name.
int JMCController::parseSelection(const char* csv, bool* sel, bool all) {
  for (uint8_t i = 0; i < _count; i++) sel[i] = all;
  if (all) return _count;
  while (*csv && isspace((unsigned char)*csv)) csv++;
  if (!*csv) return 0;
  char tok[24];
  int n = 0;
  const char* q = csv;
  if (csv[0] == 'M' || csv[0] == 'm') {
    while (q) {
      q = nextField(q, tok, sizeof(tok));
      if (!tok[0]) continue;
      char* eq = strchr(tok, '=');
      if (eq) *eq = '\0';
      int idx = motorIndexOf(tok, _count);
      if (idx < 0) return -1;
      if (!sel[idx]) { sel[idx] = true; n++; }
    }
    return n;
  }
  for (uint8_t i = 0; i < _count && q; i++) {
    q = nextField(q, tok, sizeof(tok));
    if (tok[0]) { sel[i] = true; n++; }
  }
  return n;
}

// Commands left out of this build (Uno profile): "ERR:<HEAD> not available on this board".
void JMCController::notAvailable(const char* head) {
  _out.reply();
  _out.p(F("ERR:")); _out.s(head); _out.p(F(" not available on this board"));
  _out.send();
}

void JMCController::say(const __FlashStringHelper* msg) {
  _out.reply();
  _out.p(msg);
  _out.send();
}

void JMCController::errHead(const __FlashStringHelper* cmd) {
  _out.reply();
  _out.p(F("ERR:"));
  _out.p(cmd);
}

void JMCController::listSel(const bool* sel) {
  bool first = true;
  for (uint8_t i = 0; i < _count; i++)
    if (sel[i]) { if (!first) _out.c(','); _out.n(i + 1); first = false; }
}

void JMCController::motorRange() {
  _out.p(F(" motor must be M1..M")); _out.n(_count);
}

bool JMCController::selectionError(const __FlashStringHelper* cmd, int n) {
  if (n > 0) return false;
  errHead(cmd);
  if (n < 0) motorRange();
  else {
    _out.p(F(" needs motor slots"));
#if JMC_FEATURE_HINTS
    _out.p(F(" (e.g. ")); _out.p(cmd); _out.p(F(":1,,1 or "));
    _out.p(cmd); _out.p(F(":M1,M3) - ")); _out.p(cmd); _out.p(F("A = all motors"));
#endif
  }
  _out.send();
  return true;
}

bool JMCController::valuesError(const __FlashStringHelper* cmd, int n, bool all) {
  if (n > 0) return false;
  errHead(cmd);
  if (n == -2) {
    _out.p(F(" bad value '")); _out.s(_badText); _out.p(F("' (")); _out.m(_badMotor); _out.c(')');
  } else if (all) {
    _out.p(F("A takes one value for all motors"));
#if JMC_FEATURE_HINTS
    _out.p(F(" (e.g. ")); _out.p(cmd); _out.p(F("A:1000)"));
#endif
  } else if (n < 0) {
    motorRange();
#if JMC_FEATURE_HINTS
    _out.p(F(" (e.g. ")); _out.p(cmd); _out.p(F(":M3=1000)"));
#endif
  } else {
    _out.p(F(" needs values"));
#if JMC_FEATURE_HINTS
    _out.p(F(" (e.g. ")); _out.p(cmd); _out.p(F(":1000,,3000 or "));
    _out.p(cmd); _out.p(F(":M3=1000) - ")); _out.p(cmd); _out.p(F("A:<value> = all motors"));
#endif
  }
  _out.send();
  return true;
}

//==============================================================================
//  Command dispatch
//==============================================================================
// Motor commands, in the order of the switch in handleCommand().
enum : uint8_t { C_P, C_PR, C_PS, C_PRS, C_V, C_ACC, C_DEC, C_HV, C_HVZ, C_HACC,
                 C_VS, C_L, C_BS, C_Z, C_ZO, C_ZS, C_HO, C_SL, C_S, C_LS, C_I,
                 C_R, C_SD, C_DG, C_B, C_BN, C_HN };
static const char kMotorCmds[] PROGMEM =
  "P\0PR\0PS\0PRS\0V\0ACC\0DEC\0HV\0HVZ\0HACC\0VS\0L\0BS\0Z\0ZO\0ZS\0HO\0SL\0S\0LS\0I\0R\0SD\0DG\0B\0";

// Command id of a motor-command head, or -1. B0..B3 -> C_BN, H1..H12 -> C_HN.
static int motorCmdId(const char* h) {
  const char* q = kMotorCmds;
  for (int id = 0; pgm_read_byte(q); id++) {
    if (strcmp_P(h, q) == 0) return id;
    q += strlen_P(q) + 1;
  }
  if ((h[0] == 'B' || h[0] == 'H') && h[1]) {
    for (const char* d = h + 1; *d; d++) if (!isdigit((unsigned char)*d)) return -1;
    return h[0] == 'B' ? C_BN : C_HN;
  }
  return -1;
}

// "H12" -> 12 (digits already checked by motorCmdId).
static int headNumber(const char* head) {
  int v = 0;
  for (const char* q = head + 1; *q; q++) {
    v = v * 10 + (*q - '0');
    if (v > 99) return -1;                   // H65537 must not wrap to H1 on AVR
  }
  return v;
}

void JMCController::handleCommand(char* line) {
  // trim
  while (*line && isspace((unsigned char)*line)) line++;
  size_t len = strlen(line);
  while (len && isspace((unsigned char)line[len - 1])) line[--len] = '\0';

  // optional "#<number> " tag, echoed at the start of every reply
  _tag[0] = '\0';
  if (line[0] == '#') {
    size_t k = 1;
    while (k < 10 && isdigit((unsigned char)line[k])) k++;
    if (k > 1 && line[k] == ' ') {
      memcpy(_tag, line, k); _tag[k] = ' '; _tag[k + 1] = '\0';
      line += k + 1;
      while (*line == ' ') line++;
    }
  }
  _out.setTag(_tag);

  _lastCmdMs = millis();                         // feeds the command watchdog
  _wdTripped = false;
  if (!*line) return;

  char* colon = strchr(line, ':');
  char* args = colon ? colon + 1 : line + strlen(line);
  if (colon) *colon = '\0';
  char head[8];
  size_t hn = 0;
  for (char* q = line; *q && hn < sizeof(head) - 1; q++)
    if (!isspace((unsigned char)*q)) head[hn++] = (char)toupper((unsigned char)*q);
  head[hn] = '\0';
  while (*args && isspace((unsigned char)*args)) args++;

  // ---- controller commands -------------------------------------------------
  if (HEAD_IS(head, "MC"))   { cmdMotorCount(args); return; }
  if (HEAD_IS(head, "WD"))   { cmdWatchdog(args); return; }
  if (HEAD_IS(head, "SAVE")) { cmdSave(args); return; }
  if (HEAD_IS(head, "LOAD")) { cmdLoad(); return; }
  if (HEAD_IS(head, "PING")) { say(F("PONG")); return; }
  if (HEAD_IS(head, "VER")) {
    _out.reply();
    _out.p(F("JMCMotors " JMCMOTORS_VERSION " (max ")); _out.n(JMC_MAX_MOTORS); _out.p(F(" motors)"));
    _out.send();
    return;
  }
  if (HEAD_IS(head, "X")) {
#if !JMC_FEATURE_EXTRAS
    notAvailable(head);
    return;
#endif
    say(F("System restart initiated..."));
    idleDelay(50);
#if JMC_ARCH_AVR
    asm volatile("jmp 0");
#elif JMC_ARCH_ESP32
    ESP.restart();
#endif
    return;
  }

  // ---- motor commands: CMD:<slots> individual, CMDA[:value] all ------------
  bool all = false;
  int id = motorCmdId(head);
  if (id < 0 && hn >= 2 && head[hn - 1] == 'A') {
    head[hn - 1] = '\0';
    id = motorCmdId(head);
    if (id >= 0) all = true; else head[hn - 1] = 'A';
  }

  switch (id) {
    case C_P:    cmdPosition(args, true,  all); break;
    case C_PR:   cmdPosition(args, false, all); break;
#if JMC_FEATURE_SYNC
    case C_PS:   cmdSyncMove(args, true,  all); break;
    case C_PRS:  cmdSyncMove(args, false, all); break;
#else
    case C_PS:
    case C_PRS:  notAvailable(head); break;
#endif
    case C_V:    cmdFloat(0, args, all); break;
    case C_ACC:  cmdFloat(1, args, all); break;
    case C_DEC:  cmdFloat(2, args, all); break;
    case C_HV:   cmdFloat(3, args, all); break;
#if JMC_FEATURE_EXTRAS
    case C_HVZ:  cmdFloat(4, args, all); break;
    case C_HACC: cmdFloat(5, args, all); break;
#else
    case C_HVZ:
    case C_HACC: notAvailable(head); break;
#endif
    case C_VS:   cmdJog(args, all); break;
    case C_L:    cmdLoosen(args, all); break;
#if JMC_FEATURE_EXTRA_STATUS
    case C_BS:   cmdBrakeStatus(args, all); break;
    case C_ZS:   cmdZeroStatus(args, all); break;
#else
    case C_BS:
    case C_ZS:   notAvailable(head); break;
#endif
    case C_Z:    cmdZero(args, all); break;
    case C_ZO:   cmdZeroOffset(args, all); break;
    case C_HO:   cmdHomingOffset(args, all); break;
    case C_SL:   cmdSoftLimits(args, all); break;
    case C_S:    cmdStatus(args, all); break;
    case C_LS:   cmdLimitStatus(args, all); break;
    case C_I:    cmdInit(args, all); break;
    case C_R:    cmdReset(args, all); break;
#if JMC_FULL_REPORTS
    case C_SD:   cmdStatusDetail(args, all); break;
    case C_DG:   cmdDiagnose(args, all); break;
#else
    case C_SD:
    case C_DG:   notAvailable(head); break;
#endif
    case C_B:    cmdBrake(0, args, all); break;
    case C_BN:   cmdBrake(headNumber(head), args, all); break;
    case C_HN:   cmdHoming(headNumber(head), args, all); break;
    default:
      errHead(F("unknown command ")); _out.s(head);
      _out.send();
  }
}

//==============================================================================
//  Background monitor, command watchdog
//==============================================================================
void JMCController::serviceMonitor() {
  uint32_t now = millis();
  if (now - _lastTick < MONITOR_TICK_MS) return;
  _lastTick = now;

  // Online motors every sweep; offline ones only every OFFLINE_RETRY_MS.
  int chosen = -1;
  for (uint8_t scan = 0; scan < _count; scan++) {
    _monIdx = (_monIdx + 1) % _count;
    if (_cache[_monIdx].online) { chosen = _monIdx; break; }
    if (now - _offlineTry[_monIdx] >= OFFLINE_RETRY_MS) { chosen = _monIdx; break; }
  }
  if (chosen < 0 || !_m[chosen]) return;

  MotorCache& c = _cache[chosen];
  JMCMotor* mt = _m[chosen];
  uint16_t s;
  if (!c.online) {
    _offlineTry[chosen] = now;
    s = probeStatus(mt);                     // dead drive: short timeout, no retry
  } else {
    s = mt->getStatusWord();
  }
  bool ok = (mt->lastResult() == JMC_OK);

  if (!ok) {
    if (c.online && ++c.failCnt >= 2) {
      c.online = false;
      c.failCnt = 0;
      evHead(F("EVENT:OFFLINE:"), chosen); _out.send();
    }
    return;
  }

  c.failCnt = 0;
  if (!c.online) {
    // The drive (re)appeared - perhaps power-cycled: re-initialise fully.
    reinitMotor(chosen);
    c.online = true;
    s = mt->getStatusWord();
    c.homed = c.homeErr = false;
    c.sw = s & 0x0800;
    evHead(F("EVENT:ONLINE:"), chosen);
    stWord(s); _out.p(F(" (re-initialised)")); _out.send();
  }

  bool cw = s & 0x4000, ccw = s & 0x8000, sw = s & 0x0800, fault = s & 0x0008;

  // Switch edges: CW / CCW limit, origin. An edge to "active" carries the
  // position and status word, a release just the name.
  {
    bool nowOn[3] = { cw, ccw, sw };
    bool wasOn[3] = { (bool)c.cw, (bool)c.ccw, (bool)c.sw };
    long pos = 0;
    bool havePos = false;
    for (uint8_t k = 0; k < 3; k++) {
      if (nowOn[k] == wasOn[k]) continue;
      evHead(k == 2 ? F("EVENT:ORIGIN:") : F("EVENT:LIMIT:"), chosen);
      _out.p(k == 0 ? F(":CW") : k == 1 ? F(":CCW") : F(":"));
      if (nowOn[k]) {
        if (!havePos) { pos = mt->getPosition(); havePos = true; }
        _out.p(k == 2 ? F("ON POS=") : F(" POS=")); _out.n(pos); stWord(s);
      } else {
        _out.p(k == 2 ? F("OFF") : F("_CLEARED"));
      }
      _out.send();
    }
  }

  // Homing progress (bit12 complete / bit13 error, homing mode only). bit12
  // may still be set from a previous run right after the start pulse, so
  // completion counts once it was seen low, or once the motor is at rest
  // well after the start.
  if (c.homingArmed && c.mode == JMCMotor::MODE_HOMING) {
    bool done = s & 0x1000, err = s & 0x2000, run = s & 0x0200;
    if (err && !c.homeErr) {
      c.homeErr = true; c.homingArmed = false;
      evHead(F("EVENT:HOMING_ERROR:"), chosen); stWord(s);
      _out.p(F(" (check limit/origin wiring, HV, HO)")); _out.send();
    } else if (!done) {
      c.homingBusySeen = true;
    } else if (c.homingBusySeen || (!run && (now - c.homingStart) > 1000)) {
      c.homed = true; c.homingArmed = false;
      evHead(F("EVENT:HOMED:"), chosen); _out.p(F(" POS=")); _out.n(mt->getPosition());
      stWord(s); _out.send();
    }
  }

  // Soft limits while jogging: stop at the limit in the direction of travel.
  if (mt->velocityModeRunning() && _softMin[chosen] <= _softMax[chosen]) {
    long pos = mt->getPosition();
    if (mt->lastResult() == JMC_OK) {
      bool neg = c.jogNeg;                     // direction the jog was started in
      if ((!neg && pos >= _softMax[chosen]) || (neg && pos <= _softMin[chosen])) {
        mt->stop();
        evHead(F("EVENT:SOFTLIMIT:"), chosen); _out.p(F(" POS=")); _out.n(pos);
        _out.p(F(" LIMIT=")); _out.n(neg ? _softMin[chosen] : _softMax[chosen]); _out.send();
      }
    }
  }

  if (fault && !c.fault) {
    JMCMotor::ErrorFlags e = mt->getErrorFlags();
    evHead(F("EVENT:FAULT:"), chosen); stWord(s);
    _out.p(F(" ERR=0x")); _out.hx(e.raw);
#if JMC_FEATURE_HINTS
    if (e.valid) {
      // 0x1001 bits 1..7 (bit 6 unused); bit 0 alone = generic
      static const char names[] PROGMEM = " CURRENT\0 VOLTAGE\0 TEMP\0 COMM\0 OVERPOS\0\0 PHASE\0";
      const char* q = names;
      bool named = false;
      for (uint8_t b = 1; b <= 7; b++, q += strlen_P(q) + 1)
        if ((e.raw >> b) & 1 && pgm_read_byte(q)) { _out.p(reinterpret_cast<const __FlashStringHelper*>(q)); named = true; }
      if (e.generic && !named) _out.p(F(" GENERIC"));
    }
#endif                                       // Uno: the raw ERR= code only
    _out.send();
  }
  if (!fault && c.fault) { evHead(F("EVENT:FAULT_CLEARED:"), chosen); _out.send(); }

  c.cw = cw; c.ccw = ccw; c.sw = sw; c.fault = fault; c.status = s;
}

void JMCController::evHead(const __FlashStringHelper* what, uint8_t i) {
  _out.event();
  _out.p(what);
  _out.m(i);
}

void JMCController::stWord(uint16_t s) {
  _out.p(F(" ST=0x"));
  _out.hx(s);
}

// Stop type 0 = halt on the decel ramp (B), 1..3 = quick-stop codes (B1..B3).
// Every motor: one broadcast first, so all drives stop in the same instant
// and a dead drive cannot delay the others; then each selected online drive
// is told again individually, which also confirms it heard the stop.
// fail[i] = that drive did not answer. Returns true if any failed.
bool JMCController::stopMotors(uint8_t type, const bool* sel, bool every, bool* fail) {
  if (every) {
    if (type) _bus.writeSingle(0, JMCReg::QUICKSTOP, type);
    _bus.writeSingle(0, JMCReg::CONTROL, type ? 0x000B : 0x010F);   // quick stop / halt
  }
  bool any = false;
  for (uint8_t i = 0; i < _count; i++) {
    fail[i] = false;
    if (!sel[i] || !_m[i] || !_cache[i].online) continue;
    uint16_t f0 = _bus.failures();
    if (type == 0) _m[i]->stop();
    else _m[i]->quickStop((JMCMotor::QuickStopMode)type);
    fail[i] = _bus.failures() != f0;
    if (fail[i]) any = true;
  }
  return any;
}

void JMCController::stopAll(uint8_t type) {
  bool sel[JMC_MAX_MOTORS], fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) sel[i] = true;
  stopMotors(type, sel, true, fail);
}

// Communication watchdog: no command for _wdMs -> stop everything once.
void JMCController::serviceWatchdog() {
  if (!_wdMs || _wdTripped) return;
  if ((uint32_t)(millis() - _lastCmdMs) < _wdMs) return;
  _wdTripped = true;
  cancelPendingMove();
  stopAll(0);
  _out.event();
  _out.p(F("EVENT:WATCHDOG:STOP no command for ")); _out.n(_wdMs); _out.p(F(" ms - all motors stopped"));
  _out.send();
}

//==============================================================================
//  Motion helpers
//==============================================================================
// Velocity -> position/homing handoff: brake every selected jogging motor on
// its decel ramp, wait once for the group, then let the caller switch mode.
bool JMCController::rampDownBegin(const bool* sel, bool* braking, uint32_t& deadline) {
  cancelPendingMove();
  uint16_t limit = 0;
  bool any = false;
  for (uint8_t i = 0; i < _count; i++) {
    braking[i] = sel[i] && _m[i] && _m[i]->velocityModeRunning();
    if (!braking[i]) continue;
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

void JMCController::rampDownEnd(const bool* braking) {
  for (uint8_t i = 0; i < _count; i++)
    if (braking[i]) _m[i]->endVelocityRampDown();
}

// Blocking form for Z / ZO / H (setup commands), capped at 5 s.
void JMCController::rampDownSelected(const bool* sel) {
  bool braking[JMC_MAX_MOTORS];
  uint32_t deadline;
  if (!rampDownBegin(sel, braking, deadline)) return;
  uint32_t cap = millis() + JMCMotor::BLOCKING_RAMP_CAP_MS;
  if ((int32_t)(deadline - cap) > 0) deadline = cap;
  while ((int32_t)(millis() - deadline) < 0) {
    JMCWatchdog::feed();
    if (rampDownDone(braking)) break;
    delay(5);
  }
  rampDownEnd(braking);
}

void JMCController::servicePendingMove() {
  if (!_pending.active) return;
  uint32_t now = millis();
  bool expired = (int32_t)(now - _pending.deadline) >= 0;
  if (!expired) {
    if (now - _pending.lastPoll < RAMP_POLL_MS) return;
    _pending.lastPoll = now;
    if (!rampDownDone(_pending.braking)) return;
  }
  rampDownEnd(_pending.braking);
  _pending.active = false;
  char args[JMC_CMD_BUFFER];
  strncpy(args, _pending.args, sizeof(args) - 1);
  args[sizeof(args) - 1] = '\0';
  memcpy(_tag, _pending.tag, sizeof(_tag));
  _out.setTag(_tag);
#if JMC_FEATURE_SYNC
  if (_pending.kind == 1) { cmdSyncMove(args, _pending.absolute, _pending.all); return; }
#endif
  cmdPosition(args, _pending.absolute, _pending.all);
}

void JMCController::cancelPendingMove() {
  if (!_pending.active) return;
  _pending.active = false;
  rampDownEnd(_pending.braking);
}

// Launch prepared moves. Every motor: one broadcast pair, so all drives start
// in the same instant. A subset: the bit4 edges back to back (one frame,
// about 1.5 ms at 115200, between motors), then bit4 low again.
void JMCController::launchPrepared(const bool* sel, uint8_t n, bool* fail) {
  int first = -1;
  for (uint8_t i = 0; i < _count; i++) if (sel[i] && _m[i]) { first = i; break; }
  if (first < 0) return;
  // Broadcast only when every configured motor is part of the move: a drive
  // left out would otherwise start its previously loaded target.
  if (n == _count) {
    _m[first]->broadcastEnableMovement();
    delay(2);                                 // bit 4 high for 2 ms: no reply to pace it
    _m[first]->broadcastStartMovement();
    return;
  }
  for (uint8_t i = 0; i < _count; i++)
    if (sel[i] && _m[i] && _m[i]->enableMovement() != JMC_OK && fail) fail[i] = true;
  for (uint8_t i = 0; i < _count; i++)
    if (sel[i] && _m[i] && _m[i]->startMovement() != JMC_OK && fail) fail[i] = true;
}

// Motion commands refuse an explicitly selected motor that is offline (a
// partial coordinated move is worse than none); the all-motors form skips
// offline motors. Returns false after replying when the command must stop.
bool JMCController::onlineFilter(const __FlashStringHelper* cmd, bool* sel, bool all) {
  uint8_t left = 0;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    if (_m[i] && _cache[i].online) { left++; continue; }
    if (all) { sel[i] = false; continue; }
    errHead(cmd); _out.c(' '); _out.m(i);
    _out.p(F(" offline (check power / RS-485, or send I)"));
    _out.send();
    return false;
  }
  if (!left) {
    errHead(cmd); _out.p(F(" no motor online")); _out.send();
    return false;
  }
  return true;
}

bool JMCController::checkSoftLimits(const __FlashStringHelper* cmd, const bool* sel, const long* target) {
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i] || _softMin[i] > _softMax[i]) continue;
    if (target[i] < _softMin[i] || target[i] > _softMax[i]) {
      errHead(cmd); _out.c(' '); _out.m(i);
      _out.p(F(" target ")); _out.n(target[i]); _out.p(F(" outside soft limit "));
      _out.n(_softMin[i]); _out.p(F("..")); _out.n(_softMax[i]);
      _out.send();
      return false;
    }
  }
  return true;
}

//==============================================================================
//  Command handlers
//==============================================================================
// ---- P / PR / PA / PRA ----------------------------------------------------------
void JMCController::cmdPosition(const char* args, bool absolute, bool all) {
  const __FlashStringHelper* cmd = absolute ? F("P") : F("PR");
  Fields Fv;
  int n = parseValues(args, Fv, all, PV_INT);
  if (valuesError(cmd, n, all)) return;

  if (Fv.nq) {
    if (Fv.nq != Fv.n) {
      errHead(cmd); _out.p(F(" cannot mix values and ?")); _out.send();
      return;
    }
    replyPositions(Fv.query);
    return;
  }
  if (!onlineFilter(cmd, Fv.active, all)) return;

  // PR is sent as an absolute target: actual position + delta. The drive's
  // own relative mode (control word bit 6) adds the delta to its internal
  // previous target, which the controller cannot read back (0x607A returns
  // the last value written) and which need not match the shaft after Z, ZO
  // or homing - so the result, and any soft-limit check, would be a guess.
  // A jogging motor is braked first and its position read once at rest.
  bool jogging = false;
  for (uint8_t i = 0; i < _count; i++)
    if (Fv.active[i] && _m[i] && _m[i]->velocityModeRunning()) jogging = true;

  long target[JMC_MAX_MOTORS];
  if (absolute || !jogging) {
    for (uint8_t i = 0; i < _count; i++) {
      target[i] = Fv.l[i];
      if (absolute || !Fv.active[i] || !_m[i]) continue;
      long pos = _m[i]->getPosition();
      if (_m[i]->lastResult() != JMC_OK) {
        errHead(cmd); _out.c(' '); _out.m(i); _out.p(F(" not responding")); _out.send();
        return;
      }
      target[i] = satAdd(pos, Fv.l[i]);
    }
    if (!checkSoftLimits(cmd, Fv.active, target)) return;
  }

  // Jogging motors must brake to standstill before their mode changes; the
  // move waits in _pending and starts (all together) once they are at rest.
  if (rampDownBegin(Fv.active, _pending.braking, _pending.deadline)) {
    strncpy(_pending.args, args, sizeof(_pending.args) - 1);
    _pending.args[sizeof(_pending.args) - 1] = '\0';
    memcpy(_pending.tag, _tag, sizeof(_tag));
    _pending.kind = 0;
    _pending.absolute = absolute;
    _pending.all = all;
    _pending.lastPoll = millis();
    _pending.active = true;
    return;                                   // "OK" when the move really starts
  }

  // Prepare every motor first; if any drive did not take its parameters,
  // nothing is started (a partial coordinated move is worse than none).
  uint8_t moving = 0;
  bool fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) {
    fail[i] = false;
    if (!Fv.active[i] || !_m[i]) continue;
    uint16_t f0 = _bus.failures();
    _m[i]->preparePositionMove(target[i], _storedVel[i], true);
    fail[i] = _bus.failures() != f0;
    moving++;
  }
  noteMode(Fv.active, JMCMotor::MODE_POSITION);
  if (busFailed(cmd, fail, F(" - nothing started"))) return;
  launchPrepared(Fv.active, moving, fail);
  if (busFailed(cmd, fail, F(" to the start - check it with S"))) return;
  say(F("OK"));
}

#if JMC_FEATURE_SYNC
// ---- PS / PRS / PSA / PRSA: synchronised arrival ----------------------------------
// Every selected axis starts together and arrives together: speeds and ramps
// are scaled by distance (straight line in joint space), chosen so that no
// axis exceeds its own V / ACC / DEC. PRS deltas are taken from the actual
// position (the scaling needs the exact distances).
void JMCController::cmdSyncMove(const char* args, bool absolute, bool all) {
  const __FlashStringHelper* cmd = absolute ? F("PS") : F("PRS");
  Fields Fv;
  int n = parseValues(args, Fv, all, PV_INT);
  if (valuesError(cmd, n, all)) return;
  if (Fv.nq) { errHead(cmd); _out.p(F(" takes values; read positions with P:?")); _out.send(); return; }
  if (!onlineFilter(cmd, Fv.active, all)) return;

  if (rampDownBegin(Fv.active, _pending.braking, _pending.deadline)) {
    strncpy(_pending.args, args, sizeof(_pending.args) - 1);
    _pending.args[sizeof(_pending.args) - 1] = '\0';
    memcpy(_pending.tag, _tag, sizeof(_tag));
    _pending.kind = 1;
    _pending.absolute = absolute;
    _pending.all = all;
    _pending.lastPoll = millis();
    _pending.active = true;
    return;
  }

  long target[JMC_MAX_MOTORS];
  float dist[JMC_MAX_MOTORS];
  float dmax = 0;
  for (uint8_t i = 0; i < _count; i++) {
    dist[i] = 0;
    if (!Fv.active[i]) continue;
    long pos = _m[i] ? _m[i]->getPosition() : 0;
    if (!_m[i] || _m[i]->lastResult() != JMC_OK) {
      errHead(cmd); _out.c(' '); _out.m(i); _out.p(F(" not responding")); _out.send();
      return;
    }
    target[i] = absolute ? Fv.l[i] : satAdd(pos, Fv.l[i]);
    dist[i] = fabsf((float)target[i] - (float)pos);      // no 32-bit overflow
    if (dist[i] > dmax) dmax = dist[i];
  }
  if (!checkSoftLimits(cmd, Fv.active, target)) return;
  if (dmax <= 0) { say(F("OK")); return; }

  // Common profile: the largest v / a / dc that keeps every axis within its
  // own limits once scaled by k_i = d_i / dmax.
  float v = 1e9f, a = 1e9f, dc = 1e9f;
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i] || dist[i] <= 0) continue;
    float k = dist[i] / dmax;
    float vi = fabsf(_storedVel[i]);
    if (vi / k < v)           v  = vi / k;
    if (_accelSet[i] / k < a) a  = _accelSet[i] / k;
    if (_decelSet[i] / k < dc) dc = _decelSet[i] / k;
  }

  // The drive takes speeds in 0.1 rps and ramps in 0.1 rps/s steps, so the
  // scaled values are rounded. Each phase of the move (ramp up, cruise,
  // ramp down) of an axis stretches by its own rounding ratio, so the spread
  // of those ratios bounds how far the axes can drift apart.
  bool movingSel[JMC_MAX_MOTORS];
  uint8_t moving = 0;
  // What matters for arriving together is how differently the axes round:
  // the spread of (register value / ideal value) between the axes, per
  // parameter. Axes that all round the same way stay in step.
  float rMin[3] = { 1e9f, 1e9f, 1e9f }, rMax[3] = { 0, 0, 0 };
  bool fail[JMC_MAX_MOTORS];
  memset(fail, 0, sizeof(fail));
  for (uint8_t i = 0; i < _count; i++) {
    movingSel[i] = Fv.active[i] && dist[i] > 0;
    if (!movingSel[i]) continue;
    float k = dist[i] / dmax;
    int32_t  vr = _m[i]->speedToReg(v * k);
    uint16_t ar = JMCMotor::accelToReg(a * k);
    uint16_t dr = JMCMotor::accelToReg(dc * k);
    if (vr < 1) vr = 1;
    if (ar < 1) ar = 1;
    if (dr < 1) dr = 1;
    float r[3] = { (vr / _speedScale) / (v * k), (ar / 10.0f) / (a * k), (dr / 10.0f) / (dc * k) };
    for (uint8_t q = 0; q < 3; q++) {
      if (r[q] < rMin[q]) rMin[q] = r[q];
      if (r[q] > rMax[q]) rMax[q] = r[q];
    }
    uint16_t f0 = _bus.failures();
    _m[i]->prepareScaledMove(target[i], vr, ar, dr, true);
    fail[i] = _bus.failures() != f0;
    moving++;
  }
  float worst = 0;
  for (uint8_t q = 0; q < 3; q++)
    if (rMin[q] > 0 && rMax[q] / rMin[q] - 1.0f > worst) worst = rMax[q] / rMin[q] - 1.0f;
  noteMode(movingSel, JMCMotor::MODE_POSITION);
  if (busFailed(cmd, fail, F(" - nothing started"))) return;
  launchPrepared(movingSel, moving, fail);
  if (busFailed(cmd, fail, F(" to the start - check it with S"))) return;

  _out.reply();
  _out.p(F("OK"));
  if (worst > 0.02f) {
    _out.p(F(" (sync within ")); _out.n((long)(worst * 100 + 0.5f));
    _out.p(F("% - drive resolution is 0.1 rps and 0.1 rps/s; raise V, ACC, DEC for tighter sync)"));
  }
  _out.send();
}
#endif // JMC_FEATURE_SYNC

void JMCController::replyPositions(const bool* sel) {
  long pos[JMC_MAX_MOTORS];
  bool ok[JMC_MAX_MOTORS];
  bool anyErr = false;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    pos[i] = _m[i] ? _m[i]->getPosition() : 0;
    ok[i] = _m[i] && _m[i]->lastResult() == JMC_OK;
    if (!ok[i]) anyErr = true;
  }
  _out.reply(anyErr ? F("WARN: ") : nullptr);
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    _out.item(20);
    _out.m(i); _out.c('=');
    if (ok[i]) _out.n(pos[i]); else _out.p(F("ERROR"));
  }
  _out.send();
}

// ---- V / ACC / DEC / HV / HVZ / HACC ------------------------------------------------
// Storage of V / ACC / DEC / HV / HVZ / HACC, in that order.
float* JMCController::floatSetting(uint8_t which) {
  switch (which) {
    case 0:  return _storedVel;
    case 1:  return _accelSet;
    case 2:  return _decelSet;
    case 3:  return _homingVel;
    case 4:  return _homingZeroVel;
    default: return _homingAccel;
  }
}

void JMCController::cmdFloat(uint8_t which, const char* args, bool all) {
  // Ranges: V sign = direction; HV / HVZ max 1200 rpm (manual); ramps fit
  // the 0.1 rps/s register as a signed 16-bit value.
  static const char names[] PROGMEM = "V\0ACC\0DEC\0HV\0HVZ\0HACC\0";
  const char* q = names;
  for (uint8_t k = 0; k < which; k++) q += strlen_P(q) + 1;
  const __FlashStringHelper* cmd = reinterpret_cast<const __FlashStringHelper*>(q);
  bool isSpeed = (which == 0 || which == 3 || which == 4);
  const __FlashStringHelper* unit = isSpeed ? F(" rps") : F(" rps/s");
  float lo = isSpeed ? 0.01f : 0.1f;
  // Ramps max 3276.7 rps/s = register 32767: the manual types 0x6083 / 0x6084
  // as UNSIGNED16 (section 5) and as INTEGER16 (register list, section 8).
  float hi = (which == 0) ? 1000.0f : isSpeed ? 20.0f : 3276.7f;
  float* store = floatSetting(which);

  Fields Fv;
  int n = parseValues(args, Fv, all, 0);
  if (valuesError(cmd, n, all)) return;

  if (Fv.nq) {
    if (Fv.nq != Fv.n) { errHead(cmd); _out.p(F(" cannot mix values and ?")); _out.send(); return; }
    _out.reply(nullptr);
    _out.p(cmd); _out.p(F(": "));
    for (uint8_t i = 0; i < _count; i++) {
      if (!Fv.query[i]) continue;
      float v = store[i];                          // HVZ follows HV, HACC follows ACC
      if (isnan(v)) v = (which == 4) ? _homingVel[i] : _accelSet[i];
      _out.item(16);
      _out.m(i); _out.c('='); _out.f(v, 2);
    }
    _out.p(unit);
    _out.send();
    return;
  }

  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    float v = fabsf(Fv.f[i]);
    if (v < lo || v > hi || (which && Fv.f[i] < 0)) {
      errHead(cmd); _out.p(F(" range "));
      _out.f(lo, 2); _out.p(F("..")); _out.f(hi, 1); _out.p(unit);
      _out.p(F(" (")); _out.m(i); _out.c(')'); _out.send();
      return;
    }
  }

  bool fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) {
    fail[i] = false;
    if (!Fv.active[i]) continue;
    store[i] = Fv.f[i];                            // V: used by the next move / jog
    if (!_m[i] || !_cache[i].online || which == 0) continue;
    uint16_t f0 = _bus.failures();
    if (which <= 2) _m[i]->setAccelDecel(_accelSet[i], _decelSet[i]);
    else applyHomingParams(i);
    fail[i] = _bus.failures() != f0;
  }
  if (busFailed(cmd, fail, F(" - stored, written again at the next I / R"))) return;

  if (all) {
    if (which == 0) {                              // running jogs follow (speed and direction);
      for (uint8_t i = 0; i < _count; i++)         // drives in position mode are not touched
        if (_m[i] && _cache[i].online && _m[i]->velocityModeRunning()) {
          _m[i]->setTargetVelocity(Fv.f[0]);
          _cache[i].jogNeg = Fv.f[0] < 0;
        }
    }
    _out.reply();
    _out.p(cmd); _out.p(F("A: all motors = ")); _out.f(Fv.f[0], 2); _out.p(unit);
    _out.send();
    return;
  }
  _out.reply(nullptr);
  _out.p(F("Set ")); _out.p(cmd); _out.p(F(": "));
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    _out.item(16);
    _out.m(i); _out.c('='); _out.f(Fv.f[i], 2);
  }
  _out.p(unit);
  _out.send();
}

// ---- VS / VSA: velocity mode (endless jog) -------------------------------------------
void JMCController::cmdJog(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("VS"), n)) return;
  if (!onlineFilter(F("VS"), sel, all)) return;
  cancelPendingMove();
  uint8_t moving = 0;
  bool fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) {
    fail[i] = false;
    if (!sel[i] || !_m[i]) continue;
    uint16_t f0 = _bus.failures();
    _m[i]->prepareVelocityMode(_storedVel[i]);
    fail[i] = _bus.failures() != f0;
    _cache[i].jogNeg = _storedVel[i] < 0;
    moving++;
  }
  noteMode(sel, JMCMotor::MODE_VELOCITY);
  if (busFailed(F("VS"), fail, F(" - nothing started"))) {
    for (uint8_t i = 0; i < _count; i++) if (sel[i] && _m[i]) _m[i]->stop();   // undo the prepare
    return;
  }
  launchPrepared(sel, moving, fail);
  if (busFailed(F("VS"), fail, F(" to the start - check it with S"))) return;
  _out.reply();
  _out.p(F("VS: velocity mode started"));
  if (moving != _count) {
    _out.p(F(" for M"));
    listSel(sel);
  }
  _out.p(F(" (stop with B)"));
  _out.send();
}

// ---- B / B1 / B2 / B3 ---------------------------------------------------------------------
// A bare stop always means every motor: a stop is never refused.
void JMCController::cmdBrake(int type, const char* args, bool all) {
  if (type < 0 || type > 3) { say(F("ERR:B type is B, B1, B2 or B3")); return; }
  if (!all && !*args) all = true;
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("B"), n)) return;
  cancelPendingMove();                       // stop wins over a queued move
  bool every = (n == _count);
  bool fail[JMC_MAX_MOTORS];
  if (stopMotors((uint8_t)type, sel, every, fail) &&
      busFailed(F("B"), fail, F(" - the others were stopped"))) return;
  _out.reply();
  if (type == 0) {
    if (every) _out.p(F("All motors stopped with deceleration"));
    else {
      _out.p(F("Motor "));
      listSel(sel);
      _out.p(F(" stopped with deceleration"));
    }
  } else {
    _out.p(type == 1 ? F("Deceleration brake applied to ")
         : type == 2 ? F("Fast brake applied to ") : F("Immediate brake applied to "));
    if (every) _out.p(F("all motors"));
    else {
      _out.p(F("motor "));
      listSel(sel);
    }
  }
  _out.send();
}

// ---- L --------------------------------------------------------------------------------------
void JMCController::cmdLoosen(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("L"), n)) return;
  bool fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) fail[i] = sel[i] && _m[i] && _m[i]->loosenBrake() != JMC_OK;
  if (busFailed(F("L"), fail, F(""))) return;
  _out.reply();
  if (n == _count) _out.p(F("All motor brakes loosened"));
  else {
    _out.p(F("Motor "));
    listSel(sel);
    _out.p(F(" brake loosened"));
  }
  _out.send();
}

#if JMC_FEATURE_EXTRA_STATUS
// ---- BS -------------------------------------------------------------------------------------
void JMCController::cmdBrakeStatus(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("BS"), n)) return;
  int braked = 0, freed = 0, errors = 0;
  _out.reply(F("BRAKE STATUS: "));
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
    _out.item(20);
    _out.m(i); _out.c(':');
    if (!_m[i] || _m[i]->lastResult() != JMC_OK) { _out.p(F("ERROR")); errors++; continue; }
    bool brake = (s & 0x0100) || !(s & 0x0004);          // halted or not enabled
    _out.p(brake ? F("BRAKE") : F("FREE")); _out.c('/');
    _out.p((s & 0x0200) ? F("RUN") : F("STOP"));
    if (brake) braked++; else freed++;
  }
  _out.need(24);
  _out.p(F(" | SUMMARY: "));
  _out.p(errors == n ? F("ERROR") : (braked && !freed) ? F("BRAKED") : (freed && !braked) ? F("FREE") : F("MIXED"));
  _out.send();
}
#endif

// ---- Z / ZA -----------------------------------------------------------------------------------
void JMCController::cmdZero(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("Z"), n)) return;
  if (!onlineFilter(F("Z"), sel, all)) return;
  if (setPositions(F("Z"), sel, nullptr)) return;
  _out.reply();
  if (n == _count) _out.p(F("All motors set to zero position"));
  else {
    _out.p(F("Motor "));
    listSel(sel);
    _out.p(F(" set to zero position"));
  }
  _out.send();
}

// Z / ZO: brake jogs, then declare the current position 0 (value NULL) or
// value[i] - homing method 0, nothing moves - and return to position mode.
bool JMCController::setPositions(const __FlashStringHelper* cmd, const bool* sel, const long* value) {
  rampDownSelected(sel);
  bool fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) {
    fail[i] = false;
    if (!sel[i] || !_m[i]) continue;
    uint16_t f0 = _bus.failures();
    long want = value ? value[i] : 0;
    long got = 0;
    // Method 0 with the offset (manual p.20). The manual does not say which
    // sign the offset takes, so the result is read back: a drive that
    // applies it the other way round is set again with the sign flipped.
#if JMC_FEATURE_EXTRAS
    const uint8_t tries = 2;
#else
    const uint8_t tries = 1;                 // Uno: verified and reported, not corrected
#endif
    for (uint8_t attempt = 0; attempt < tries; attempt++) {
      _m[i]->setCurrentPosition(attempt ? -want : want);
      uint32_t t0 = millis();                // homing complete = status bit 12
      do { idleDelay(5); } while (!(_m[i]->getStatusWord() & 0x1000) && millis() - t0 < 200);
      got = _m[i]->getPosition();
      if (got == want || want == 0 || got != -want) break;
    }
    _m[i]->setOperationMode(JMCMotor::MODE_POSITION);
    noteModeOne(i, JMCMotor::MODE_POSITION);
    fail[i] = _bus.failures() != f0;
    if (!fail[i] && got != want) {
      errHead(cmd); _out.c(' '); _out.m(i); _out.p(F(" reads ")); _out.n(got);
      _out.p(F(" after setting ")); _out.n(want);
      _out.send();
      return true;
    }
    if (!fail[i]) _cache[i].zeroSet = true;
  }
  return busFailed(cmd, fail, F(" - position not set"));
}

// ---- ZO / ZOA ----------------------------------------------------------------------------------
void JMCController::cmdZeroOffset(const char* args, bool all) {
  Fields Fv;
  int n = parseValues(args, Fv, all, PV_INT);
  if (valuesError(F("ZO"), n, all)) return;
  if (Fv.nq) { say(F("ERR:ZO takes values, not ? (read positions with P:?)")); return; }
  if (!onlineFilter(F("ZO"), Fv.active, all)) return;
  if (setPositions(F("ZO"), Fv.active, Fv.l)) return;
  _out.reply(F("ZO: "));
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    _out.item(20);
    _out.m(i); _out.c('='); _out.n(Fv.l[i]);
  }
  _out.need(34);
  _out.p(F(" (current position redefined)"));
  _out.send();
}

#if JMC_FEATURE_EXTRA_STATUS
// ---- ZS ------------------------------------------------------------------------------------------
void JMCController::cmdZeroStatus(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("ZS"), n)) return;
  bool allSet = true;
  _out.reply(F("ZERO STATUS: "));
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    long p = _m[i] ? _m[i]->getPosition() : 0;
    _out.item(28);
    _out.m(i); _out.c(':');
    if (!_m[i] || _m[i]->lastResult() != JMC_OK) { _out.p(F("ERROR")); allSet = false; continue; }
    _out.p(_cache[i].zeroSet ? F("ZERO") : F("NOT_SET"));
    _out.c('('); _out.n(p); _out.c(')');
    if (!_cache[i].zeroSet) allSet = false;
  }
  _out.need(20);
  _out.p(allSet ? F(" | ALL_ZERO_SET") : F(" | ZERO_NOT_SET"));
  _out.send();
}
#endif

// ---- HO / HOA --------------------------------------------------------------------------------------
void JMCController::cmdHomingOffset(const char* args, bool all) {
  Fields Fv;
  int n = parseValues(args, Fv, all, PV_INT);
  if (valuesError(F("HO"), n, all)) return;
  bool read = Fv.nq;
  if (read && Fv.nq != Fv.n) { say(F("ERR:HO cannot mix values and ?")); return; }
  for (uint8_t i = 0; i < _count && !read; i++) {
    if (!Fv.active[i]) continue;
    _homingOffset[i] = Fv.l[i];
    if (_m[i] && _cache[i].online && _m[i]->setHomingOffset(Fv.l[i]) != JMC_OK) {
      errHead(F("HO")); _out.c(' '); _out.m(i); _out.p(F(" not accepted"));
#if JMC_FEATURE_HINTS
      _out.p(F(" - stored, written again at the next H / I / R"));
#endif
      _out.send();
      return;
    }
  }
  _out.reply(F("HO: "));
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    _out.item(20);
    _out.m(i); _out.c('='); _out.n(_homingOffset[i]);
  }
  if (!read) { _out.need(44); _out.p(F(" steps after sensor (used at next homing)")); }
  _out.send();
}

// ---- H1..H12 / H<n>A ----------------------------------------------------------------------------------
void JMCController::cmdHoming(int method, const char* args, bool all) {
  if (method < 1 || method > 12) { say(F("ERR:homing method H1..H12")); return; }
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("H"), n)) return;
  if (!onlineFilter(F("H"), sel, all)) return;
  // Manual 6.3: methods 1-4 home at most at 300 r/min (5 rps); only methods
  // 9-12 accept a negative home offset.
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    float hv  = isnan(_homingVel[i]) ? _defHomingVel : _homingVel[i];
    float hvz = isnan(_homingZeroVel[i]) ? hv : _homingZeroVel[i];
    bool fast = method <= 4 && (hv > 5.0f || hvz > 5.0f);
    bool negOff = method <= 8 && _homingOffset[i] < 0;
    if (fast || negOff) {
      errHead(F("H")); _out.n(method); _out.c(' '); _out.m(i);
      _out.p(fast ? F(" HV/HVZ above 5 rps (H1..H4: max 300 r/min)")
                  : F(" negative HO needs H9..H12"));
      _out.send();
      return;
    }
  }
  rampDownSelected(sel);
  uint32_t now = millis();
  bool fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) {
    fail[i] = false;
    if (!sel[i] || !_m[i]) continue;
    uint16_t f0 = _bus.failures();
    applyHomingParams(i);
    _m[i]->performHoming((uint8_t)method);
    fail[i] = _bus.failures() != f0;
    MotorCache& c = _cache[i];
    c.mode = JMCMotor::MODE_HOMING;
    c.homingArmed = true;
    c.homingBusySeen = false;
    c.homeErr = false;
    c.homed = false;
    c.homingStart = now;
  }
  if (busFailed(F("H"), fail, F(" - homing may not have started"))) return;
  _out.reply();
  _out.p(F("Homing method ")); _out.n(method);
  if (n == _count) _out.p(F(" started for all motors"));
  else {
    _out.p(F(" started for M"));
    listSel(sel);
  }
  _out.send();
}

// ---- SL / SLA: software position limits ---------------------------------------------------------------
void JMCController::outLimits(uint8_t i) {
  if (_softMin[i] <= _softMax[i]) { _out.n(_softMin[i]); _out.c('/'); _out.n(_softMax[i]); }
  else _out.p(F("OFF"));
}

void JMCController::cmdSoftLimits(const char* args, bool all) {
  Fields Fv;
  int n = parseValues(args, Fv, all, PV_INT | PV_PAIR | PV_OFF);
  if (valuesError(F("SL"), n, all)) return;
  bool read = Fv.nq;
  if (read && Fv.nq != Fv.n) { say(F("ERR:SL cannot mix values and ?")); return; }
  if (!read) {
    for (uint8_t i = 0; i < _count; i++) {
      if (!Fv.active[i] || Fv.off[i]) continue;
      if (!Fv.pair[i] || Fv.l[i] >= Fv.l2[i]) {
        _out.reply(); _out.p(F("ERR:SL value must be min/max with min < max, or OFF (")); _out.m(i); _out.c(')'); _out.send();
        return;
      }
    }
    for (uint8_t i = 0; i < _count; i++) {
      if (!Fv.active[i]) continue;
      if (Fv.off[i]) { _softMin[i] = 1; _softMax[i] = 0; }
      else { _softMin[i] = Fv.l[i]; _softMax[i] = Fv.l2[i]; }
    }
    if (all) {
      _out.reply();
      _out.p(F("SLA: all motors = "));
      outLimits(0);
      _out.send();
      return;
    }
  }
  _out.reply(F("SL: "));
  for (uint8_t i = 0; i < _count; i++) {
    if (!Fv.active[i]) continue;
    _out.item(32);
    _out.m(i); _out.c('='); outLimits(i);
  }
  _out.send();
}

// ---- S / SA: status word ------------------------------------------------------------------------------
void JMCController::outCompact(uint8_t i) {
  uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
  _out.m(i); _out.c(':');
  if (!_m[i] || _m[i]->lastResult() != JMC_OK) { _out.p(F("ERROR")); return; }
  _out.p((s & 0x0008) ? F("ERR") : (s & 0x0004) ? F("RUN") : F("OFF")); _out.c('|');
  _out.p((s & 0x0200) ? F("MOV") : F("STP")); _out.c('|');
  _out.p((s & 0x0400) ? F("TR") : F("NR"));
  _out.p(F("|POS=")); _out.n(_m[i]->getPosition());
  _out.p(F("|VEL=")); _out.f(_m[i]->getVelocity(), 1);
}

void JMCController::cmdStatus(const char* args, bool all) {
  // S:<slots>[:FORMAT]  or  SA[:FORMAT]
  char sel_[JMC_CMD_BUFFER];
  const char* fmt = "";
  const char* selPart = args;
  if (all) { fmt = args; selPart = ""; }
  else {
    const char* c2 = strchr(args, ':');
    if (c2) {
      size_t k = (size_t)(c2 - args);
      if (k >= sizeof(sel_)) k = sizeof(sel_) - 1;
      memcpy(sel_, args, k); sel_[k] = '\0';
      selPart = sel_;
      fmt = c2 + 1;
    }
  }
  char f[10];
  size_t fn = 0;
  for (const char* q = fmt; *q && fn < sizeof(f) - 1; q++)
    if (!isspace((unsigned char)*q)) f[fn++] = (char)toupper((unsigned char)*q);
  f[fn] = '\0';
  if (!fn) strcpy(f, "RAW");

  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(selPart, sel, all);
  if (selectionError(F("S"), n)) return;

  if (HEAD_IS(f, "RAW")) {
    _out.reply(F("STATUS:"));
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
      _out.item(16);
      _out.m(i); _out.c('=');
      if (!_m[i] || _m[i]->lastResult() != JMC_OK) _out.p(F("ERROR"));
      else { _out.p(F("0x")); _out.hx(s); }
    }
    _out.send();
    return;
  }
  if (HEAD_IS(f, "COMPACT")) {
    _out.reply();
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      _out.item(64);
      outCompact(i);
    }
    _out.send();
    return;
  }
#if JMC_FULL_REPORTS
  if (HEAD_IS(f, "STATE")) {
    _out.reply(F("STATES:"));
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
      _out.item(28);
      _out.m(i); _out.c(':');
      if (!_m[i] || _m[i]->lastResult() != JMC_OK) _out.p(F("ERROR")); else _out.p(stateName(s));
    }
    _out.send();
    return;
  }
  if (HEAD_IS(f, "BITS") || HEAD_IS(f, "FLAGS")) {
    bool bits = HEAD_IS(f, "BITS");
    _out.reply();
    bool firstLine = true;
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
      _out.need(bits ? 28 : 140);
      if (!firstLine) _out.c('\n');
      firstLine = false;
      _out.m(i); _out.c(':');
      if (!_m[i] || _m[i]->lastResult() != JMC_OK) { _out.p(F("ERROR")); continue; }
      if (bits) {
        for (int b = 15; b >= 0; b--) { _out.c((s & (1 << b)) ? '1' : '0'); if (b == 12 || b == 8 || b == 4) _out.c(' '); }
      } else {
        bool any = false;
        for (uint8_t b = 0; b < 16; b++)
          if (s & (1 << b)) { if (any) _out.c('|'); _out.p(flagName(b, _cache[i].mode)); any = true; }
        if (!any) _out.p(F("NONE"));
      }
    }
    _out.send();
    return;
  }
  if (HEAD_IS(f, "DETAIL")) {
    int left = n;
    for (uint8_t i = 0; i < _count; i++) {
      if (!sel[i]) continue;
      left--;
      _out.reply();
      outDetail(i);
      if (n == 1) _out.send(); else _out.sendPart(left == 0);
    }
    return;
  }
  say(F("ERR:S formats RAW|COMPACT|DETAIL|STATE|BITS|FLAGS"));
#else
  say(F("ERR:S formats on this board: RAW|COMPACT"));
#endif
}

// ---- LS / LSA: limit switches, origin switch, homing state -------------------------------------------------
void JMCController::cmdLimitStatus(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("LS"), n)) return;
  _out.reply(F("LIMITS: "));
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
    _out.item(48);
    _out.m(i); _out.c(':');
    if (!_m[i] || _m[i]->lastResult() != JMC_OK) { _out.p(F("ERROR")); continue; }
    MotorCache& c = _cache[i];
    bool homed = (c.mode == JMCMotor::MODE_HOMING) ? (s & 0x1000) : c.homed;
    bool herr  = (c.mode == JMCMotor::MODE_HOMING) ? (s & 0x2000) : c.homeErr;
    _out.p(F("CW="));    _out.c((s & 0x4000) ? '1' : '0');
    _out.p(F(" CCW="));  _out.c((s & 0x8000) ? '1' : '0');
    _out.p(F(" SW="));   _out.c((s & 0x0800) ? '1' : '0');
    _out.p(F(" HOMED=")); _out.p(herr ? F("ERR") : homed ? F("1") : c.homingArmed ? F("BUSY") : F("0"));
    _out.p(F(" MODE="));
    _out.p(c.mode == JMCMotor::MODE_HOMING ? F("HOM") : c.mode == JMCMotor::MODE_VELOCITY ? F("VEL") : F("POS"));
  }
  _out.send();
}

// ---- R / RA -------------------------------------------------------------------------------------------------
void JMCController::cmdReset(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("R"), n)) return;
  cancelPendingMove();
  bool fail[JMC_MAX_MOTORS];
  for (uint8_t i = 0; i < _count; i++) {
    fail[i] = false;
    if (!sel[i]) continue;
    if (!_m[i]) { fail[i] = true; continue; }
    _m[i]->alarmReset();
    fail[i] = (reinitMotor(i) != JMC_OK);
    // Fault gone: say so now. Still there: forget it, so the monitor reports
    // it again as a new EVENT:FAULT.
    uint16_t st = _m[i]->getStatusWord();
    if (_cache[i].fault && _m[i]->lastResult() == JMC_OK && !(st & 0x0008)) {
      evHead(F("EVENT:FAULT_CLEARED:"), i);
      _out.send();
    }
    _cache[i].fault = false;
    _cache[i].homeErr = false;               // alarm cleared; 'homed' is kept
  }
  _out.reply();
  if (n == _count) _out.p(F("All motors reset complete"));
  else {
    _out.p(F("Motor "));
    bool first = true;
    for (uint8_t i = 0; i < _count; i++) if (sel[i]) {
      if (!first) _out.c(',');
      _out.n(i + 1);
      if (fail[i]) _out.p(F("(FAIL)"));
      first = false;
    }
    _out.p(F(" reset complete"));
  }
  _out.send();
}

// ---- I / IA ---------------------------------------------------------------------------------------------------
void JMCController::cmdInit(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("I"), n)) return;
  cancelPendingMove();
  int found = 0;
  _out.reply();
  _out.p(F("MOTOR INITIALIZATION:\n"));
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    slot(i);
    resolveDefaults(i);
    JMCResult r = reinitMotor(i);
    _out.need(110);
    _out.p(F("Motor ")); _out.n(i + 1); _out.p(F(" (Slave ID ")); _out.n(_m[i]->slaveId()); _out.p(F("): "));
    if (r == JMC_OK) {
      found++;
      uint16_t s = _m[i]->getStatusWord();
      _out.p(F("DETECTED - Initialized in Position mode\n  Status: 0x")); _out.hx(s);
      _out.p(F(" (")); _out.p(stateName(s)); _out.p(F(")\n  Position: ")); _out.n(_m[i]->getPosition()); _out.c('\n');
    } else {
      _out.p(F("NOT DETECTED (")); _out.p(JMCBus::resultName(r));
#if JMC_FEATURE_HINTS
      _out.p(F(")\n  Check: Power, Address, RS485 wiring\n"));
#else
      _out.p(F(")\n"));
#endif
    }
  }
  _out.need(80);
  _out.p(F("Total motors detected: ")); _out.n(found); _out.p(F(" of ")); _out.n(n);
#if JMC_FEATURE_HINTS
  if (found == n) _out.p(F("\nSUCCESS: All expected motors detected and initialized!"));
#endif
  _out.send();
}

// ---- MC -----------------------------------------------------------------------------------------------------------
void JMCController::cmdMotorCount(const char* args) {
  if (!*args || strchr(args, '?')) {
    _out.reply();
    _out.p(F("MC: ")); _out.n(_count); _out.p(F(" motors (slave IDs 1..")); _out.n(_count); _out.c(')');
    _out.send();
    return;
  }
#if !JMC_FEATURE_EXTRAS
  notAvailable("MC:n");
#else
  long n;
  if (!parseLong(args, n) || n < 1 || n > JMC_MAX_MOTORS) {
    _out.reply(); _out.p(F("ERR:MC range 1..")); _out.n(JMC_MAX_MOTORS); _out.send();
    return;
  }
  uint8_t oldCount = _count;
  _count = (uint8_t)n;
  if (_monIdx >= _count) _monIdx = -1;
  int online = 0;
  for (uint8_t i = 0; i < _count; i++) {
    JMCWatchdog::feed();
    if (i < oldCount && _m[i]) { if (_cache[i].online) online++; continue; }
    if (initMotorSlot(i)) online++;
  }
  _out.reply();
  _out.p(F("MC: ")); _out.n(_count); _out.p(F(" motors, ")); _out.n(online);
  _out.p(F(" online (SAVE to keep across reboot)"));
  _out.send();
#endif
}

// ---- WD: communication watchdog -------------------------------------------------------------------------------
void JMCController::cmdWatchdog(const char* args) {
  if (!*args || strchr(args, '?')) {
    _out.reply();
    if (!_wdMs) _out.p(F("WD: off"));
    else {
      _out.p(F("WD: ")); _out.n(_wdMs); _out.p(F(" ms"));
#if JMC_FEATURE_HINTS
      _out.p(F(" - all motors stop when no command arrives for that long"));
#endif
    }
    _out.send();
    return;
  }
  long ms;
  if (!parseLong(args, ms) || (ms != 0 && (ms < 50 || ms > 65000))) {
    say(F("ERR:WD range 0 (off) or 50..65000 ms"));
    return;
  }
  setCommandWatchdog((uint16_t)ms);
  _out.reply();
  if (!ms) _out.p(F("WD: off"));
  else {
    _out.p(F("WD: ")); _out.n(ms); _out.p(F(" ms"));
#if JMC_FEATURE_HINTS
    _out.p(F(" - send any command (PING) more often than that"));
#endif
  }
  _out.send();
}

// ---- SAVE / LOAD ---------------------------------------------------------------------------------------------------
#if JMC_FEATURE_SAVE
void JMCController::cmdSave(const char* args) {
  char a[8];
  size_t k = 0;
  for (const char* q = args; *q && k < sizeof(a) - 1; q++) if (!isspace((unsigned char)*q)) a[k++] = (char)toupper((unsigned char)*q);
  a[k] = '\0';
  if (HEAD_IS(a, "?")) {
    if (!hasSaved())              say(F("SAVE: nothing stored - sketch values in use"));
    else if (savedMatchesSketch()) say(F("SAVE: settings stored and applied at boot (SAVE:CLEAR to forget)"));
    else say(F("SAVE: settings stored but IGNORED - sketch was re-flashed with other values (LOAD applies them, SAVE re-arms)"));
    return;
  }
  if (HEAD_IS(a, "CLEAR")) { clearSaved(); say(F("SAVE: cleared - sketch values apply at next boot")); return; }
  if (k) { say(F("ERR:SAVE use SAVE, SAVE:? or SAVE:CLEAR")); return; }
  if (!saveSettings()) { say(F("ERR:SAVE EEPROM too small")); return; }
  _out.reply();
  _out.p(F("SAVE: ")); _out.n(_count);
  _out.p(F(" motors stored (V, ACC, DEC, HV, HVZ, HACC, HO, SL, WD, MC)"));
  _out.send();
}

void JMCController::cmdLoad() {
  if (!loadSettings(true)) { say(F("ERR:LOAD nothing stored (or checksum failed)")); return; }
  _out.reply();
  _out.p(F("LOAD: ")); _out.n(_count); _out.p(F(" motors loaded and applied"));
  _out.send();
}
#else
void JMCController::cmdSave(const char*) { notAvailable("SAVE"); }
void JMCController::cmdLoad()            { notAvailable("LOAD"); }
#endif

//==============================================================================
//  Verbose reports (not on 2 KB boards)
//==============================================================================
#if JMC_FULL_REPORTS
void JMCController::outDetail(uint8_t i) {
  uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
  if (!_m[i] || _m[i]->lastResult() != JMC_OK) {
    _out.m(i); _out.p(F(": NO REPLY (")); _out.p(JMCBus::resultName(_m[i] ? _m[i]->lastResult() : JMC_ERR_TIMEOUT)); _out.c(')');
    return;
  }
  _out.m(i); _out.p(F(" (ID ")); _out.n(_m[i]->slaveId()); _out.p(F(")\n  Status: 0x")); _out.hx(s);
  _out.p(F(" (")); _out.p(stateName(s)); _out.p(F(")\n  Mode:   ")); _out.p(modeName(_cache[i].mode));
  _out.p(F("\n  Flags:  "));
  bool any = false;
  for (uint8_t b = 0; b < 16; b++) if (s & (1 << b)) { if (any) _out.c('|'); _out.p(flagName(b, _cache[i].mode)); any = true; }
  if (!any) _out.p(F("NONE"));
  _out.p(F("\n  Bits:   "));
  for (int b = 15; b >= 0; b--) { _out.c((s & (1 << b)) ? '1' : '0'); if (b == 12 || b == 8 || b == 4) _out.c(' '); }
  _out.p(F("\n  Pos:    ")); _out.n(_m[i]->getPosition());
  _out.p(F("\n  Vel:    ")); _out.f(_m[i]->getVelocity(), 1); _out.p(F(" rps"));
  if (s & 0x0008) { JMCMotor::ErrorFlags e = _m[i]->getErrorFlags(); _out.p(F("\n  Error reg 0x1001 = 0x")); _out.hx(e.raw); }
}

void JMCController::outHuman(uint8_t i) {
  uint16_t s = _m[i] ? _m[i]->getStatusWord() : 0;
  if (!_m[i] || _m[i]->lastResult() != JMC_OK) {
    _out.p(F("=== MOTOR ")); _out.n(i + 1); _out.p(F(" === NOT RESPONDING ("));
    _out.p(JMCBus::resultName(_m[i] ? _m[i]->lastResult() : JMC_ERR_TIMEOUT));
    _out.p(F(") - check power, slave ID, RS485 wiring"));
    return;
  }
  long pos = _m[i]->getPosition();
  float vel = _m[i]->getVelocity();
  uint8_t mode = _cache[i].mode;
  _out.p(F("=== MOTOR ")); _out.n(i + 1); _out.p(F(" (Slave ID ")); _out.n(_m[i]->slaveId());
  _out.p(F(") Status 0x")); _out.hx(s); _out.p(F(" ===\nState: ")); _out.p(stateName(s));
  if (s & 0x0040) _out.p(F(" (entering initialization)"));
  _out.p(F("\nMotor: ")); _out.p((s & 0x0004) ? F("ENABLED") : F("DISABLED"));
  _out.p((s & 0x0200) ? F(", RUNNING") : F(", stopped"));
  if (s & 0x0100) _out.p(F(", HALT active"));
  if (!(s & 0x0020) && (s & 0x000F) == 0x7) _out.p(F(", QUICK-STOP active"));
  _out.p(F("\nMode: ")); _out.p(modeName(mode));
  _out.p(F("\nPosition: ")); _out.n(pos); _out.p(F(" steps (target "));
  _out.p((s & 0x0400) ? F("REACHED") : F("NOT reached"));
  _out.p(F(")\nVelocity: ")); _out.f(vel, 1); _out.p(F(" rps\nHoming: "));
  if (mode == JMCMotor::MODE_HOMING) {
    _out.p((s & 0x2000) ? F("ERROR") : (s & 0x1000) ? F("complete") : _cache[i].homingArmed ? F("IN PROGRESS") : F("not done"));
  } else {
    _out.p(_cache[i].homeErr ? F("ERROR (last run failed)") : _cache[i].homed ? F("homed") : F("not homed since boot"));
  }
  _out.p(F(" | Origin switch (SW): ")); _out.p((s & 0x0800) ? F("ON") : F("off"));
  _out.p(F("\nLimits: CW ")); _out.p((s & 0x4000) ? F("*** ACTIVE ***") : F("clear"));
  _out.p(F(" | CCW ")); _out.p((s & 0x8000) ? F("*** ACTIVE ***") : F("clear"));
  if (_softMin[i] <= _softMax[i]) { _out.p(F(" | Soft ")); _out.n(_softMin[i]); _out.p(F("..")); _out.n(_softMax[i]); }
  _out.c('\n');
  if (s & 0x2000) {
    _out.p(mode == JMCMotor::MODE_POSITION ? F("OVER-POSITION ERROR (bit13)\n")
         : mode == JMCMotor::MODE_VELOCITY ? F("MAX ACCELERATION reached (bit13)\n") : F("HOMING ERROR (bit13)\n"));
  }
  if (s & 0x0080) _out.p(F("WARNING active (bit7)\n"));
  if (s & 0x0008) {
    JMCMotor::ErrorFlags e = _m[i]->getErrorFlags();
    _out.p(F("FAULT:"));
    bool named = false;
    if (e.valid) {
      if (e.current)       { _out.p(F(" over-current"));          named = true; }
      if (e.voltage)       { _out.p(F(" voltage-error"));         named = true; }
      if (e.temperature)   { _out.p(F(" over-temperature"));      named = true; }
      if (e.communication) { _out.p(F(" communication-error"));   named = true; }
      if (e.overPosition)  { _out.p(F(" over-position-protect")); named = true; }
      if (e.phaseLoss)     { _out.p(F(" motor-phase-loss"));      named = true; }
      if (e.generic && !named) { _out.p(F(" generic-error"));     named = true; }
    }
    if (!named) _out.p(F(" unknown"));
    if (e.valid) { _out.p(F(" (0x1001=0x")); _out.hx(e.raw); _out.c(')'); }
    _out.p(F(" -> send R:M")); _out.n(i + 1); _out.p(F(" to reset\n"));
  } else {
    _out.p(F("Health: OK, no faults\n"));
  }
}

void JMCController::cmdStatusDetail(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("SD"), n)) return;
  int left = n;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    left--;
    _out.reply();
    outHuman(i);
    if (n == 1) _out.send(); else _out.sendPart(left == 0);
  }
}

void JMCController::outDiagnose(uint8_t i) {
  uint8_t slave = _m[i]->slaveId();
  uint16_t w[2];
  JMCResult r;
  _out.p(F("DG ")); _out.m(i); _out.p(F(": "));
  r = _bus.readHolding(slave, JMCReg::FORMAT, w, 1);
  _out.p(F("fmt6000="));   if (r == JMC_OK) _out.n(w[0]); else _out.p(JMCBus::resultName(r));
  r = _bus.readHolding(slave, JMCReg::STATUS, w, 1);
  _out.p(F(" st6041="));   if (r == JMC_OK) { _out.p(F("0x")); _out.hx(w[0]); } else _out.p(JMCBus::resultName(r));
  r = _bus.readHolding(slave, JMCReg::MODE_DISP, w, 1);
  _out.p(F(" mode6061=")); if (r == JMC_OK) _out.n((int16_t)w[0]); else _out.p(JMCBus::resultName(r));
  r = _bus.readHolding(slave, JMCReg::ERROR, w, 1);
  _out.p(F(" err1001="));  if (r == JMC_OK) { _out.p(F("0x")); _out.hx(w[0]); } else _out.p(JMCBus::resultName(r));
  r = _bus.readHolding(slave, JMCReg::TARGET_POS, w, 2);
  _out.p(F("\ntgt607A=")); if (r == JMC_OK) { _out.n(w[0]); _out.c(','); _out.n(w[1]); } else _out.p(JMCBus::resultName(r));
  uint16_t tv[2] = {0, 0};
  r = _bus.readHolding(slave, JMCReg::TARGET_VEL, tv, 2);
  bool velOk = (r == JMC_OK);
  _out.p(F(" vel6081="));  if (r == JMC_OK) { _out.n(tv[0]); _out.c(','); _out.n(tv[1]); } else _out.p(JMCBus::resultName(r));
  r = _bus.readHolding(slave, JMCReg::ACCEL, w, 1);
  bool accOk = (r == JMC_OK);
  uint16_t accVal = w[0];
  _out.p(F(" acc6083="));  if (r == JMC_OK) _out.n(w[0]); else _out.p(JMCBus::resultName(r));
  r = _bus.readHolding(slave, JMCReg::DECEL, w, 1);
  _out.p(F(" dec6084="));  if (r == JMC_OK) _out.n(w[0]); else _out.p(JMCBus::resultName(r));
  r = _bus.readHolding(slave, JMCReg::POS_ACTUAL, w, 2);
  _out.p(F(" pos6064="));  if (r == JMC_OK) { _out.n(w[0]); _out.c(','); _out.n(w[1]); } else _out.p(JMCBus::resultName(r));
  // Write tests: re-write values just read from the drive (no motion);
  // skipped when the read failed, so DG never changes a setting.
  if (accOk && velOk) {
    r = _bus.writeSingle(slave, JMCReg::ACCEL, accVal);
    _out.p(F("\nWRITE06=")); _out.p(JMCBus::resultName(r));
    if (r == JMC_ERR_EXCEPTION) { _out.p(F("(exc")); _out.n(_bus.lastException()); _out.c(')'); }
    r = _bus.writeMultiple(slave, JMCReg::TARGET_VEL, tv, 2);
    _out.p(F(" WRITE10=")); _out.p(JMCBus::resultName(r));
    if (r == JMC_ERR_EXCEPTION) { _out.p(F("(exc")); _out.n(_bus.lastException()); _out.c(')'); }
  } else {
    _out.p(F("\nWRITE tests skipped (read failed)"));
  }
  _out.p(F(" libScale=")); _out.n((long)_speedScale);
}

void JMCController::cmdDiagnose(const char* args, bool all) {
  bool sel[JMC_MAX_MOTORS];
  int n = parseSelection(args, sel, all);
  if (selectionError(F("DG"), n)) return;
  int left = n;
  for (uint8_t i = 0; i < _count; i++) {
    if (!sel[i]) continue;
    left--;
    JMCWatchdog::feed();
    _out.reply();
    if (_m[i]) outDiagnose(i); else { _out.p(F("DG ")); _out.m(i); _out.p(F(": not initialised")); }
    if (n == 1) _out.send(); else _out.sendPart(left == 0);
  }
}
#endif // JMC_FULL_REPORTS
