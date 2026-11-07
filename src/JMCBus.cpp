/*==============================================================================
  JMCBus.cpp  -  Modbus-RTU transport + processor watchdog
  Author: LAGARI A <lagariscience@gmail.com>  |  License: MIT
==============================================================================*/
#include "JMCMotors.h"

#if JMC_HAS_CONTROLLINO_RS485
  #include <Controllino.h>
#endif
#if JMC_ARCH_AVR
  #include <avr/wdt.h>
#endif
#if JMC_ARCH_ESP32
  #include <esp_task_wdt.h>
  #if __has_include(<esp_arduino_version.h>)
    #include <esp_arduino_version.h>
  #endif
#endif

//==============================================================================
//  Processor watchdog
//==============================================================================
namespace {
  bool s_wdtActive = false;
}

bool JMCWatchdog::begin(uint16_t timeoutMs) {
#if JMC_ARCH_AVR
  uint8_t to;
  if      (timeoutMs <= 250)  to = WDTO_250MS;
  else if (timeoutMs <= 500)  to = WDTO_500MS;
  else if (timeoutMs <= 1000) to = WDTO_1S;
  else if (timeoutMs <= 2000) to = WDTO_2S;
#if defined(WDTO_4S)
  else if (timeoutMs <= 4000) to = WDTO_4S;
  else                        to = WDTO_8S;
#else
  else                        to = WDTO_2S;
#endif
  wdt_reset();
  wdt_enable(to);
  s_wdtActive = true;
  return true;
#elif JMC_ARCH_ESP32
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t cfg = {};
    cfg.timeout_ms = timeoutMs;
    cfg.idle_core_mask = 0;
    cfg.trigger_panic = true;
    if (esp_task_wdt_reconfigure(&cfg) != ESP_OK && esp_task_wdt_init(&cfg) != ESP_OK)
      return false;
  #else
    uint32_t sec = (timeoutMs + 999) / 1000;
    if (sec < 1) sec = 1;
    esp_task_wdt_init(sec, true);
  #endif
  esp_task_wdt_add(NULL);            // the task running loop()
  s_wdtActive = true;
  return true;
#else
  (void)timeoutMs;
  return false;
#endif
}

void JMCWatchdog::feed() {
  if (!s_wdtActive) return;
#if JMC_ARCH_AVR
  wdt_reset();
#elif JMC_ARCH_ESP32
  esp_task_wdt_reset();
#endif
}

bool JMCWatchdog::active() { return s_wdtActive; }

//==============================================================================
//  JMCBus
//==============================================================================

// Largest frame built or received: write-multiple of 4 x 32-bit registers.
static const uint8_t JMC_MAX_FRAME = 32;

JMCBus::JMCBus(Stream& port) : _port(&port) {}

void JMCBus::begin(uint32_t baud, int8_t dePin, int8_t rePin) {
  _controllino = false;
  _baud  = baud ? baud : 115200;
  _dePin = dePin;
  _rePin = rePin;
  if (_dePin >= 0) pinMode(_dePin, OUTPUT);
  if (_rePin >= 0) pinMode(_rePin, OUTPUT);
  // Modbus inter-frame silence: 3.5 characters of 10 bits, at least 350 us.
  uint32_t gap = 35000000UL / _baud;
  _gapUs = gap < 350 ? 350 : (gap > 65000 ? 65000 : (uint16_t)gap);
  rxEnable();
  _lastActivityUs = micros();
}

#if JMC_HAS_CONTROLLINO_RS485
void JMCBus::beginControllino(uint32_t baud) {
  Serial3.begin(baud, SERIAL_8N1);        // JMC default: 8 data, 1 stop, no parity
  begin(baud, -1, -1);
  _controllino = true;
  Controllino_RS485Init();
  Controllino_RS485RxEnable();
}
#endif

uint16_t JMCBus::frameTimeMs(uint8_t bytes) const {
  uint32_t us = (uint32_t)bytes * 10000000UL / _baud;   // 10 bits per byte
  return (uint16_t)((us + 999) / 1000);
}

void JMCBus::txEnable() {
#if JMC_HAS_CONTROLLINO_RS485
  if (_controllino) { Controllino_RS485TxEnable(); return; }
#endif
  if (_rePin >= 0) digitalWrite(_rePin, HIGH);   // receiver off (RE active-low)
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);   // driver on
}

void JMCBus::rxEnable() {
#if JMC_HAS_CONTROLLINO_RS485
  if (_controllino) { Controllino_RS485RxEnable(); return; }
#endif
  if (_dePin >= 0) digitalWrite(_dePin, LOW);
  if (_rePin >= 0) digitalWrite(_rePin, LOW);
}

void JMCBus::interFrameGuard() {
  uint32_t elapsed = micros() - _lastActivityUs;
  if (elapsed < _gapUs) delayMicroseconds(_gapUs - elapsed);
}

void JMCBus::drainRx() {
  while (_port->available() > 0) _port->read();
}

uint16_t JMCBus::crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;   // sent low byte first
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
    JMCWatchdog::feed();
    interFrameGuard();
    drainRx();

    txEnable();
    _port->write(req, totalReq);
    _port->flush();                    // returns when the last bit is on the wire
    delayMicroseconds(50);             // transceiver settle before releasing DE
    rxEnable();
    _lastActivityUs = micros();

    if (slave == 0) {                  // broadcast: no response
      delayMicroseconds(_gapUs);
      _lastActivityUs = micros();
      return JMC_OK;
    }

    uint32_t start = millis();
    // Some transceivers hear their own transmission: drop that echo first.
    if (_echo) {
      uint8_t dropped = 0;
      while (dropped < totalReq && (uint16_t)(millis() - start) <= _timeoutMs) {
        if (_port->available() > 0) { _port->read(); dropped++; }
      }
    }

    uint8_t idx = 0;
    while (idx < expectedRespLen) {
      if (_port->available() > 0) {
        resp[idx++] = (uint8_t)_port->read();
        // An exception frame is 5 bytes: stop waiting once it is complete.
        if (idx == 5 && resp[1] == (funcCode | 0x80)) break;
      } else if ((uint16_t)(millis() - start) > _timeoutMs) {
        break;
      }
    }
    _lastActivityUs = micros();

    if (idx < 5)          { lastErr = JMC_ERR_TIMEOUT;  continue; }
    if (resp[0] != slave) { lastErr = JMC_ERR_BADFRAME; continue; }

    if (resp[1] == (funcCode | 0x80)) {            // exception frame
      uint16_t rc = crc16(resp, 3);
      if (((rc & 0xFF) == resp[3]) && ((rc >> 8) == resp[4])) {
        _lastException = resp[2];
        // 6 = busy, 10 = CRC error "requesting retransmission" (manual p.3): retry.
        if (resp[2] == 6 || resp[2] == 10) { lastErr = JMC_ERR_EXCEPTION; continue; }
        return failed(JMC_ERR_EXCEPTION);
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
  return failed(lastErr);
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
  if (slave == 0) return JMC_ERR_PARAM;            // broadcast reads are illegal
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

const __FlashStringHelper* JMCBus::resultName(JMCResult r) {
  switch (r) {
    case JMC_OK:            return F("OK");
    case JMC_ERR_TIMEOUT:   return F("TIMEOUT");
    case JMC_ERR_CRC:       return F("CRC");
    case JMC_ERR_EXCEPTION: return F("EXCEPTION");
    case JMC_ERR_BADFRAME:  return F("BADFRAME");
    case JMC_ERR_PARAM:     return F("PARAM");
    default:                return F("?");
  }
}
