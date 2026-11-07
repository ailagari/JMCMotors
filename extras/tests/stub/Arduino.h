// Minimal Arduino core for building JMCMotors on a PC (host tests only).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

typedef uint8_t byte;
typedef bool boolean;

class __FlashStringHelper;
#define F(s)              (reinterpret_cast<const __FlashStringHelper*>(s))
#define PSTR(s)           (s)
#define PROGMEM
#define pgm_read_byte(p)  (*(const uint8_t*)(p))
#define strcmp_P          strcmp
#define strlen_P          strlen

#define HIGH   1
#define LOW    0
#define INPUT  0
#define OUTPUT 1

// Simulated clock: every call to micros()/millis() moves time on a little,
// so busy-wait loops (bus timeouts) always end.
extern uint64_t g_us;
inline uint32_t micros() { g_us += 5; return (uint32_t)g_us; }
inline uint32_t millis() { g_us += 5; return (uint32_t)(g_us / 1000); }
inline void delay(uint32_t ms) { g_us += (uint64_t)ms * 1000; }
inline void delayMicroseconds(uint32_t us) { g_us += us; }

extern int g_pinState[128];
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int v) { if (pin >= 0 && pin < 128) g_pinState[pin] = v; }

class Print {
  public:
    virtual size_t write(uint8_t b) = 0;
    virtual size_t write(const uint8_t* buf, size_t n) {
      size_t k = 0;
      while (n--) k += write(*buf++);
      return k;
    }
    virtual ~Print() {}
};

class Stream : public Print {
  public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() {}
};
