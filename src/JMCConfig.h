/*==============================================================================
  JMCConfig.h  -  board detection and per-board sizing for JMCMotors
==============================================================================
  Everything here is chosen automatically from the board you compile for.
  Supported and compile-tested:
    - CONTROLLINO MAXI / MEGA (built-in RS-485 on Serial3; the MAXI Automation
      has no RS-485 port - use a TTL-to-RS-485 module on Serial1/2 there)
    - Arduino Mega 2560 and other large AVRs
    - Arduino Uno / Nano / Pro Mini (ATmega328P, 2 KB RAM: "small" profile)
    - ESP32 (any Arduino-ESP32 2.x or 3.x board)

  PlatformIO / build-flag users may override any JMC_* value below with
  -D, e.g. -DJMC_MAX_MOTORS=8. (The Arduino IDE does not pass #defines from
  a sketch into libraries, so in the IDE the board defaults apply.)
==============================================================================*/
#ifndef JMCConfig_h
#define JMCConfig_h

#include <Arduino.h>

#define JMCMOTORS_VERSION "0.6.0"     // reported by the VER command

// ---- architecture -------------------------------------------------------------
#if defined(ARDUINO_ARCH_ESP32)
  #define JMC_ARCH_ESP32 1
#else
  #define JMC_ARCH_ESP32 0
#endif
#if defined(__AVR__)
  #define JMC_ARCH_AVR 1
#else
  #define JMC_ARCH_AVR 0
#endif

// CONTROLLINO variants define these in their pins_arduino.h.
#if defined(CONTROLLINO_MAXI) || defined(CONTROLLINO_MEGA)
  #define JMC_HAS_CONTROLLINO_RS485 1
#else
  #define JMC_HAS_CONTROLLINO_RS485 0
#endif

// 2 KB RAM class: Uno, Nano, Pro Mini, Leonardo/Micro, CONTROLLINO MINI.
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || \
    defined(__AVR_ATmega328PB__) || defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega32U4__)
  #define JMC_SMALL_RAM 1
#else
  #define JMC_SMALL_RAM 0
#endif

// ---- sizes --------------------------------------------------------------------
// Most motors the controller can manage (Modbus allows 32 per RS-485 bus).
#ifndef JMC_MAX_MOTORS
  #if JMC_SMALL_RAM
    #define JMC_MAX_MOTORS 6
  #else
    #define JMC_MAX_MOTORS 32
  #endif
#endif

// Longest command line accepted (longer ones are rejected, never truncated).
#ifndef JMC_CMD_BUFFER
  #if JMC_SMALL_RAM
    #define JMC_CMD_BUFFER 96
  #else
    #define JMC_CMD_BUFFER 512
  #endif
#endif

// Reply packet buffer. Long per-motor lists continue in further packets
// marked :MORE / :END, so this bounds RAM, not what can be reported.
#ifndef JMC_REPLY_BUFFER
  #if JMC_SMALL_RAM
    #define JMC_REPLY_BUFFER 128
  #else
    #define JMC_REPLY_BUFFER 420
  #endif
#endif

// Verbose reports (SD plain-language status, DG register dump and the S
// formats STATE / BITS / FLAGS / DETAIL). Left out on 2 KB boards, where
// they do not fit; S RAW / COMPACT and LS carry the same information.
#ifndef JMC_FULL_REPORTS
  #if JMC_SMALL_RAM
    #define JMC_FULL_REPORTS 0
  #else
    #define JMC_FULL_REPORTS 1
  #endif
#endif

// Feature switches for 32 KB-flash boards (measured on an Uno, see README).
// SAVE / LOAD to EEPROM. Off on the Uno: it is flashed over the same USB
// cable it is controlled through, so the admin changes the sketch instead.
#ifndef JMC_FEATURE_SAVE
  #if JMC_SMALL_RAM
    #define JMC_FEATURE_SAVE 0
  #else
    #define JMC_FEATURE_SAVE 1
  #endif
#endif
// PS / PRS synchronised-arrival moves (about 3 KB of float code). Off on
// the Uno, where it does not fit; P/PA still start every motor together.
#ifndef JMC_FEATURE_SYNC
  #if JMC_SMALL_RAM
    #define JMC_FEATURE_SYNC 0
  #else
    #define JMC_FEATURE_SYNC 1
  #endif
#endif
// BS (brake status) and ZS (zero status) - S:..:COMPACT and P:? report the
// same information. Off on the Uno for space.
#ifndef JMC_FEATURE_EXTRA_STATUS
  #if JMC_SMALL_RAM
    #define JMC_FEATURE_EXTRA_STATUS 0
  #else
    #define JMC_FEATURE_EXTRA_STATUS 1
  #endif
#endif
// Conveniences: MC:n (motor count at runtime - MC:? always works), X (soft
// restart), and HVZ / HACC as separate settings (without them they follow
// HV and ACC, the default anyway). Off on the Uno for space.
#ifndef JMC_FEATURE_EXTRAS
  #if JMC_SMALL_RAM
    #define JMC_FEATURE_EXTRAS 0
  #else
    #define JMC_FEATURE_EXTRAS 1
  #endif
#endif
// Example hints in error replies ("ERR:P needs values (e.g. ...)"). The
// "ERR:<CMD> ..." start of every error is the same on every board.
#ifndef JMC_FEATURE_HINTS
  #if JMC_SMALL_RAM
    #define JMC_FEATURE_HINTS 0
  #else
    #define JMC_FEATURE_HINTS 1
  #endif
#endif

// EEPROM area used by SAVE / LOAD (ESP32 emulates EEPROM in flash).
#ifndef JMC_EEPROM_SIZE
  #define JMC_EEPROM_SIZE 1536
#endif

#endif // JMCConfig_h
