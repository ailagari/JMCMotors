// EEPROM for host tests (AVR-style API).
#pragma once
#include <stdint.h>

class EEPROMClass {
  public:
    uint8_t mem[4096];
    EEPROMClass() { for (unsigned i = 0; i < sizeof(mem); i++) mem[i] = 0xFF; }
    uint8_t read(int a) const   { return mem[a]; }
    void write(int a, uint8_t v) { mem[a] = v; }
    void update(int a, uint8_t v) { mem[a] = v; }
    uint16_t length() const      { return sizeof(mem); }
};
extern EEPROMClass EEPROM;
