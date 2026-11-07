// Simulated JMC drives on an RS-485 bus, for host tests.
//
// Answers Modbus-RTU FC03 / FC06 / FC10 (broadcast slave 0 for writes),
// verifies every request's CRC, and models what the library relies on:
// control word 0x6040 (enable, bit4 edge, bit5 immediate, bit6 relative,
// bit8 halt, quick stop, reset), modes 1 / 3 / 6, homing method 0 (set
// position) and a timed homing run, limit / origin inputs, faults.
// Moves run at constant speed (no ramps) at 4000 steps per revolution.
// Directions follow the manual's homing descriptions (section 6.3): the CW
// limit is the end reached moving left (negative counts), CCW moving right.
// 0x6000 (32-bit word order) is 0 by default; set "format = 1" to test the
// other order.
#pragma once
#include <Arduino.h>
#include <vector>
#include <map>
#include <string>

struct FakeDrive {
  bool     present = true;
  std::map<uint16_t, uint16_t> reg;
  int32_t  pos = 0, target = 0;
  bool     moving = false;
  double   posF = 0;              // sub-step position
  uint16_t ctrl = 0;
  uint16_t mode = 1;
  bool     cw = false, ccw = false, sw = false, fault = false;
  uint16_t errReg = 0;
  bool     homed = false, homing = false, homeErr = false;
  uint64_t homingDoneUs = 0;
  // Physical homing (autoSwitches = true): the inputs follow the axis, the
  // switches sit at these positions; a homing run moves per the manual's
  // method (odd = left/negative first, even = right) until its switch.
  bool     autoSwitches = false;
  bool     hasSW = true;
  int32_t  cwAt = -20000, swAt = 3000, ccwAt = 20000;
  int      homeDir = 0, homeTarget = 0;   // target: 1 = CW, 2 = CCW, 3 = SW
  bool     homeReversed = false;
  bool     cwTouched = false, ccwTouched = false;   // during the last homing run
  uint64_t lastUs = 0;
  uint64_t arrivedUs = 0;         // when the last position move finished
  int      modeSwitchWhileSpinning = 0;
  int      latches = 0;           // bit4 rising edges in position mode
  int32_t  maxPos = -2147483647, minPos = 2147483647;   // extremes reached (moves)
  uint16_t format = 0;            // 0x6000: 0 = high word first, 1 = low word first
  uint16_t rejectReg = 0;         // writes to this register get an exception (0 = none)
  uint8_t  rejectCode = 3;        // the exception code sent
  int      rejectTimes = -1;      // how many times to reject (-1 = always)
  int      offsetMode = 0;        // method 0: 0 = position := +offset, 1 = := -offset, 2 = := 0

  // 32-bit registers are objects of two words (manual p.14, p.27, p.32): a
  // 0x10 write of 2 words to 0x6099 sets 0x6099 only, not 0x609A.
  std::map<uint16_t, int32_t> obj32;
  static bool is32(uint16_t r) {
    return r == 0x6064 || r == 0x606C || r == 0x607A || r == 0x607C ||
           r == 0x6081 || r == 0x6099 || r == 0x609B;
  }
  int32_t get32(uint16_t r) { return obj32[r]; }
  int32_t read32obj(uint16_t r) { return r == 0x6064 ? pos : r == 0x606C ? velReg() : obj32[r]; }
  void write32obj(uint16_t r, uint16_t w0, uint16_t w1) {
    uint32_t v = format ? ((uint32_t)w1 << 16 | w0) : ((uint32_t)w0 << 16 | w1);
    obj32[r] = (int32_t)v;
  }
  uint16_t word32(int32_t v, bool second) const {   // register word of a 32-bit value
    bool high = format ? second : !second;
    return high ? (uint16_t)((uint32_t)v >> 16) : (uint16_t)((uint32_t)v & 0xFFFF);
  }
  bool enabled() const { return (ctrl & 0x000F) == 0x000F; }
  bool halted()  const { return ctrl & 0x0100; }
  int32_t velReg() {               // actual velocity, x10 rps
    if (fault || !enabled() || halted()) return 0;
    if (mode == 3) return get32(0x6081);
    if (mode == 1 && moving) { int32_t v = get32(0x6081); return target >= pos ? v : -v; }
    return 0;
  }
  void update(uint64_t now) {
    double dt = (now - lastUs) / 1e6;
    lastUs = now;
    // halt pauses a position move (released, it resumes: manual p.18); a
    // fault or disable cancels it
    if (fault || !enabled() || halted()) { if (mode == 1 && !halted()) moving = false; return; }
    double stepsPerSec = fabs(get32(0x6081) / 10.0) * 4000.0;
    if (mode == 3) {
      double v = get32(0x6081) / 10.0 * 4000.0;
      if ((v < 0 && cw) || (v > 0 && ccw)) return;
      posF += v * dt;
      if (autoSwitches) { if (posF < cwAt) posF = cwAt; if (posF > ccwAt) posF = ccwAt; }
      pos = (int32_t)lround(posF);
    } else if (mode == 1 && moving) {
      double d = target - posF, step = stepsPerSec * dt;
      if ((d < 0 && cw) || (d > 0 && ccw)) { moving = false; return; }
      if (fabs(d) <= step) { posF = target; moving = false; arrivedUs = now; }
      else posF += d > 0 ? step : -step;
      if (posF > maxPos) maxPos = (int32_t)posF;
      if (posF < minPos) minPos = (int32_t)posF;
      if (autoSwitches) {
        if (posF <= cwAt)  { posF = cwAt;  moving = false; }
        if (posF >= ccwAt) { posF = ccwAt; moving = false; }
      }
      pos = (int32_t)lround(posF);
    } else if (mode == 6 && homing && autoSwitches) {
      double v = fabs(get32(0x6099) / 10.0) * 4000.0;
      posF += homeDir * v * dt;
      pos = (int32_t)lround(posF);
      sense();
      bool atTarget = (homeTarget == 1 && cw) || (homeTarget == 2 && ccw) || (homeTarget == 3 && sw);
      if (atTarget) {
        homing = false; homed = true;
        renumber(get32(0x607C));
      } else if ((homeDir < 0 && cw) || (homeDir > 0 && ccw)) {
        if (homeTarget == 3 && !homeReversed) { homeDir = -homeDir; homeReversed = true; }   // methods 3/4/7/8/11/12
        else { homing = false; homeErr = true; }                                              // nowhere left to look
      }
    } else if (mode == 6 && homing && now >= homingDoneUs) {
      homing = false; homed = true;
      posF = pos = get32(0x607C); target = pos;
    }
    if (autoSwitches) sense();
  }
  // The position counter is redefined: the switches stay where they are
  // physically, so their coordinates move with the new numbering.
  void renumber(int32_t newPos) {
    int32_t d = newPos - pos;
    cwAt += d; swAt += d; ccwAt += d;
    posF = pos = target = newPos;
  }
  void sense() {                   // inputs active within 50 steps of a switch / beyond a limit
    cw = pos <= cwAt;
    ccw = pos >= ccwAt;
    if (homing) { if (cw) cwTouched = true; if (ccw) ccwTouched = true; }
    sw = hasSW && pos >= swAt - 50 && pos <= swAt + 50;
  }
  uint16_t status() {
    uint16_t s = 0;
    if (enabled()) s |= 0x0037;
    else if ((ctrl & 0x0F) == 0x0B) s |= 0x0017;   // quick stop active: X00X 0111 (p.7)
    else s |= 0x0003;
    if (halted()) s |= 0x0100;
    if (velReg() != 0 || homing) s |= 0x0200;
    // Manual p.13: with control-word bit 5 = 1 the set-point bit (12) stays 1
    // throughout; bit 10 = target reached.
    if (mode == 1) s |= moving ? 0x1000 : 0x1400;
    if (mode == 3 && velReg() == 0) s |= 0x1000;
    if (mode == 6 && homed) s |= 0x1400;
    if (mode == 6 && homeErr) s |= 0x2000;
    if (sw) s |= 0x0800;
    if (cw) s |= 0x4000;
    if (ccw) s |= 0x8000;
    if (fault) s = (uint16_t)((s & ~0x000F) | 0x000F | 0x0008);
    return s;
  }
  uint16_t readReg(uint16_t r) {
    switch (r) {
      case 0x6041: return status();
      case 0x6061: return mode;
      case 0x1001: return errReg;
      case 0x6000: return format;
      default:     return reg[r];
    }
  }
  void writeReg(uint16_t r, uint16_t v, uint64_t now) {
    if (r == 0x6060) {
      if (velReg() != 0 && mode == 3 && v != 3) modeSwitchWhileSpinning++;
      if (v != mode) { moving = false; homing = false; }   // same mode: the target stays
      mode = v;
      return;
    }
    reg[r] = v;
    if (r != 0x6040) return;
    uint16_t prev = ctrl;
    ctrl = v;
    if (v & 0x0080) { fault = false; errReg = 0; }
    bool edge = !(prev & 0x0010) && (v & 0x0010);
    if (!edge || !enabled() || halted()) return;
    if (mode == 1) {
      int32_t t = get32(0x607A);
      target = (v & 0x0040) ? target + t : t;
      moving = (target != pos);
      latches++;
    } else if (mode == 6) {
      if (reg[0x6098] == 0) {                                              // set position, complete at once
        int32_t o = get32(0x607C);
        renumber(offsetMode == 1 ? -o : offsetMode == 2 ? 0 : o);
        homed = true;
      }
      else {
        homing = true; homed = false; homeErr = false; homingDoneUs = now + 200000;
        int m = reg[0x6098];
        homeDir = (m % 2) ? -1 : 1;                        // odd: left first
        bool toSwitch = (m == 3 || m == 4 || m == 7 || m == 8 || m == 11 || m == 12);
        homeTarget = toSwitch ? 3 : (m % 2 ? 1 : 2);
        homeReversed = false;
        cwTouched = ccwTouched = false;
      }
    }
  }
};

class FakeBus : public Stream {
  public:
    FakeDrive d[33];                       // index = slave ID
    std::vector<std::vector<uint8_t>> log; // every request frame
    int crcErrors = 0;

    size_t write(uint8_t b) override { _tx.push_back(b); return 1; }
    void flush() override { process(); }
    int available() override { return (int)(_rx.size() - _rp); }
    int read() override { return _rp < _rx.size() ? _rx[_rp++] : -1; }
    int peek() override { return _rp < _rx.size() ? _rx[_rp] : -1; }

    void tick() { for (int i = 1; i <= 32; i++) d[i].update(g_us); }

    static uint16_t crc(const uint8_t* p, size_t n) {
      uint16_t c = 0xFFFF;
      for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
      }
      return c;
    }
    bool sawFrame(const std::vector<uint8_t>& f) const {
      for (auto& x : log) if (x == f) return true;
      return false;
    }

  private:
    std::vector<uint8_t> _tx, _rx;
    size_t _rp = 0;

    void reply(std::vector<uint8_t> r) {
      uint16_t c = crc(r.data(), r.size());
      r.push_back(c & 0xFF); r.push_back(c >> 8);
      _rx.insert(_rx.end(), r.begin(), r.end());
    }
    void process() {
      std::vector<uint8_t> f = _tx;
      _tx.clear();
      if (_rp >= _rx.size()) { _rx.clear(); _rp = 0; }
      if (f.size() < 4) return;
      log.push_back(f);
      uint16_t c = crc(f.data(), f.size() - 2);
      if ((c & 0xFF) != f[f.size() - 2] || (c >> 8) != f[f.size() - 1]) { crcErrors++; return; }
      tick();
      uint8_t slave = f[0], fc = f[1];
      uint16_t r = (uint16_t)(f[2] << 8 | f[3]);
      int first = slave ? slave : 1, last = slave ? slave : 32;
      for (int s = first; s <= last; s++) {
        FakeDrive& dr = d[s];
        if (!dr.present) continue;
        if ((fc == 0x06 || fc == 0x10) && dr.rejectReg) {
          uint16_t n = (fc == 0x06) ? 1 : f[5];
          if (dr.rejectReg >= r && dr.rejectReg < r + n && dr.rejectTimes != 0) {
            if (dr.rejectTimes > 0) dr.rejectTimes--;
            if (slave) reply(std::vector<uint8_t>{ slave, (uint8_t)(fc | 0x80), dr.rejectCode });
            continue;
          }
        }
        if (fc == 0x06) {
          dr.writeReg(r, (uint16_t)(f[4] << 8 | f[5]), g_us);
          if (slave) reply(std::vector<uint8_t>(f.begin(), f.begin() + 6));
        } else if (fc == 0x10) {
          uint8_t n = f[5];
          if (n == 2 && FakeDrive::is32(r))
            dr.write32obj(r, (uint16_t)(f[7] << 8 | f[8]), (uint16_t)(f[9] << 8 | f[10]));
          else
            for (uint8_t k = 0; k < n; k++)
              dr.writeReg((uint16_t)(r + k), (uint16_t)(f[7 + 2 * k] << 8 | f[8 + 2 * k]), g_us);
          if (slave) reply(std::vector<uint8_t>(f.begin(), f.begin() + 6));
        } else if (fc == 0x03 && slave) {
          uint8_t n = f[5];
          std::vector<uint8_t> out = { slave, 0x03, (uint8_t)(2 * n) };
          if (n == 2 && FakeDrive::is32(r)) {
            int32_t v = dr.read32obj(r);
            uint16_t a = dr.word32(v, false), b = dr.word32(v, true);
            out.push_back(a >> 8); out.push_back(a & 0xFF); out.push_back(b >> 8); out.push_back(b & 0xFF);
          } else
            for (uint8_t k = 0; k < n; k++) {
              uint16_t v = dr.readReg((uint16_t)(r + k));
              out.push_back(v >> 8); out.push_back(v & 0xFF);
            }
          reply(out);
        }
      }
    }
};
