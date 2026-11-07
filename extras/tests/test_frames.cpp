// Frame-level check against the JMC Modbus-RTU manual V2.2: for every command,
// the exact register writes / reads the library puts on the bus, in order.
// Also checks, over every frame of the run, the manual's general rules.
#include <JMCMotors.h>
#include <EEPROM.h>
#include "fake_drives.h"
#include <cstdio>
#include <string>
#include <vector>
#include <deque>

uint64_t g_us = 0;
int g_pinState[128];
EEPROMClass EEPROM;

class FakeLink : public JMCLink {
  public:
    std::deque<std::string> in;
    std::vector<std::string> replies, events;
    int receive(char* buf, size_t cap) override {
      if (in.empty()) return 0;
      std::string s = in.front(); in.pop_front();
      if (s.size() > cap - 1) return -1;
      memcpy(buf, s.c_str(), s.size() + 1);
      return (int)s.size();
    }
    void reply(const char* m, size_t n) override { replies.push_back(std::string(m, n)); }
    void event(const char* m, size_t n) override { events.push_back(std::string(m, n)); }
};

static FakeBus  port;
static JMCBus   bus(port);
static FakeLink pc;
static int pass = 0, fail = 0;
static std::vector<std::vector<uint8_t>> allFrames;

// A frame decoded: function 03 = read, 06 = write one, 10 = write several.
struct Fr { int slave, fc, reg, n; std::vector<int32_t> v; };
static Fr decode(const std::vector<uint8_t>& f) {
  Fr d; d.slave = f[0]; d.fc = f[1]; d.reg = f[2] << 8 | f[3]; d.n = 1;
  if (d.fc == 0x06) d.v.push_back((int16_t)(f[4] << 8 | f[5]) & 0xFFFF);
  if (d.fc == 0x03) d.n = f[4] << 8 | f[5];
  if (d.fc == 0x10) {
    d.n = f[4] << 8 | f[5];
    if (d.n == 2) d.v.push_back((int32_t)((uint32_t)(f[7] << 8 | f[8]) << 16 | (f[9] << 8 | f[10])));  // 0x6000 = 0
    else for (int k = 0; k < d.n; k++) d.v.push_back(f[7 + 2 * k] << 8 | f[8 + 2 * k]);
  }
  return d;
}
static std::string show(const Fr& d) {
  char b[64];
  if (d.fc == 0x03) snprintf(b, sizeof b, "R%d 0x%04X x%d", d.slave, d.reg, d.n);
  else snprintf(b, sizeof b, "%s%d 0x%04X=%ld", d.fc == 6 ? "W" : "W32 ", d.slave, d.reg, d.v.empty() ? 0L : (long)d.v[0]);
  return b;
}
struct Exp { int slave, fc, reg; long val; bool anyVal; };
static Exp W(int s, int r, long v)   { return { s, 0x06, r, v, false }; }
static Exp W32(int s, int r, long v) { return { s, 0x10, r, v, false }; }
static Exp R(int s, int r)           { return { s, 0x03, r, 0, true }; }

static void check(bool ok, const std::string& what, const std::string& got = "") {
  if (ok) { pass++; return; }
  fail++;
  printf("FAIL: %s\n      got: %s\n", what.c_str(), got.c_str());
}
static void runFor(JMCController& c, uint32_t ms) {
  uint64_t end = g_us + (uint64_t)ms * 1000;
  while (g_us < end) { c.run(); port.tick(); delay(1); }
}
static std::vector<std::string> cmd(JMCController& c, const std::string& line) {
  pc.replies.clear();
  pc.in.push_back(line);
  c.run();
  return pc.replies;
}
static void keepLog() { for (auto& f : port.log) allFrames.push_back(f); port.log.clear(); }

// Run a command; check the reply starts with `reply`, and that the expected
// frames appear on the bus in this order (other frames may come between).
static void seq(JMCController& c, const std::string& line, const std::string& reply,
                std::vector<Exp> want, const char* manual) {
  keepLog();
  auto r = cmd(c, line);
  std::string got = r.empty() ? "" : r[0];
  check(got.rfind(reply, 0) == 0, line + "  reply  " + reply, got);
  std::vector<Fr> fr;
  for (auto& f : port.log) fr.push_back(decode(f));
  size_t k = 0;
  std::string trace;
  for (auto& d : fr) {
    trace += show(d) + "; ";
    if (k < want.size() && d.slave == want[k].slave && d.fc == want[k].fc && d.reg == want[k].reg &&
        (want[k].anyVal || (!d.v.empty() && d.v[0] == want[k].val))) k++;
  }
  if (k < want.size()) {
    Fr m = { want[k].slave, want[k].fc, want[k].reg, 1, { (int32_t)want[k].val } };
    check(false, line + "  frames per manual " + manual + ": missing #" + std::to_string(k + 1) + " " + show(m), trace);
  } else pass++;
}

int main() {
  for (int i = 1; i <= 32; i++) port.d[i].present = (i <= 3);
  bus.begin(115200, 7);
  JMCController jmc(bus, pc);
  jmc.setMotorCount(3);
  jmc.setMotionDefaults(0.5, 0.4, 0.3, 0.3);   // ACC 5, DEC 4, V 3, HV 3 (x0.1)
  jmc.begin();
  // ---- power-up / init (p.16 routine) -------------------------------------------
  {
    std::vector<Fr> fr; for (auto& f : port.log) fr.push_back(decode(f));
    std::vector<Exp> want = { R(1, 0x6000), W(1, 0x6040, 0x01), W(1, 0x6040, 0x03), W(1, 0x6040, 0x0F),
                              W(1, 0x6060, 1), W(1, 0x6083, 5), W(1, 0x6084, 4), W(1, 0x6085, 4),
                              W(1, 0x605D, 1), R(1, 0x6041) };
    size_t k = 0;
    for (auto& d : fr) if (k < want.size() && d.slave == want[k].slave && d.fc == want[k].fc && d.reg == want[k].reg &&
                           (want[k].anyVal || (!d.v.empty() && d.v[0] == want[k].val))) k++;
    check(k == want.size(), "begin: 0x6000 read, 0x01/0x03/0x0F, mode 1, ramps, halt code 1 (p.16, p.7)", std::to_string(k));
  }
  runFor(jmc, 50);

  // ---- position mode: forward / reverse, single motor (p.10-16) -----------------
  seq(jmc, "V:1", "Set V: M1=1.00 rps", {}, "");
  // already in position mode and enabled after begin(): mode and enable are not sent again
  seq(jmc, "P:4000", "OK", { W32(1, 0x6081, 10), W32(1, 0x607A, 4000), W(1, 0x6040, 0x3F), W(1, 0x6040, 0x2F) }, "p.16");
  runFor(jmc, 1200);
  seq(jmc, "P:-4000", "OK", { W32(1, 0x607A, -4000), W(1, 0x6040, 0x3F), W(1, 0x6040, 0x2F) }, "p.16 (negative INT32)");
  {
    // a repeated move with the same speed is exactly three frames to the drive
    int n = 0; for (auto& f : port.log) if (f[0] == 1 && f[1] != 0x03) n++;
    check(n == 3, "streaming: a position update with an unchanged speed = 3 frames (607A, 0x3F, 0x2F)", std::to_string(n));
  }
  runFor(jmc, 2200);
  seq(jmc, "V:-1", "Set V: M1=-1.00 rps", {}, "");
  seq(jmc, "P:0", "OK", { W32(1, 0x607A, 0) }, "speed magnitude for position moves");
  {
    bool vel = false; for (auto& f : port.log) if (f[0] == 1 && f[1] == 0x10 && (f[2] << 8 | f[3]) == 0x6081) vel = true;
    check(!vel, "V:-1 has the same magnitude as V:1: speed register not re-sent");
  }
  runFor(jmc, 1200);
  seq(jmc, "PR:-1500", "OK", { R(1, 0x6064), W32(1, 0x607A, -1500), W(1, 0x6040, 0x3F), W(1, 0x6040, 0x2F) }, "PR = actual position + delta, absolute");
  runFor(jmc, 600);
  seq(jmc, "PR:2500", "OK", { R(1, 0x6064), W32(1, 0x607A, 1000), W(1, 0x6040, 0x3F) }, "PR forward");
  runFor(jmc, 600);
  // ---- several motors: broadcast vs back-to-back --------------------------------
  seq(jmc, "V:1,1,1", "Set V:", {}, "");
  seq(jmc, "PA:2000", "OK", { W32(1, 0x607A, 2000), W32(2, 0x607A, 2000), W32(3, 0x607A, 2000),
                             W(0, 0x6040, 0x3F), W(0, 0x6040, 0x2F) }, "p.2 broadcast 0x06");
  runFor(jmc, 600);
  seq(jmc, "P:-1000,,3000", "OK", { W32(1, 0x607A, -1000), W32(3, 0x607A, 3000),
                                   W(1, 0x6040, 0x3F), W(3, 0x6040, 0x3F), W(1, 0x6040, 0x2F), W(3, 0x6040, 0x2F) }, "subset: unicast edges back to back");
  runFor(jmc, 1200);
  seq(jmc, "PRA:-500", "OK", { R(1, 0x6064), R(2, 0x6064), R(3, 0x6064), W(0, 0x6040, 0x3F), W(0, 0x6040, 0x2F) }, "PRA broadcast start");
  runFor(jmc, 600);
  seq(jmc, "V:1,,0.5", "Set V:", {}, "");
  seq(jmc, "PS:4000,,0", "OK", { R(1, 0x6064), R(3, 0x6064), W(1, 0x6083, 5), W(1, 0x6084, 4), W32(1, 0x6081, 10), W32(1, 0x607A, 4000) }, "PS scaled ramps + speed");
  runFor(jmc, 2000);

  // ---- velocity mode, forward / reverse (p.18-19) ------------------------------------
  seq(jmc, "V:-0.5", "Set V: M1=-0.50 rps", {}, "");
  seq(jmc, "VS:M1", "VS: velocity mode started for M1", { W(1, 0x6040, 0x10F), W(1, 0x6060, 3), W32(1, 0x6081, -5),
                                                          W(1, 0x6040, 0x1F), W(1, 0x6040, 0x0F) }, "p.19 pause, mode 3, speed, start");
  runFor(jmc, 200);
  check(port.d[1].velReg() == -5, "drive 1 jogging in reverse at -0.5 rps", std::to_string(port.d[1].velReg()));
  seq(jmc, "VA:0.8", "VA: all motors = 0.80 rps", { W32(1, 0x6081, 8) }, "p.19 speed change while running, jogging drives only");
  runFor(jmc, 50);
  check(port.d[1].velReg() == 8, "running jog follows VA, now forward", std::to_string(port.d[1].velReg()));
  seq(jmc, "B", "All motors stopped with deceleration", { W(0, 0x6040, 0x10F), W32(1, 0x6081, 0), W(1, 0x6040, 0x10F) }, "p.19 0x010F stops");
  runFor(jmc, 50);
  seq(jmc, "VSA", "VS: velocity mode started", { W(1, 0x6060, 3), W(2, 0x6060, 3), W(3, 0x6060, 3), W(0, 0x6040, 0x1F), W(0, 0x6040, 0x0F) }, "all motors: broadcast start");
  runFor(jmc, 50);

  // ---- stops (p.7: 0x605A / 0x605D) -------------------------------------------------
  seq(jmc, "B1:M1", "Deceleration brake applied to motor 1", { W(1, 0x605A, 1), W(1, 0x6040, 0x000B) }, "p.7 code 1; 0x000B = quick stop with the enable bits kept (p.5-6)");
  seq(jmc, "B2:M2", "Fast brake applied to motor 2", { W(2, 0x605A, 2), W(2, 0x6040, 0x000B) }, "p.7 code 2");
  seq(jmc, "B3:M3", "Immediate brake applied to motor 3", { W(3, 0x605A, 3), W(3, 0x6040, 0x000B) }, "p.7 code 3");
  seq(jmc, "B3", "Immediate brake applied to all motors", { W(0, 0x605A, 3), W(0, 0x6040, 0x000B), W(1, 0x605A, 3) }, "broadcast then per drive");
  // a quick-stopped jog must not resume on the next enable (R / I)
  seq(jmc, "VS:M1", "VS: velocity mode started for M1", {}, "");
  runFor(jmc, 100);
  seq(jmc, "B2:M1", "Fast brake applied to motor 1", { W(1, 0x605A, 2), W(1, 0x6040, 0x000B), W32(1, 0x6081, 0) }, "quick stop clears the jog speed");
  seq(jmc, "R:M1", "Motor 1 reset complete", { W(1, 0x6040, 0x0080), W(1, 0x6040, 0x03), W32(1, 0x6081, 0), W(1, 0x6040, 0x0F) }, "speed 0 before 0x000F");
  runFor(jmc, 100);
  check(port.d[1].velReg() == 0 && port.d[1].modeSwitchWhileSpinning == 0, "after B2 + R the motor is still at rest");
  runFor(jmc, 50);
  // after B mid-move, the next P keeps the halt until its own start pulse
  seq(jmc, "V:1", "Set V: M1=1.00 rps", {}, "");
  seq(jmc, "P:40000", "OK", {}, "");
  runFor(jmc, 200);
  seq(jmc, "B:M1", "Motor 1 stopped with deceleration", { W(1, 0x6040, 0x010F) }, "");
  runFor(jmc, 50);
  {
    keepLog();
    cmd(jmc, "P:0");
    bool released = false;   // a 0x000F before the 0x3F edge would release the halt early
    for (auto& f : port.log) {
      Fr d = decode(f);
      if (d.slave == 1 && d.fc == 6 && d.reg == 0x6040) {
        if (d.v[0] == 0x3F) break;
        if ((d.v[0] & 0x0100) == 0 && (d.v[0] & 0x0F) == 0x0F) released = true;
      }
    }
    check(!released, "after B the halt stays on until the new move's start pulse (no early 0x000F)");
  }
  runFor(jmc, 1500);
  seq(jmc, "L:M2", "Motor 2 brake loosened", { W(2, 0x6040, 0x0003) }, "p.16 0x0003 = motor off, brake open");
  seq(jmc, "R:M1", "Motor 1 reset complete", { W(1, 0x6040, 0x0080), W(1, 0x6040, 0x01), W(1, 0x6040, 0x03), W(1, 0x6040, 0x0F) }, "p.5 bit 7 reset, then power-up");
  seq(jmc, "IA", "MOTOR INITIALIZATION:", { W(1, 0x6040, 0x0F), W(2, 0x6040, 0x0F), W(3, 0x6040, 0x0F) }, "p.16 power-up, all");

  // ---- ramps and homing parameters (p.9-10, units x10) ----------------------------------
  seq(jmc, "ACC:2", "Set ACC: M1=2.00 rps/s", { W(1, 0x6083, 20), W(1, 0x6084, 4), W(1, 0x6085, 4) }, "p.9 x10");
  seq(jmc, "DEC:3", "Set DEC: M1=3.00 rps/s", { W(1, 0x6083, 20), W(1, 0x6084, 30), W(1, 0x6085, 30) }, "p.9 x10, quick-stop ramp = DEC");
  seq(jmc, "HV:2", "Set HV: M1=2.00 rps", { W32(1, 0x6099, 20) }, "p.10 / p.27 x10");
  seq(jmc, "HVZ:1", "Set HVZ: M1=1.00 rps", { W32(1, 0x609B, 10) }, "p.10 x10");
  seq(jmc, "HACC:5", "Set HACC: M1=5.00 rps/s", { W(1, 0x609A, 50) }, "p.10 x10");
  seq(jmc, "HO:300", "HO: M1=300", { W32(1, 0x607C, 300) }, "p.8-9 INT32");

  // ---- homing, every method (p.20-27) ------------------------------------------------------
  for (int m = 1; m <= 12; m++) {
    std::string c = "H" + std::to_string(m) + ":M1";
    // the first run switches to homing mode; later runs find the drive still in it
    std::vector<Exp> want = { W(1, 0x609A, 50), W32(1, 0x6099, 20), W32(1, 0x609B, 10), W32(1, 0x607C, 300) };
    if (m == 1) want.push_back(W(1, 0x6060, 6));
    want.push_back(W(1, 0x6098, m)); want.push_back(W(1, 0x6040, 0x0F)); want.push_back(W(1, 0x6040, 0x1F));
    seq(jmc, c, "Homing method " + std::to_string(m) + " started for M1", want, "p.27 routine (parameters before the start bit)");
    pc.events.clear();
    runFor(jmc, 400);
    bool homed = false;
    for (auto& e : pc.events) if (e.find("EVENT:HOMED:M1 POS=300") != std::string::npos) homed = true;
    check(homed, c + ": EVENT:HOMED:M1 POS=300 after the run");
  }
  seq(jmc, "HO:-300", "HO: M1=-300", { W32(1, 0x607C, -300) }, "negative offset stored");
  for (int m = 1; m <= 12; m++) {
    std::string c = "H" + std::to_string(m) + ":M1";
    auto r = cmd(jmc, c);
    bool refused = !r.empty() && r[0].find("negative HO needs H9..H12") != std::string::npos;
    check(m <= 8 ? refused : (!r.empty() && r[0].rfind("Homing method", 0) == 0),
          c + (m <= 8 ? ": negative HO refused (p.25-26)" : ": negative HO accepted (p.25-26)"), r.empty() ? "" : r[0]);
    runFor(jmc, 400);
  }
  seq(jmc, "HO:0", "HO: M1=0", {}, "");
  seq(jmc, "HV:6", "Set HV: M1=6.00 rps", {}, "");
  for (int m = 1; m <= 12; m++) {
    auto r = cmd(jmc, "H" + std::to_string(m) + ":M1");
    bool refused = !r.empty() && r[0].find("above 5 rps") != std::string::npos;
    check(m <= 4 ? refused : !refused, "H" + std::to_string(m) + " at 6 rps: " + (m <= 4 ? "refused (300 r/min, p.21-22)" : "allowed (1200 r/min, p.23-26)"), r.empty() ? "" : r[0]);
    runFor(jmc, 400);
  }
  seq(jmc, "HV:0.3", "Set HV: M1=0.30 rps", {}, "");
  seq(jmc, "HV:20.1", "ERR:HV range 0.01..20.0 rps", {}, "");

  // ---- position set (homing method 0, p.20) --------------------------------------------------
  seq(jmc, "Z:M2", "Motor 2 set to zero position", { W(2, 0x6060, 6), W(2, 0x6098, 0), W32(2, 0x607C, 0), W(2, 0x6040, 0x0F), W(2, 0x6040, 0x1F), W(2, 0x6060, 1) }, "p.20 method 0");
  seq(jmc, "ZO:M2=-12345", "ZO: M2=-12345", { W(2, 0x6098, 0), W32(2, 0x607C, -12345), W(2, 0x6040, 0x1F), W(2, 0x6060, 1) }, "method 0 with offset");
  seq(jmc, "P:M2", "M2=-12345", { R(2, 0x6064) }, "p.8 INT32 readback");

  // ---- status reads (p.6-8) -----------------------------------------------------------------------
  seq(jmc, "S:M1", "STATUS:M1=0x", { R(1, 0x6041) }, "p.6");
  seq(jmc, "S:M1:COMPACT", "M1:RUN|", { R(1, 0x6041), R(1, 0x6064), R(1, 0x606C) }, "p.6, p.8");
  seq(jmc, "LS:M1", "LIMITS: M1:CW=", { R(1, 0x6041) }, "p.6-7 bits 11/14/15");
  seq(jmc, "SD:M1", "=== MOTOR 1", { R(1, 0x6041) }, "");
  seq(jmc, "DG:M1", "DG M1: fmt6000=0", { R(1, 0x6000), R(1, 0x6041), R(1, 0x6061), R(1, 0x1001), R(1, 0x607A), R(1, 0x6081), R(1, 0x6083), R(1, 0x6084), R(1, 0x6064) }, "p.4-9");

  // ---- write-skipping: everything is sent again when the drive may have changed ---------------------
  seq(jmc, "V:1,1,1", "Set V:", {}, "");
  seq(jmc, "P:500", "OK", { W(1, 0x6060, 1), W32(1, 0x607A, 500) }, "leaving homing mode: mode written");
  runFor(jmc, 300);
  seq(jmc, "P:600", "OK", { W32(1, 0x607A, 600), W(1, 0x6040, 0x3F) }, "");
  runFor(jmc, 300);
  seq(jmc, "B1:M1", "Deceleration brake applied to motor 1", {}, "");
  seq(jmc, "P:700", "OK", { W(1, 0x6040, 0x0F), W32(1, 0x607A, 700), W(1, 0x6040, 0x3F) }, "after a quick stop: enable sent again");
  runFor(jmc, 300);
  port.d[1].fault = true; port.d[1].errReg = 2; runFor(jmc, 100);
  port.d[1].fault = false; port.d[1].errReg = 0; runFor(jmc, 100);
  seq(jmc, "P:800", "OK", { W(1, 0x6040, 0x0F), W(1, 0x6060, 1), W32(1, 0x6081, 10), W32(1, 0x607A, 800) }, "after the monitor saw a fault: full sequence again");
  runFor(jmc, 300);
  seq(jmc, "VS:M1", "VS: velocity mode started for M1", {}, "");
  runFor(jmc, 100);
  seq(jmc, "B:M1", "Motor 1 stopped with deceleration", {}, "");
  runFor(jmc, 100);
  seq(jmc, "P:0", "OK", { W(1, 0x6060, 1), W32(1, 0x6081, 10), W32(1, 0x607A, 0) }, "after a jog: position mode and speed written again");
  runFor(jmc, 1500);
  seq(jmc, "PA:100", "OK", { W32(1, 0x607A, 100), W32(2, 0x607A, 100), W32(3, 0x607A, 100), W(0, 0x6040, 0x3F), W(0, 0x6040, 0x2F) }, "");
  runFor(jmc, 300);
  {
    keepLog();
    cmd(jmc, "PA:200");
    int n = 0; for (auto& f : port.log) if (f[1] != 0x03) n++;
    check(n == 5, "all three motors: 3 target frames + 2 broadcast pulses = 5 frames per update", std::to_string(n));
  }
  runFor(jmc, 300);

  // ---- the manual's general rules, over every frame of the run --------------------------------------
  keepLog();
  int badFc = 0, bcRead = 0, readWO = 0, multi = 0, w6on32 = 0, w32on16 = 0;
  const int r32[] = { 0x6064, 0x606C, 0x607A, 0x607C, 0x6081, 0x6099, 0x609B };
  auto is32 = [&](int r) { for (int x : r32) if (x == r) return true; return false; };
  for (auto& f : allFrames) {
    Fr d = decode(f);
    if (d.fc != 0x03 && d.fc != 0x06 && d.fc != 0x10) badFc++;
    if (d.slave == 0 && d.fc == 0x03) bcRead++;
    if (d.fc == 0x03 && (d.reg == 0x6040 || d.reg == 0x6060)) readWO++;
    if (d.fc == 0x10 && d.n != 2) multi++;
    if (d.fc == 0x06 && is32(d.reg)) w6on32++;
    if (d.fc == 0x10 && !is32(d.reg)) w32on16++;
  }
  check(badFc == 0, "only function codes 03 / 06 / 10 (p.3)", std::to_string(badFc));
  check(bcRead == 0, "never reads from broadcast address 0 (p.2)", std::to_string(bcRead));
  check(readWO == 0, "never reads write-only 0x6040 / 0x6060 (p.32)", std::to_string(readWO));
  check(multi == 0, "0x10 writes are always one 32-bit register (p.32 tip)", std::to_string(multi));
  check(w6on32 == 0, "32-bit registers never written with 0x06 (p.29)", std::to_string(w6on32));
  check(w32on16 == 0, "16-bit registers never written with 0x10 as 32-bit", std::to_string(w32on16));
  int bcVel = 0;
  for (auto& f : allFrames) { Fr d = decode(f); if (d.slave == 0 && d.reg == 0x6081) bcVel++; }
  check(bcVel == 0, "a signed speed is never broadcast into drives that may be in position mode", std::to_string(bcVel));
  check(port.crcErrors == 0, "every frame CRC-valid (p.3, p.33)", std::to_string(port.crcErrors));
  check(allFrames.size() > 300, "frames examined: " + std::to_string(allFrames.size()));
  for (int i = 1; i <= 3; i++)
    check(port.d[i].modeSwitchWhileSpinning == 0, "drive " + std::to_string(i) + ": mode never changed while turning",
          std::to_string(port.d[i].modeSwitchWhileSpinning));

  printf("frame check: %d passed, %d failed (%zu frames)\n", pass, fail, allFrames.size());
  return fail ? 1 : 0;
}
