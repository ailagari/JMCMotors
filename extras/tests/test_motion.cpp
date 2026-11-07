// Behaviour check against the JMC Modbus-RTU manual V2.2, with drives that
// move physically: limit and origin switches sit on the axis, homing runs per
// the manual's method descriptions (section 6.3). Forward / reverse, single /
// several motors, every homing method, limits, status formats, resets.
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

static void check(bool ok, const std::string& what, const std::string& got = "") {
  if (ok) { pass++; return; }
  fail++;
  printf("FAIL: %s\n      got: %s\n", what.c_str(), got.c_str());
}
static std::string joined(const std::vector<std::string>& v) {
  std::string s; for (auto& x : v) { if (!s.empty()) s += " || "; s += x; } return s;
}
static void runFor(JMCController& c, uint32_t ms) {
  uint64_t end = g_us + (uint64_t)ms * 1000;
  while (g_us < end) { c.run(); port.tick(); delay(1); }
}
static std::vector<std::string> cmd(JMCController& c, const std::string& line) {
  pc.replies.clear(); pc.in.push_back(line); c.run(); return pc.replies;
}
static void expect(JMCController& c, const std::string& line, const std::string& want) {
  auto r = cmd(c, line);
  check(r.size() == 1 && r[0] == want, line + "  ->  " + want, joined(r));
}
static void expectHas(JMCController& c, const std::string& line, const std::string& part) {
  auto r = cmd(c, line);
  check(joined(r).find(part) != std::string::npos, line + "  ->  ...'" + part + "'...", joined(r));
}
static bool ev(const std::string& part) {
  for (auto& e : pc.events) if (e.find(part) != std::string::npos) return true;
  return false;
}
static void settle(JMCController& c, uint32_t ms = 8000) {
  uint64_t end = g_us + (uint64_t)ms * 1000;
  while (g_us < end) {
    c.run(); port.tick(); delay(1);
    bool any = false;
    for (int i = 1; i <= 3; i++) if (port.d[i].moving || port.d[i].homing) any = true;
    if (!any) { runFor(c, 60); return; }
  }
}
// Put drive i back at physical "0" with the switches at their places.
static void place(JMCController& c, int i, bool sw = true) {
  FakeDrive& d = port.d[i];
  d.cwAt = -20000; d.swAt = 3000; d.ccwAt = 20000; d.hasSW = sw;
  d.pos = 0; d.posF = 0; d.target = 0; d.moving = false;
  d.sense();
  expect(c, "ZO:M" + std::to_string(i) + "=0", "ZO: M" + std::to_string(i) + "=0 (current position redefined)");
  runFor(c, 60);
}

int main() {
  for (int i = 1; i <= 32; i++) port.d[i].present = (i <= 3);
  for (int i = 1; i <= 3; i++) port.d[i].autoSwitches = true;
  bus.begin(115200, 7);
  JMCController jmc(bus, pc);
  jmc.setMotorCount(3);
  jmc.begin();
  runFor(jmc, 50);
  for (int i = 1; i <= 3; i++) place(jmc, i);
  expect(jmc, "V:2,2,2", "Set V: M1=2.00,M2=2.00,M3=2.00 rps");    // 8000 steps/s

  // ---- forward / reverse, one motor ------------------------------------------------
  expect(jmc, "P:10000", "OK");
  runFor(jmc, 300);
  expectHas(jmc, "S:M1:COMPACT", "M1:RUN|MOV|NR|");
  settle(jmc);
  expectHas(jmc, "S:M1:COMPACT", "M1:RUN|STP|TR|POS=10000|");
  expect(jmc, "P:-10000", "OK");
  settle(jmc);
  expect(jmc, "P:?", "M1=-10000");
  expect(jmc, "PR:4000", "OK");  settle(jmc);  expect(jmc, "P:?", "M1=-6000");
  expect(jmc, "PR:-4000", "OK"); settle(jmc);  expect(jmc, "P:?", "M1=-10000");
  expect(jmc, "P:0", "OK");      settle(jmc);

  // ---- several motors: same and mixed directions -----------------------------------------
  expect(jmc, "PA:5000", "OK");   settle(jmc);  expect(jmc, "PA:?", "M1=5000,M2=5000,M3=5000");
  {
    long t12 = (long)port.d[1].arrivedUs - (long)port.d[2].arrivedUs, t13 = (long)port.d[1].arrivedUs - (long)port.d[3].arrivedUs;
    check(labs(t12) < 3000 && labs(t13) < 3000, "PA: one broadcast start, all three arrive together", std::to_string(t12) + "/" + std::to_string(t13) + " us");
  }
  expect(jmc, "PA:-5000", "OK");  settle(jmc);  expect(jmc, "PA:?", "M1=-5000,M2=-5000,M3=-5000");
  expect(jmc, "P:1000,-1000,0", "OK"); settle(jmc); expect(jmc, "PA:?", "M1=1000,M2=-1000,M3=0");
  expect(jmc, "PRA:2500", "OK");  settle(jmc);  expect(jmc, "PA:?", "M1=3500,M2=1500,M3=2500");
  expect(jmc, "PR:M2=-3000,M3=500", "OK"); settle(jmc); expect(jmc, "PA:?", "M1=3500,M2=-1500,M3=3000");
  expectHas(jmc, "PS:-3500,1500,0", "OK"); settle(jmc); expect(jmc, "PA:?", "M1=-3500,M2=1500,M3=0");
  expect(jmc, "PA:0", "OK");      settle(jmc);

  // ---- jogs in both directions, single / all stops -----------------------------------------
  expect(jmc, "V:1,-1,0.5", "Set V: M1=1.00,M2=-1.00,M3=0.50 rps");
  expect(jmc, "VSA", "VS: velocity mode started (stop with B)");
  runFor(jmc, 300);
  check(port.d[1].velReg() == 10 && port.d[2].velReg() == -10 && port.d[3].velReg() == 5,
        "VSA: M1 forward, M2 reverse, M3 forward slow",
        std::to_string(port.d[1].velReg()) + "," + std::to_string(port.d[2].velReg()) + "," + std::to_string(port.d[3].velReg()));
  expect(jmc, "B:,1", "Motor 2 stopped with deceleration");
  runFor(jmc, 50);
  check(port.d[2].velReg() == 0 && port.d[1].velReg() == 10, "B:,1 stops only motor 2");
  expect(jmc, "BA", "All motors stopped with deceleration");
  runFor(jmc, 50);
  check(port.d[1].velReg() == 0 && port.d[3].velReg() == 0, "BA stops the rest");
  check(port.d[1].pos > 0 && port.d[2].pos < 0 && port.d[3].pos > 0, "positions moved the right way",
        std::to_string(port.d[1].pos) + "," + std::to_string(port.d[2].pos) + "," + std::to_string(port.d[3].pos));
  expect(jmc, "V:2,2,2", "Set V: M1=2.00,M2=2.00,M3=2.00 rps");
  expect(jmc, "PA:0", "OK"); settle(jmc);

  // ---- hardware limits (CW = left / negative end, CCW = right / positive) ------------------
  pc.events.clear();
  expect(jmc, "P:-30000", "OK");
  settle(jmc);
  check(ev("EVENT:LIMIT:M1:CW POS=-20000"), "running into the CW limit: EVENT:LIMIT:M1:CW at -20000", joined(pc.events));
  expect(jmc, "LS:M1", "LIMITS: M1:CW=1 CCW=0 SW=0 HOMED=0 MODE=POS");
  expect(jmc, "P:-25000", "OK");  runFor(jmc, 400);  expect(jmc, "P:?", "M1=-20000");   // blocked
  expect(jmc, "P:0", "OK");       settle(jmc);
  check(ev("EVENT:LIMIT:M1:CW_CLEARED"), "leaving it: CW_CLEARED", joined(pc.events));
  pc.events.clear();
  expect(jmc, "P:30000", "OK");
  settle(jmc);
  check(ev("EVENT:LIMIT:M1:CCW POS=20000"), "running into the CCW limit: EVENT:LIMIT:M1:CCW at 20000", joined(pc.events));
  expect(jmc, "P:25000", "OK");  runFor(jmc, 400);  expect(jmc, "P:?", "M1=20000");
  expect(jmc, "P:0", "OK");      settle(jmc);
  check(ev("EVENT:LIMIT:M1:CCW_CLEARED"), "CCW_CLEARED", joined(pc.events));
  // the same with a jog
  pc.events.clear();
  expect(jmc, "V:-2", "Set V: M1=-2.00 rps");
  expect(jmc, "VS:M1", "VS: velocity mode started for M1 (stop with B)");
  runFor(jmc, 4000);
  check(port.d[1].pos == -20000 && ev("EVENT:LIMIT:M1:CW"), "a reverse jog stops at the CW limit", std::to_string(port.d[1].pos));
  expect(jmc, "B", "All motors stopped with deceleration");
  expect(jmc, "V:2", "Set V: M1=2.00 rps");
  expect(jmc, "P:0", "OK"); settle(jmc);

  // ---- homing, every method, with real switches (manual p.20-26) -----------------------------
  expect(jmc, "HV:4", "Set HV: M1=4.00 rps");       // <= 5 rps: valid for every method
  expect(jmc, "HVZ:1", "Set HVZ: M1=1.00 rps");
  expect(jmc, "HO:0", "HO: M1=0 steps after sensor (used at next homing)");
  for (int m = 1; m <= 12; m++) {
    place(jmc, 1);
    pc.events.clear();
    std::string c = "H" + std::to_string(m) + ":M1";
    expect(jmc, c, "Homing method " + std::to_string(m) + " started for M1");
    runFor(jmc, 60);
    expectHas(jmc, "LS:M1", "HOMED=BUSY MODE=HOM");
    settle(jmc, 10000);
    bool toSwitch = (m == 3 || m == 4 || m == 7 || m == 8 || m == 11 || m == 12);
    int target = toSwitch ? 3 : (m % 2 ? 1 : 2);
    const char* name = target == 1 ? "CW limit" : target == 2 ? "CCW limit" : "origin switch";
    FakeDrive& d = port.d[1];
    int32_t at = target == 1 ? d.cwAt : target == 2 ? d.ccwAt : d.swAt;
    check(labs(at) <= 60, c + ": ends on the " + name + " (manual p.21-26)", "switch now at " + std::to_string(at));
    check(ev("EVENT:HOMED:M1 POS=0"), c + ": EVENT:HOMED:M1 POS=0", joined(pc.events));
    if (target == 1) check(ev("EVENT:LIMIT:M1:CW"), c + ": EVENT:LIMIT:M1:CW on the way", joined(pc.events));
    if (target == 2) check(ev("EVENT:LIMIT:M1:CCW"), c + ": EVENT:LIMIT:M1:CCW on the way", joined(pc.events));
    if (target == 3) check(ev("EVENT:ORIGIN:M1:ON"), c + ": EVENT:ORIGIN:M1:ON at the end", joined(pc.events));
    // the path on the axis (a momentary limit contact while reversing is shorter
    // than a status poll, so it is checked on the simulated axis itself)
    if (m % 2 == 1 && target == 3)   // left first, origin to the right: reverses at the CW limit
      check(d.cwTouched && !d.ccwTouched, c + ": went left, reversed at the CW limit, then to the origin (p.22, 24, 26)");
    if (m % 2 == 0 && target == 3)   // right first: meets the origin before the CCW limit
      check(!d.ccwTouched && !d.cwTouched, c + ": went right and found the origin before any limit (p.22, 24, 26)");
    auto ls = cmd(jmc, "LS:M1");
    check(!ls.empty() && ls[0].find("HOMED=1 MODE=HOM") != std::string::npos, c + ": LS shows HOMED=1", joined(ls));
  }
  // home offset is applied
  place(jmc, 1);
  expect(jmc, "HO:250", "HO: M1=250 steps after sensor (used at next homing)");
  pc.events.clear();
  expect(jmc, "H4:M1", "Homing method 4 started for M1");
  settle(jmc, 10000);
  check(ev("EVENT:HOMED:M1 POS=250"), "homing with HO:250 ends at POS=250", joined(pc.events));
  expect(jmc, "HO:0", "HO: M1=0 steps after sensor (used at next homing)");
  // a missing origin switch: homing error, then reset
  place(jmc, 1, false);
  pc.events.clear();
  expect(jmc, "H3:M1", "Homing method 3 started for M1");
  settle(jmc, 15000);
  check(ev("EVENT:HOMING_ERROR:M1"), "no origin switch: EVENT:HOMING_ERROR:M1 (status bit 13)", joined(pc.events));
  expectHas(jmc, "LS:M1", "HOMED=ERR MODE=HOM");
  expectHas(jmc, "S:M1:FLAGS", "HOME_ERROR");
  expect(jmc, "R:M1", "Motor 1 reset complete");
  expectHas(jmc, "LS:M1", "HOMED=0 MODE=POS");
  // several motors at once
  for (int i = 1; i <= 3; i++) place(jmc, i);
  expect(jmc, "HV:4,4,4", "Set HV: M1=4.00,M2=4.00,M3=4.00 rps");
  pc.events.clear();
  expect(jmc, "H8A", "Homing method 8 started for all motors");
  settle(jmc, 10000);
  check(ev("EVENT:HOMED:M1") && ev("EVENT:HOMED:M2") && ev("EVENT:HOMED:M3"), "H8A: all three homed", joined(pc.events));
  expect(jmc, "LSA", "LIMITS: M1:CW=0 CCW=0 SW=1 HOMED=1 MODE=HOM,M2:CW=0 CCW=0 SW=1 HOMED=1 MODE=HOM,M3:CW=0 CCW=0 SW=1 HOMED=1 MODE=HOM");

  // ---- status formats decode the mode-dependent bits (manual p.6) ------------------------------
  expectHas(jmc, "S:M1:FLAGS", "HOME_REACHED|SWORIGIN|HOMED");       // homing mode: bits 10, 11, 12
  expectHas(jmc, "S:M1:DETAIL", "Mode:   homing");
  expect(jmc, "PA:1000", "OK");  settle(jmc);
  expectHas(jmc, "S:M1:FLAGS", "REACHED|SETPOINT_OK");              // position mode names
  expectHas(jmc, "S:M1:STATE", "STATES:M1:Device enable");
  expectHas(jmc, "S:M1", "STATUS:M1=0x1437");
  expectHas(jmc, "S:M1:BITS", "M1:0001 0100 0011 0111");
  expectHas(jmc, "BSA", "BRAKE STATUS: M1:FREE/STOP");
  expect(jmc, "B", "All motors stopped with deceleration");
  expectHas(jmc, "BS:M1", "M1:BRAKE/STOP");                          // halt bit 8 set
  expectHas(jmc, "S:M1:FLAGS", "HALT");
  expect(jmc, "L:M1", "Motor 1 brake loosened");
  expectHas(jmc, "S:M1:STATE", "Power on");                          // 0x0003: motor off
  expectHas(jmc, "SD:M1", "Motor: DISABLED");
  expectHas(jmc, "I:M1", "DETECTED - Initialized in Position mode");
  expectHas(jmc, "S:M1:STATE", "Device enable");

  // ---- faults and resets ------------------------------------------------------------------------
  pc.events.clear();
  port.d[2].fault = true; port.d[2].errReg = 0x02;
  runFor(jmc, 100);
  check(ev("EVENT:FAULT:M2") && ev("ERR=0x2 CURRENT"), "fault: EVENT:FAULT:M2 ... CURRENT", joined(pc.events));
  expectHas(jmc, "S:M2:STATE", "Fault alarm");
  expectHas(jmc, "SD:M2", "FAULT: over-current");
  expectHas(jmc, "S:M2:COMPACT", "M2:ERR|");
  expect(jmc, "RA", "All motors reset complete");
  check(ev("EVENT:FAULT_CLEARED:M2"), "RA clears it: EVENT:FAULT_CLEARED:M2", joined(pc.events));
  expectHas(jmc, "S:M2:STATE", "Device enable");
  expect(jmc, "P:,2000", "OK"); settle(jmc); expect(jmc, "P:,?", "M2=2000");   // moves again after the reset
  // power-cycled drive
  pc.events.clear();
  port.d[3].present = false; runFor(jmc, 300);
  port.d[3].pos = 0; port.d[3].posF = 0; port.d[3].ctrl = 0; port.d[3].mode = 1;
  port.d[3].present = true;  runFor(jmc, 3500);
  check(ev("EVENT:OFFLINE:M3") && ev("EVENT:ONLINE:M3"), "power cycle: OFFLINE then ONLINE (re-initialised)", joined(pc.events));
  expect(jmc, "P:,,1500", "OK"); settle(jmc); expect(jmc, "P:,,?", "M3=1500");

  // ---- findings of the independent audit ------------------------------------------------------
  expect(jmc, "V:2,2,2", "Set V: M1=2.00,M2=2.00,M3=2.00 rps");
  expect(jmc, "PA:0", "OK"); settle(jmc);
  // a halt mid-move must not let the drive resume its old target (p.18)
  expect(jmc, "P:40000", "OK");
  runFor(jmc, 300);
  expect(jmc, "B:M1", "Motor 1 stopped with deceleration");
  runFor(jmc, 100);
  int32_t stopAt = port.d[1].pos;
  port.d[1].maxPos = stopAt;
  expect(jmc, "P:0", "OK");
  settle(jmc);
  check(port.d[1].maxPos <= stopAt, "after B, P:0 goes straight back - no lurch towards the old target",
        "stopped at " + std::to_string(stopAt) + ", reached " + std::to_string(port.d[1].maxPos));
  expect(jmc, "P:?", "M1=0");
  expect(jmc, "P:40000", "OK");
  runFor(jmc, 300);
  expect(jmc, "B:M1", "Motor 1 stopped with deceleration");
  runFor(jmc, 100);
  stopAt = port.d[1].pos;
  port.d[1].maxPos = stopAt;
  expect(jmc, "Z:M1", "Motor 1 set to zero position");
  runFor(jmc, 200);
  check(port.d[1].maxPos <= stopAt, "after B, Z does not move the shaft", std::to_string(port.d[1].maxPos - stopAt) + " steps");
  // Z / ZO read back: a drive applying the offset with the other sign is corrected
  port.d[1].offsetMode = 1;
  expect(jmc, "ZO:M1=5000", "ZO: M1=5000 (current position redefined)");
  expect(jmc, "P:?", "M1=5000");
  expect(jmc, "ZO:M1=-700", "ZO: M1=-700 (current position redefined)");
  expect(jmc, "P:?", "M1=-700");
  // ... and one that ignores it is reported, not trusted
  port.d[1].offsetMode = 2;
  expect(jmc, "ZO:M1=5000", "ERR:ZO M1 reads 0 after setting 5000");
  expect(jmc, "Z:M1", "Motor 1 set to zero position");
  port.d[1].offsetMode = 0;
  // quick stop: the state reads "Quick stop active" (state table p.7), C++ API too
  expect(jmc, "B1:M1", "Deceleration brake applied to motor 1");
  expectHas(jmc, "S:M1:STATE", "Quick stop active");
  check(jmc.motor(0)->getDetailedStatus().quickStopActive, "getDetailedStatus().quickStopActive true in quick stop");
  expectHas(jmc, "I:M1", "DETECTED");
  check(!jmc.motor(0)->getDetailedStatus().quickStopActive, "quickStopActive false when enabled (bit 5 = 1)");
  // a busy drive (exception 6) is retried, as the manual asks (p.3)
  port.d[1].rejectReg = 0x6040; port.d[1].rejectCode = 6; port.d[1].rejectTimes = 1;
  expect(jmc, "L:M1", "Motor 1 brake loosened");
  check(port.d[1].rejectTimes == 0, "the busy reply was answered with a retry");
  port.d[1].rejectReg = 0; port.d[1].rejectTimes = -1; port.d[1].rejectCode = 3;
  expectHas(jmc, "I:M1", "DETECTED");

  check(port.crcErrors == 0, "every frame CRC-valid", std::to_string(port.crcErrors));
  for (int i = 1; i <= 3; i++)
    check(port.d[i].modeSwitchWhileSpinning == 0, "drive " + std::to_string(i) + ": mode never changed while turning",
          std::to_string(port.d[i].modeSwitchWhileSpinning));
  printf("motion check: %d passed, %d failed\n", pass, fail);
  return fail ? 1 : 0;
}
