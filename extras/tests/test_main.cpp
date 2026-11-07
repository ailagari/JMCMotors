// JMCMotors host tests: the real library sources, a fake Arduino core and
// simulated drives (fake_drives.h). Build and run with run_tests.sh.
//   default build   = the CONTROLLINO / Mega / ESP32 profile (32 motors)
//   -DSMALL_PROFILE = the Uno profile (6 motors, features left out)
#include <JMCMotors.h>
#include <JMCLinkSerial.h>
#include <EEPROM.h>
#include "fake_drives.h"
#include <cstdio>
#include <string>
#include <vector>
#include <deque>

// Expected reply in the full profile / in the Uno profile.
#ifdef SMALL_PROFILE
  #define PICK(full, uno) (uno)
#else
  #define PICK(full, uno) (full)
#endif

uint64_t g_us = 0;
int g_pinState[128];
EEPROMClass EEPROM;

// ---------------------------------------------------------------- PC link
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
  std::string s;
  for (auto& x : v) { if (!s.empty()) s += " || "; s += x; }
  return s;
}
static void runFor(JMCController& c, uint32_t ms) {
  uint64_t end = g_us + (uint64_t)ms * 1000;
  while (g_us < end) { c.run(); port.tick(); delay(1); }
}
// Send one command, return every reply it produced.
static std::vector<std::string> cmd(JMCController& c, const std::string& line) {
  pc.replies.clear();
  pc.in.push_back(line);
  c.run();
  return pc.replies;
}
static void expect(JMCController& c, const std::string& line, const std::string& want) {
  auto r = cmd(c, line);
  check(r.size() == 1 && r[0] == want, line + "  ->  " + want, joined(r));
}
static void expectHas(JMCController& c, const std::string& line, const std::string& part) {
  auto r = cmd(c, line);
  check(joined(r).find(part) != std::string::npos, line + "  ->  ...'" + part + "'...", joined(r));
}
// A frame from the manual, written as hex text, CRC included.
static std::vector<uint8_t> hexFrame(const char* h) {
  std::vector<uint8_t> f;
  while (*h) {
    while (*h == ' ') h++;
    if (!*h) break;
    f.push_back((uint8_t)strtol(std::string(h, 2).c_str(), nullptr, 16));
    h += 2;
  }
  return f;
}
static void sawManual(const char* hex, const std::string& what) {
  std::vector<uint8_t> f = hexFrame(hex);
  uint16_t c = FakeBus::crc(f.data(), f.size() - 2);
  check((c & 0xFF) == f[f.size() - 2] && (c >> 8) == f[f.size() - 1], "manual frame CRC agrees with ours: " + std::string(hex));
  check(port.sawFrame(f), what + "  =  manual frame " + hex);
}
static bool eventHas(const std::string& part) {
  for (auto& e : pc.events) if (e.find(part) != std::string::npos) return true;
  return false;
}
static void waitArrive(JMCController& c, uint32_t ms = 3000) {
  uint64_t end = g_us + (uint64_t)ms * 1000;
  while (g_us < end) {
    c.run(); port.tick(); delay(1);
    bool any = false;
    for (int i = 1; i <= 4; i++) if (port.d[i].present && port.d[i].moving) any = true;
    if (!any) return;
  }
}

int main() {
  for (int i = 1; i <= 32; i++) port.d[i].present = (i == 1 || i == 3 || i == 4);
  bus.begin(115200, 7);
  JMCController jmc(bus, pc);
  jmc.setMotorCount(4);
  uint8_t found = jmc.begin();
  check(found == 3, "begin() finds drives 1, 3, 4", std::to_string(found));
  check(eventHas("Motor ready"), "boot event 'Motor ready'", joined(pc.events));
  check(port.crcErrors == 0, "every request frame has a valid CRC", std::to_string(port.crcErrors));

  // ---- basics ----------------------------------------------------------------
#ifdef SMALL_PROFILE
  expect(jmc, "VER", "JMCMotors 0.6.0 (max 6 motors)");
#else
  expect(jmc, "VER", "JMCMotors 0.6.0 (max 32 motors)");
#endif
  expect(jmc, "PING", "PONG");
  expect(jmc, PICK("#17 PING", "PING"), PICK("#17 PONG", "PONG"));   // Uno: no reply tags
  expect(jmc, "p:?", "M1=0");
  expect(jmc, "FOO", "ERR:unknown command FOO");
  expectHas(jmc, "IA", "Total motors detected: 3 of 4");

  // ---- strict parsing ----------------------------------------------------------
  expect(jmc, "P:12abc", "ERR:P bad value '12abc' (M1)");
  expect(jmc, "P:1e3", "ERR:P bad value '1e3' (M1)");
  expect(jmc, "V:fast", "ERR:V bad value 'fast' (M1)");
  expect(jmc, "P:2147483648", "ERR:P bad value '2147483648' (M1)");
  expect(jmc, "P:1000,?", "ERR:P cannot mix values and ?");
  expect(jmc, "SL:M1=-2147483648/2147483647", "SL: M1=-2147483648/2147483647");
  expect(jmc, "SLA:OFF", "SLA: all motors = OFF");
  {
    pc.replies.clear();
    pc.in.push_back(std::string(600, 'x'));
    jmc.run();
    check(pc.replies.size() == 1 && pc.replies[0] == "ERR:command too long", "600-char line refused", joined(pc.replies));
  }
#ifdef SMALL_PROFILE
  expect(jmc, "P:", "ERR:P needs values");
#else
  expect(jmc, "P:", "ERR:P needs values (e.g. P:1000,,3000 or P:M3=1000) - PA:<value> = all motors");
#endif

  // ---- negative absolute move: exact frame + round trip ------------------------
  port.log.clear();
  expect(jmc, "P:-1000", "OK");
  {
    std::vector<uint8_t> f = { 0x01, 0x10, 0x60, 0x7A, 0x00, 0x02, 0x04, 0xFF, 0xFF, 0xFC, 0x18 };
    uint16_t c = FakeBus::crc(f.data(), f.size());
    f.push_back(c & 0xFF); f.push_back(c >> 8);
    check(f[11] == 0x9D && f[12] == 0xD8, "CRC of the P:-1000 target frame is 9D D8");
    check(port.sawFrame(f), "P:-1000 writes 01 10 60 7A 00 02 04 FF FF FC 18 9D D8");
    // control word sequence after the target: 0x3F then 0x2F
    std::vector<uint8_t> a = { 0x01, 0x06, 0x60, 0x40, 0x00, 0x3F }, b = { 0x01, 0x06, 0x60, 0x40, 0x00, 0x2F };
    uint16_t ca = FakeBus::crc(a.data(), 6), cb = FakeBus::crc(b.data(), 6);
    a.push_back(ca & 0xFF); a.push_back(ca >> 8); b.push_back(cb & 0xFF); b.push_back(cb >> 8);
    check(port.sawFrame(a) && port.sawFrame(b), "start pulse 0x3F then 0x2F on the control word");
  }
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=-1000");
  expect(jmc, "PR:-500", "OK");
  waitArrive(jmc);
  expect(jmc, "P:M1", "M1=-1500");
  expect(jmc, "PR:2500", "OK");
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=1000");

  // ---- byte-for-byte against the JMC Modbus-RTU manual (sections 5-7) -----------------
  port.log.clear();
  expectHas(jmc, "I:M1", "DETECTED");          // the power-up routine again
  sawManual("01 06 60 40 00 01 57 DE", "begin: drive initialization");
  sawManual("01 06 60 40 00 03 D6 1F", "begin: drive on, brake open");
  sawManual("01 06 60 40 00 0F D6 1A", "begin: operation enabled");
  sawManual("01 06 60 60 00 01 56 14", "begin: position mode");
  port.log.clear();
  expect(jmc, "ACC:10", "Set ACC: M1=10.00 rps/s");
  sawManual("01 06 60 83 00 64 67 C9", "ACC:10 -> 0x6083 = 100");
  expect(jmc, "DEC:10", "Set DEC: M1=10.00 rps/s");
  sawManual("01 06 60 84 00 64 D6 08", "DEC:10 -> 0x6084 = 100");
  expect(jmc, "ACC:0.5", "Set ACC: M1=0.50 rps/s");
  expect(jmc, "DEC:0.5", "Set DEC: M1=0.50 rps/s");
  expect(jmc, "V:1", "Set V: M1=1.00 rps");
  port.log.clear();
  expect(jmc, "P:200000", "OK");
  sawManual("01 10 60 81 00 02 04 00 00 00 0A 12 06", "V:1 -> 0x6081 = 10");
  sawManual("01 10 60 7A 00 02 04 00 03 0D 40 29 96", "P:200000 -> 0x607A (0x6000 = 0)");
  expect(jmc, "B", "All motors stopped with deceleration");
  expect(jmc, "ZO:M1=1000", "ZO: M1=1000 (current position redefined)");
  {
    // the drive's reply to a 32-bit write, as printed in the manual
    std::vector<uint8_t> fb = hexFrame("01 10 60 7A 00 02 7E 11");
    uint16_t c = FakeBus::crc(fb.data(), 6);
    check((c & 0xFF) == 0x7E && (c >> 8) == 0x11, "CRC of the manual's write reply 01 10 60 7A 00 02 = 7E 11");
  }
  // the other 32-bit word order (0x6000 = 1)
  port.d[1].format = 1;
  expectHas(jmc, "I:M1", "DETECTED");
  port.log.clear();
  expect(jmc, "P:200000", "OK");
  sawManual("01 10 60 7A 00 02 04 0D 40 00 03 9F 8F", "P:200000 -> 0x607A (0x6000 = 1)");
  expect(jmc, "B", "All motors stopped with deceleration");
  expect(jmc, "ZO:M1=-123456", "ZO: M1=-123456 (current position redefined)");
  expect(jmc, "P:?", "M1=-123456");          // negative readback in the other word order
  port.d[1].format = 0;
  expectHas(jmc, "I:M1", "DETECTED");
  expect(jmc, "ZO:M1=1000", "ZO: M1=1000 (current position redefined)");
  // velocity mode routine (manual 6.2)
  port.log.clear();
  expect(jmc, "VS:M1", "VS: velocity mode started for M1 (stop with B)");
  sawManual("01 06 60 40 01 0F D7 8A", "VS: pause (halt) before the speed is written");
  sawManual("01 06 60 60 00 03 D7 D5", "VS: velocity mode");
  sawManual("01 06 60 40 00 0F D6 1A", "VS: release = start");
  expect(jmc, "B", "All motors stopped with deceleration");
  runFor(jmc, 50);
  // homing routine (manual 6.3)
  port.log.clear();
  expect(jmc, "HV:10", "Set HV: M1=10.00 rps");
  sawManual("01 10 60 99 00 02 04 00 00 00 64 93 40", "HV:10 -> 0x6099 = 100");
  expect(jmc, "HVZ:10", PICK("Set HVZ: M1=10.00 rps", "ERR:HVZ not available on this board"));
  sawManual("01 10 60 9B 00 02 04 00 00 00 64 12 99", "HVZ:10 -> 0x609B = 100");
  expect(jmc, "HACC:100", PICK("Set HACC: M1=100.00 rps/s", "ERR:HACC not available on this board"));
#ifndef SMALL_PROFILE
  sawManual("01 06 60 9A 03 E8 B7 5B", "HACC:100 -> 0x609A = 1000");
#endif
  expect(jmc, "HO:1000", "HO: M1=1000 steps after sensor (used at next homing)");
  sawManual("01 10 60 7C 00 02 04 00 00 03 E8 5C 62", "HO:1000 -> 0x607C");
  expect(jmc, "H1:M1", "ERR:H1 M1 HV/HVZ above 5 rps (H1..H4: max 300 r/min)");
  expect(jmc, "H5:M1", "Homing method 5 started for M1");     // 5-12: up to 1200 r/min
  runFor(jmc, 400);
  expect(jmc, "HV:2", "Set HV: M1=2.00 rps");
  expect(jmc, "HVZ:1", PICK("Set HVZ: M1=1.00 rps", "ERR:HVZ not available on this board"));
  expect(jmc, "Z:M1", "Motor 1 set to zero position");   // back to position mode, so H1 sends the whole routine
  port.log.clear();
  expect(jmc, "H1:M1", "Homing method 1 started for M1");
  sawManual("01 06 60 60 00 06 17 D6", "H1: homing mode");
  sawManual("01 06 60 98 00 01 D7 E5", "H1: homing method 1");
  sawManual("01 06 60 40 00 1F D7 D6", "H1: start (bit 4)");
  runFor(jmc, 400);
  expect(jmc, "HO:-500", "HO: M1=-500 steps after sensor (used at next homing)");
  expect(jmc, "H3:M1", "ERR:H3 M1 negative HO needs H9..H12");
  expect(jmc, "H8:M1", "ERR:H8 M1 negative HO needs H9..H12");
  pc.events.clear();
  expect(jmc, "H9:M1", "Homing method 9 started for M1");
  runFor(jmc, 400);
  check(eventHas("EVENT:HOMED:M1 POS=-500"), "H9 with a negative HO ends at -500", joined(pc.events));
  expect(jmc, "HO:0", "HO: M1=0 steps after sensor (used at next homing)");
  expect(jmc, "HV:0.3", "Set HV: M1=0.30 rps");
  expect(jmc, "HVZ:0.3", PICK("Set HVZ: M1=0.30 rps", "ERR:HVZ not available on this board"));
  expect(jmc, "HACC:0.5", PICK("Set HACC: M1=0.50 rps/s", "ERR:HACC not available on this board"));
  expect(jmc, "Z:M1", "Motor 1 set to zero position");

  // ---- PR is relative to where the shaft is, also after ZO / Z ------------------------
  expect(jmc, "V:1", "Set V: M1=1.00 rps");
  expect(jmc, "P:5000", "OK");
  waitArrive(jmc);
  expect(jmc, "ZO:M1=0", "ZO: M1=0 (current position redefined)");
  expect(jmc, "PR:100", "OK");
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=100");                // not 5100
  {
    // the target written is absolute (bit 6 never set by PR)
    bool rel = false;
    for (auto& f : port.log) if (f.size() >= 6 && f[1] == 0x06 && f[2] == 0x60 && f[3] == 0x40 && (f[5] & 0x40)) rel = true;
    check(!rel, "PR never uses the drive's relative bit (6)");
  }
  expect(jmc, "Z:M1", "Motor 1 set to zero position");

  // ---- the negative direction ---------------------------------------------------------
  expect(jmc, "V:1", "Set V: M1=1.00 rps");
  expect(jmc, "P:-4000", "OK");
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=-4000");
  expect(jmc, "PR:-4000", "OK");
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=-8000");
  expect(jmc, "PR:12000", "OK");
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=4000");
  // negative jog stops at the lower soft limit
  expect(jmc, "SL:M1=-3000/100000", "SL: M1=-3000/100000");
  expect(jmc, "V:-1", "Set V: M1=-1.00 rps");
  expect(jmc, "VS:M1", "VS: velocity mode started for M1 (stop with B)");
  runFor(jmc, 300);
  expectHas(jmc, "S:M1:COMPACT", "|VEL=-1.0");
  pc.events.clear();
  runFor(jmc, 2500);
  check(eventHas("EVENT:SOFTLIMIT:M1 POS=") && eventHas("LIMIT=-3000"), "negative jog stopped at -3000", joined(pc.events));
  expect(jmc, "P:-5000", "ERR:P M1 target -5000 outside soft limit -3000..100000");
  expectHas(jmc, "PR:-2147483647", "ERR:PR M1 target -2147");                       // no overflow
  expect(jmc, "PR:-2147483647", "ERR:PR M1 target -2147483648 outside soft limit -3000..100000");
  expect(jmc, "SLA:OFF", "SLA: all motors = OFF");
  expect(jmc, "B", "All motors stopped with deceleration");
  // the CW limit (left / negative end) blocks negative moves; positive ones still run
  expect(jmc, "V:1", "Set V: M1=1.00 rps");
  expect(jmc, "P:0", "OK");
  waitArrive(jmc);
  pc.events.clear();
  port.d[1].cw = true;
  runFor(jmc, 50);
  check(eventHas("EVENT:LIMIT:M1:CW POS=0"), "EVENT:LIMIT:M1:CW while standing", joined(pc.events));
  expect(jmc, "LS:M1", "LIMITS: M1:CW=1 CCW=0 SW=0 HOMED=1 MODE=POS");
  expect(jmc, "P:-1000", "OK");
  runFor(jmc, 500);
  expect(jmc, "P:?", "M1=0");                 // the drive refuses to move into the limit
  expect(jmc, "P:1000", "OK");
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=1000");              // away from it works
  port.d[1].cw = false;
  runFor(jmc, 50);
  expect(jmc, "P:0", "OK");
  waitArrive(jmc);

  // ---- failures in the middle of a command: OK only when every drive took it ---------
  expect(jmc, "V:1,,1", "Set V: M1=1.00,M3=1.00 rps");
  port.d[3].present = false;                   // M3 stops answering; the monitor has not noticed yet
  {
    int latches1 = port.d[1].latches;
    expect(jmc, "P:100,,500", PICK("ERR:P M3 not accepted (TIMEOUT) - nothing started", "ERR:P M3 not accepted (TIMEOUT)"));
    check(port.d[1].latches == latches1 && !port.d[1].moving, "coordinated move aborted: M1 did not start either");
    expect(jmc, "VS:M1,M3", PICK("ERR:VS M3 not accepted (TIMEOUT) - nothing started", "ERR:VS M3 not accepted (TIMEOUT)"));
    runFor(jmc, 20);
    check(port.d[1].velReg() == 0, "VS aborted: M1 not left jogging");
    // by now the monitor has noticed (two failed reads) and reported it offline
    runFor(jmc, 200);
    check(eventHas("EVENT:OFFLINE:M3"), "the silent drive is reported EVENT:OFFLINE:M3", joined(pc.events));
    expect(jmc, "ACC:0.3,,0.3", "Set ACC: M1=0.30,M3=0.30 rps/s");          // stored for M3's re-init
    expect(jmc, "Z:,,1", "ERR:Z M3 offline (check power / RS-485, or send I)");
    expect(jmc, "B", "All motors stopped with deceleration");               // offline M3 skipped (it got the broadcast)
  }
  port.d[3].present = true;
  runFor(jmc, 3500);
  check(port.d[3].reg[0x6083] == 3, "the ACC stored while M3 was offline is written when it comes back", std::to_string(port.d[3].reg[0x6083]));
  {
    // a drive that goes silent right before B (not yet known offline)
    expect(jmc, "V:1", "Set V: M1=1.00 rps");
    expect(jmc, "P:3000", "OK");
    runFor(jmc, 100);
    port.d[3].present = false;
    port.log.clear();
    expect(jmc, "B", PICK("ERR:B M3 not accepted (TIMEOUT) - the others were stopped", "ERR:B M3 not accepted (TIMEOUT)"));
    {
      std::vector<uint8_t> bc = { 0x00, 0x06, 0x60, 0x40, 0x01, 0x0F };
      uint16_t c = FakeBus::crc(bc.data(), 6); bc.push_back(c & 0xFF); bc.push_back(c >> 8);
      check(!port.log.empty() && port.log[0] == bc, "B for all motors starts with one broadcast halt (00 06 60 40 01 0F)");
    }
    runFor(jmc, 20);
    int32_t p1 = port.d[1].pos;
    runFor(jmc, 50);
    check(port.d[1].halted() && port.d[1].pos == p1, "B stopped M1 although M3 was silent");
  }
  port.d[3].present = true;
  runFor(jmc, 3500);                            // M3 back online if the monitor marked it
  expect(jmc, "ACC:0.5,,0.5", "Set ACC: M1=0.50,M3=0.50 rps/s");
  // a drive that answers with a Modbus exception - the manual's own example
  // (section 7.5): "01 86 0B 03 A7" = function 06 rejected, code 11
  {
    std::vector<uint8_t> ex = hexFrame("01 86 0B 03 A7");
    uint16_t c = FakeBus::crc(ex.data(), 3);
    check((c & 0xFF) == 0x03 && (c >> 8) == 0xA7, "CRC of the manual's exception reply 01 86 0B = 03 A7");
    port.d[1].rejectReg = 0x6040;
    port.d[1].rejectCode = 0x0B;
    expect(jmc, "L:M1", "ERR:L M1 not accepted (EXCEPTION)");
    check(bus.lastException() == 11, "exception code 11 recorded", std::to_string(bus.lastException()));
    port.d[1].rejectCode = 3;
  }
  port.d[1].rejectReg = 0x607A;
  expect(jmc, "P:0", PICK("ERR:P M1 not accepted (EXCEPTION) - nothing started", "ERR:P M1 not accepted (EXCEPTION)"));
  port.d[1].rejectReg = 0;
  expect(jmc, "P:0", "OK");
  waitArrive(jmc);
  expect(jmc, "P:?", "M1=0");

  // ---- offline handling ------------------------------------------------------------
  expect(jmc, "P:,500", "ERR:P M2 offline (check power / RS-485, or send I)");
  expect(jmc, "PA:0", "OK");
  waitArrive(jmc);
  expect(jmc, "PA:?", "WARN: M1=0,M2=ERROR,M3=0,M4=0");

  // ---- per-motor speeds on PA -----------------------------------------------------
  expect(jmc, "V:1,,0.5,0.25", "Set V: M1=1.00,M3=0.50,M4=0.25 rps");
  expect(jmc, "PA:4000", "OK");
  check(port.d[1].get32(0x6081) == 10 && port.d[3].get32(0x6081) == 5 && port.d[4].get32(0x6081) == 3,
        "PA writes each motor's own V (10, 5, 3 x0.1 rps)",
        std::to_string(port.d[1].get32(0x6081)) + "," + std::to_string(port.d[3].get32(0x6081)) + "," + std::to_string(port.d[4].get32(0x6081)));
  waitArrive(jmc, 6000);
  expect(jmc, "PA:0", "OK");
  waitArrive(jmc, 6000);

  // ---- soft limits ------------------------------------------------------------------
  expect(jmc, "SL:M1=-100/100", "SL: M1=-100/100");
  expect(jmc, "P:500", "ERR:P M1 target 500 outside soft limit -100..100");
  expect(jmc, "P:50", "OK");
  waitArrive(jmc);
  expect(jmc, "PR:60", "ERR:PR M1 target 110 outside soft limit -100..100");
  expect(jmc, "SLA:?", "SL: M1=-100/100,M2=OFF,M3=OFF,M4=OFF");
  expect(jmc, "SL:M1=5/1", "ERR:SL value must be min/max with min < max, or OFF (M1)");

  // jog stops at the limit in the direction it was started in, even after V changes sign
  expect(jmc, "SL:M1=-100000/2000", "SL: M1=-100000/2000");
  expect(jmc, "V:1", "Set V: M1=1.00 rps");
  expect(jmc, "VS:M1", "VS: velocity mode started for M1 (stop with B)");
  expect(jmc, "V:-1", "Set V: M1=-1.00 rps");
  pc.events.clear();
  runFor(jmc, 1500);
  check(eventHas("EVENT:SOFTLIMIT:M1 POS=") && eventHas("LIMIT=2000"), "jog stopped at +2000 with EVENT:SOFTLIMIT", joined(pc.events));
  check(port.d[1].velReg() == 0, "motor 1 no longer turning after the soft limit");
  expect(jmc, "SLA:OFF", "SLA: all motors = OFF");
  expect(jmc, "B", "All motors stopped with deceleration");

  // ---- jog, then P: braked first, OK deferred, tagged -------------------------------
  expect(jmc, "V:1", "Set V: M1=1.00 rps");
  expect(jmc, "VS:M1", "VS: velocity mode started for M1 (stop with B)");
  runFor(jmc, 200);
  {
    auto r = cmd(jmc, PICK("#6 P:0", "P:0"));
    check(r.empty(), "P during a jog: no immediate reply", joined(r));
    pc.replies.clear();
    runFor(jmc, 1500);
    check(joined(pc.replies) == PICK("#6 OK", "OK"), "deferred reply (tagged '#6 OK' where tags exist) once the move starts", joined(pc.replies));
    check(port.d[1].modeSwitchWhileSpinning == 0, "mode never switched while the shaft turned");
  }
  waitArrive(jmc, 6000);
  expect(jmc, "P:?", "M1=0");

#ifndef SMALL_PROFILE
  // ---- synchronised arrival ------------------------------------------------------------
  expect(jmc, "V:0.6,,0.6", "Set V: M1=0.60,M3=0.60 rps");
  expect(jmc, "ACC:0.6,,0.6", "Set ACC: M1=0.60,M3=0.60 rps/s");   // halves round exactly
  expect(jmc, "DEC:0.6,,0.6", "Set DEC: M1=0.60,M3=0.60 rps/s");
  expect(jmc, "PS:4000,,2000", "OK");
  check(port.d[1].get32(0x6081) == 6 && port.d[3].get32(0x6081) == 3, "PS scales M3 to half speed (6 / 3)",
        std::to_string(port.d[1].get32(0x6081)) + "/" + std::to_string(port.d[3].get32(0x6081)));
  check(port.d[1].reg[0x6083] == 6 && port.d[3].reg[0x6083] == 3, "PS scales M3's acceleration (6 / 3)",
        std::to_string(port.d[1].reg[0x6083]) + "/" + std::to_string(port.d[3].reg[0x6083]));
  waitArrive(jmc, 6000);
  {
    long dt = (long)port.d[1].arrivedUs - (long)port.d[3].arrivedUs;
    check(labs(dt) < 20000, "M1 and M3 arrive within 20 ms of each other", std::to_string(dt) + " us");
  }
  expect(jmc, "P:?,,?", "M1=4000,M3=2000");
  expect(jmc, "P:0,,0", "OK");
  check(port.d[3].reg[0x6083] == 6, "a normal move restores M3's own ACC", std::to_string(port.d[3].reg[0x6083]));
  waitArrive(jmc, 6000);
  expect(jmc, "V:0.5,,0.25", "Set V: M1=0.50,M3=0.25 rps");
  expectHas(jmc, "PS:4000,,2000", "OK (sync within 20% - drive resolution is 0.1 rps and 0.1 rps/s");
  waitArrive(jmc, 9000);
  expect(jmc, "V:0.6,,0.6", "Set V: M1=0.60,M3=0.60 rps");
  expect(jmc, "PS:0,,0", "OK");             // equal distances: equal rounding, no warning
  waitArrive(jmc, 9000);
  expect(jmc, "PS:?", "ERR:PS takes values; read positions with P:?");
  expect(jmc, "PSA:0", "OK");
#else
  expect(jmc, "PS:4000", "ERR:PS not available on this board");
#endif

  // ---- command watchdog ------------------------------------------------------------------
  expect(jmc, "WD:30", "ERR:WD range 0 (off) or 50..65000 ms");
  expect(jmc, "WD:100", PICK("WD: 100 ms - send any command (PING) more often than that", "WD: 100 ms"));
  expect(jmc, "VS:M1", "VS: velocity mode started for M1 (stop with B)");
  pc.events.clear();
  runFor(jmc, 60);
  expect(jmc, "PING", "PONG");              // feeds it
  runFor(jmc, 60);
  check(!eventHas("WATCHDOG"), "watchdog fed by PING does not trip", joined(pc.events));
  runFor(jmc, 150);
  check(eventHas(PICK("EVENT:WATCHDOG:STOP no command for 100 ms - all motors stopped", "EVENT:WATCHDOG:STOP")), "watchdog trips after 100 ms", joined(pc.events));
  check(port.d[1].velReg() == 0, "watchdog stopped the jog");
  expect(jmc, "WD:?", PICK("WD: 100 ms - all motors stop when no command arrives for that long", "WD: 100 ms"));
  expect(jmc, "WD:0", "WD: off");

  // ---- events ------------------------------------------------------------------------------
  pc.events.clear();
  port.d[1].cw = true;  runFor(jmc, 100);
  port.d[1].cw = false; runFor(jmc, 100);
  port.d[1].sw = true;  runFor(jmc, 100);
  port.d[1].sw = false; runFor(jmc, 100);
  check(eventHas("EVENT:LIMIT:M1:CW POS="), "EVENT:LIMIT:M1:CW", joined(pc.events));
  check(eventHas("EVENT:LIMIT:M1:CW_CLEARED"), "EVENT:LIMIT:M1:CW_CLEARED", joined(pc.events));
  check(eventHas("EVENT:ORIGIN:M1:ON POS="), "EVENT:ORIGIN:M1:ON", joined(pc.events));
  check(eventHas("EVENT:ORIGIN:M1:OFF"), "EVENT:ORIGIN:M1:OFF", joined(pc.events));
  pc.events.clear();
  port.d[3].fault = true; port.d[3].errReg = 0x22;  runFor(jmc, 100);
  check(eventHas("EVENT:FAULT:M3 ST=0x") && eventHas(PICK("ERR=0x22 CURRENT OVERPOS", "ERR=0x22")), "EVENT:FAULT with the error register (names: full profile)", joined(pc.events));
  expect(jmc, "R:,,1", "Motor 3 reset complete");
  check(eventHas("EVENT:FAULT_CLEARED:M3"), "R sends EVENT:FAULT_CLEARED:M3", joined(pc.events));
  pc.events.clear();
  port.d[3].fault = true; port.d[3].errReg = 0x02;  runFor(jmc, 100);
  port.d[3].reg[0x6040] = 0;                 // a fault that survives the reset:
  {
    struct Sticky { static void on(FakeDrive& d) { d.fault = true; } };
    pc.events.clear();
    cmd(jmc, "R:,,1");
    Sticky::on(port.d[3]);                     // comes straight back
    runFor(jmc, 100);
    check(eventHas("EVENT:FAULT:M3"), "a fault that persists after R is reported again", joined(pc.events));
  }
  port.d[3].fault = false; port.d[3].errReg = 0; cmd(jmc, "R:,,1"); runFor(jmc, 100);
  pc.events.clear();
  port.d[4].present = false; runFor(jmc, 300);
  check(eventHas("EVENT:OFFLINE:M4"), "EVENT:OFFLINE:M4", joined(pc.events));
  port.d[4].present = true;  runFor(jmc, 3500);
  check(eventHas("EVENT:ONLINE:M4 ST=0x") && eventHas("(re-initialised)"), "EVENT:ONLINE:M4 (re-initialised)", joined(pc.events));

  // ---- zero, position, homing ------------------------------------------------------------
  expect(jmc, "P:777", "OK");
  waitArrive(jmc);
  expect(jmc, "Z:M1", "Motor 1 set to zero position");
  expect(jmc, "P:?", "M1=0");
  expect(jmc, "ZO:M1=5000", "ZO: M1=5000 (current position redefined)");
  expect(jmc, "P:?", "M1=5000");
  expect(jmc, "ZO:?", "ERR:ZO takes values, not ? (read positions with P:?)");
  expect(jmc, "HO:M1=250", "HO: M1=250 steps after sensor (used at next homing)");
  expect(jmc, "HO:?", "HO: M1=250");
  expect(jmc, "H65537:M1", "ERR:homing method H1..H12");
  expect(jmc, "H13:M1", "ERR:homing method H1..H12");
  pc.events.clear();
  expect(jmc, "H3:M1", "Homing method 3 started for M1");
  check(port.d[1].get32(0x607C) == 250, "homing re-applies HO (ZO had overwritten 0x607C)", std::to_string(port.d[1].get32(0x607C)));
  runFor(jmc, 500);
  check(eventHas("EVENT:HOMED:M1 POS=250"), "EVENT:HOMED:M1 POS=250", joined(pc.events));
  expect(jmc, "LS:M1", "LIMITS: M1:CW=0 CCW=0 SW=0 HOMED=1 MODE=HOM");
  expect(jmc, "P:0", "OK");
  waitArrive(jmc);

  // ---- stops, speeds, ramps ------------------------------------------------------------------
  expect(jmc, "B2:1,,1", "Fast brake applied to motor 1,3");
  expect(jmc, "B", "All motors stopped with deceleration");
  expect(jmc, "B4", "ERR:B type is B, B1, B2 or B3");
  expect(jmc, "HACCA:0.2", PICK("HACCA: all motors = 0.20 rps/s", "ERR:HACC not available on this board"));
  expect(jmc, "HV:30", "ERR:HV range 0.01..20.0 rps (M1)");
  expect(jmc, "ACC:-1", "ERR:ACC range 0.10..3276.7 rps/s (M1)");
  expect(jmc, "ACC:3276.8", "ERR:ACC range 0.10..3276.7 rps/s (M1)");   // register 32767 max (INT16 in the manual's list)
  expect(jmc, "ACC:3276.7", "Set ACC: M1=3276.70 rps/s");
  check(port.d[1].reg[0x6083] == 32767, "ACC:3276.7 -> 0x6083 = 32767", std::to_string(port.d[1].reg[0x6083]));
  expect(jmc, "ACC:0.2", "Set ACC: M1=0.20 rps/s");
  expect(jmc, "HVZ:?", PICK("HVZ: M1=0.30 rps", "ERR:HVZ not available on this board"));
  expect(jmc, "ACC:0.2,,0.4", "Set ACC: M1=0.20,M3=0.40 rps/s");
  check(port.d[1].reg[0x6083] == 2 && port.d[3].reg[0x6083] == 4, "ACC written to 0x6083 (2 / 4)");
  expect(jmc, "VA:0.5", "VA: all motors = 0.50 rps");
  expect(jmc, "S:M1:COMPACT", "M1:RUN|STP|TR|POS=0|VEL=0.0");

  // ---- serial link (Uno USB / RS-232): line framing ------------------------------------------
  {
    struct FakeSerial : public Stream {
      std::string in, out;
      size_t ip = 0;
      size_t write(uint8_t b) override { out += (char)b; return 1; }
      int available() override { return (int)(in.size() - ip); }
      int read() override { return ip < in.size() ? (unsigned char)in[ip++] : -1; }
      int peek() override { return ip < in.size() ? (unsigned char)in[ip] : -1; }
    } ser;
    JMCLinkSerial sl(ser);
    JMCController sc(bus, sl);
    sc.setMotorCount(4);
    sc.begin();
    check(ser.out == "Motor ready\r\n", "serial: boot message ends with CR LF", ser.out);
    auto feed = [&](const std::string& s, int runs) { ser.out.clear(); ser.in += s; for (int k = 0; k < runs; k++) sc.run(); return ser.out; };
    check(feed("PING\r\n", 2) == "PONG\r\n", "serial: PING CR LF -> PONG CR LF");
    check(feed("PING\n", 1) == "PONG\r\n", "serial: LF alone ends a line");
    check(feed("PI", 1).empty() && feed("NG\r", 1) == "PONG\r\n", "serial: a line split across reads");
    check(feed("\r\n\r\n", 2).empty(), "serial: empty lines are ignored");
    std::string longLine(JMC_CMD_BUFFER + 20, 'x');
    check(feed(longLine + "\n", 1) == "ERR:command too long\r\n", "serial: over-long line refused", ser.out);
    check(feed(PICK("#3 P:?\n", "P:?\n"), 1) == PICK("#3 M1=0\r\n", "M1=0\r\n"), "serial: next line after an over-long one works", ser.out);
  }

#ifndef SMALL_PROFILE
  // ---- SAVE / LOAD and the sketch fingerprint ----------------------------------------------
  expect(jmc, "V:0.7", "Set V: M1=0.70 rps");
  expect(jmc, "SAVE", "SAVE: 4 motors stored (V, ACC, DEC, HV, HVZ, HACC, HO, SL, WD, MC)");
  expect(jmc, "SAVE:?", "SAVE: settings stored and applied at boot (SAVE:CLEAR to forget)");
  expect(jmc, "V:0.9", "Set V: M1=0.90 rps");
  expect(jmc, "LOAD", "LOAD: 4 motors loaded and applied");
  expect(jmc, "V:?", "V: M1=0.70 rps");
  {
    // the same sketch values reboot -> the block applies
    JMCController same(bus, pc);
    same.setMotorCount(4);
    same.begin();
    expect(same, "V:?", "V: M1=0.70 rps");
    // a re-flash with other values -> the sketch wins
    JMCController reflashed(bus, pc);
    reflashed.setMotorCount(4);
    reflashed.setMotionDefaults(0.5, 0.5, 0.4, 0.3);
    reflashed.begin();
    expect(reflashed, "V:?", "V: M1=0.40 rps");
    expectHas(reflashed, "SAVE:?", "IGNORED");
  }
  EEPROM.mem[20] ^= 0x55;                    // corrupt the stored block
  expect(jmc, "LOAD", "ERR:LOAD nothing stored (or checksum failed)");
  expect(jmc, "SAVE:CLEAR", "SAVE: cleared - sketch values apply at next boot");
  expect(jmc, "SAVE:?", "SAVE: nothing stored - sketch values in use");
#else
  expect(jmc, "SAVE", "ERR:SAVE not available on this board");
  expect(jmc, "LOAD", "ERR:LOAD not available on this board");
  expect(jmc, "SD:M1", "ERR:SD not available on this board");
  expect(jmc, "BSA", "ERR:BS not available on this board");
  expect(jmc, "X", "ERR:X not available on this board");
  expect(jmc, "S:M1:FLAGS", "ERR:S formats on this board: RAW|COMPACT");
#endif

  // ---- motor count and multi-packet replies -----------------------------------------------
#ifdef SMALL_PROFILE
  expect(jmc, "MC:6", "ERR:MC:n not available on this board");         // set in the sketch on the Uno
  expect(jmc, "MC:?", "MC: 4 motors (slave IDs 1..4)");
#else
  expect(jmc, "MC:33", "ERR:MC range 1..32");
  expect(jmc, "MC:32", "MC: 32 motors, 3 online (SAVE to keep across reboot)");
#endif
  {
    auto r = cmd(jmc, "LSA");
    bool ok = r.size() >= 2;
    for (size_t i = 0; i < r.size() && ok; i++) {
      ok = r[i].rfind("LIMITS: ", 0) == 0;
      const char* end = (i + 1 == r.size()) ? ":END" : ":MORE";
      ok = ok && r[i].size() > 5 && r[i].compare(r[i].size() - strlen(end), strlen(end), end) == 0;
    }
    check(ok, "LSA splits into packets, each 'LIMITS: ...', :MORE ... :END", joined(r));
    for (auto& x : r) check(x.size() <= JMC_REPLY_BUFFER, "packet fits the reply buffer", std::to_string(x.size()));
  }
  expect(jmc, "MC:4", PICK("MC: 4 motors, 3 online (SAVE to keep across reboot)", "ERR:MC:n not available on this board"));
  expect(jmc, "MC:?", "MC: 4 motors (slave IDs 1..4)");

  // ---- 49.7 days of uptime: millis() rolls over --------------------------------------------
  {
    g_us = (4294967296ULL - 300) * 1000;      // 300 ms before millis() wraps to 0
    for (int i = 1; i <= 32; i++) port.d[i].lastUs = g_us;
    runFor(jmc, 20);
    expect(jmc, "WD:200", PICK("WD: 200 ms - send any command (PING) more often than that", "WD: 200 ms"));
    pc.events.clear();
    runFor(jmc, 150);                          // still before the wrap
    check(!eventHas("WATCHDOG"), "rollover: no early watchdog trip", joined(pc.events));
    runFor(jmc, 400);                          // across the wrap
    int trips = 0;
    for (auto& e : pc.events) if (e.find("EVENT:WATCHDOG") != std::string::npos) trips++;
    check(trips == 1, "rollover: watchdog trips exactly once across the wrap", std::to_string(trips));
    expect(jmc, "WD:0", "WD: off");
    pc.events.clear();
    port.d[1].cw = true;  runFor(jmc, 100);
    port.d[1].cw = false; runFor(jmc, 100);
    check(eventHas("EVENT:LIMIT:M1:CW POS=") && eventHas("CW_CLEARED"), "rollover: monitor keeps reporting events", joined(pc.events));
    expect(jmc, "P:500", "OK");
    waitArrive(jmc);
    expect(jmc, "P:?", "M1=500");
  }

  check(port.crcErrors == 0, "no request frame ever had a bad CRC", std::to_string(port.crcErrors));
  for (int i = 1; i <= 4; i++)
    check(port.d[i].modeSwitchWhileSpinning == 0, "drive " + std::to_string(i) + ": no mode switch while spinning");

  printf("%s: %d passed, %d failed\n",
#ifdef SMALL_PROFILE
         "Uno profile",
#else
         "full profile",
#endif
         pass, fail);
  return fail ? 1 : 0;
}
