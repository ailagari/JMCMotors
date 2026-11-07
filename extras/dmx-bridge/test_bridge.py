#!/usr/bin/env python3
"""
Tests for jmc_dmx_bridge.py - no hardware needed.

    python test_bridge.py

1. Packet parsing and value mapping (Art-Net, sACN, 8 / 16 bit, min / max, clamping).
2. A live run on this PC: a simulated JMCMotors controller on 127.0.0.1 answers
   like the firmware does, and real Art-Net and sACN packets are sent to the
   bridge. The test checks the position commands that reach the controller.
"""

import re
import socket
import struct
import threading
import time

import jmc_dmx_bridge as jb

PASS = FAIL = 0


def check(cond, what, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
    else:
        FAIL += 1
        print("FAIL: %s %s" % (what, detail))


# --------------------------------------------------------------------------- packet builders
def artnet_packet(universe, data, seq=0):
    data = bytes(data)
    return (b"Art-Net\x00" + struct.pack("<H", 0x5000) + bytes([0, 14, seq & 0xFF, 0,
            universe & 0xFF, (universe >> 8) & 0x7F]) + struct.pack(">H", len(data)) + data)


def sacn_packet(universe, data, seq=0, terminated=False, start_code=0):
    data = bytes(data)
    n = len(data)
    dmp = struct.pack(">HBBHHH", 0x7000 | (10 + n + 1), 0x02, 0xA1, 0, 1, n + 1) + bytes([start_code]) + data
    name = b"test".ljust(64, b"\x00")
    framing = (struct.pack(">HI", 0x7000 | (77 + len(dmp)), 0x00000002) + name +
               bytes([100]) + struct.pack(">H", 0) + bytes([seq & 0xFF, 0x40 if terminated else 0]) +
               struct.pack(">H", universe))
    root = (struct.pack(">HH", 0x0010, 0) + jb.SACN_ID +
            struct.pack(">HI", 0x7000 | (22 + len(framing) + len(dmp)), 0x00000004) + bytes(16))
    return root + framing + dmp


def dmx_with(values, size=512):
    """values: {channel (1-based): byte}"""
    d = bytearray(size)
    for ch, b in values.items():
        d[ch - 1] = b
    return d


def put16(values, ch, v):
    values[ch] = (v >> 8) & 0xFF
    values[ch + 1] = v & 0xFF
    return values


# --------------------------------------------------------------------------- 1. parsing and mapping
def unit_tests():
    d = dmx_with(put16({}, 1, 0x1234))
    u, data = jb.parse_artnet(artnet_packet(0, d))
    check(u == 0 and len(data) == 512 and data[0] == 0x12 and data[1] == 0x34, "Art-Net ArtDmx parsed")
    u, _ = jb.parse_artnet(artnet_packet(0x0123, d))
    check(u == 0x0123, "Art-Net port-address = Net<<8 | SubUni", hex(u))
    check(jb.parse_artnet(b"Art-Net\x00" + struct.pack("<H", 0x2000) + bytes(20)) is None, "ArtPoll ignored")
    check(jb.parse_artnet(b"garbage") is None, "short / foreign packet ignored")
    short = artnet_packet(1, bytes(24))
    check(len(jb.parse_artnet(short)[1]) == 24, "Art-Net short universe (24 channels)")

    u, data = jb.parse_sacn(sacn_packet(7, d))
    check(u == 7 and data[0] == 0x12 and data[1] == 0x34 and len(data) == 512, "sACN data packet parsed")
    check(jb.parse_sacn(sacn_packet(7, d, terminated=True)) is None, "sACN stream-terminated packet ignored")
    check(jb.parse_sacn(sacn_packet(7, d, start_code=0xDD)) is None, "sACN non-zero start code ignored")
    check(jb.sacn_group(1) == "239.255.0.1" and jb.sacn_group(258) == "239.255.1.2", "sACN multicast group")

    check(jb.value16(d, 1) == 0x1234, "16-bit: coarse, fine")
    check(jb.read_value(d, 1, 8) == 0x12, "8-bit: one channel")
    check(jb.value16(d, 512) == 0, "16-bit on the last channel: fine reads 0, no crash")
    check(jb.read_value(b"", 5, 16) == 0, "empty universe reads 0")

    m = jb.default_motor(1)
    m.update(dmx_min=0, dmx_max=65535, steps_min=0, steps_max=4000)
    check(jb.map_steps(0, m) == 0 and jb.map_steps(65535, m) == 4000, "16-bit full range to steps")
    check(jb.map_steps(32768, m) == 2000, "middle", str(jb.map_steps(32768, m)))
    m.update(steps_min=-5000, steps_max=5000)
    check(jb.map_steps(0, m) == -5000 and jb.map_steps(65535, m) == 5000, "negative positions")
    m.update(dmx_min=65535, dmx_max=0, steps_min=0, steps_max=4000)
    check(jb.map_steps(0, m) == 4000 and jb.map_steps(65535, m) == 0, "DMX min > max reverses the direction")
    m.update(dmx_min=10000, dmx_max=20000, steps_min=0, steps_max=1000)
    check(jb.map_steps(0, m) == 0 and jb.map_steps(65535, m) == 1000, "values outside DMX min/max are clamped")
    m8 = jb.default_motor(1)
    m8.update(bits=8, dmx_min=0, dmx_max=255, steps_min=0, steps_max=2550)
    check(jb.map_steps(128, m8) == 1280, "8-bit to steps")

    cfg = jb.default_config()
    check(jb.check_config(cfg) == [], "default settings are valid", str(jb.check_config(cfg)))
    cfg["motors"][1]["motor"] = 1
    check(any("used twice" in p for p in jb.check_config(cfg)), "a motor used twice is refused")
    cfg = jb.default_config()
    cfg["motors"][0]["channel"] = 512
    check(any("511" in p for p in jb.check_config(cfg)), "16-bit on channel 512 refused (no fine channel)")
    cfg["motors"][0].update(bits=8, dmx_max=65535)
    check(any("0..255" in p for p in jb.check_config(cfg)), "8-bit DMX max above 255 refused")
    cfg["motors"][0].update(dmx_max=255)
    check(jb.check_config(cfg) == [], "8-bit on channel 512 is fine", str(jb.check_config(cfg)))
    cfg["input"].update(artnet=False, sacn=False)
    check(any("Art-Net" in p for p in jb.check_config(cfg)), "no input protocol refused")

    loaded = jb.merge(jb.default_config(), {"controller": {"ip": "10.0.0.5"}})
    check(loaded["controller"]["ip"] == "10.0.0.5" and loaded["controller"]["port"] == 8001,
          "settings file merges into defaults")

    check(jb.parse_version("JMCMotors 0.6.0 (max 32 motors)") == (0, 6, 0), "firmware version parsed")
    check(jb.parse_version("JMCMotors 0.5.1") < jb.MIN_FIRMWARE, "older firmware detected")
    check(jb.parse_version("hello") is None, "a device that is not a JMCMotors controller")
    lib = open(jb.os.path.join(jb.os.path.dirname(jb.os.path.abspath(jb.__file__)), "..", "..",
                               "library.properties"), encoding="utf-8").read()
    check("version=%s\n" % jb.VERSION in lib, "bridge version = library version (released together)", jb.VERSION)


# --------------------------------------------------------------------------- 2. simulated controller
class FakeController:
    """Answers like the JMCMotors firmware (the commands the bridge uses)."""
    P_LINE = re.compile(r"^P:M\d+=-?\d+(,M\d+=-?\d+)*$")

    def __init__(self, motors=3, reply_port=None, bus_ms_per_motor=2.0):
        self.motors = motors
        self.bus = bus_ms_per_motor / 1000.0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.port = self.sock.getsockname()[1]
        self.sock.settimeout(0.1)
        self.reply_port = reply_port
        self.log = []                  # (time, line)
        self.pos = {}
        self.wd = 0
        self.bad = []
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _reply(self, text):
        self.sock.sendto(text.encode(), ("127.0.0.1", self.reply_port))

    def _run(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            line = data.decode().strip()
            self.log.append((time.time(), line))
            if line == "VER":
                self._reply("JMCMotors 0.6.0 (max 32 motors)")
            elif line == "MC:?":
                self._reply("MC: %d motors (slave IDs 1..%d)" % (self.motors, self.motors))
            elif line.startswith("WD:"):
                self.wd = int(line[3:])
                self._reply("WD: off" if self.wd == 0 else "WD: %d ms" % self.wd)
            elif line == "PING":
                self._reply("PONG")
            elif re.match(r"^(V|ACC|DEC):M\d+=[\d.]+$", line):
                self._reply("Set %s: %s" % (line.split(":")[0], line.split(":")[1]))
            elif line == "B":
                self._reply("Deceleration brake applied to all motors")
            elif self.P_LINE.match(line):
                pairs = [p.split("=") for p in line[2:].split(",")]
                if any(int(k[1:]) > self.motors for k, _ in pairs):
                    self._reply("ERR:P motor must be M1..M%d (e.g. P:M3=1000)" % self.motors)
                    continue
                time.sleep(self.bus * len(pairs))              # bus time, like the real controller
                for k, val in pairs:
                    self.pos[int(k[1:])] = int(val)
                self._reply("OK")
            else:
                self.bad.append(line)
                self._reply("ERR:unknown command")

    def p_lines(self):
        return [l for _, l in self.log if l.startswith("P:")]

    def close(self):
        self.running = False
        self.sock.close()


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def wait_for(cond, timeout=2.0):
    t = time.time() + timeout
    while time.time() < t:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


def live_tests():
    reply_port, art_port, sacn_port = free_port(), free_port(), free_port()
    ctl = FakeController(motors=3, reply_port=reply_port)
    cfg = jb.default_config()
    cfg["controller"].update(link="udp", ip="127.0.0.1", port=ctl.port, reply_port=reply_port)
    cfg["input"].update(artnet=True, sacn=True, listen_ip="0.0.0.0", artnet_port=art_port, sacn_port=sacn_port)
    cfg["stream"].update(max_rate_hz=40, deadband_steps=2, watchdog_ms=600, smooth_ms=0, source_timeout_s=0.6)
    m1 = dict(jb.default_motor(1), universe=0, channel=1, steps_min=0, steps_max=4000)          # Art-Net 16-bit
    m2 = dict(jb.default_motor(2), universe=0, channel=3, steps_min=-1000, steps_max=1000)      # Art-Net 16-bit
    m3 = dict(jb.default_motor(3), universe=5, channel=10, bits=8, dmx_min=0, dmx_max=255,      # sACN 8-bit
              steps_min=0, steps_max=255)
    cfg["motors"] = [m1, m2, m3]
    check(jb.check_config(cfg) == [], "live-test settings valid", str(jb.check_config(cfg)))
    logs = []
    br = jb.Bridge(cfg, log=logs.append)
    br.start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def art(values):
        tx.sendto(artnet_packet(0, dmx_with(values)), ("127.0.0.1", art_port))

    def sacn(values, universe=5):
        tx.sendto(sacn_packet(universe, dmx_with(values)), ("127.0.0.1", sacn_port))

    try:
        # start-up: version, motor count, watchdog, speed and ramps
        check(wait_for(lambda: br.controller_count == 3), "bridge read the motor count (MC:?)", str(br.controller_count))
        check(wait_for(lambda: ctl.wd == 600), "watchdog set on the controller", str(ctl.wd))
        setup = [l for _, l in ctl.log]
        check("V:M1=2.0" in setup and "ACC:M2=5.0" in setup and "DEC:M3=5.0" in setup, "speed / ramps sent per motor", str(setup))

        # Art-Net only: motors 1 and 2 move, motor 3 (sACN) has no data -> not sent
        art(put16(put16({}, 1, 32768), 3, 65535))
        check(wait_for(lambda: ctl.pos.get(1) == 2000 and ctl.pos.get(2) == 1000), "Art-Net 16-bit -> steps", str(ctl.pos))
        check(3 not in ctl.pos, "motor without DMX data is not moved", str(ctl.pos))

        # sACN for motor 3 too: now all three are mapped -> every update carries all motors (broadcast start)
        sacn({10: 128})
        check(wait_for(lambda: ctl.pos.get(3) == 128), "sACN 8-bit -> steps", str(ctl.pos))
        art(put16(put16({}, 1, 0), 3, 65535))
        sacn({10: 128})
        check(wait_for(lambda: ctl.pos.get(1) == 0), "new Art-Net value followed", str(ctl.pos))
        last = ctl.p_lines()[-1]
        check(last == "P:M1=0,M2=1000,M3=128", "all mapped motors sent together (broadcast start path)", last)

        # deadband: a 1-step change is not sent
        n = len(ctl.p_lines())
        sacn({10: 128})
        art(put16(put16({}, 1, 16), 3, 65535))            # 16/65535*4000 = 0.98 -> 1 step
        time.sleep(0.2)
        check(len(ctl.p_lines()) == n, "change below the deadband not sent", str(ctl.p_lines()[n:]))

        # rate limit: 200 packets in 1 s -> at most ~40 commands
        n = len(ctl.p_lines())
        t0 = time.time()
        k = 0
        while time.time() - t0 < 1.0:
            art(put16(put16({}, 1, (k * 300) % 65535), 3, 65535))
            sacn({10: 128})
            k += 1
            time.sleep(0.005)
        time.sleep(0.15)
        sent = len(ctl.p_lines()) - n
        check(10 <= sent <= 44, "rate limited to max_rate_hz while flooded (%d packets)" % k, "%d commands" % sent)
        final = 32768
        art(put16(put16({}, 1, final), 3, 65535))
        sacn({10: 128})
        check(wait_for(lambda: ctl.pos.get(1) == 2000), "after a flood the newest value wins", str(ctl.pos))

        # one command in flight: next send only after the reply
        times = [t for t, l in ctl.log if l.startswith("P:")]
        gaps = [b - a for a, b in zip(times, times[1:])]
        check(min(gaps) >= 0.02, "commands spaced (waits for each reply, rate limit)", "min gap %.3f" % min(gaps))

        # source lost: no more P commands, but PING keeps the watchdog fed
        time.sleep(0.8)                                  # source_timeout 0.6 s
        n = len(ctl.p_lines())
        pings0 = sum(1 for _, l in ctl.log if l == "PING")
        time.sleep(0.9)
        check(len(ctl.p_lines()) == n, "lost DMX source: motors hold, nothing sent")
        pings = sum(1 for _, l in ctl.log if l == "PING") - pings0
        check(pings >= 3, "PING keepalive while idle (watchdog 600 ms)", str(pings))
        stamps = [t for t, l in ctl.log if l in ("PING",) or l.startswith("P:")]
        check(max(b - a for a, b in zip(stamps[-4:], stamps[-3:])) < 0.6, "never quiet longer than the watchdog")

        # STOP: B goes out at once, and nothing moves the motors until Resume
        art(put16(put16({}, 1, 8000), 3, 65535))
        sacn({10: 50})
        br.emergency_stop()
        check(wait_for(lambda: any(l == "B" for _, l in ctl.log)), "STOP MOTORS sends B")
        b_at = max(t for t, l in ctl.log if l == "B")
        for k in range(10):                               # DMX keeps changing after STOP
            art(put16(put16({}, 1, 20000 + 3000 * k), 3, 65535))
            sacn({10: 60 + k})
            time.sleep(0.03)
        time.sleep(0.2)
        after = [l for t, l in ctl.log if t > b_at and l.startswith("P:")]
        check(not after, "after STOP no position command is sent, even when DMX changes", str(after))
        check(br.snapshot()["paused"], "bridge reports paused")
        br.resume()
        check(wait_for(lambda: ctl.pos.get(3) == 69), "Resume follows DMX again", str(ctl.pos))

        # wrong motor number on the controller: error reported, bridge keeps running
        cfg["motors"][2]["motor"] = 9
        sacn({10: 200})
        art(put16(put16({}, 1, 65535), 3, 65535))
        check(wait_for(lambda: br.stats["errors"] > 0), "ERR reply counted", str(br.stats))
        check(any("ERR:P motor must be" in l for l in logs), "ERR reply shown in the log")
        cfg["motors"][2]["motor"] = 3
        sacn({10: 200})
        art(put16(put16({}, 1, 65535), 3, 65535))
        check(wait_for(lambda: ctl.pos.get(1) == 4000 and ctl.pos.get(3) == 200), "recovers after the error", str(ctl.pos))
        check(not ctl.bad, "every command was valid firmware syntax", str(ctl.bad))
    finally:
        br.stop()
        time.sleep(0.1)
        check(ctl.wd == 0, "leaving switches the controller watchdog off (WD:0)", str(ctl.wd))
        ctl.close()
        tx.close()


def smoothing_test():
    reply_port, art_port = free_port(), free_port()
    ctl = FakeController(motors=1, reply_port=reply_port)
    cfg = jb.default_config()
    cfg["controller"].update(ip="127.0.0.1", port=ctl.port, reply_port=reply_port)
    cfg["input"].update(artnet=True, sacn=False, artnet_port=art_port)
    cfg["stream"].update(max_rate_hz=50, smooth_ms=200, watchdog_ms=0, deadband_steps=1, send_settings=False)
    cfg["motors"] = [dict(jb.default_motor(1), bits=8, channel=1, dmx_min=0, dmx_max=255, steps_min=0, steps_max=2550)]
    br = jb.Bridge(cfg, log=lambda m: None)
    br.start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        tx.sendto(artnet_packet(0, dmx_with({1: 0})), ("127.0.0.1", art_port))
        check(wait_for(lambda: ctl.pos.get(1) == 0), "smoothing: first value sent directly", str(ctl.pos))
        time.sleep(0.1)
        n = len(ctl.p_lines())
        tx.sendto(artnet_packet(0, dmx_with({1: 255})), ("127.0.0.1", art_port))   # one jump 0 -> 2550
        for _ in range(50):                                                         # DMX keeps arriving (2.5 s)
            time.sleep(0.05)
            tx.sendto(artnet_packet(0, dmx_with({1: 255})), ("127.0.0.1", art_port))
        steps = [int(l.split("=")[1]) for l in ctl.p_lines()[n:]]
        check(len(steps) >= 8, "smoothing: a jump becomes a glide of many small targets", str(steps))
        check(steps == sorted(steps) and steps[-1] == 2550, "smoothing: glide is monotonic and lands on target", str(steps))
        check(steps and steps[0] < 2550 * 0.4, "smoothing: first step is a fraction of the jump", str(steps[:3]))
    finally:
        br.stop()
        ctl.close()
        tx.close()


if __name__ == "__main__":
    unit_tests()
    live_tests()
    smoothing_test()
    print("%d passed, %d failed" % (PASS, FAIL))
    raise SystemExit(1 if FAIL else 0)
