#!/usr/bin/env python3
"""
JMC DMX Bridge - Art-Net / sACN (E1.31) to JMC motors, in real time.

Runs on the same PC as Resolume, MadMapper, a lighting console or anything
else that sends DMX over the network. Each motor follows one 16-bit DMX
value (coarse + fine channel): the value is mapped to a step position and
sent to the JMCMotors controller (CONTROLLINO / ESP32 / Mega over UDP, or an
Uno over a USB COM port) as position commands.

Ready-made apps (Windows .exe, macOS .app, Linux) are on the GitHub Releases
page, built from this file - no Python needed. From source:

    python jmc_dmx_bridge.py                       settings window
    python jmc_dmx_bridge.py --headless            no window: stream with the saved settings
    python jmc_dmx_bridge.py --config other.json   another settings file

Needs Python 3.8+ (standard library only). The USB / COM-port connection
also needs pyserial:  python -m pip install pyserial

Part of JMCMotors (MIT) - https://github.com/ailagari/JMCMotors
Guide: extras/dmx-bridge/README.md
"""

import argparse
import collections
import json
import math
import os
import queue
import select
import shutil
import socket
import socketserver
import struct
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
from urllib.request import urlopen

VERSION = "0.6.0"            # released together with the JMCMotors library of the same version
MIN_FIRMWARE = (0, 6, 0)     # oldest controller firmware the bridge works with (MC:?, WD, PING)
ARTNET_PORT = 6454
SACN_PORT = 5568
FROZEN = getattr(sys, "frozen", False)      # running as the packaged app (.exe / .app)


def settings_dir():
    """Where the settings file lives.
    From source: next to this file. Packaged on Windows / Linux: next to the
    app if that folder is writable (a portable folder), otherwise the user's
    home. Packaged on macOS: ~/Library/Application Support/JMC DMX Bridge."""
    if not FROZEN:
        return os.path.dirname(os.path.abspath(__file__))
    if sys.platform == "darwin":
        d = os.path.join(os.path.expanduser("~"), "Library", "Application Support", "JMC DMX Bridge")
    else:
        d = os.path.dirname(os.path.abspath(sys.executable))
        if not os.access(d, os.W_OK):
            d = os.path.join(os.path.expanduser("~"), "JMC DMX Bridge")
    os.makedirs(d, exist_ok=True)
    return d


DEFAULT_CONFIG = os.path.join(settings_dir(), "jmc_dmx_bridge.json")


def parse_version(text):
    """'JMCMotors 0.6.0 (max 32 motors)' -> (0, 6, 0), or None."""
    try:
        if not text.startswith("JMCMotors "):
            return None
        return tuple(int(x) for x in text.split()[1].split(".")[:3])
    except (IndexError, ValueError):
        return None


# --------------------------------------------------------------------------- settings
def default_motor(n):
    return {
        "enabled": True,
        "motor": n,             # controller motor number (drive slave ID)
        "universe": 0,          # as it arrives: Art-Net port-address (0 = first) or sACN universe (1 = first)
        "bits": 16,             # 16 = coarse + fine channel (0..65535); 8 = one channel (0..255)
        "channel": 2 * n - 1,   # first channel 1..512; in 16-bit mode the fine channel is the next one
        "dmx_min": 0,           # DMX value that means steps_min (0..65535 in 16-bit, 0..255 in 8-bit)
        "dmx_max": 65535,       # DMX value that means steps_max
        "steps_min": 0,
        "steps_max": 4000,
        "speed": 2.0,           # rps - how fast the motor may follow
        "accel": 5.0,           # rps/s - ramps (acceleration and deceleration)
    }


def default_config():
    return {
        "controller": {
            "link": "udp",            # "udp" (CONTROLLINO / ESP32 / Mega) or "serial" (Uno over USB)
            "ip": "192.168.1.101",
            "port": 8001,
            "reply_port": 8000,       # the PC_PORT set in the sketch: replies and events come here
            "com_port": "COM3",
            "baud": 57600,
        },
        "input": {
            "artnet": True,
            "sacn": True,
            "listen_ip": "0.0.0.0",   # 0.0.0.0 = every network card, also 127.0.0.1
            "artnet_port": ARTNET_PORT,
            "sacn_port": SACN_PORT,
        },
        "stream": {
            "max_rate_hz": 40,        # most position updates per second
            "deadband_steps": 2,      # smaller changes are not sent (DMX noise)
            "smooth_ms": 0,           # glide between DMX values (8-bit sources: 100..200); 0 = off
            "watchdog_ms": 1000,      # controller stops the motors if the bridge goes quiet; 0 = off
            "send_settings": True,    # write speed / ramps to the controller when starting
            "source_timeout_s": 2.0,  # a universe that stops arriving: motors hold position
        },
        "motors": [default_motor(1), default_motor(2)],
    }


def merge(base, override):
    """Recursively merge loaded settings into defaults (keeps new keys working)."""
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path):
    cfg = default_config()
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            cfg = merge(cfg, json.load(f))
        cfg["motors"] = [merge(default_motor(i + 1), m) for i, m in enumerate(cfg.get("motors", []))]
    return cfg


def save_config(path, cfg):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    os.replace(tmp, path)


# --------------------------------------------------------------------------- DMX parsing
def parse_artnet(data):
    """ArtDmx -> (universe, bytes) or None. Universe = 15-bit port-address."""
    if len(data) < 18 or data[:8] != b"Art-Net\x00":
        return None
    opcode = data[8] | (data[9] << 8)
    if opcode != 0x5000:                       # only ArtDmx carries channel data
        return None
    universe = data[14] | ((data[15] & 0x7F) << 8)
    length = (data[16] << 8) | data[17]
    return universe, bytes(data[18:18 + min(length, 512)])


SACN_ID = b"ASC-E1.17\x00\x00\x00"


def parse_sacn(data):
    """E1.31 data packet -> (universe, bytes) or None."""
    if len(data) < 126 or data[4:16] != SACN_ID:
        return None
    if struct.unpack(">I", data[18:22])[0] != 0x00000004:     # root vector: E1.31 data
        return None
    if struct.unpack(">I", data[40:44])[0] != 0x00000002:     # framing vector: DMP
        return None
    if data[112] & 0x40:                                       # stream terminated
        return None
    universe = struct.unpack(">H", data[113:115])[0]
    count = struct.unpack(">H", data[123:125])[0]              # property values incl. start code
    if data[125] != 0x00:                                      # only start code 0 (dimmer data)
        return None
    return universe, bytes(data[126:126 + max(0, min(count - 1, 512))])


def sacn_group(universe):
    return "239.255.%d.%d" % ((universe >> 8) & 0xFF, universe & 0xFF)


def value16(dmx, channel):
    """16-bit value from coarse channel `channel` (1-based) and the next one."""
    hi = dmx[channel - 1] if 1 <= channel <= len(dmx) else 0
    lo = dmx[channel] if 0 <= channel < len(dmx) else 0
    return (hi << 8) | lo


def read_value(dmx, channel, bits):
    """The motor's DMX value: 16-bit (coarse + fine) or 8-bit (one channel)."""
    if int(bits) == 8:
        return dmx[channel - 1] if 1 <= channel <= len(dmx) else 0
    return value16(dmx, channel)


def map_steps(v, m, exact=False):
    """DMX value -> steps, linear between the min / max pairs, clamped.
    exact=True returns the unrounded value (used by smoothing)."""
    d0, d1 = int(m["dmx_min"]), int(m["dmx_max"])
    s0, s1 = int(m["steps_min"]), int(m["steps_max"])
    if d1 == d0:
        return s0
    t = (v - d0) / float(d1 - d0)
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    s = s0 + t * (s1 - s0)
    return s if exact else int(round(s))


# --------------------------------------------------------------------------- controller links
class UdpLink:
    def __init__(self, ip, port, reply_port, on_line):
        self.addr = (ip, int(port))
        self.on_line = on_line
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", int(reply_port)))
        self.sock.settimeout(0.2)
        self.running = True
        threading.Thread(target=self._rx, daemon=True).start()

    def send(self, line):
        self.sock.sendto(line.encode("ascii"), self.addr)

    def _rx(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            self.on_line(data.decode("ascii", "replace").strip())

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


class SerialLink:
    def __init__(self, com_port, baud, on_line):
        try:
            import serial  # pyserial
        except ImportError:
            raise RuntimeError("The COM-port link needs pyserial: python -m pip install pyserial")
        self.port = serial.Serial(com_port, int(baud), timeout=0.2)
        self.on_line = on_line
        self.running = True
        time.sleep(2.0)                     # an Uno resets when the port opens
        threading.Thread(target=self._rx, daemon=True).start()

    def send(self, line):
        self.port.write((line + "\n").encode("ascii"))

    def _rx(self):
        buf = b""
        while self.running:
            try:
                chunk = self.port.read(256)
            except Exception:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("ascii", "replace").strip()
                if text:
                    self.on_line(text)

    def close(self):
        self.running = False
        try:
            self.port.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- the bridge
class Bridge:
    """DMX in -> steps -> controller. Thread-safe; the GUI reads snapshot()."""

    def __init__(self, cfg, log=print):
        self.cfg = cfg
        self.log = log
        self.lock = threading.Lock()
        self.universes = {}          # universe -> {"data", "proto", "src", "t", "count"}
        self.replies = queue.Queue()
        self.dirty = threading.Event()
        self.running = False
        self.link = None
        self.controller_count = None
        self.version = ""
        self.sent = {}               # motor -> last steps sent
        self.targets = {}            # motor -> current target from DMX
        self.filtered = {}           # motor -> smoothed target (smooth_ms)
        self.stats = {"commands": 0, "errors": 0, "last_reply": "", "rate": 0.0}
        self.threads = []
        self.in_socks = []
        self.in_proto = {}           # socket -> "Art-Net" / "sACN"
        self._last_err, self._last_err_t = None, 0.0
        self.paused = False          # after STOP: no position commands until resume()
        self.cmd_lock = threading.Lock()

    # ---- lifecycle
    def start(self):
        c = self.cfg["controller"]
        if c["link"] == "serial":
            self.link = SerialLink(c["com_port"], c["baud"], self._on_line)
            self.log("Controller: %s at %s baud" % (c["com_port"], c["baud"]))
        else:
            self.link = UdpLink(c["ip"], c["port"], c["reply_port"], self._on_line)
            self.log("Controller: %s:%s (replies on port %s)" % (c["ip"], c["port"], c["reply_port"]))
        self._open_inputs()
        self.running = True
        for fn in (self._rx_dmx, self._tx):
            t = threading.Thread(target=fn, daemon=True)
            t.start()
            self.threads.append(t)

    def stop(self):
        if not self.running:
            return
        self.running = False
        self.dirty.set()
        for t in self.threads:
            t.join(timeout=1.5)
        self.threads = []
        if self.link:
            try:
                if int(self.cfg["stream"]["watchdog_ms"]) > 0:
                    self.link.send("WD:0")   # leaving on purpose: no watchdog stop
            except OSError:
                pass
            self.link.close()
            self.link = None
        for s in self.in_socks:
            s.close()
        self.in_socks = []
        self.log("Bridge stopped")

    def emergency_stop(self):
        """B: every motor stops on its ramp, sent at once. The stream pauses
        (no position commands) until resume(), so nothing restarts the motors."""
        if self.link:
            with self.cmd_lock:              # no position command can slip in after B
                self.paused = True
                self.link.send("B")
            self.log("STOP sent (B) - streaming paused, press Resume to follow DMX again")

    def resume(self):
        with self.cmd_lock:
            self.paused = False
            self.sent = {}                   # send every current target again
        self.dirty.set()
        self.log("Streaming resumed")

    # ---- DMX input
    def _open_inputs(self):
        inp = self.cfg["input"]
        ip = inp.get("listen_ip", "0.0.0.0")
        if inp.get("artnet"):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.bind((ip, int(inp.get("artnet_port", ARTNET_PORT))))
            self.in_proto[s] = "Art-Net"
            self.in_socks.append(s)
            self.log("Listening for Art-Net on %s:%s" % (ip, inp.get("artnet_port", ARTNET_PORT)))
        if inp.get("sacn"):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0" if ip in ("", "0.0.0.0") else ip, int(inp.get("sacn_port", SACN_PORT))))
            joined = []
            for u in sorted({int(m["universe"]) for m in self.cfg["motors"] if m.get("enabled")}):
                if 1 <= u <= 63999:
                    try:
                        mreq = socket.inet_aton(sacn_group(u)) + socket.inet_aton("0.0.0.0" if ip in ("", "0.0.0.0") else ip)
                        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                        joined.append(u)
                    except OSError as e:
                        self.log("sACN: could not join multicast for universe %d (%s) - unicast still works" % (u, e))
            self.in_proto[s] = "sACN"
            self.in_socks.append(s)
            self.log("Listening for sACN on port %s%s" % (inp.get("sacn_port", SACN_PORT),
                     (" (multicast universes %s)" % ", ".join(map(str, joined))) if joined else ""))

    def _rx_dmx(self):
        while self.running:
            if not self.in_socks:
                time.sleep(0.1)
                continue
            try:
                ready, _, _ = select.select(self.in_socks, [], [], 0.2)
            except (OSError, ValueError):
                break
            for s in ready:
                try:
                    data, addr = s.recvfrom(1500)
                except OSError:
                    continue
                parsed = parse_artnet(data) if self.in_proto[s] == "Art-Net" else parse_sacn(data)
                if not parsed:
                    continue
                universe, dmx = parsed
                now = time.time()
                with self.lock:
                    u = self.universes.setdefault(universe, {"count": 0})
                    u.update(data=dmx, proto=self.in_proto[s], src=addr[0], t=now)
                    u["count"] += 1
                self.dirty.set()

    # ---- controller replies
    def _on_line(self, text):
        if text.startswith("EVENT:") or text == "Motor ready":
            self.log(text)
            return
        self.replies.put(text)

    def _command(self, line, timeout=0.5, motion=False):
        """Send one command and wait for its reply (one in flight at a time).
        motion=True: not sent while paused (returns None)."""
        with self.cmd_lock:
            if motion and self.paused:
                return None
            while not self.replies.empty():
                try:
                    self.replies.get_nowait()
                except queue.Empty:
                    break
            self.link.send(line)
        try:
            reply = self.replies.get(timeout=timeout)
        except queue.Empty:
            reply = None
        with self.lock:
            self.stats["last_reply"] = reply if reply is not None else "(no reply)"
            if reply is None or reply.startswith("ERR"):
                self.stats["errors"] += 1
        if reply is None or reply.startswith("ERR"):
            text = reply if reply is not None else "no reply"
            now = time.time()
            if text != self._last_err or now - self._last_err_t > 5.0:   # a repeating error: once per 5 s
                self.log("%s -> %s" % (line, text))
                self._last_err, self._last_err_t = text, now
        return reply

    # ---- the stream
    def compute_targets(self, exact=False):
        """motor -> steps for every enabled motor whose universe is live."""
        out = {}
        now = time.time()
        timeout = float(self.cfg["stream"].get("source_timeout_s", 2.0))
        with self.lock:
            for m in self.cfg["motors"]:
                if not m.get("enabled"):
                    continue
                u = self.universes.get(int(m["universe"]))
                if not u or now - u["t"] > timeout:
                    continue                       # no data: hold position
                v = read_value(u["data"], int(m["channel"]), m.get("bits", 16))
                out[int(m["motor"])] = map_steps(v, m, exact)
            self.targets = {n: int(round(s)) for n, s in out.items()}
        return out

    def smooth(self, raw, dt):
        """Glide toward the DMX targets (first-order, time constant smooth_ms).
        Returns rounded steps and whether a motor is still gliding."""
        tau = float(self.cfg["stream"].get("smooth_ms", 0)) / 1000.0
        out, gliding = {}, False
        for n, r in raw.items():
            f = self.filtered.get(n)
            if tau <= 0 or f is None:
                f = r                               # off, or the first value: no glide
            else:
                f += (r - f) * (1.0 - math.exp(-max(0.0, dt) / tau))
                if abs(r - f) < 0.5:
                    f = r
                else:
                    gliding = True
            self.filtered[n] = f
            out[n] = int(round(f))
        for n in list(self.filtered):
            if n not in raw:
                del self.filtered[n]                # source lost: start fresh when it returns
        return out, gliding

    def _setup_controller(self):
        reply = self._command("VER", timeout=1.0)
        if reply is None:
            self.log("No reply from the controller - check IP / port / reply port / firewall (still trying)")
        else:
            self.version = reply
            self.log("Controller: " + reply)
            fw = parse_version(reply)
            if fw is None:
                self.log("WARNING: the device on that address did not answer like a JMCMotors controller")
            elif fw < MIN_FIRMWARE:
                self.log("WARNING: controller firmware %s is older than %s - update the JMCMotors library "
                         "(Arduino Library Manager) and upload the sketch again" %
                         (".".join(map(str, fw)), ".".join(map(str, MIN_FIRMWARE))))
        reply = self._command("MC:?", timeout=1.0)
        if reply and reply.startswith("MC:"):
            try:
                self.controller_count = int(reply.split()[1])
            except (IndexError, ValueError):
                self.controller_count = None
        st = self.cfg["stream"]
        if int(st.get("watchdog_ms", 0)) > 0:
            self._command("WD:%d" % int(st["watchdog_ms"]))
        if st.get("send_settings", True):
            for m in self.cfg["motors"]:
                if not m.get("enabled"):
                    continue
                n = int(m["motor"])
                self._command("V:M%d=%s" % (n, float(m["speed"])))
                self._command("ACC:M%d=%s" % (n, float(m["accel"])))
                self._command("DEC:M%d=%s" % (n, float(m["accel"])))
        return reply is not None

    def _tx(self):
        try:
            self._setup_controller()
        except OSError as e:
            self.log("Controller link error: %s" % e)
        st = self.cfg["stream"]
        period = 1.0 / max(1.0, float(st.get("max_rate_hz", 40)))
        deadband = int(st.get("deadband_steps", 2))
        keepalive = max(0.1, int(st.get("watchdog_ms", 1000)) / 3000.0) if int(st.get("watchdog_ms", 0)) > 0 else 1.0
        last_send = 0.0
        last_setup = time.time()
        last_calc = time.time()
        gliding = False
        window_t, window_n = time.time(), 0
        while self.running:
            if not gliding:
                self.dirty.wait(timeout=min(period, keepalive))
            self.dirty.clear()
            if not self.running:
                break
            now = time.time()
            wait = period - (now - last_send)
            if wait > 0:                                   # rate limit: wait, then take the newest values
                time.sleep(wait)
            now = time.time()
            targets, gliding = self.smooth(self.compute_targets(exact=True), now - last_calc)
            last_calc = now
            changed = {n: s for n, s in targets.items()
                       if n not in self.sent or abs(s - self.sent[n]) >= max(1, deadband)}
            try:
                if not self.version and time.time() - last_setup > 3.0:
                    last_setup = time.time()       # controller was not answering at start: try again
                    self._setup_controller()
                if changed and not self.paused:
                    # all of the controller's motors mapped: send them all, so the
                    # controller can start them with one broadcast (fastest path)
                    if self.controller_count and len(targets) == self.controller_count:
                        batch = targets
                    else:
                        batch = changed
                    line = "P:" + ",".join("M%d=%d" % (n, batch[n]) for n in sorted(batch))
                    reply = self._command(line, motion=True)
                    if reply == "OK":
                        self.sent.update(batch)
                    last_send = time.time()
                    window_n += 1
                    with self.lock:
                        self.stats["commands"] += 1
                elif time.time() - last_send >= keepalive:
                    self._command("PING")                 # keeps the controller's watchdog fed
                    last_send = time.time()
            except OSError as e:
                self.log("Controller link error: %s" % e)
                time.sleep(0.5)
            if time.time() - window_t >= 1.0:
                with self.lock:
                    self.stats["rate"] = window_n / (time.time() - window_t)
                window_t, window_n = time.time(), 0

    # ---- for the window
    def snapshot(self):
        now = time.time()
        with self.lock:
            unis = [(u, d.get("proto", ""), d.get("src", ""), now - d.get("t", 0), d.get("count", 0))
                    for u, d in sorted(self.universes.items())]
            return {"universes": unis, "targets": dict(self.targets), "sent": dict(self.sent), "paused": self.paused,
                    "stats": dict(self.stats), "version": self.version, "count": self.controller_count,
                    "dmx": {u: d.get("data", b"") for u, d in self.universes.items()}}


def check_config(cfg):
    """Readable problems with the settings, or []."""
    p = []
    seen = set()
    for i, m in enumerate(cfg["motors"], 1):
        if not m.get("enabled"):
            continue
        n = int(m["motor"])
        if not 1 <= n <= 32:
            p.append("Row %d: motor must be 1..32" % i)
        if n in seen:
            p.append("Row %d: motor %d is used twice" % (i, n))
        seen.add(n)
        bits = int(m.get("bits", 16))
        top = 255 if bits == 8 else 65535
        if bits not in (8, 16):
            p.append("Row %d: bits must be 8 or 16" % i)
        if bits == 16 and not 1 <= int(m["channel"]) <= 511:
            p.append("Row %d: channel must be 1..511 in 16-bit mode (the fine channel is the next one)" % i)
        if bits == 8 and not 1 <= int(m["channel"]) <= 512:
            p.append("Row %d: channel must be 1..512" % i)
        if not (0 <= int(m["universe"]) <= 63999):
            p.append("Row %d: universe must be 0..63999" % i)
        for k in ("dmx_min", "dmx_max"):
            if not 0 <= int(m[k]) <= top:
                p.append("Row %d: %s must be 0..%d in %d-bit mode" % (i, k.replace("_", " ").upper(), top, bits))
        if int(m["dmx_min"]) == int(m["dmx_max"]):
            p.append("Row %d: DMX min and max must differ" % i)
        if not 0.01 <= float(m["speed"]) <= 1000:
            p.append("Row %d: speed must be 0.01..1000 rps" % i)
        if not 0.1 <= float(m["accel"]) <= 3276.7:
            p.append("Row %d: ramp must be 0.1..3276.7 rps/s" % i)
    if not cfg["input"].get("artnet") and not cfg["input"].get("sacn"):
        p.append("Tick Art-Net, sACN or both")
    st = cfg["stream"]
    if not 1 <= float(st.get("max_rate_hz", 40)) <= 200:
        p.append("Max updates / s must be 1..200")
    if not 0 <= float(st.get("smooth_ms", 0)) <= 5000:
        p.append("Smoothing must be 0..5000 ms")
    wd = int(st.get("watchdog_ms", 0))
    if wd != 0 and not 300 <= wd <= 65000:
        p.append("Watchdog must be 0 (off) or 300..65000 ms")
    return p


# --------------------------------------------------------------------------- app: settings page (local web UI)
UI_PORT = 8090
APP_ID = "jmc-dmx-bridge"


def asset(name):
    """A file shipped with the program (inside the packaged app, or next to this file)."""
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, name)


def serial_ports():
    try:
        from serial.tools import list_ports
        return sorted(p.device for p in list_ports.comports())
    except Exception:
        return []


def normalize_config(raw):
    """Settings from the page or a file -> (clean settings, problems)."""
    problems = []
    try:
        cfg = merge(default_config(), raw if isinstance(raw, dict) else {})
        motors = raw.get("motors", []) if isinstance(raw, dict) else []
        cfg["motors"] = []
        for i, m in enumerate(motors if isinstance(motors, list) else [], 1):
            m = merge(default_motor(i), m if isinstance(m, dict) else {})
            try:
                for k in ("motor", "universe", "channel", "bits", "dmx_min", "dmx_max", "steps_min", "steps_max"):
                    m[k] = int(float(m[k]))
                for k in ("speed", "accel"):
                    m[k] = float(m[k])
                m["enabled"] = bool(m.get("enabled", True))
            except (TypeError, ValueError):
                problems.append("Row %d: every field must be a number" % i)
            cfg["motors"].append(m)
        c, st = cfg["controller"], cfg["stream"]
        c["port"], c["reply_port"], c["baud"] = int(c["port"]), int(c["reply_port"]), int(c["baud"])
        for k in ("max_rate_hz", "smooth_ms", "source_timeout_s"):
            st[k] = float(st[k])
        st["deadband_steps"], st["watchdog_ms"] = int(float(st["deadband_steps"])), int(float(st["watchdog_ms"]))
        for k in ("artnet", "sacn"):
            cfg["input"][k] = bool(cfg["input"][k])
        st["send_settings"] = bool(st["send_settings"])
    except (TypeError, ValueError, KeyError, AttributeError):
        return default_config(), ["Settings are not valid - check the numbers"]
    if not problems:
        problems = check_config(cfg)
    return cfg, problems


class App:
    """Settings, the running bridge and the log - shared by the page and headless mode."""

    def __init__(self, path, echo=False):
        self.path = path
        self.cfg, _ = normalize_config(load_config(path))
        self.bridge = None
        self.lock = threading.Lock()
        self.logs = collections.deque(maxlen=600)
        self.log_id = 0
        self.echo = echo
        self.quit = threading.Event()

    def log(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        with self.lock:
            self.log_id += 1
            self.logs.append((self.log_id, line))
        if self.echo:
            print(line, flush=True)

    def state(self, since=0):
        with self.lock:
            logs = [l for i, l in self.logs if i > since]
            last = self.log_id
        out = {"app": APP_ID, "version": VERSION, "settings_path": self.path, "cfg": self.cfg,
               "running": self.bridge is not None, "ports": serial_ports(), "log": logs, "log_id": last}
        b = self.bridge
        if b:
            s = b.snapshot()
            rows = []
            for m in b.cfg["motors"]:
                data = s["dmx"].get(int(m["universe"]))
                n = int(m["motor"])
                rows.append({"dmx": read_value(data, int(m["channel"]), m.get("bits", 16)) if data else None,
                             "target": s["targets"].get(n), "sent": s["sent"].get(n)})
            out["live"] = {"rows": rows, "paused": s["paused"], "stats": s["stats"], "controller": s["version"],
                           "count": s["count"],
                           "universes": [{"universe": u, "proto": p, "src": src, "age": round(age, 1), "count": c}
                                         for u, p, src, age, c in s["universes"]]}
        return out

    def save(self, raw):
        cfg, problems = normalize_config(raw)
        if problems:
            return problems
        save_config(self.path, cfg)
        self.cfg = cfg
        self.log("Settings saved to " + self.path)
        return []

    def start(self, raw=None):
        if self.bridge:
            return ["The bridge is already running"]
        problems = self.save(raw) if raw is not None else check_config(self.cfg)
        if problems:
            return problems
        b = Bridge(json.loads(json.dumps(self.cfg)), self.log)
        try:
            b.start()
        except (OSError, RuntimeError) as e:
            b.stop()
            msg = "Could not start: %s - is another program (or a second bridge) using the port?" % e
            self.log(msg)
            return [msg]
        self.bridge = b
        return []

    def stop(self):
        b, self.bridge = self.bridge, None
        if b:
            b.stop()

    def estop(self):
        if self.bridge:
            self.bridge.emergency_stop()
        else:
            self.log("STOP: the bridge is not running (nothing was streaming)")

    def resume(self):
        if self.bridge:
            self.bridge.resume()


class UiHandler(BaseHTTPRequestHandler):
    """The settings page and its JSON API. Only answers this PC (127.0.0.1), checks
    the Host header (no DNS rebinding) and needs a custom header on every change
    (another web page cannot send it), so no website can drive the motors."""
    app = None
    port = 0
    server_version = "JMCDMXBridge"

    def log_message(self, *args):
        pass

    def _host_ok(self):
        h = (self.headers.get("Host") or "").lower()
        return h in ("127.0.0.1:%d" % self.port, "localhost:%d" % self.port)

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if not self._host_ok():
            return self._send(403, {"error": "forbidden"})
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            with open(asset("ui.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if url.path == "/favicon.ico" and os.path.exists(asset("icon.ico")):
            with open(asset("icon.ico"), "rb") as f:
                return self._send(200, f.read(), "image/x-icon")
        if url.path == "/api/ping":
            return self._send(200, {"app": APP_ID, "version": VERSION})
        if url.path == "/api/state":
            try:
                since = int(parse_qs(url.query).get("since", ["0"])[0])
            except ValueError:
                since = 0
            return self._send(200, self.app.state(since))
        return self._send(404, {"error": "not found"})

    def do_POST(self):
        if not self._host_ok() or self.headers.get("X-JMC-Bridge") != "1":
            return self._send(403, {"error": "forbidden"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n > 1000000:
                return self._send(413, {"error": "too large"})
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._send(400, {"error": "bad JSON"})
        a, path = self.app, urlparse(self.path).path
        if path == "/api/save":
            problems = a.save(body.get("cfg"))
        elif path == "/api/start":
            problems = a.start(body.get("cfg"))
        elif path == "/api/stop":
            a.stop()
            problems = []
        elif path == "/api/estop":
            a.estop()
            problems = []
        elif path == "/api/resume":
            a.resume()
            problems = []
        elif path == "/api/check":
            problems = normalize_config(body.get("cfg"))[1]
        elif path == "/api/quit":
            self._send(200, {"ok": True, "problems": []})
            a.quit.set()
            return None
        else:
            return self._send(404, {"error": "not found"})
        return self._send(200, {"ok": not problems, "problems": problems})


class UiServer(ThreadingHTTPServer):
    """Local page server. The port is exclusive (Windows would otherwise let a
    second copy bind the same port), and no slow host-name lookup at start."""
    daemon_threads = True
    allow_reuse_address = not sys.platform.startswith("win")

    def server_bind(self):
        if sys.platform.startswith("win") and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = self.server_address[:2]


def ping_ui(port):
    """True if a JMC DMX Bridge already serves its page on this port."""
    try:
        with urlopen("http://127.0.0.1:%d/api/ping" % port, timeout=1.0) as r:
            return json.loads(r.read().decode()).get("app") == APP_ID
    except Exception:
        return False


def start_ui_server(app, port):
    """-> (server, port), or (None, port) when another bridge already serves its page there."""
    for p in range(port, port + 10):
        handler = type("Handler", (UiHandler,), {"app": app, "port": p})
        try:
            srv = UiServer(("127.0.0.1", p), handler)
        except OSError:
            if ping_ui(p):
                return None, p
            continue
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        return srv, p
    raise RuntimeError("no free port for the settings page (%d..%d)" % (port, port + 9))


def find_app_browser():
    """A Chromium browser that can open the page as an app window (no tabs or address bar)."""
    cands = []
    if sys.platform.startswith("win"):
        for env in ("PROGRAMFILES(X86)", "PROGRAMFILES", "LOCALAPPDATA"):
            base = os.environ.get(env)
            if base:
                cands += [os.path.join(base, r"Microsoft\Edge\Application\msedge.exe"),
                          os.path.join(base, r"Google\Chrome\Application\chrome.exe"),
                          os.path.join(base, r"BraveSoftware\Brave-Browser\Application\brave.exe")]
    elif sys.platform == "darwin":
        cands = ["/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
                 "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
                 "/Applications/Chromium.app/Contents/MacOS/Chromium",
                 "/Applications/Brave Browser.app/Contents/MacOS/Brave Browser"]
    else:
        cands = [shutil.which(n) for n in ("google-chrome", "chromium", "chromium-browser",
                                           "microsoft-edge", "brave-browser")]
    for c in cands:
        if c and os.path.exists(c):
            return c
    return None


def open_window(url):
    """Open the page in its own app window. Returns the window's process, or None
    when it opened in the normal browser (then the page's Quit button ends the app)."""
    exe = find_app_browser()
    if exe:
        profile = os.path.join(settings_dir(), "window")
        try:
            return subprocess.Popen([exe, "--app=" + url, "--user-data-dir=" + profile,
                                     "--window-size=1320,900", "--no-first-run",
                                     "--no-default-browser-check", "--disable-features=Translate"])
        except OSError:
            pass
    webbrowser.open(url)
    return None


def run_app(path, headless=False, port=UI_PORT):
    app = App(path, echo=headless)
    if headless and not os.path.exists(path):
        save_config(path, app.cfg)
        print("Wrote default settings to %s - edit them (here or on the settings page), then start again." % path)
        return
    srv, port = start_ui_server(app, port)
    url = "http://127.0.0.1:%d/" % port
    if srv is None:                                   # already running: show its window
        if headless:
            print("A JMC DMX Bridge is already running (%s)" % url)
            sys.exit(1)
        open_window(url)
        return
    app.log("JMC DMX Bridge %s - settings: %s" % (VERSION, path))
    win = None
    if headless:
        app.log("Settings page: " + url)
        problems = app.start()
        if problems:
            app.log("Not started: " + "; ".join(problems))
            sys.exit(1)
        app.log("Running - Ctrl+C to stop.")
    else:
        t0 = time.time()
        win = open_window(url)
        app.log("Press Start, then send DMX from Resolume / MadMapper. Esc or STOP MOTORS = stop every motor.")
    try:
        while not app.quit.is_set():
            if win is not None and win.poll() is not None:
                if time.time() - t0 < 4.0:
                    win = None                       # handed over to a running browser: use Quit instead
                else:
                    app.log("Window closed")
                    break
            app.quit.wait(0.5)
    except KeyboardInterrupt:
        pass
    app.stop()
    srv.shutdown()


if __name__ == "__main__":
    if sys.stdout is None:          # packaged app without a console: write messages to a log file
        sys.stdout = sys.stderr = open(os.path.join(settings_dir(), "jmc_dmx_bridge.log"), "a", buffering=1)
    ap = argparse.ArgumentParser(description="Art-Net / sACN to JMC motors")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="settings file (JSON)")
    ap.add_argument("--headless", action="store_true", help="no window: start streaming with the saved settings")
    ap.add_argument("--ui-port", type=int, default=UI_PORT, help="port of the settings page on this PC (default 8090)")
    ap.add_argument("--version", action="version", version="JMC DMX Bridge " + VERSION)
    a = ap.parse_args()
    run_app(a.config, headless=a.headless, port=a.ui_port)
