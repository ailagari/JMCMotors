#!/usr/bin/env python3
"""
JMC DMX Bridge - Art-Net / sACN (E1.31) to JMC motors, in real time.

Runs on the same PC as Resolume, MadMapper, a lighting console or anything
else that sends DMX over the network. Each motor follows one 16-bit DMX
value (coarse + fine channel): the value is mapped to a step position and
sent to the JMCMotors controller (CONTROLLINO / ESP32 / Mega over UDP, or an
Uno over a USB COM port) as position commands.

    python jmc_dmx_bridge.py                       settings window
    python jmc_dmx_bridge.py --headless            no window, settings file only
    python jmc_dmx_bridge.py --config other.json   another settings file

Needs Python 3.8+ (standard library only). The USB / COM-port connection
also needs pyserial:  python -m pip install pyserial

Part of JMCMotors (MIT) - https://github.com/ailagari/JMCMotors
Guide: extras/dmx-bridge/README.md
"""

import argparse
import json
import math
import os
import queue
import select
import socket
import struct
import sys
import threading
import time

VERSION = "1.0.0"
ARTNET_PORT = 6454
SACN_PORT = 5568
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "jmc_dmx_bridge.json")


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


# --------------------------------------------------------------------------- settings window
def run_gui(path):
    import tkinter as tk
    from tkinter import ttk, messagebox, filedialog

    cfg = load_config(path)
    state = {"bridge": None, "path": path}
    root = tk.Tk()
    root.title("JMC DMX Bridge %s" % VERSION)
    root.minsize(980, 640)

    def var(value):
        return tk.StringVar(value=str(value))

    # ---- controller / input / stream settings
    top = ttk.Frame(root, padding=8)
    top.pack(fill="x")
    c, i, st = cfg["controller"], cfg["input"], cfg["stream"]
    v = {
        "link": tk.StringVar(value=c["link"]), "ip": var(c["ip"]), "port": var(c["port"]),
        "reply_port": var(c["reply_port"]), "com_port": var(c["com_port"]), "baud": var(c["baud"]),
        "artnet": tk.BooleanVar(value=i["artnet"]), "sacn": tk.BooleanVar(value=i["sacn"]),
        "listen_ip": var(i["listen_ip"]), "max_rate_hz": var(st["max_rate_hz"]),
        "deadband_steps": var(st["deadband_steps"]), "watchdog_ms": var(st["watchdog_ms"]),
        "smooth_ms": var(st["smooth_ms"]), "send_settings": tk.BooleanVar(value=st["send_settings"]),
    }
    box = ttk.LabelFrame(top, text="Controller", padding=6)
    box.grid(row=0, column=0, sticky="nsew", padx=4)
    ttk.Radiobutton(box, text="Network (UDP)", variable=v["link"], value="udp").grid(row=0, column=0, sticky="w")
    ttk.Radiobutton(box, text="USB / COM port", variable=v["link"], value="serial").grid(row=0, column=1, sticky="w")
    for r, (lab, key) in enumerate([("Controller IP", "ip"), ("Controller port", "port"), ("Reply port (PC_PORT)", "reply_port")], 1):
        ttk.Label(box, text=lab).grid(row=r, column=0, sticky="w")
        ttk.Entry(box, textvariable=v[key], width=16).grid(row=r, column=1, sticky="w")
    ttk.Label(box, text="COM port / baud").grid(row=4, column=0, sticky="w")
    f = ttk.Frame(box); f.grid(row=4, column=1, sticky="w")
    ttk.Entry(f, textvariable=v["com_port"], width=8).pack(side="left")
    ttk.Entry(f, textvariable=v["baud"], width=7).pack(side="left", padx=4)

    box = ttk.LabelFrame(top, text="DMX input", padding=6)
    box.grid(row=0, column=1, sticky="nsew", padx=4)
    ttk.Checkbutton(box, text="Art-Net (port 6454)", variable=v["artnet"]).grid(row=0, column=0, sticky="w")
    ttk.Checkbutton(box, text="sACN / E1.31 (port 5568)", variable=v["sacn"]).grid(row=1, column=0, sticky="w")
    ttk.Label(box, text="Listen on IP (0.0.0.0 = all)").grid(row=2, column=0, sticky="w")
    ttk.Entry(box, textvariable=v["listen_ip"], width=16).grid(row=3, column=0, sticky="w")

    box = ttk.LabelFrame(top, text="Streaming", padding=6)
    box.grid(row=0, column=2, sticky="nsew", padx=4)
    for r, (lab, key) in enumerate([("Max updates / s", "max_rate_hz"), ("Deadband (steps)", "deadband_steps"),
                                    ("Smoothing ms (0 = off)", "smooth_ms"), ("Watchdog ms (0 = off)", "watchdog_ms")]):
        ttk.Label(box, text=lab).grid(row=r, column=0, sticky="w")
        ttk.Entry(box, textvariable=v[key], width=8).grid(row=r, column=1, sticky="w")
    ttk.Checkbutton(box, text="Send speed / ramps on start", variable=v["send_settings"]).grid(row=4, column=0, columnspan=2, sticky="w")

    btns = ttk.Frame(top, padding=(4, 0))
    btns.grid(row=0, column=3, sticky="nsew")
    start_btn = ttk.Button(btns, text="Start")
    start_btn.pack(fill="x", pady=2)
    stop_btn = tk.Button(btns, text="STOP MOTORS", bg="#c62828", fg="white", font=("Segoe UI", 11, "bold"))
    stop_btn.pack(fill="x", pady=2)
    resume_btn = ttk.Button(btns, text="Resume", command=lambda: state["bridge"] and state["bridge"].resume())
    resume_btn.pack(fill="x", pady=2)
    ttk.Button(btns, text="Save settings", command=lambda: do_save()).pack(fill="x", pady=2)
    ttk.Button(btns, text="Open settings…", command=lambda: do_open()).pack(fill="x", pady=2)

    # ---- motor table
    mid = ttk.LabelFrame(root, text="Motors - one DMX value each (16-bit: channel + next channel; 8-bit: one channel)", padding=6)
    mid.pack(fill="both", expand=True, padx=8)
    cols = [("On", 4), ("Motor", 6), ("Universe", 8), ("Channel", 8), ("Bits", 5), ("DMX min", 8), ("DMX max", 8),
            ("Steps min", 10), ("Steps max", 10), ("Speed rps", 9), ("Ramp rps/s", 9), ("DMX now", 9), ("Target", 10), ("Sent", 10)]
    canvas = tk.Canvas(mid, highlightthickness=0, height=230)
    sb = ttk.Scrollbar(mid, orient="vertical", command=canvas.yview)
    table = ttk.Frame(canvas)
    table.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.create_window((0, 0), window=table, anchor="nw")
    canvas.configure(yscrollcommand=sb.set)
    canvas.pack(side="left", fill="both", expand=True)
    sb.pack(side="right", fill="y")
    rows = []
    keys = ["motor", "universe", "channel", "bits", "dmx_min", "dmx_max", "steps_min", "steps_max", "speed", "accel"]

    def header():
        for col, (name, w) in enumerate(cols):
            ttk.Label(table, text=name, width=w, anchor="w", font=("Segoe UI", 9, "bold")).grid(row=0, column=col, padx=1)

    def add_row(m):
        r = len(rows) + 1
        row = {"enabled": tk.BooleanVar(value=m["enabled"])}
        ttk.Checkbutton(table, variable=row["enabled"]).grid(row=r, column=0)
        for col, k in enumerate(keys, 1):
            row[k] = var(m.get(k, 16) if k == "bits" else m[k])
            if k == "bits":
                ttk.Combobox(table, textvariable=row[k], values=("16", "8"), width=3,
                             state="readonly").grid(row=r, column=col, padx=1, pady=1)
            else:
                ttk.Entry(table, textvariable=row[k], width=cols[col][1]).grid(row=r, column=col, padx=1, pady=1)

        def bits_changed(*_):                  # keep DMX max in range when switching 8 / 16 bit
            mx = row["dmx_max"].get().strip()
            if row["bits"].get() == "8" and mx == "65535":
                row["dmx_max"].set("255")
            elif row["bits"].get() == "16" and mx == "255":
                row["dmx_max"].set("65535")
        row["bits"].trace_add("write", bits_changed)
        for col, k in enumerate(["dmx_now", "target", "sent"], len(keys) + 1):
            row[k] = tk.StringVar(value="-")
            ttk.Label(table, textvariable=row[k], width=cols[col][1]).grid(row=r, column=col)
        row["widgets"] = table.grid_slaves(row=r)
        rows.append(row)

    def remove_row():
        if rows:
            for w in rows.pop()["widgets"]:
                w.destroy()

    header()
    for m in cfg["motors"]:
        add_row(m)
    tb = ttk.Frame(root, padding=(8, 2))
    tb.pack(fill="x")
    ttk.Button(tb, text="Add motor", command=lambda: add_row(default_motor(len(rows) + 1))).pack(side="left")
    ttk.Button(tb, text="Remove last", command=remove_row).pack(side="left", padx=4)
    ttk.Label(tb, text="Swap min and max to reverse a direction. Speed: how fast a motor may follow the DMX value.").pack(side="left", padx=12)

    # ---- incoming + log
    bottom = ttk.Frame(root, padding=8)
    bottom.pack(fill="both", expand=True)
    inc = ttk.LabelFrame(bottom, text="Incoming DMX (use these universe numbers)", padding=4)
    inc.pack(side="left", fill="y")
    inc_list = tk.Listbox(inc, width=44, height=10, font=("Consolas", 9))
    inc_list.pack(fill="y", expand=True)
    lg = ttk.LabelFrame(bottom, text="Log - controller replies and events", padding=4)
    lg.pack(side="left", fill="both", expand=True, padx=(8, 0))
    log_box = tk.Text(lg, height=10, font=("Consolas", 9), state="disabled")
    log_box.pack(fill="both", expand=True)
    status = tk.StringVar(value="Stopped")
    ttk.Label(root, textvariable=status, relief="sunken", anchor="w", padding=4).pack(fill="x", side="bottom")
    log_q = queue.Queue()

    def log(msg):
        log_q.put(time.strftime("%H:%M:%S ") + msg)

    def gather():
        out = default_config()
        out["controller"] = {"link": v["link"].get(), "ip": v["ip"].get().strip(), "port": int(v["port"].get()),
                             "reply_port": int(v["reply_port"].get()), "com_port": v["com_port"].get().strip(),
                             "baud": int(v["baud"].get())}
        out["input"] = {"artnet": v["artnet"].get(), "sacn": v["sacn"].get(), "listen_ip": v["listen_ip"].get().strip(),
                        "artnet_port": cfg["input"].get("artnet_port", ARTNET_PORT),
                        "sacn_port": cfg["input"].get("sacn_port", SACN_PORT)}
        out["stream"] = {"max_rate_hz": float(v["max_rate_hz"].get()), "deadband_steps": int(v["deadband_steps"].get()),
                         "smooth_ms": float(v["smooth_ms"].get()), "watchdog_ms": int(v["watchdog_ms"].get()), "send_settings": v["send_settings"].get(),
                         "source_timeout_s": cfg["stream"].get("source_timeout_s", 2.0)}
        motors = []
        for row in rows:
            m = {"enabled": row["enabled"].get()}
            for k in keys:
                m[k] = float(row[k].get()) if k in ("speed", "accel") else int(float(row[k].get()))
            motors.append(m)
        out["motors"] = motors
        problems = check_config(out)
        if problems:
            raise ValueError("\n".join(problems))
        return out

    def do_save():
        try:
            save_config(state["path"], gather())
            log("Settings saved to " + state["path"])
        except ValueError as e:
            messagebox.showerror("Settings", str(e))

    def do_open():
        p = filedialog.askopenfilename(filetypes=[("Settings", "*.json")])
        if p:
            if state["bridge"]:
                toggle()
            root.destroy()
            run_gui(p)

    def toggle():
        if state["bridge"]:
            state["bridge"].stop()
            state["bridge"] = None
            start_btn.configure(text="Start")
            status.set("Stopped")
            return
        try:
            new = gather()
        except ValueError as e:
            messagebox.showerror("Settings", str(e))
            return
        cfg.update(new)
        b = Bridge(cfg, log)
        try:
            b.start()
        except (OSError, RuntimeError) as e:
            messagebox.showerror("Start", "Could not start: %s\n\nIs another program (or a second bridge) using the port?" % e)
            b.stop()
            return
        state["bridge"] = b
        start_btn.configure(text="Stop bridge")

    start_btn.configure(command=toggle)
    stop_btn.configure(command=lambda: state["bridge"].emergency_stop() if state["bridge"] else None)

    def tick():
        while not log_q.empty():
            line = log_q.get_nowait()
            log_box.configure(state="normal")
            log_box.insert("end", line + "\n")
            if int(log_box.index("end-1c").split(".")[0]) > 400:
                log_box.delete("1.0", "100.0")
            log_box.see("end")
            log_box.configure(state="disabled")
        b = state["bridge"]
        if b:
            snap = b.snapshot()
            inc_list.delete(0, "end")
            for u, proto, src, age, count in snap["universes"]:
                inc_list.insert("end", "%-7s universe %-5d from %-15s %s" %
                                (proto, u, src, "live" if age < 2 else "lost %.0fs" % age))
            for row in rows:
                try:
                    n, u, ch = int(row["motor"].get()), int(row["universe"].get()), int(row["channel"].get())
                    bits = int(row["bits"].get())
                except ValueError:
                    continue
                data = snap["dmx"].get(u)
                row["dmx_now"].set(str(read_value(data, ch, bits)) if data else "-")
                row["target"].set(str(snap["targets"].get(n, "-")))
                row["sent"].set(str(snap["sent"].get(n, "-")))
            s = snap["stats"]
            status.set(("PAUSED after STOP - press Resume | " if snap["paused"] else "Running - ") + "%s | motors on controller: %s | %.0f updates/s | commands %d, errors %d | last reply: %s" %
                       (snap["version"] or "controller not answering", snap["count"] or "?", s["rate"],
                        s["commands"], s["errors"], s["last_reply"]))
        root.after(100, tick)

    def on_close():
        if state["bridge"]:
            state["bridge"].stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.bind("<Escape>", lambda e: state["bridge"] and state["bridge"].emergency_stop())
    log("JMC DMX Bridge %s - settings: %s" % (VERSION, state["path"]))
    log("Press Start, then send DMX from Resolume / MadMapper. Esc or STOP MOTORS = stop every motor.")
    tick()
    root.mainloop()


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


def run_headless(path):
    cfg = load_config(path)
    if not os.path.exists(path):
        save_config(path, cfg)
        print("Wrote default settings to %s - edit it, then start again." % path)
        return
    problems = check_config(cfg)
    if problems:
        print("Settings problems:\n  " + "\n  ".join(problems))
        sys.exit(1)
    b = Bridge(cfg, lambda m: print(time.strftime("%H:%M:%S ") + m, flush=True))
    b.start()
    print("Running - Ctrl+C to stop.", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        b.stop()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Art-Net / sACN to JMC motors")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="settings file (JSON)")
    ap.add_argument("--headless", action="store_true", help="run without the settings window")
    a = ap.parse_args()
    if a.headless:
        run_headless(a.config)
    else:
        run_gui(a.config)
