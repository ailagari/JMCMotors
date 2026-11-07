# JMC DMX Bridge

Move JMC motors live from **Resolume, MadMapper, a lighting console or any
software that sends Art-Net or sACN**. The bridge runs on the same PC as
Resolume. It turns each motor's DMX value into a step position and sends it to
the JMCMotors controller (CONTROLLINO, ESP32 or Mega over the network, or an
Uno over USB), many times a second.

```
Resolume / MadMapper ──Art-Net or sACN──▶ JMC DMX Bridge ──UDP or USB──▶ controller ──RS-485──▶ JMC drives
        (same PC)                          (this program)                 (JMCMotors sketch)
```

- **Inputs:** Art-Net (port 6454) and sACN / E1.31 (port 5568, unicast or multicast).
- **Several controllers:** one per IP (or COM port), **up to 32 motors each**, all from one window. Each controller streams on its own, so a slow bus never holds up the others.
- **Per motor:** universe, channel, **16-bit** (channel + next channel, 0–65535) or **8-bit** (one channel, 0–255), DMX min/max, step min/max, speed, ramp.
- **Safety:** STOP MOTORS button (or Esc); the motors stay stopped until you press Resume. If the bridge or PC dies, the controller's watchdog stops every motor. If DMX stops arriving, the motors hold their position.
- **Smooth:** sends only the newest value, at most 40 updates/s (adjustable). Optional smoothing turns 8-bit staircases into glides.

---

## 1. Before you start

**Hardware**

- A controller running the `JMCMotors_Controller` sketch. For the network you need a CONTROLLINO, an ESP32, or a Mega + Ethernet shield. An Uno has no room for the network code, so use it over USB.
- **RS-485** between the controller and the drives: a TTL-to-**RS-485** module (MAX485 / MAX13487 type) on Arduino and ESP32. CONTROLLINO has it built in. A TTL-to-RS-232 converter will **not** talk to the drives, because the JMC "-RC" drives use RS-485. See [HARDWARE.md](../HARDWARE.md).
- **No DMX shield is needed.** The DMX comes over the network into the bridge. A DMX shield only matters if you want a wired XLR DMX cable straight into the Arduino, which is a different setup.

**Three pieces, one version number**

| Piece | Where to get it |
|---|---|
| Arduino library (controller firmware) | Arduino IDE → **Library Manager** → search **JMCMotors** → Install. Then File → Examples → JMCMotors → `JMCMotors_Controller` |
| JMC DMX Bridge app | **[GitHub Releases](https://github.com/ailagari/JMCMotors/releases/latest)** → download the zip for your computer |
| Your show software | Resolume Arena, MadMapper, a lighting console, QLC+, TouchDesigner… anything that sends Art-Net or sACN |

Use the app and library with the **same version number** (e.g. both 0.6.0). The bridge reads the controller's version when it starts and warns in its log if the firmware is too old.

**Install the app (no Python needed)**

| Computer | Download | Run |
|---|---|---|
| Windows 10 / 11 | `JMC-DMX-Bridge-<version>-windows-x64.zip` | unzip, double-click `JMC-DMX-Bridge.exe`. The app isn't code-signed, so SmartScreen may warn: click **More info → Run anyway** |
| macOS (Apple Silicon) | `JMC-DMX-Bridge-<version>-macos-arm64.zip` | unzip, move `JMC DMX Bridge.app` to Applications. The first time, **right-click → Open** (it isn't notarised) |
| Linux (x64) | `JMC-DMX-Bridge-<version>-linux-x64.zip` | unzip, `chmod +x JMC-DMX-Bridge`, then `./JMC-DMX-Bridge` |

The app includes everything, including USB/COM-port support. It opens **its own window**, in the same design as the [JMCMotors reference page](https://ailagari.github.io/JMCMotors/). The window is drawn by the Edge or Chrome already on the computer (built into Windows 10/11), in app mode: no tabs, no address bar. **Closing the window quits the app**, and the motors hold their position. Double-clicking the app again while it runs brings its window back. With no Edge, Chrome, Chromium or Brave installed, the page opens in your normal browser; then use **Quit** at the bottom of the page to end the app. The page is only reachable from this computer (`127.0.0.1`), never from the network. **Settings** are saved in `jmc_dmx_bridge.json` next to the app (Windows / Linux, so a USB stick works as a portable setup) or in `~/Library/Application Support/JMC DMX Bridge` (macOS). Messages in headless mode go to `jmc_dmx_bridge.log` in the same place.

<details><summary>Run from source instead (Intel Mac, other systems, developers)</summary>

Install Python 3.8+ ([python.org](https://www.python.org/downloads/); on Windows tick "Add python.exe to PATH"). For the USB link, run `python -m pip install pyserial`. Then run `python jmc_dmx_bridge.py`, or double-click `start_bridge.bat` on Windows. To build the app yourself: `python -m pip install pyinstaller pyserial`, then `python build_app.py`. The result lands in `dist/`.
</details>

**Get the controller working first.** Before the bridge, send `P:1000` from the serial monitor or the [interactive reference](https://ailagari.github.io/JMCMotors/) and see the motor move. Set soft limits (`SL`) and home or zero (`H` / `Z`) every axis. The bridge sends **absolute** positions, so step 0 must be where you expect it.

---

## 2. Set up the controller sketch (network)

In `JMCMotors_Controller.ino`, STEP 3:

| Setting | Set it to |
|---|---|
| `BOARD_IP_ADDR` | a free address on the PC's network, e.g. `192, 168, 1, 101` |
| `BOARD_UDP_PORT` | `8001` (the bridge's **Controller port**) |
| `PC_IP_ADDR` | **the PC's IP address** (the PC that runs Resolume and the bridge) |
| `PC_UDP_PORT` | `8000` (the bridge's **Reply port**) |

To find the PC's IP, open a command prompt, type `ipconfig`, and read the "IPv4 Address" of the wired network card. Upload the sketch. The PC and the controller must be on the same network, for example both `192.168.1.x` with mask `255.255.255.0`.

For an **Uno over USB**, use `PC_LINK LINK_USB`. In the bridge, pick **USB / COM port** and enter the COM port from Arduino IDE > Tools > Port. The baud is `57600` for an Uno and `115200` for the other boards.

---

## 3. Start the bridge

Open **JMC DMX Bridge** (or, from source, `start_bridge.bat` / `python jmc_dmx_bridge.py`).

> **Windows Firewall:** the first time, Windows asks whether JMC-DMX-Bridge (or Python, from source) may use the network. Tick **Private networks** and click **Allow**. If you miss it, no DMX and no controller replies get through. To fix it, go to Windows Security > Firewall > "Allow an app through firewall" and tick it. macOS asks the same ("accept incoming network connections"): click **Allow**.

Fill in the window, top to bottom:

1. **DMX input:** tick Art-Net, sACN or both. Leave Listen on IP as `0.0.0.0`.
2. **Replies:** the reply port is the sketch's `PC_UDP_PORT` (`8000`). It is one port for every controller.
3. **Controllers:** one panel per controller. Name it (e.g. "Stage left"), pick **Network**, and enter its IP (`BOARD_IP_ADDR`) and port (`8001`). For an Uno, pick **USB / COM** and choose its port. **+ Add controller** adds the next one; it picks the next IP and the next free universe for you.
4. **Motors:** one row per motor, **up to 32 per controller**. **Motor** is the drive's number on that controller (M1, M2…). **+ Add motor**, **+ Add 8** and **Fill to 32** continue the numbering and the channels. They move to the next universe when one is full, so 32 motors in 16-bit mode use channels 1–63 of one universe. Section 6 explains each column.
5. Click **Save settings**, then **Start**.

The header should show **Streaming**, and each controller panel shows its firmware, its motor count and **Streaming**. If a panel says *Not answering*, see Troubleshooting.

**Several controllers:** in every controller's sketch, set its own `BOARD_IP_ADDR`, the same `PC_IP_ADDR` (this PC) and the same `PC_UDP_PORT`. The bridge sorts their replies by sender. Each controller runs its own RS-485 bus with its own drive IDs 1–32, so two controllers can both have an M1.

**The window adapts to its size.** Wide, each controller's motors are a table. Narrower (half a screen, a tablet), each motor becomes a card. On a phone-width window, one card per row. The fold arrow ▾ on a controller, or **Collapse all**, hides controllers you aren't working on.

---

## 4. Connect Resolume Arena — step by step

DMX output is in **Resolume Arena** only, not Avenue. Resolume sends Art-Net and builds its DMX from **pixels**: a fixture reads the brightness of a spot in your composition and sends it as a channel value. White = 255, black = 0. Resolume sends **8-bit** values only, so use **8-bit** rows in the bridge with some smoothing.

### 4.1 Point Resolume's Art-Net output at the bridge

1. **Arena > Preferences > DMX:** pick the wired network card for Art-Net output. Set **Delay** to `0` (the default 40 ms adds lag), and **Framerate** to 30–44.
2. **Output > Advanced Output…** Click **+** and add a **Lumiverse**.
3. Right-click the Lumiverse. Set **Target IP** to **IP Address** and enter **the PC's own IP** (from `ipconfig`), or `127.0.0.1`. Bridge and Resolume are on the same PC, so this sends the data to the bridge.
4. Set **Subnet = 0** and **Universe = 0**. The bridge will then see this Lumiverse as **universe 0**.
5. Keep the Lumiverse's **Opacity 100 %, Brightness and Contrast neutral, Color white**. These settings scale every value, which would shift your motor positions.

### 4.2 Make a one-channel "motor" fixture

1. Next to the Fixture drop-down, click the **gear icon** to open the **Fixture Editor**.
2. Click **+** to make a new fixture and name it `JMC Motor`.
3. Set **Width 1, Height 1** (one pixel).
4. Set **Color Space** to **L** (luminance, one channel per pixel).
5. Set **Gamma Correction** to **1.0**. The default 2.5 bends the curve: half brightness would land at about 18 % of the travel instead of 50 %.

### 4.3 Place one fixture per motor

1. In Advanced Output, add a `JMC Motor` fixture to the Lumiverse for each motor.
2. In **Input Selection**, place each fixture over its own area of the composition, for example motor 1 top-left, motor 2 top-right. That area's brightness becomes the motor's position.
3. Go to the Lumiverse's **DMX Output** tab and set each fixture's **Start Channel**: motor 1 → 1, motor 2 → 2, and so on.

### 4.4 Match the bridge

| Bridge column | Value |
|---|---|
| Universe | `0`, as shown in the bridge's **Incoming DMX** list |
| Channel | the fixture's Start Channel |
| Bits | **8** |
| DMX min / max | `0` / `255` (black → steps min, white → steps max) |
| Steps min / max | the travel you want, e.g. `0` / `20000` |

Set **Smoothing** to about `150` ms. 8-bit gives 256 positions over the whole travel, and smoothing makes the motor glide between them instead of stepping.

### 4.5 Drive it

- Put a **Solid Color** (white) source on a layer that covers the fixture's area, and animate the layer's **Opacity**. You can also map Opacity to a dashboard knob, MIDI fader or OSC. Black = steps min, full white = steps max.
- Any content works. A gradient wipe, a mask or an animated shape over the fixture's pixel all become motion.
- Watch **DMX now / Target / Sent** in the bridge to see what arrives.

> **Need finer than 1/256 of the travel?** Use a source that sends 16-bit DMX (MadMapper, a lighting console, QLC+, TouchDesigner…), or split the travel into a smaller range per motor.

---

## 5. Connect MadMapper and other DMX software

Menu names vary between versions; the idea is the same everywhere:

1. **Add an Art-Net or sACN output** with the destination IP set to **this PC** (`ipconfig`, or `127.0.0.1`). Pick a universe.
2. **Patch a fixture with a 16-bit channel** if the software has one. Look for a "16-bit", "fine" or "pan + pan fine" channel type. It sends coarse on channel N and fine on N+1. Use **Bits = 16** in the bridge and set Channel = N.
3. If only 8-bit channels exist (dimmer or pixel mapping), use **Bits = 8** and some smoothing, as with Resolume.
4. **Start the bridge, send something, and read the universe number from Incoming DMX.** Use that number in the motor rows. Tools disagree on whether the first universe is 0 or 1. The list shows what actually arrives.

**Universe numbers in the bridge**

- **Art-Net:** the bridge uses the 15-bit port-address, `Net × 256 + Subnet × 16 + Universe`. Net 0, Subnet 0, Universe 0 is universe 0. Subnet 1, Universe 2 is universe 18.
- **sACN:** the universe number itself (usually starting at 1). For multicast sACN, the bridge joins the group for each universe in its motor rows when you press Start.

---

## 6. Settings reference

**Motor rows**

| Column | Meaning |
|---|---|
| On | untick to ignore a row |
| Motor | the motor number on the controller (`M1` = 1); each motor may appear once |
| Universe | see section 5 |
| Channel | 16-bit: the coarse channel (1–511); the fine channel is the next one. 8-bit: the channel (1–512) |
| Bits | `16` = two channels, value 0–65535 · `8` = one channel, value 0–255 |
| DMX min / max | the DMX values that mean Steps min / Steps max. Values outside are clamped. **Swap them to reverse the direction** |
| Steps min / max | the positions in steps (negative is fine). Keep them inside the controller's soft limits |
| Speed rps | the fastest the motor may move while following the DMX value |
| Ramp rps/s | acceleration and deceleration |
| DMX now · Target · Sent | live: the value arriving, the step position it maps to, the last position the controller accepted |

**Streaming**

| Setting | Default | Meaning |
|---|---|---|
| Max updates / s | 40 | the most position commands per second, per controller. The RS-485 bus sets the real limit; each controller panel shows an estimate: about 30/s with 6 motors and about 11/s with 32 motors at 115200 baud |
| Deadband (steps) | 2 | smaller changes are not sent, which filters noise |
| Smoothing ms | 0 | glide toward each new value; 100–200 for 8-bit sources, 0 for 16-bit |
| Watchdog ms | 1000 | the controller stops every motor if the bridge goes quiet this long; the bridge sends `PING` to keep it alive. 0 = off |
| Send speed / ramps on start | on | writes each row's Speed and Ramp to the controller when you press Start |

**Save settings** writes `jmc_dmx_bridge.json` (Start saves too). **Export** downloads the current settings as a file and **Import…** loads one, so you can keep one file per show.

**Without the window** (a show PC, a service): run `JMC-DMX-Bridge.exe --headless` (or `python jmc_dmx_bridge.py --headless`). It starts streaming with the saved settings at once. The first run writes a default `jmc_dmx_bridge.json`; edit it, then run again. While it runs, its page is still at `http://127.0.0.1:8090` on that PC for monitoring. Use `--config show2.json` for another file, `--ui-port` for another page port, and `--version` to show the version.

---

## 7. How it moves the motors

- Each update is one command per controller, for example `P:M1=1200,M2=-300`. The bridge waits for that controller's `OK` before sending its next one, so commands never pile up and the newest value always wins. Controllers run side by side: a busy 32-motor bus does not slow down a 4-motor one.
- When every motor on the controller has a row, each update includes all of them. The controller then starts all the drives with one broadcast frame, which is the fastest path.
- The controller skips registers that have not changed. A position update with the same speed is 3 short frames per drive.
- A new target replaces the old one immediately (it does not wait for the motor to arrive). The drive moves toward the newest target with your Speed and Ramp, so the motor follows the DMX like a smoothed fader.

---

## 8. Safety

- **STOP MOTORS** (or **Esc**) sends `B`: every motor brakes on its deceleration ramp at once, and the bridge **pauses**. No position is sent, even if the DMX keeps changing, until you press **Resume**. On Resume the motors move to the current DMX positions at their Speed.
- **Watchdog:** if the PC, Python or the network dies, the controller stops all motors after *Watchdog ms*. Pressing Stop bridge turns the watchdog off on purpose, so the motors don't stop on a planned exit.
- **DMX lost:** if a universe stops arriving for 2 s, its motors hold their position. They do not jump to 0.
- **On Start**, each motor moves to its current DMX position at its Speed. Keep a safe Speed until you trust the setup.
- Set **soft limits** on the controller (`SL`) as the last line of defence, and keep the hardware E-stop on the drives' power.

---

## 9. Troubleshooting

| Symptom | Check |
|---|---|
| *Not answering* on a controller panel | Ping the controller IP. Is the Controller port 8001 and the Reply port the sketch's `PC_UDP_PORT`? Is the app allowed through the Windows Firewall? Is the PC on the same subnet? |
| Incoming DMX list stays empty | Firewall (as above). Is Resolume's Target IP this PC? Is the right network card picked in Resolume's DMX preferences? Another program on port 6454? Try sACN or `127.0.0.1` |
| "Could not start … port" | Another bridge, or a program that receives Art-Net/sACN (Resolume's DMX input, a visualiser), holds the port. Close it or switch protocol |
| Universe arrives with a different number | Use the number shown in Incoming DMX (section 5) |
| Motor moves the wrong way | Swap DMX min and max |
| Motor positions not linear in Resolume | Set Gamma Correction 1.0 in the fixture; keep the Lumiverse brightness/contrast neutral |
| Motor jerks in small steps | 8-bit source: set Smoothing 100–200 ms, or use a 16-bit source |
| Motor lags behind | Raise Speed / Ramp, lower Smoothing, set Resolume's DMX Delay to 0 |
| `ERR:P M3 offline` in the log | Drive 3 isn't answering on RS-485. Check its power, wiring and ID; send `I` on the controller |
| Motors stop and the log shows `EVENT:WATCHDOG:STOP` | The bridge went quiet longer than the watchdog (PC busy, network drop). Raise Watchdog ms, or check the network |

---

## 10. Test without hardware

`python test_bridge.py` checks the packet parsing and the mapping. It then runs the bridge against a simulated controller on this PC with real Art-Net and sACN packets: start-up, 8/16-bit, all-motor updates, deadband, rate limit under flooding, keepalive, lost source, STOP, error recovery and smoothing. It also runs two controllers on one reply port, one with 32 motors. Last, it tests the settings page: every button's API, and that requests from another website or a foreign host name are refused.
