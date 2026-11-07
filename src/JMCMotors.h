/*==============================================================================
  JMCMotors.h  -  JMC integrated servo/stepper drives over Modbus-RTU (RS-485)
==============================================================================
  Author : LAGARI A <lagariscience@gmail.com>
  License: MIT
  Repo   : https://github.com/ailagari/JMCMotors
  Docs   : https://ailagari.github.io/JMCMotors/  (interactive command reference)

  Drives: JMC IHSS / IHT "-RC" series (Modbus-RTU, RS-485 two-wire).
  Boards: CONTROLLINO MAXI / MEGA, Arduino Uno / Nano / Mega, ESP32.
  Verified against the JMC Modbus-RTU manual V2.2.

  QUICK START - CONTROLLINO MAXI, commands over Ethernet UDP
  ---------------------------------------------------------
    #include <Ethernet.h>
    #include <JMCMotors.h>
    #include <JMCLinkUDP.h>

    EthernetUDP   udp;
    JMCLinkUDP    pcLink(udp);   // not "link": that name clashes on ESP32
    JMCBus        bus(Serial3);              // built-in RS-485
    JMCController jmc(bus, pcLink);

    void setup() {
      byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
      Ethernet.begin(mac, IPAddress(192, 168, 1, 101));
      pcLink.begin(8001, IPAddress(192, 168, 1, 100), 8000);
      bus.beginControllino(115200);
      jmc.setMotorCount(6);                  // slave IDs 1..6
      jmc.begin();                           // detects motors, sends "Motor ready"
    }
    void loop() { jmc.run(); }               // non-blocking: commands + monitor

  Uno and ESP32 sketches: see the examples (TTL-to-RS-485 module on any
  serial port, commands over USB serial, Ethernet or WiFi).

  LAYERS
    JMCBus        RS-485 Modbus-RTU transport (CRC-checked, retries, timeouts)
    JMCMotor      one drive: position / velocity / homing, unit scaling
    JMCController N drives + text command system + event monitor + watchdogs
    JMCLink       where commands come from (JMCLinkUDP.h, JMCLinkSerial.h)

  UNITS: speed = rps, accel = rps/s, position = encoder steps (drive P17).
==============================================================================*/
#ifndef JMCMotors_h
#define JMCMotors_h

#include <Arduino.h>
#include "JMCConfig.h"
#include "JMCLink.h"

//==============================================================================
//  Result codes
//==============================================================================
enum JMCResult : uint8_t {
  JMC_OK = 0,          // transaction completed, response verified
  JMC_ERR_TIMEOUT,     // no (or incomplete) response before the timeout
  JMC_ERR_CRC,         // response CRC did not match
  JMC_ERR_EXCEPTION,   // slave returned a Modbus exception frame
  JMC_ERR_BADFRAME,    // wrong slave / function / byte-count in response
  JMC_ERR_PARAM        // invalid argument supplied by the caller
};

//==============================================================================
//  Processor watchdog (opt-in). AVR: hardware WDT. ESP32: task watchdog.
//  Fed automatically by the library while it waits on the bus or a ramp.
//  Note: some old ATmega2560 bootloaders reboot-loop after a watchdog reset;
//  test once on your board before relying on it.
//==============================================================================
namespace JMCWatchdog {
  bool begin(uint16_t timeoutMs);   // false when the board cannot do it
  void feed();
  bool active();
}

//==============================================================================
//  JMCBus - Modbus-RTU over an RS-485 transceiver
//==============================================================================
class JMCBus {
  public:
    // Any serial port: HardwareSerial (Serial1/2/3), SoftwareSerial, ...
    explicit JMCBus(Stream& port);

    // Start the serial port yourself first (Serial2.begin(115200, SERIAL_8N1,
    // RX, TX) on ESP32, mySoft.begin(38400) for SoftwareSerial ...), then:
    //   dePin: transceiver driver-enable (HIGH = transmit), -1 for modules
    //          with automatic direction control.
    //   rePin: receiver-enable if wired separately (LOW = receive), else -1.
    // baud must match the port and the drives (P41); it sets the timing.
    void begin(uint32_t baud, int8_t dePin = -1, int8_t rePin = -1);

#if JMC_HAS_CONTROLLINO_RS485
    // CONTROLLINO MAXI / MEGA built-in RS-485: starts Serial3 and the
    // on-board transceiver. Construct the bus with JMCBus bus(Serial3).
    void beginControllino(uint32_t baud = 115200);
#endif

    void setResponseTimeout(uint16_t ms) { _timeoutMs = ms; }
    void setRetries(uint8_t n)           { _retries = n; }
    void setInterFrameGap(uint16_t us)   { _gapUs = us; }
    // For transceivers that hear their own transmission (RE held low):
    // discard the echoed request before reading the reply.
    void setEchoSuppression(bool on)     { _echo = on; }

    uint16_t responseTimeout() const { return _timeoutMs; }
    uint8_t  retries() const         { return _retries; }
    uint32_t baud() const            { return _baud; }
    uint16_t frameTimeMs(uint8_t bytes) const;   // time on the wire, rounded up

    // Modbus operations. Slave 0 = broadcast (writes only, no response).
    JMCResult writeSingle(uint8_t slave, uint16_t reg, uint16_t value);
    JMCResult writeMultiple(uint8_t slave, uint16_t reg,
                            const uint16_t* words, uint8_t count);
    JMCResult readHolding(uint8_t slave, uint16_t reg,
                          uint16_t* out, uint8_t count);

    uint8_t lastException() const { return _lastException; }
    // Transactions that failed after all retries (wraps at 65535), and the
    // result of the most recent one. Lets a caller check that a whole
    // sequence of writes really reached the drive.
    uint16_t  failures() const    { return _failures; }
    JMCResult lastFailure() const { return _lastFailure; }
    static const __FlashStringHelper* resultName(JMCResult r);
    static uint16_t crc16(const uint8_t* data, uint16_t len);

  private:
    Stream*  _port;
    int8_t   _dePin          = -1;
    int8_t   _rePin          = -1;
    bool     _controllino    = false;
    bool     _echo           = false;
    uint32_t _baud           = 115200;
    uint16_t _timeoutMs      = 60;
    uint8_t  _retries        = 2;
    uint16_t _gapUs          = 350;    // >= 3.5 character times
    uint32_t _lastActivityUs = 0;
    uint8_t  _lastException  = 0;
    uint16_t  _failures      = 0;
    JMCResult _lastFailure   = JMC_OK;
    JMCResult failed(JMCResult r) { _failures++; _lastFailure = r; return r; }

    void txEnable();
    void rxEnable();
    void interFrameGuard();
    void drainRx();
    JMCResult transact(uint8_t* req, uint8_t reqLen,
                       uint8_t* resp, uint8_t expectedRespLen,
                       uint8_t slave, uint8_t funcCode);
};

//==============================================================================
//  JMCMotor - one drive on the bus
//==============================================================================

// JMC Modbus register map (manual section 8, "Register list").
namespace JMCReg {
  const uint16_t ERROR       = 0x1001; // RO  error register
  const uint16_t FORMAT      = 0x6000; // RW  32-bit word order (0 = MSB first)
  const uint16_t QUICKSTOP   = 0x605A; // RW  quick-stop code
  const uint16_t HALTCODE    = 0x605D; // RW  halt code
  const uint16_t CONTROL     = 0x6040; // WO  control word
  const uint16_t STATUS      = 0x6041; // RO  status word
  const uint16_t MODE        = 0x6060; // WO  modes of operation
  const uint16_t MODE_DISP   = 0x6061; // RO  mode display
  const uint16_t POS_ACTUAL  = 0x6064; // RO  position actual value (INT32)
  const uint16_t VEL_ACTUAL  = 0x606C; // RO  velocity actual value (INT32)
  const uint16_t TARGET_POS  = 0x607A; // RW  target position (INT32)
  const uint16_t HOME_OFFSET = 0x607C; // RW  home offset (INT32)
  const uint16_t TARGET_VEL  = 0x6081; // RW  target velocity (INT32)
  const uint16_t ACCEL       = 0x6083; // RW  acceleration (UINT16)
  const uint16_t DECEL       = 0x6084; // RW  deceleration (UINT16)
  const uint16_t QS_DECEL    = 0x6085; // RW  quick-stop deceleration (UINT16)
  const uint16_t HOME_METHOD = 0x6098; // RW  homing method (INT16)
  const uint16_t HOME_SPEED  = 0x6099; // RW  speed to find mech. origin (U32)
  const uint16_t HOME_ACCEL  = 0x609A; // RW  homing acceleration (UINT16)
  const uint16_t HOME_ZEROSPD= 0x609B; // RW  speed to find zero origin (U32)
}

class JMCMotor {
  public:
    enum OperationMode : uint8_t { MODE_POSITION = 1, MODE_VELOCITY = 3, MODE_HOMING = 6 };
    enum QuickStopMode : uint8_t {
      QUICK_STOP_DECEL     = 1,   // current deceleration ramp
      QUICK_STOP_FAST      = 2,   // quick-stop ramp
      QUICK_STOP_IMMEDIATE = 3    // stop immediately
    };

    struct MotorStatus {           // decoded status word (0x6041)
      bool readyToInit, initComplete, motorEnabled, errorStatus;
      bool driveWorking, quickStopActive, initState, warning;
      bool motorHalt, motorRunning, targetReached, swMechanicalOrigin;
      bool homingComplete, overPositionError, cwLimit, ccwLimit;
      bool valid;                  // false if the status read failed
    };
    struct ErrorFlags {            // decoded error register (0x1001)
      bool generic, current, voltage, temperature;
      bool communication, overPosition, phaseLoss;
      uint16_t raw;
      bool valid;
    };

    JMCMotor(JMCBus& bus, uint8_t slaveId);

    JMCResult begin(float acceleration = 1.0f, float deceleration = 1.0f);
    bool isPresent();

    // Configuration
    JMCResult setAccelDecel(float accel_rps_s, float decel_rps_s);
    JMCResult setOperationMode(OperationMode mode);
    JMCResult loosenBrake();
    JMCResult enable();
    JMCResult alarmReset();

    // Position mode. A move is prepare -> enableMovement (bit4 rising edge
    // starts it) -> startMovement (bit4 low again, ready for the next one).
    JMCResult moveToPosition(long steps, float velocity_rps, bool absolute = true);
    JMCResult preparePositionMove(long steps, float velocity_rps, bool absolute = true);
    // Synchronised moves: raw register values (speed counts, ramp counts)
    // chosen by the caller. The motor's own ramps are restored automatically
    // before its next normal move or jog.
    JMCResult prepareScaledMove(long steps, int32_t speedReg, uint16_t accelReg,
                                uint16_t decelReg, bool absolute = true);
    JMCResult enableMovement();
    JMCResult startMovement();
    long getPosition();
    long getTargetPosition();      // 0x607A: what a relative move adds to

    // Velocity mode
    JMCResult setVelocity(float velocity_rps);
    JMCResult setTargetVelocity(float velocity_rps);
    JMCResult prepareVelocityMode(float velocity_rps);
    float getVelocity();

    // Leaving velocity mode: a mode write while the shaft turns stops it dead,
    // so brake on the decel ramp first (done automatically by
    // setOperationMode). The three-part form lets a caller brake several
    // motors together (see JMCController).
    static const uint16_t BLOCKING_RAMP_CAP_MS = 5000;
    JMCResult stopVelocityMode(uint16_t timeout_ms = 0);
    JMCResult beginVelocityRampDown();
    bool      isSpinning(float eps_rps = 0.2f);
    JMCResult endVelocityRampDown();
    bool      velocityModeRunning() const { return _velRunning; }
    uint16_t  rampDownTime_ms() const     { return _rampMs; }

    // Homing
    JMCResult setHomingVelocity(float velocity_rps);      // 0x6099 search speed
    JMCResult setHomingZeroVelocity(float velocity_rps);  // 0x609B zero-find speed
    JMCResult setHomingAccel(float accel_rps_s);          // 0x609A
    JMCResult setHomingOffset(long steps);                // 0x607C, sign = direction
    JMCResult performHoming(uint8_t method);              // methods 0..12
    JMCResult setZeroPosition();                          // current position -> 0
    JMCResult setCurrentPosition(long value);             // current position -> value

    // Stopping
    JMCResult stop();                                     // controlled halt (bit8)
    JMCResult quickStop(QuickStopMode mode = QUICK_STOP_FAST);
    JMCResult eStop();

    // Status / diagnostics
    uint16_t    getStatusWord();
    MotorStatus getDetailedStatus();
    ErrorFlags  getErrorFlags();
    bool        isRunning();
    bool        isTargetReached();
    bool        hasFault();

    // Broadcast (slave 0 -> all drives at once). The launch frames use this
    // motor's prepared control word, so prepare it (or any motor) first.
    JMCResult broadcastPrepareMove(long steps, float velocity_rps, bool absolute = true);
    JMCResult broadcastSetVelocity(float velocity_rps);
    JMCResult broadcastEnableMovement();
    JMCResult broadcastStartMovement();
    JMCResult broadcastStop();

    // Unit conversion. Speed registers = rps x scale (10 per the JMC manual,
    // 1 if a drive measurably runs 10x too fast: JMCController::setDriveP45(0)).
    void     setSpeedScale(float countsPerRps) { if (countsPerRps > 0) _speedScale = countsPerRps; }
    float    speedScale() const                { return _speedScale; }
    int32_t  speedToReg(float rps) const;
    static uint16_t accelToReg(float rps_s);   // register = rps/s x 10

    uint8_t   slaveId() const    { return _slave; }
    JMCResult lastResult() const { return _last; }
    JMCBus&   bus() const        { return _bus; }
    float     accel() const      { return _accel; }
    float     decel() const      { return _decel; }

  private:
    JMCBus&   _bus;
    uint8_t   _slave;
    float     _accel = 1.0f;
    float     _decel = 1.0f;
    bool      _formatMSBFirst = true;   // 0x6000 == 0 (JMC default)
    bool      _velRunning = false;      // velocity mode commanded, not yet stopped
    bool      _halted     = false;      // last control word: enabled with halt (bit 8)
    bool      _rampsScaled = false;     // a synchronised move changed 0x6083/84
    uint16_t  _moveBase = 0x000F;
    uint16_t  _rampMs = 0;
    float     _speedScale = 10.0f;
    JMCResult _last = JMC_OK;

    void restoreRamps();
    JMCResult write32(uint8_t slave, uint16_t reg, int32_t value);
    JMCResult read32(uint16_t reg, int32_t& out);
    JMCResult ctrl(uint16_t value);
};

//==============================================================================
//  JMCOut - fixed-size reply builder (no String, no heap)
//==============================================================================
class JMCOut {
  public:
    explicit JMCOut(JMCLink& link) : _link(link) {}

    // A reply to the current command. header (flash string, may be NULL) is
    // repeated at the start of every continuation packet of a long list.
    void reply(const __FlashStringHelper* header = nullptr);
    void event();                                  // an unsolicited event
    void setTag(const char* tag);                  // "#12 " prefix for replies ("" = none)

    void p(const __FlashStringHelper* s);          // flash string
    void s(const char* s);                         // RAM string
    void c(char ch);
    void n(long v);                                // decimal
    void hx(uint16_t v);                           // lowercase hex digits
    void f(float v, uint8_t decimals);             // fixed-point
    void m(uint8_t index);                         // "M<index+1>"

    // Lists: item() before each element (comma between elements);
    // need(bytes) continues in a new packet when the element would not fit.
    void item(uint8_t reserve = 64);
    void need(uint8_t bytes);
    void resetList() { _first = true; }

    void send();                                   // finish (":END" if continued)
    void sendPart(bool last);                      // one-packet-per-motor reports
    size_t length() const { return _len; }

  private:
    JMCLink& _link;
    char     _buf[JMC_REPLY_BUFFER + 1];
    size_t   _len = 0;
    bool     _isEvent = false;
    bool     _first = true;
    bool     _continued = false;
    const __FlashStringHelper* _header = nullptr;
    char     _tag[12] = {0};

    void begin();
    void flushMore();
};

//==============================================================================
//  JMCController - N drives + text command system + event monitor
//==============================================================================
class JMCController {
  public:
    static const uint8_t MAX_MOTORS = JMC_MAX_MOTORS;

    JMCController(JMCBus& bus, JMCLink& link);

    // ---- configuration (before begin) ---------------------------------------
    void setMotorCount(uint8_t count);                 // 1..MAX_MOTORS, slave IDs 1..count
    void setMotionDefaults(float accel_rps_s, float decel_rps_s,
                           float velocity_rps, float homingVel_rps);
    // Speed-unit correction: setDriveP45(0) only if a motor measurably runs
    // 10x faster than commanded (V:1 + VS should give exactly 60 rev/min).
    void setDriveP45(uint8_t p45);

    // Per-motor values (motor IDs are 1-based). A value <= 0 means "use the
    // global default". Re-applied on every boot and whenever a drive comes
    // back online.
    void setMotorVelocity(uint8_t motorId, float rps);
    void setMotorAccelDecel(uint8_t motorId, float accel_rps_s, float decel_rps_s);
    void setMotorHomingVelocity(uint8_t motorId, float rps);       // HV
    void setMotorHomingZeroVelocity(uint8_t motorId, float rps);   // HVZ (0 = same as HV)
    void setMotorHomingAccel(uint8_t motorId, float rps_s);        // HACC (0 = same as accel)
    void setMotorHomingOffset(uint8_t motorId, long steps);        // HO
    // Software position limits: moves outside [minSteps, maxSteps] are
    // refused and jogs are stopped at the limit. Homing is not limited.
    void setMotorSoftLimits(uint8_t motorId, long minSteps, long maxSteps);
    void clearMotorSoftLimits(uint8_t motorId);

    // Communication watchdog: when no command arrives for timeoutMs, every
    // motor is stopped on its decel ramp and EVENT:WATCHDOG is sent. Any
    // command (PING is the cheapest) feeds it. 0 = off (default).
    void setCommandWatchdog(uint16_t timeoutMs);
    // Processor watchdog (see JMCWatchdog). Call after begin().
    bool enableHardwareWatchdog(uint16_t timeoutMs = 4000) { return JMCWatchdog::begin(timeoutMs); }

    // ---- saved settings (SAVE / LOAD commands) -------------------------------
    // The sketch's values are the admin's configuration. SAVE keeps changes
    // made over the link on a deployed unit; the block is applied at boot
    // only while the sketch values it was saved under are still the ones
    // flashed, so re-flashing different values always wins.
    bool saveSettings();
    bool loadSettings(bool force = false);
    void clearSaved();
    bool hasSaved();
    bool savedMatchesSketch();

    // ---- lifecycle -------------------------------------------------------------
    // Start the bus (and the network, if any) first. Detects and initialises
    // the motors, applies a matching SAVEd block, sends "Motor ready".
    // Returns the number of motors found.
    uint8_t begin();
    // Call from loop(): one command, one monitor tick, watchdog service.
    void run();

    // ---- direct access ------------------------------------------------------------
    JMCMotor* motor(uint8_t index);                    // 0-based, NULL if out of range
    JMCBus&   busRef()             { return _bus; }
    uint8_t   motorCount() const   { return _count; }
    bool      isOnline(uint8_t index) const { return index < _count && _cache[index].online; }

    // Parsed per-motor fields (public for the unit tests).
    struct Fields {
      bool  active[JMC_MAX_MOTORS];
      bool  query[JMC_MAX_MOTORS];
      long  l[JMC_MAX_MOTORS];
      float f[JMC_MAX_MOTORS];
      long  l2[JMC_MAX_MOTORS];     // second number of an "a/b" pair
      bool  pair[JMC_MAX_MOTORS];   // field was "a/b"
      bool  off[JMC_MAX_MOTORS];    // field was OFF
      int   n, nq;
    };

  private:
    static const uint32_t MONITOR_TICK_MS  = 8;
    static const uint32_t OFFLINE_RETRY_MS = 3000;
    static const uint32_t RAMP_POLL_MS     = 50;

    struct MotorCache {
      uint16_t status;
      uint8_t  failCnt;
      uint8_t  mode;               // JMCMotor::OperationMode last commanded
      uint32_t homingStart;
      bool online : 1;
      bool cw : 1, ccw : 1, sw : 1, fault : 1;
      bool zeroSet : 1;
      bool homed : 1, homingArmed : 1, homingBusySeen : 1, homeErr : 1;
      bool jogNeg : 1;             // running jog direction (V < 0), for soft limits
    };

    struct PendingMove {
      bool     active = false;
      uint8_t  kind = 0;           // 0 = P/PR, 1 = PS/PRS
      bool     absolute = true;
      bool     all = false;
      bool     braking[JMC_MAX_MOTORS];
      char     args[JMC_CMD_BUFFER];
      char     tag[12];
      uint32_t deadline = 0;
      uint32_t lastPoll = 0;
    };
    char _tag[12] = {0};                  // "#12 " of the command being run

    JMCBus&    _bus;
    JMCLink&   _link;
    JMCOut     _out;
    JMCMotor*  _m[JMC_MAX_MOTORS];       // NULL until the slot is first used
    // Fixed storage for the motor objects: no heap use at all.
    alignas(JMCMotor) uint8_t _pool[JMC_MAX_MOTORS][sizeof(JMCMotor)];
    JMCMotor* slot(uint8_t i);            // create motor i in _pool on first use
    uint8_t    _count = 6;
    MotorCache _cache[JMC_MAX_MOTORS];
    uint32_t   _offlineTry[JMC_MAX_MOTORS];

    float _defAccel = 0.5f, _defDecel = 0.5f;
    float _defVel   = 0.3f, _defHomingVel = 0.3f;
    float _speedScale = 10.0f;
    float _storedVel[JMC_MAX_MOTORS];
    float _accelSet[JMC_MAX_MOTORS];
    float _decelSet[JMC_MAX_MOTORS];
    float _homingVel[JMC_MAX_MOTORS];
    float _homingZeroVel[JMC_MAX_MOTORS];
    float _homingAccel[JMC_MAX_MOTORS];
    long  _homingOffset[JMC_MAX_MOTORS];
    long  _softMin[JMC_MAX_MOTORS];       // softMin > softMax = no limits
    long  _softMax[JMC_MAX_MOTORS];

    uint16_t _wdMs = 0;                   // command watchdog, 0 = off
    uint32_t _lastCmdMs = 0;
    bool     _wdTripped = false;
    uint16_t _sketchCrc = 0;

    PendingMove _pending;
    uint32_t _lastTick = 0;
    int      _monIdx = -1;
    char     _cmd[JMC_CMD_BUFFER];

    // engine
    void handleCommand(char* line);
    void serviceMonitor();
    void serviceWatchdog();
    bool initMotorSlot(uint8_t i);
    void applyHomingParams(uint8_t i);
    void resolveDefaults(uint8_t i);
    uint16_t sketchFingerprint();
    void noteMode(const bool* sel, uint8_t mode);
    void noteModeOne(uint8_t i, uint8_t mode);
    void idleDelay(uint16_t ms);

    // parsing
    enum : uint8_t { PV_INT = 1, PV_PAIR = 2, PV_OFF = 4 };
    int  parseValues(const char* csv, Fields& F, bool all, uint8_t flags);
    int  parseSelection(const char* csv, bool* sel, bool all);
    bool selectionError(const __FlashStringHelper* cmd, int n);
    bool valuesError(const __FlashStringHelper* cmd, int n, bool all);
    void say(const __FlashStringHelper* msg);
    void notAvailable(const char* head);
    void errHead(const __FlashStringHelper* cmd);      // reply "ERR:<cmd>"
    void listSel(const bool* sel);                     // "1,3,4"
    void motorRange();                                 // " motor must be M1..Mn"
    void evHead(const __FlashStringHelper* what, uint8_t i);   // event "EVENT:X:Mi"
    void stWord(uint16_t s);                           // " ST=0x...."
    char    _badText[16];                 // offending field for "bad value" errors
    uint8_t _badMotor = 0;

    // motion helpers
    bool rampDownBegin(const bool* sel, bool* braking, uint32_t& deadline);
    bool rampDownDone(const bool* braking);
    void rampDownEnd(const bool* braking);
    void rampDownSelected(const bool* sel);
    void servicePendingMove();
    void cancelPendingMove();
    void launchPrepared(const bool* sel, uint8_t n, bool* fail = nullptr);
    bool busFailed(const __FlashStringHelper* cmd, const bool* fail, const __FlashStringHelper* note);
    bool onlineFilter(const __FlashStringHelper* cmd, bool* sel, bool all);
    bool checkSoftLimits(const __FlashStringHelper* cmd, const bool* sel, const long* target);
    void stopAll(uint8_t type);
    bool stopMotors(uint8_t type, const bool* sel, bool every, bool* fail);

    // command handlers ('all' = the A-suffixed form)
    void cmdPosition(const char* args, bool absolute, bool all);
#if JMC_FEATURE_SYNC
    void cmdSyncMove(const char* args, bool absolute, bool all);
#endif
    void replyPositions(const bool* sel);
    void cmdFloat(uint8_t which, const char* args, bool all);
    float* floatSetting(uint8_t which);
    void cmdJog(const char* args, bool all);
    void cmdBrake(int type, const char* args, bool all);
    void cmdLoosen(const char* args, bool all);
#if JMC_FEATURE_EXTRA_STATUS
    void cmdBrakeStatus(const char* args, bool all);
    void cmdZeroStatus(const char* args, bool all);
#endif
    void cmdZero(const char* args, bool all);
    void cmdZeroOffset(const char* args, bool all);
    void cmdHomingOffset(const char* args, bool all);
    void cmdHoming(int method, const char* args, bool all);
    void cmdSoftLimits(const char* args, bool all);
    void outLimits(uint8_t i);                         // "min/max" or "OFF"
    bool setPositions(const __FlashStringHelper* cmd, const bool* sel, const long* value);
    JMCResult reinitMotor(uint8_t i);
    uint16_t  probeStatus(JMCMotor* mt);
    void cmdStatus(const char* args, bool all);
    void cmdLimitStatus(const char* args, bool all);
    void cmdReset(const char* args, bool all);
    void cmdInit(const char* args, bool all);
    void cmdMotorCount(const char* args);
    void cmdWatchdog(const char* args);
    void cmdSave(const char* args);
    void cmdLoad();
    void outCompact(uint8_t i);
#if JMC_FULL_REPORTS
    void cmdStatusDetail(const char* args, bool all);
    void cmdDiagnose(const char* args, bool all);
    void outHuman(uint8_t i);
    void outDetail(uint8_t i);
    void outDiagnose(uint8_t i);
#endif
};

#endif // JMCMotors_h
