/*==============================================================================
  JMCMotors.h  -  servo/stepper drive control library for Controllino
==============================================================================
  Author : LAGARI A <lagariscience@gmail.com>
  License: MIT
  Repo   : https://github.com/ailagari/JMCMotors

  Complete control stack for integrated servo/stepper drives (IHSS/IHT
  "-RC" series) over Modbus-RTU / RS-485, with a ready-to-use UDP command
  system (JMC_CONTROLLINO_V3).

  Verified against the JMC Modbus-RTU manual V2.2 and the IHSS60-R/RC
  user manual.

  QUICK START (Controllino MAXI + built-in RS-485)
  ------------------------------------------------
    #include <JMCMotors.h>

    JMCController jmc;                       // uses Serial3 (Controllino RS-485)

    void setup() {
      jmc.setStationId(1);                   // derives MAC + IP (192.168.1.101)
      jmc.setMotorCount(6);                  // slave IDs 1..6
      jmc.begin();                           // bus + Ethernet + motors, sends "Motor ready"
    }

    void loop() { jmc.run(); }               // non-blocking: commands + monitor

  That's the whole sketch - the full UDP command system is then live.
  Every motor command exists in a per-motor form (positional slots:
  field N = motor N, empty = skip) and an all-motors form:
    P / PR / PA / PRA   position moves (abs / rel; A = all motors) + '?' queries
    V / VA / VS         run speed (per motor / all) ; VS[:sel] jog
    AD / ADA            accel+decel per motor / all
    S / SD / LS / BS / ZS / DG   status: formats, detailed, limits+sensors,
                                 brakes, zero, register dump
    B[0-3] / L          stop (4 types) / loosen brakes
    Z / ZO / ZOA        zero & position redefinition
    HV / HVA / HO / HOA / H1..H12   homing speed, offset, start
    R / I / MC / X      reset / detect / motor count / reboot
  ...plus automatic EVENT push: limit switches (CW/CCW), origin switch,
  homing complete / error, faults, online/offline.

  Networking is static, no pairing/auto-discovery: set both the
  controller's own IP/port and the PC's IP/port with setNetwork() (or
  setStationId() for the controller side). Events always go to the
  configured PC address; command replies go there and also back to the
  command's sender - see setNetwork() below.

  LAYERS (all in this one header)
  -------------------------------
    JMCBus        shared RS-485 Modbus-RTU transport (CRC-checked, retries)
    JMCMotor      one drive: position/velocity/homing, correct unit scaling
    JMCController N motors + Ethernet UDP command system + event monitor

  UNITS: speed = rps, accel = rps/s, position = encoder steps (drive P17).
==============================================================================*/
#ifndef JMCMotors_h
#define JMCMotors_h

#define JMCMOTORS_VERSION "0.5.0"   // reported by the VER command

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

//==============================================================================
//  JMCBus - hardened Modbus-RTU (RS-485) transport
//==============================================================================

// Result / error codes returned by every bus transaction.
enum JMCResult : uint8_t {
  JMC_OK = 0,          // transaction completed, response verified
  JMC_ERR_TIMEOUT,     // no (or incomplete) response before the timeout
  JMC_ERR_CRC,         // response CRC did not match
  JMC_ERR_EXCEPTION,   // slave returned a Modbus exception frame
  JMC_ERR_BADFRAME,    // wrong slave / function / byte-count in response
  JMC_ERR_PARAM        // invalid argument supplied by the caller
};

class JMCBus {
  public:
    explicit JMCBus(HardwareSerial& serial);

    // Controllino built-in RS-485 (typically Serial3).
    void beginControllino(uint32_t baud);

    // Generic RS-485 transceiver: dePin = driver enable (HIGH = TX).
    // Pass rePin too if DE and RE are wired separately.
    void begin(uint32_t baud, int dePin, int rePin = -1);

    // Tuning (sensible defaults; override any time).
    void setResponseTimeout(uint16_t ms) { _timeoutMs = ms; }
    void setRetries(uint8_t n)           { _retries = n; }
    void setInterFrameGap(uint16_t us)   { _gapUs = us; }

    // Modbus operations. Slave 0 = broadcast (writes only, no response).
    JMCResult writeSingle(uint8_t slave, uint16_t reg, uint16_t value);
    JMCResult writeMultiple(uint8_t slave, uint16_t reg,
                            const uint16_t* words, uint8_t count);
    JMCResult readHolding(uint8_t slave, uint16_t reg,
                          uint16_t* out, uint8_t count);

    uint8_t lastException() const { return _lastException; }
    static const char* resultName(JMCResult r);
    static uint16_t crc16(const uint8_t* data, uint16_t len);

  private:
    HardwareSerial& _serial;
    int      _dePin          = -1;
    int      _rePin          = -1;
    bool     _useControllino = false;
    uint16_t _timeoutMs      = 60;
    uint8_t  _retries        = 2;
    uint16_t _gapUs          = 350;   // >= 3.5 char times @ 115200
    uint32_t _lastActivityUs = 0;
    uint8_t  _lastException  = 0;

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
    enum OperationMode : uint8_t {
      MODE_POSITION = 1,
      MODE_VELOCITY = 3,
      MODE_HOMING   = 6
    };

    enum QuickStopMode : uint8_t {
      QUICK_STOP_DECEL     = 1,   // current deceleration ramp
      QUICK_STOP_FAST      = 2,   // quick-stop ramp
      QUICK_STOP_IMMEDIATE = 3    // stop immediately
    };

    // Decoded status word (0x6041).
    struct MotorStatus {
      bool readyToInit, initComplete, motorEnabled, errorStatus;
      bool driveWorking, quickStopActive, initState, warning;
      bool motorHalt, motorRunning, targetReached, swMechanicalOrigin;
      bool homingComplete, overPositionError, cwLimit, ccwLimit;
      bool valid;                 // false if the status read failed
    };

    // Decoded error register (0x1001).
    struct ErrorFlags {
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

    // Position mode
    JMCResult moveToPosition(long steps, float velocity_rps, bool absolute = true);
    JMCResult preparePositionMove(long steps, float velocity_rps, bool absolute = true);
    JMCResult enableMovement();
    JMCResult startMovement();
    long getPosition();

    // Velocity mode
    JMCResult setVelocity(float velocity_rps);
    JMCResult setTargetVelocity(float velocity_rps);
    JMCResult prepareVelocityMode(float velocity_rps);
    float getVelocity();

    // Leaving velocity mode. A bare 0x6060 mode write while the shaft is
    // turning makes the drive drop the velocity setpoint and latch position:
    // the motor stops DEAD, no ramp, and only then runs the new move.
    // stopVelocityMode() brakes on the normal decel ramp (0x6084) first and is
    // called automatically by setOperationMode(), so P / PR / H / Z after a jog
    // no longer slam the mechanics. The three-part form lets a caller ramp
    // several motors down together (see JMCController::rampDownSelected).
    static const uint16_t BLOCKING_RAMP_CAP_MS = 5000;   // see stopVelocityMode
    JMCResult stopVelocityMode(uint16_t timeout_ms = 0); // 0 = ramp time + margin
    JMCResult beginVelocityRampDown();                   // start the decel ramp
    bool      isSpinning(float eps_rps = 0.2f);          // still turning?
    JMCResult endVelocityRampDown();                     // release halt, ramp done
    bool      velocityModeRunning() const { return _velRunning; }
    uint16_t  rampDownTime_ms() const     { return _rampMs; } // valid after begin

    // Homing
    JMCResult setHomingVelocity(float velocity_rps);      // 0x6099 search speed
    JMCResult setHomingZeroVelocity(float velocity_rps);  // 0x609B zero-find speed
    JMCResult setHomingAccel(float accel_rps_s);
    // Steps to move AFTER the origin sensor triggers (0x607C). Sign = direction.
    JMCResult setHomingOffset(long steps);
    JMCResult performHoming(uint8_t method);       // methods 0..12
    JMCResult setZeroPosition();                   // current position -> 0
    JMCResult setCurrentPosition(long value);      // current position -> value

    // Stopping
    JMCResult stop();                              // controlled halt
    JMCResult quickStop(QuickStopMode mode = QUICK_STOP_FAST);
    JMCResult eStop();

    // Status / diagnostics
    uint16_t    getStatusWord();
    MotorStatus getDetailedStatus();
    ErrorFlags  getErrorFlags();
    bool        isRunning();
    bool        isTargetReached();
    bool        hasFault();

    // Broadcast (slave 0 -> all drives at once)
    JMCResult broadcastPrepareMove(long steps, float velocity_rps, bool absolute = true);
    JMCResult broadcastSetVelocity(float velocity_rps);
    JMCResult broadcastEnableMovement();
    JMCResult broadcastStartMovement();
    JMCResult broadcastStop();

    // Speed-register scale: counts written per 1 rps. Default 10, per the JMC
    // Modbus manual ("setting value is 10 times the actual value") -> 0.1 rps
    // resolution, decimal speeds like 0.3 rps work. If a drive measurably runs
    // 10x FASTER than commanded, it uses whole-rps units: set scale 1
    // (JMCController::setDriveP45(0) does this for all motors).
    void  setSpeedScale(float countsPerRps) { if (countsPerRps > 0) _speedScale = countsPerRps; }
    float speedScale() const                { return _speedScale; }

    uint8_t   slaveId() const    { return _slave; }
    JMCResult lastResult() const { return _last; }
    JMCBus&   bus() const        { return _bus; }

  private:
    JMCBus&   _bus;
    uint8_t   _slave;
    float     _accel = 1.0f;
    float     _decel = 1.0f;
    bool      _formatMSBFirst = true;   // 0x6000 == 0 (JMC default)
    uint16_t  _moveBase = 0x000F;
    bool      _velRunning = false;   // velocity mode commanded, not yet stopped
    uint16_t  _rampMs = 0;           // decel time estimate from the last ramp-down
    float     _speedScale = 10.0f;      // counts per rps (Modbus manual: rps x 10)
    JMCResult _last = JMC_OK;

    int32_t spdReg(float rps) const;    // rps -> speed register counts
    static uint16_t toAccelReg(float rps_s);

    JMCResult write32(uint8_t slave, uint16_t reg, int32_t value);
    JMCResult read32(uint16_t reg, int32_t& out);
    JMCResult ctrl(uint16_t value);
};

//==============================================================================
//  JMCController - N motors + Ethernet UDP command system + event monitor
//==============================================================================
//  Implements the JMC_CONTROLLINO_V3 command set over UDP and a non-blocking
//  background monitor that pushes EVENT packets (CW/CCW limit switches,
//  origin switch, homing complete/error, faults, online/offline) to the PC.
//  See extras/PROTOCOL.md for the full command reference.
//==============================================================================

class JMCController {
  public:
    // RS-485 allows up to 32 Modbus slaves per bus (IHSS60-R/RC datasheet).
    static const uint8_t JMC_MAX_MOTORS = 32;

    explicit JMCController(HardwareSerial& rs485Serial = Serial3);

    // ---- Configuration (call before begin) ---------------------------------
    // Fixed, permanent addressing - no pairing, no auto-adopt. localIp/
    // localPort is this controller's own address; pcIp/pcPort is where
    // every reply and event is sent, for the life of the sketch. Command
    // replies are also sent back to the command's sender (IP + source
    // port) when that differs, so any UDP tool sees its answers.
    void setNetwork(const byte* mac,
                    IPAddress localIp, uint16_t localPort,
                    IPAddress pcIp,    uint16_t pcPort);
    // One number = the unit's whole network identity. Derives a unique MAC
    // (DE:AD:BE:EF:00:<id>) and IP (ipBase + id; default 192.168.1.100+id,
    // so station 1 = .101), listening port 8001. With one ID per board,
    // duplicate MACs/IPs cannot happen. Use IDs 1..99. Still uses whatever
    // PC address/port was last set (setNetwork(), or 192.168.1.100:8000 by
    // default) - call setNetwork() INSTEAD if you need full manual control
    // of both sides at once.
    void setStationId(uint8_t id, IPAddress ipBase = IPAddress(192, 168, 1, 100));

    void setMotorCount(uint8_t count);                 // 1..32 (slave IDs 1..count)
    void setMotionDefaults(float accel_rps_s, float decel_rps_s,
                           float velocity_rps, float homingVel_rps);

    // Speed-unit correction. The library default follows the JMC Modbus
    // manual: speed registers = rps x 10 (0.1 rps resolution, decimals work).
    // Verify on first bring-up with V:1 + VS -> exactly 60 shaft revs/min.
    // If a motor measurably runs 10x FASTER than commanded, its drive uses
    // whole-rps units: call setDriveP45(0) (scale 1, whole rps only).
    // setDriveP45(1) restores the default x10.
    void setDriveP45(uint8_t p45);

    // ---- Per-motor hardcoded settings (optional; motor IDs are 1-based) ----
    // Runtime commands (V / HV / HO) are lost on a power failure. These let
    // the sketch hardcode per-motor values instead: they are re-applied on
    // every boot and whenever a drive comes back online. Call before or
    // after begin(); runtime commands can still override until next reboot.
    // A value <= 0 means "use the global default" (a 0 velocity would be a
    // motor that never moves - the typical mistake of a settings table with
    // fewer rows than motors).
    void setMotorVelocity(uint8_t motorId, float rps);        // run speed
    void setMotorHomingVelocity(uint8_t motorId, float rps);  // homing search speed (HV)
    void setMotorHomingZeroVelocity(uint8_t motorId, float rps); // slow zero-find speed (HVZ; 0 = same as HV)
    void setMotorHomingAccel(uint8_t motorId, float rps_s);   // homing ramp (HACC; 0 = same as accel)
    void setMotorHomingOffset(uint8_t motorId, long steps);   // past sensor
    // Gentle ramps for geared mechanisms (rps/s; resolution 0.1 rps/s).
    void setMotorAccelDecel(uint8_t motorId, float accel_rps_s, float decel_rps_s);

    // ---- Saved settings (SAVE / LOAD commands, Controllino EEPROM) --------
    // V, ACC, DEC, HV, HVZ, HACC, HO and the motor count. When a saved block
    // exists it is applied in begin() AFTER the sketch values, so SAVE:CLEAR
    // (or clearSaved()) returns the unit to what the sketch says.
    bool saveSettings();        // write the live values; false = EEPROM too small
    bool loadSettings();        // apply a saved block; false = none stored
    void clearSaved();
    bool hasSaved();

    // ---- Lifecycle ----------------------------------------------------------
    // Brings up RS-485 (Controllino mode), Ethernet/UDP and all motors.
    // Sends "Motor ready" to the PC when init is done. Returns motors found.
    uint8_t begin(uint32_t baud = 115200);

    // Call from loop(). Never blocks: processes one UDP command if pending,
    // then runs one background-monitor tick.
    void run();

    // ---- Direct access for advanced sketches --------------------------------
    JMCMotor* motor(uint8_t index);                    // 0-based, NULL if bad
    JMCBus&   busRef() { return _bus; }
    uint8_t   motorCount() const { return _count; }

    // Parsed per-motor value fields (P, V, ZO, HV, HO, ACC, DEC, HVZ, HACC).
    struct Fields {
      bool  active[JMC_MAX_MOTORS];        // slot carries something
      bool  query[JMC_MAX_MOTORS];         // that something is '?'
      long  l[JMC_MAX_MOTORS];
      float f[JMC_MAX_MOTORS];
      int   n, nq;                         // active count, query count
    };

  private:
    // Monitor timing. Limit-switch latency ~= count x (tick + ~5 ms read):
    // 6 motors @ 8 ms tick -> a CW/CCW hit is pushed in well under 100 ms.
    static const uint32_t MONITOR_TICK_MS  = 8;
    static const uint32_t OFFLINE_RETRY_MS = 3000;
    static const uint32_t RAMP_POLL_MS     = 50;    // velocity re-read while braking
    static const uint16_t PROBE_TIMEOUT_MS = 12;
    static const int      CMD_BUFFER_SIZE  = 512;   // fits P: with 32 positions

    struct MotorCache {
      bool     online;
      uint8_t  failCnt;
      uint16_t status;
      bool     cw, ccw, fault, zeroSet;
      // Status bits 10/12/13 change meaning with the drive's operation mode
      // (manual, 0x6041 table): bit12 is "homing complete" ONLY in homing
      // mode - in position mode it means "ready for a new set-point". So the
      // controller remembers the mode it last commanded and decodes
      // accordingly. 'sw' is the origin-switch input (bit11), 'homed' is
      // latched when a homing run completes and cleared by a drive re-init.
      uint8_t  mode;             // JMCMotor::OperationMode last commanded
      bool     sw;               // origin switch input (bit11) last seen
      bool     homed;            // a homing run completed since last re-init
      bool     homingArmed;      // H<n> issued, waiting for completion
      bool     homingBusySeen;   // bit12 observed low after the start
      bool     homeErr;          // homing error (bit13 in homing mode) seen
      uint32_t homingStart;      // millis() when H<n> was issued
    };

    // A P/PR/PA that arrives while motors are jogging cannot be launched right
    // away: the drives must brake to standstill first (see rampDownBegin).
    // The move waits here and servicePendingMove() launches it from run(), so
    // the loop keeps answering commands and pushing events while they brake.
    struct PendingMove {
      bool     active    = false;
      bool     absolute  = true;
      bool     broadcast = false;         // PA vs P/PR
      bool     braking[JMC_MAX_MOTORS];   // motors still being waited on
      String   args;                      // the command, re-run once at rest
      uint32_t deadline = 0;              // ms: run it anyway past this point
      uint32_t lastPoll = 0;
    };

    JMCBus      _bus;
    JMCMotor*   _m[JMC_MAX_MOTORS];
    uint8_t     _count = 6;
    float       _storedVel[JMC_MAX_MOTORS];
    MotorCache  _cache[JMC_MAX_MOTORS];
    uint32_t    _offlineTry[JMC_MAX_MOTORS];

    EthernetUDP _udp;
    byte        _mac[6];
    IPAddress   _localIp, _pcIp;        // both fixed; set via setNetwork()/setStationId()
    uint16_t    _localPort = 8001, _pcPort = 8000;
    uint8_t     _stationId = 0;

    // Gentle factory defaults: 0.3 rps = 18 RPM run speed, soft 0.5 rps/s
    // ramps, slow 0.3 rps homing. Raise per installation with
    // setMotionDefaults() or at runtime with the V / VA / HV commands.
    float _defAccel = 0.5f, _defDecel = 0.5f;
    float _defVel   = 0.3f, _defHomingVel = 0.3f;
    float _speedScale = 10.0f;             // counts per rps (Modbus manual: rps x 10)
    float _homingVel[JMC_MAX_MOTORS];      // per-motor homing speed (NAN = default)
    float _homingZeroVel[JMC_MAX_MOTORS];  // slow zero-find speed (NAN = follow HV)
    float _homingAccel[JMC_MAX_MOTORS];    // homing ramp rps/s (NAN = follow accel)
    float _accelSet[JMC_MAX_MOTORS];       // per-motor accel rps/s (NAN = default)
    float _decelSet[JMC_MAX_MOTORS];       // per-motor decel rps/s (NAN = default)
    long  _homingOffset[JMC_MAX_MOTORS];   // steps after sensor trigger (HO cmd)
    bool  _chunked = false;                // this reply already spilled into :MORE packets
    void applyHomingParams(uint8_t i);     // write HV/HVZ/HACC/HO of motor i to its drive

    PendingMove _pending;
    uint32_t _lastTick = 0;
    int      _monIdx   = -1;
    char     _cmdBuf[CMD_BUFFER_SIZE];

    // Comms. Events go only to the fixed, configured PC address; command
    // replies go there AND back to whoever sent the last command, so any UDP
    // tool sees its answers without having to listen on PC_PORT.
    IPAddress _senderIp;
    uint16_t  _senderPort = 0;             // 0 = no command received yet
    void sendTo(IPAddress ip, uint16_t port, const String& msg);
    void reply(const String& msg);
    void reply(const __FlashStringHelper* msg) { reply(String(msg)); }
    void pushEvent(const String& msg) { sendTo(_pcIp, _pcPort, msg); }
    void pushEvent(const __FlashStringHelper* msg) { sendTo(_pcIp, _pcPort, String(msg)); }
    // Long per-motor lists go out in several packets so a 32-motor reply never
    // needs a kilobyte of String on an 8 KB AVR. Returns true when a packet
    // was sent and 'out' restarted with the header (caller resets separators).
    static const uint16_t REPLY_CHUNK_CHARS = 300;
    bool flushIfLong(String& out, const String& header);

    // Parsing (v0.5 grammar): CMD:<slots or M-names> = individual motors,
    // CMDA[:value] = all motors. See parseValues / parseSelection.
    int  parseValues(const String& csv, Fields& F, bool all);
    int  parseSelection(const String& csv, bool* sel, bool all);
    bool selectionError(const __FlashStringHelper* cmd, int n);   // replies ERR, true if n <= 0
    bool valuesError(const __FlashStringHelper* cmd, int n, bool all);
    static bool isMotorCommand(const String& base);
    String replyChunkEnd(const String& out);   // appends :END when the reply was chunked

    // Velocity -> position/homing handoff: ramp the selected motors down
    // together on their decel ramps, so no drive gets a dead-stop mode
    // switch and they all resume from standstill in the same instant.
    bool rampDownBegin(const bool* sel, bool* braking, uint32_t& deadline);
    bool rampDownDone(const bool* braking);
    void rampDownEnd(const bool* braking);
    void rampDownSelected(const bool* sel);   // blocking form (Z / ZO / H)

    // Deferred launch of a P/PR/PA that had to wait for a jog to brake.
    void servicePendingMove();
    void cancelPendingMove();
    void launchMove(const bool* sel, const long* target, bool absolute, bool broadcast);

    // Engine
    void handleUdp();
    void serviceMonitor();
    bool initMotorSlot(uint8_t i);   // allocate + probe + init one motor slot

    // Mode bookkeeping for the mode-aware status decode (see MotorCache).
    void noteMode(const bool* sel, uint8_t mode);
    void noteModeOne(uint8_t i, uint8_t mode);

    // Command handlers (v0.5). 'all' = the A-suffixed form was used.
    void handlePosition(const String& args, bool absolute, bool all);
    void replyPositions(const bool* sel);
    void handleStatus(const String& args, bool all);
    void handleStatusDetail(const String& args, bool all);
    String humanStatus(int i);
    String detailStatus(int i);
    String compactStatus(int i);
    void   replyMulti(const String& packet, bool last);   // :MORE / :END markers
    void handleBrake(int type, const String& args, bool all);
    void handleLoosen(const String& args, bool all);
    void handleBrakeStatus(const String& args, bool all);
    void handleZero(const String& args, bool all);
    void handleZeroOffset(const String& args, bool all);
    void handleZeroStatus(const String& args, bool all);
    void handleLimitStatus(const String& args, bool all);
    void handleReset(const String& args, bool all);
    void handleInit(const String& args, bool all);
    void handleMotorCount(const String& args);
    void handleDiagnose(const String& args, bool all);
    String diagnose(uint8_t i);
    // Per-motor float settings sharing one handler: V, ACC, DEC, HV, HVZ, HACC.
    enum FloatSetting : uint8_t { SET_V, SET_ACC, SET_DEC, SET_HV, SET_HVZ, SET_HACC };
    void handleFloatSetting(FloatSetting which, const String& args, bool all);
    void handleVelocityStart(const String& args, bool all);
    void handleHomingOffset(const String& args, bool all);
    void handleHoming(int method, const String& args, bool all);
    void handleSave(const String& args);
    void handleLoad();
};

#endif // JMCMotors_h
