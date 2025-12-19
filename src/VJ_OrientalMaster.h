#pragma once

#include <Arduino.h>
#include <ModbusMaster.h>

// VJ_OrientalMaster
// - Controls up to 10 Oriental Motor AZ-series drives (AZD-C(D)/AZD-CX) via Modbus RTU.
// - Single shared RS-485 bus (Stream).

class VJ_OrientalMaster {
public:
  static constexpr uint8_t MAX_MOTORS = 10;

  enum Input : uint8_t {
    START,
    ZHOME,
    STOP,
    FREE,
    RESET
  };

  enum Output : uint8_t {
    READY,
    ALARM,
    BUSY,
    MOVE,
    INPOS,
    RAW
  };

  struct SMPFields {
    bool hasOpType{false};
    uint16_t opType{0};

    bool hasPos{false};
    int32_t pos{0};

    bool hasSpd{false};
    int32_t spd{0};

    bool hasAcc{false};
    int32_t acc{0};

    bool hasDec{false};
    int32_t dec{0};

    bool hasCur{false};
    uint16_t cur{0};

    bool hasOpDataNo{false};
    uint16_t opDataNo{0};
  };

  using EventCallback = void (*)(uint8_t id, const char* msg);

  VJ_OrientalMaster();

  bool begin(Stream& bus);

  void setEventCallback(EventCallback cb);
  void setPollIntervalMs(uint32_t intervalMs);
  void setInterframeDelayMs(uint16_t delayMs);

  // Reduce blocking in case of missing slave response (prevents WDT in bad wiring cases).
  // If the underlying ModbusMaster supports setTimeout()/setResponseTimeout(), we apply it.
  void setModbusTimeoutMs(uint16_t timeoutMs);

  void setResetPulseMs(uint16_t pulseMs);

  bool MPA(uint8_t id,
           int32_t R_POS,
           int32_t R_SPD,
           int32_t R_ACC,
           int32_t R_DEC,
           int32_t R_CUR,
           int32_t R_FBP,
           int32_t R_CMP);

  bool SMP(uint8_t id, const SMPFields& f);

  bool SIN(uint8_t id, Input input, bool state);
  bool SIN(uint8_t id, const char* inputName, bool state);

  bool SIP(uint8_t id, Input input);
  bool SIP(uint8_t id, const char* inputName);

  bool GOU(uint8_t id, Output output, bool& value);
  bool GOU(uint8_t id, uint16_t& rawWord);

  bool GFP(uint8_t id, int32_t& value);
  bool GCP(uint8_t id, int32_t& value);

  bool getPresentAlarmCode(uint8_t id, uint16_t& alarmCode);

  void update();

  // ===== Direct Data helpers for Variant A (continuous speed) =====
  // Trigger values per manual: -4 = Operating speed trigger/update. 
  bool DDOSetTrigger(uint8_t id, int16_t trigger);
  bool DDOSetOperatingSpeed(uint8_t id, int32_t speedHz); // signed, scaled by R_SPD

  // Optional: set forwarding destination (0=execution, 1=buffer)
  bool DDOSetForwardingDestination(uint8_t id, uint16_t dest);

  bool execute(const String& cmd, String& reply);

private:
  struct MotorState {
    bool used{false};
    uint8_t id{0};

    int32_t rPos{1}, rSpd{1}, rAcc{1}, rDec{1}, rCur{1};
    int32_t rFbp{1}, rCmp{1};

    uint16_t opType{0};
    int32_t pos{0};
    int32_t spd{0};
    int32_t acc{0};
    int32_t dec{0};
    uint16_t cur{0};
    uint16_t opDataNo{0};

    bool lastReady{false};
    bool lastAlarm{false};
    bool lastMove{false};
    bool lastInPos{false};
    bool outInit{false};
  };

  Stream* _bus{nullptr};
  ModbusMaster _node;

  MotorState _motors[MAX_MOTORS];

  EventCallback _cb{nullptr};

  uint32_t _pollIntervalMs{100};
  uint16_t _interframeDelayMs{4};
  uint16_t _resetPulseMs{20};
  uint16_t _mbTimeoutMs{200};   // keep small to avoid WDT on missing slave
  uint32_t _lastPollMs{0};

  // registers
  static constexpr uint16_t REG_DDO_BASE = 0x0058;
  static constexpr uint16_t REG_DDO_WORDS = 16;

  static constexpr uint16_t REG_DDO_SPD_UP  = 0x005E;
  static constexpr uint16_t REG_DDO_TRIG_UP = 0x0066;
  static constexpr uint16_t REG_DDO_FWD_UP  = 0x0068;

  static constexpr uint16_t REG_IN_AUTO_UP = 0x0078;
  static constexpr uint16_t REG_IN_REF_UP  = 0x007C;
  static constexpr uint16_t REG_OUT_LO     = 0x007F;

  static constexpr uint16_t REG_PRES_ALM_UP = 0x0080;

  static constexpr uint16_t REG_FBPOS_UP   = 0x0120;
  static constexpr uint16_t REG_CMDPOS_UP  = 0x0122;

  MotorState* findMotor(uint8_t id);
  MotorState* ensureMotor(uint8_t id);

  void mbGap();
  void beginTxn(uint8_t id);

  static uint16_t hi16(int32_t v);
  static uint16_t lo16(int32_t v);

  static int32_t clampI32(int64_t v);
  static uint16_t clampU16(int32_t v, uint16_t lo, uint16_t hi);

  int32_t scaleMul(int32_t value, int32_t ratio) const;
  int32_t scaleDiv(int32_t value, int32_t ratio) const;

  bool readHolding(uint8_t id, uint16_t addr, uint16_t qty, uint16_t* out);
  bool writeSingle(uint8_t id, uint16_t addr, uint16_t value);
  bool writeMultiple(uint8_t id, uint16_t addr, const uint16_t* values, uint16_t qty);

  bool readOutRaw(uint8_t id, uint16_t& raw);
  bool readPresentAlarm(uint8_t id, uint16_t& alarmCode);
  bool read32(uint8_t id, uint16_t addrUpper, int32_t& value);

  static bool parseInt(const String& s, int32_t& out);
  static bool parseUInt(const String& s, uint32_t& out);

  static bool parseInputName(const String& n, Input& out);
  static bool parseOutputName(const String& n, Output& out);

  void emitEvent(uint8_t id, const char* tag, bool v);
};