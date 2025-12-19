#pragma once

#include <Arduino.h>
#include <ModbusMaster.h>

// VJ_OrientalMaster
// - Controls up to 10 Oriental Motor AZ-series drives (AZD-C(D)) via Modbus RTU.
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

  // Event callback: message formatted as required e.g. "RDY(1)", "ALM(0)", "MOV(1)", "IPO(0)"
  using EventCallback = void (*)(uint8_t id, const char* msg);

  VJ_OrientalMaster();

  // Bind bus (Stream) used for Modbus RTU.
  // NOTE: You must init RS485 (baud/parity/half-duplex) in your main program.
  bool begin(Stream& bus);

  void setEventCallback(EventCallback cb);
  void setPollIntervalMs(uint32_t intervalMs);
  void setInterframeDelayMs(uint16_t delayMs);

  // Register/configure a motor (slave id) and define scaling ratios.
  // Ratios: values sent TO the drive are multiplied by R_POS/R_SPD/R_ACC/R_DEC/R_CUR.
  // Returned feedback/command position are divided by R_FBP/R_CMP.
  bool MPA(uint8_t id,
           int32_t R_POS,
           int32_t R_SPD,
           int32_t R_ACC,
           int32_t R_DEC,
           int32_t R_CUR,
           int32_t R_FBP,
           int32_t R_CMP);

  // Send motion parameters (Direct Data Operation) to a motor.
  // Unspecified fields keep the last value for that motor.
  bool SMP(uint8_t id, const SMPFields& f);

  // Inputs (reference) - set a specific input to state (like old example, writes a mask).
  bool SIN(uint8_t id, Input input, bool state);
  bool SIN(uint8_t id, const char* inputName, bool state);

  // Inputs (pulse) - auto-off pulse command.
  bool SIP(uint8_t id, Input input);
  bool SIP(uint8_t id, const char* inputName);

  // Outputs
  bool GOU(uint8_t id, Output output, bool& value);
  bool GOU(uint8_t id, uint16_t& rawWord);

  // Positions
  bool GFP(uint8_t id, int32_t& value); // feedback position (scaled by R_FBP)
  bool GCP(uint8_t id, int32_t& value); // command position  (scaled by R_CMP)

  // Call often in loop(): handles output polling + change notifications.
  void update();

  // Optional string command interface (same syntax you described):
  //   MPA(1,100,1,1,1,1,10,10)
  //   SMP(I1,O3,P2000,S5000,A1500000,D1500000,C1000)
  //   SIN(RESET,1)
  //   SIP(RESET)
  //   GOU(READY)
  //   GFP(VALUE)
  //   GCP(VALUE)
  // Replies are set in 'reply'. Returns true if command understood.
  bool execute(const String& cmd, String& reply);

private:
  struct MotorState {
    bool used{false};
    uint8_t id{0};

    // ratios
    int32_t rPos{1}, rSpd{1}, rAcc{1}, rDec{1}, rCur{1};
    int32_t rFbp{1}, rCmp{1};

    // last SMP values (drive-units after scaling)
    uint16_t opType{0};
    int32_t pos{0};
    int32_t spd{0};
    int32_t acc{0};
    int32_t dec{0};
    uint16_t cur{0};
    uint16_t opDataNo{0};

    // last output bits
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
  uint32_t _lastPollMs{0};

  // registers
  static constexpr uint16_t REG_DDO_BASE = 0x0058;
  static constexpr uint16_t REG_DDO_WORDS = 16;

  static constexpr uint16_t REG_IN_AUTO_UP = 0x0078;
  static constexpr uint16_t REG_IN_REF_UP  = 0x007C;
  static constexpr uint16_t REG_OUT_LO     = 0x007F;

  static constexpr uint16_t REG_FBPOS_UP   = 0x0120;
  static constexpr uint16_t REG_CMDPOS_UP  = 0x0122;

  // helpers
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
  bool read32(uint8_t id, uint16_t addrUpper, int32_t& value);

  static bool parseInt(const String& s, int32_t& out);
  static bool parseUInt(const String& s, uint32_t& out);

  static bool parseInputName(const String& n, Input& out);
  static bool parseOutputName(const String& n, Output& out);

  void emitEvent(uint8_t id, const char* tag, bool v);
};
