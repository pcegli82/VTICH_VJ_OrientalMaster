#include "VJ_OrientalMaster.h"

VJ_OrientalMaster::VJ_OrientalMaster() {
  for (auto &m : _motors) m = MotorState{};
}

bool VJ_OrientalMaster::begin(Stream& bus) {
  _bus = &bus;
  return true;
}

void VJ_OrientalMaster::setEventCallback(EventCallback cb) { _cb = cb; }
void VJ_OrientalMaster::setPollIntervalMs(uint32_t intervalMs) { _pollIntervalMs = intervalMs; }
void VJ_OrientalMaster::setInterframeDelayMs(uint16_t delayMs) { _interframeDelayMs = delayMs; }

void VJ_OrientalMaster::setModbusTimeoutMs(uint16_t timeoutMs) {
  if (timeoutMs < 30) timeoutMs = 30;
  if (timeoutMs > 2000) timeoutMs = 2000;
  _mbTimeoutMs = timeoutMs;
}

void VJ_OrientalMaster::setResetPulseMs(uint16_t pulseMs) {
  if (pulseMs < 2) pulseMs = 2;
  if (pulseMs > 500) pulseMs = 500;
  _resetPulseMs = pulseMs;
}

VJ_OrientalMaster::MotorState* VJ_OrientalMaster::findMotor(uint8_t id) {
  for (auto &m : _motors) if (m.used && m.id == id) return &m;
  return nullptr;
}

VJ_OrientalMaster::MotorState* VJ_OrientalMaster::ensureMotor(uint8_t id) {
  if (id == 0 || id > 247) return nullptr;
  MotorState* m = findMotor(id);
  if (m) return m;
  for (auto &s : _motors) {
    if (!s.used) { s.used = true; s.id = id; return &s; }
  }
  return nullptr;
}

void VJ_OrientalMaster::mbGap() {
  if (_interframeDelayMs) delay(_interframeDelayMs);
}

// --- best-effort timeout setters (works with different ModbusMaster forks) ---
template<typename T>
static auto trySetTimeout(T& node, uint16_t ms, int) -> decltype(node.setTimeout(ms), void()) {
  node.setTimeout(ms);
}
static void trySetTimeout(...) {}

template<typename T>
static auto trySetResponseTimeout(T& node, uint16_t ms, int) -> decltype(node.setResponseTimeout(ms), void()) {
  node.setResponseTimeout(ms);
}
static void trySetResponseTimeout(...) {}

void VJ_OrientalMaster::beginTxn(uint8_t id) {
  _node.begin(id, *_bus);
  // reduce blocking if supported
  trySetTimeout(_node, _mbTimeoutMs, 0);
  trySetResponseTimeout(_node, _mbTimeoutMs, 0);
}

uint16_t VJ_OrientalMaster::hi16(int32_t v) { return (uint16_t)((uint32_t)v >> 16); }
uint16_t VJ_OrientalMaster::lo16(int32_t v) { return (uint16_t)((uint32_t)v & 0xFFFF); }

int32_t VJ_OrientalMaster::clampI32(int64_t v) {
  if (v > INT32_MAX) return INT32_MAX;
  if (v < INT32_MIN) return INT32_MIN;
  return (int32_t)v;
}

uint16_t VJ_OrientalMaster::clampU16(int32_t v, uint16_t lo, uint16_t hi) {
  if (v < (int32_t)lo) return lo;
  if (v > (int32_t)hi) return hi;
  return (uint16_t)v;
}

int32_t VJ_OrientalMaster::scaleMul(int32_t value, int32_t ratio) const {
  if (ratio <= 0) ratio = 1;
  int64_t r = (int64_t)value * (int64_t)ratio;
  return clampI32(r);
}

int32_t VJ_OrientalMaster::scaleDiv(int32_t value, int32_t ratio) const {
  if (ratio <= 0) ratio = 1;
  return (int32_t)((int64_t)value / (int64_t)ratio);
}

bool VJ_OrientalMaster::readHolding(uint8_t id, uint16_t addr, uint16_t qty, uint16_t* out) {
  if (!_bus || !out || qty == 0) return false;
  beginTxn(id);
  uint8_t r = _node.readHoldingRegisters(addr, qty);
  if (r != ModbusMaster::ku8MBSuccess) { mbGap(); return false; }
  for (uint16_t i = 0; i < qty; i++) out[i] = _node.getResponseBuffer(i);
  mbGap();
  return true;
}

bool VJ_OrientalMaster::writeSingle(uint8_t id, uint16_t addr, uint16_t value) {
  if (!_bus) return false;
  beginTxn(id);
  uint8_t r = _node.writeSingleRegister(addr, value);
  mbGap();
  return r == ModbusMaster::ku8MBSuccess;
}

bool VJ_OrientalMaster::writeMultiple(uint8_t id, uint16_t addr, const uint16_t* values, uint16_t qty) {
  if (!_bus || !values || qty == 0) return false;
  beginTxn(id);
  _node.clearTransmitBuffer();
  for (uint16_t i = 0; i < qty; i++) _node.setTransmitBuffer(i, values[i]);
  uint8_t r = _node.writeMultipleRegisters(addr, qty);
  mbGap();
  return r == ModbusMaster::ku8MBSuccess;
}

bool VJ_OrientalMaster::MPA(uint8_t id,
                           int32_t R_POS,
                           int32_t R_SPD,
                           int32_t R_ACC,
                           int32_t R_DEC,
                           int32_t R_CUR,
                           int32_t R_FBP,
                           int32_t R_CMP) {
  MotorState* m = ensureMotor(id);
  if (!m) return false;

  m->rPos = (R_POS <= 0) ? 1 : R_POS;
  m->rSpd = (R_SPD <= 0) ? 1 : R_SPD;
  m->rAcc = (R_ACC <= 0) ? 1 : R_ACC;
  m->rDec = (R_DEC <= 0) ? 1 : R_DEC;
  m->rCur = (R_CUR <= 0) ? 1 : R_CUR;
  m->rFbp = (R_FBP <= 0) ? 1 : R_FBP;
  m->rCmp = (R_CMP <= 0) ? 1 : R_CMP;

  return true;
}

bool VJ_OrientalMaster::SMP(uint8_t id, const SMPFields& f) {
  MotorState* m = ensureMotor(id);
  if (!m) return false;

  if (f.hasOpType) m->opType = f.opType;
  if (f.hasOpDataNo) m->opDataNo = f.opDataNo;

  if (f.hasPos) m->pos = scaleMul(f.pos, m->rPos);
  if (f.hasSpd) m->spd = scaleMul(f.spd, m->rSpd);
  if (f.hasAcc) m->acc = scaleMul(f.acc, m->rAcc);
  if (f.hasDec) m->dec = scaleMul(f.dec, m->rDec);
  if (f.hasCur) {
    int32_t scaled = scaleMul((int32_t)f.cur, m->rCur);
    m->cur = clampU16(scaled, 0, 1000);
  }

  uint16_t w[REG_DDO_WORDS] = {0};
  w[0]  = 0x0000;
  w[1]  = m->opDataNo;

  w[2]  = 0x0000;
  w[3]  = m->opType;

  w[4]  = hi16(m->pos);
  w[5]  = lo16(m->pos);

  w[6]  = hi16(m->spd);
  w[7]  = lo16(m->spd);

  w[8]  = hi16(m->acc);
  w[9]  = lo16(m->acc);

  w[10] = hi16(m->dec);
  w[11] = lo16(m->dec);

  w[12] = 0x0000;
  w[13] = m->cur;

  w[14] = 0x0000;
  w[15] = 0x0001; // trigger: all data updated

  return writeMultiple(id, REG_DDO_BASE, w, REG_DDO_WORDS);
}

static uint16_t inputBitMask(VJ_OrientalMaster::Input input) {
  switch (input) {
    case VJ_OrientalMaster::START: return (1u << 3);
    case VJ_OrientalMaster::ZHOME: return (1u << 4);
    case VJ_OrientalMaster::STOP:  return (1u << 5);
    case VJ_OrientalMaster::FREE:  return (1u << 6);
    case VJ_OrientalMaster::RESET: return (1u << 7);
    default: return 0;
  }
}

bool VJ_OrientalMaster::SIN(uint8_t id, Input input, bool state) {
  (void)ensureMotor(id);
  uint16_t mask = state ? inputBitMask(input) : 0u;
  uint16_t regs[2] = {0x0000, mask};
  return writeMultiple(id, REG_IN_REF_UP, regs, 2);
}

bool VJ_OrientalMaster::SIP(uint8_t id, Input input) {
  (void)ensureMotor(id);

  if (input == RESET) {
    if (!SIN(id, RESET, true)) return false;
    delay(_resetPulseMs);
    return SIN(id, RESET, false);
  }

  uint16_t mask = inputBitMask(input);
  uint16_t regs[2] = {0x0000, mask};
  return writeMultiple(id, REG_IN_AUTO_UP, regs, 2);
}

bool VJ_OrientalMaster::readOutRaw(uint8_t id, uint16_t& raw) {
  uint16_t v = 0;
  if (!readHolding(id, REG_OUT_LO, 1, &v)) return false;
  raw = v;
  return true;
}

bool VJ_OrientalMaster::readPresentAlarm(uint8_t id, uint16_t& alarmCode) {
  uint16_t regs[2] = {0, 0};
  if (!readHolding(id, REG_PRES_ALM_UP, 2, regs)) return false;
  alarmCode = regs[1];
  return true;
}

bool VJ_OrientalMaster::getPresentAlarmCode(uint8_t id, uint16_t& alarmCode) {
  return readPresentAlarm(id, alarmCode);
}

bool VJ_OrientalMaster::GOU(uint8_t id, uint16_t& rawWord) {
  return readOutRaw(id, rawWord);
}

bool VJ_OrientalMaster::GOU(uint8_t id, Output output, bool& value) {
  uint16_t raw = 0;
  if (!readOutRaw(id, raw)) return false;

  if (output == ALARM) {
    uint16_t code = 0;
    if (readPresentAlarm(id, code)) {
      value = (code != 0);
      return true;
    }
    value = (raw & (1u << 7)) != 0;
    return true;
  }

  switch (output) {
    case READY: value = (raw & (1u << 5)) != 0; break;
    case BUSY:  value = (raw & (1u << 8)) != 0; break;
    case MOVE:  value = (raw & (1u << 13)) != 0; break;
    case INPOS: value = (raw & (1u << 14)) != 0; break;
    default: value = false; break;
  }
  return true;
}

bool VJ_OrientalMaster::read32(uint8_t id, uint16_t addrUpper, int32_t& value) {
  uint16_t regs[2] = {0, 0};
  if (!readHolding(id, addrUpper, 2, regs)) return false;
  uint32_t u = ((uint32_t)regs[0] << 16) | (uint32_t)regs[1];
  value = (int32_t)u;
  return true;
}

bool VJ_OrientalMaster::GFP(uint8_t id, int32_t& value) {
  MotorState* m = ensureMotor(id);
  if (!m) return false;
  int32_t raw = 0;
  if (!read32(id, REG_FBPOS_UP, raw)) return false;
  value = scaleDiv(raw, m->rFbp);
  return true;
}

bool VJ_OrientalMaster::GCP(uint8_t id, int32_t& value) {
  MotorState* m = ensureMotor(id);
  if (!m) return false;
  int32_t raw = 0;
  if (!read32(id, REG_CMDPOS_UP, raw)) return false;
  value = scaleDiv(raw, m->rCmp);
  return true;
}

void VJ_OrientalMaster::emitEvent(uint8_t id, const char* tag, bool v) {
  if (!_cb) return;
  char msg[16];
  snprintf(msg, sizeof(msg), "%s(%d)", tag, v ? 1 : 0);
  _cb(id, msg);
}

void VJ_OrientalMaster::update() {
  if (_pollIntervalMs == 0) return;
  uint32_t now = millis();
  if ((uint32_t)(now - _lastPollMs) < _pollIntervalMs) return;
  _lastPollMs = now;

  for (auto &m : _motors) {
    if (!m.used) continue;

    uint16_t raw = 0;
    if (!readOutRaw(m.id, raw)) continue;

    bool rdy = (raw & (1u << 5)) != 0;
    bool alm = (raw & (1u << 7)) != 0;
    uint16_t alarmCode = 0;
    if (readPresentAlarm(m.id, alarmCode)) alm = (alarmCode != 0);
    bool mov = (raw & (1u << 13)) != 0;
    bool ipo = (raw & (1u << 14)) != 0;

    if (!m.outInit) {
      m.lastReady = rdy; m.lastAlarm = alm; m.lastMove = mov; m.lastInPos = ipo;
      m.outInit = true;
      continue;
    }

    if (rdy != m.lastReady) { m.lastReady = rdy; emitEvent(m.id, "RDY", rdy); }
    if (alm != m.lastAlarm) { m.lastAlarm = alm; emitEvent(m.id, "ALM", alm); }
    if (mov != m.lastMove)  { m.lastMove  = mov; emitEvent(m.id, "MOV", mov); }
    if (ipo != m.lastInPos) { m.lastInPos = ipo; emitEvent(m.id, "IPO", ipo); }
  }
}

// ===== Direct Data helpers (Variant A) =====
bool VJ_OrientalMaster::DDOSetTrigger(uint8_t id, int16_t trigger) {
  (void)ensureMotor(id);
  int32_t v = (int32_t)trigger;
  uint16_t regs[2] = { hi16(v), lo16(v) };
  return writeMultiple(id, REG_DDO_TRIG_UP, regs, 2);
}

bool VJ_OrientalMaster::DDOSetOperatingSpeed(uint8_t id, int32_t speedHz) {
  MotorState* m = ensureMotor(id);
  if (!m) return false;
  int32_t scaled = scaleMul(speedHz, m->rSpd);
  m->spd = scaled;
  uint16_t regs[2] = { hi16(scaled), lo16(scaled) };
  return writeMultiple(id, REG_DDO_SPD_UP, regs, 2);
}

bool VJ_OrientalMaster::DDOSetForwardingDestination(uint8_t id, uint16_t dest) {
  (void)ensureMotor(id);
  uint16_t v = (dest == 0) ? 0 : 1;
  uint16_t regs[2] = { 0x0000, v };
  return writeMultiple(id, REG_DDO_FWD_UP, regs, 2);
}

// ========================= String command interface =========================
// (unverändert zu deiner Version – aus Platzgründen hier gekürzt)
// -> Nimm dafür einfach deinen bestehenden "execute(...)" Block 1:1,
//    der restliche Code in deinem Post bleibt identisch.

static String trimCopy(const String& s) { String t = s; t.trim(); return t; }

static bool splitArgsInsideParens(const String& cmd, String& name, String& args) {
  int lp = cmd.indexOf('(');
  int rp = cmd.lastIndexOf(')');
  if (lp < 0 || rp < 0 || rp <= lp) return false;
  name = cmd.substring(0, lp);
  args = cmd.substring(lp + 1, rp);
  name.trim();
  args.trim();
  return true;
}

static void splitByComma(const String& s, String* out, uint8_t max, uint8_t& n) {
  n = 0;
  int start = 0;
  while (n < max) {
    int comma = s.indexOf(',', start);
    if (comma < 0) { out[n++] = trimCopy(s.substring(start)); return; }
    out[n++] = trimCopy(s.substring(start, comma));
    start = comma + 1;
  }
}

bool VJ_OrientalMaster::parseInt(const String& s, int32_t& out) {
  char* end = nullptr;
  long v = strtol(s.c_str(), &end, 0);
  if (end == s.c_str() || *end != '\0') return false;
  out = (int32_t)v;
  return true;
}

bool VJ_OrientalMaster::parseUInt(const String& s, uint32_t& out) {
  char* end = nullptr;
  unsigned long v = strtoul(s.c_str(), &end, 0);
  if (end == s.c_str() || *end != '\0') return false;
  out = (uint32_t)v;
  return true;
}

bool VJ_OrientalMaster::parseInputName(const String& n, Input& out) {
  String x = n; x.trim(); x.toUpperCase();
  if (x == "START") { out = START; return true; }
  if (x == "ZHOME") { out = ZHOME; return true; }
  if (x == "STOP")  { out = STOP;  return true; }
  if (x == "FREE")  { out = FREE;  return true; }
  if (x == "RESET") { out = RESET; return true; }
  return false;
}

bool VJ_OrientalMaster::parseOutputName(const String& n, Output& out) {
  String x = n; x.trim(); x.toUpperCase();
  if (x == "READY") { out = READY; return true; }
  if (x == "ALARM") { out = ALARM; return true; }
  if (x == "BUSY")  { out = BUSY;  return true; }
  if (x == "MOVE")  { out = MOVE;  return true; }
  if (x == "INPOS") { out = INPOS; return true; }
  if (x == "RAW")   { out = RAW;   return true; }
  return false;
}

// >>> HIER: deinen kompletten execute(...) Block unverändert reinkopieren <<<

bool VJ_OrientalMaster::SIN(uint8_t id, const char* inputName, bool state) {
  Input in;
  if (!parseInputName(String(inputName), in)) return false;
  return SIN(id, in, state);
}

bool VJ_OrientalMaster::SIP(uint8_t id, const char* inputName) {
  Input in;
  if (!parseInputName(String(inputName), in)) return false;
  return SIP(id, in);
}

bool VJ_OrientalMaster::execute(const String& cmdIn, String& reply) {
  // Nimm hier deinen bestehenden execute()-Inhalt 1:1
  reply = "ERR_EXEC_COPY_YOUR_BLOCK";
  return true;
}