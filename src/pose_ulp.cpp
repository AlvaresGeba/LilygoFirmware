// The raise-to-wake ULP program; see pose_ulp.h.
//
// The ESP32-S3 has no RTC I2C pins on GPIO 10/11, so the bus is bit-banged
// through the RTC GPIO registers. Both lines are open-drain pads (set up by
// the CPU) whose output latch the program writes: 0 pulls the line low,
// 1 releases it to the pull-ups. The pads can never drive a line high, so a
// device holding a line low (ACK, data) never fights the ULP.
//
// One run (every 100 ms in deep sleep, ~0.8 ms long):
//   1. bus idle check: SCL low -> error, skip; SDA low -> 9 clocks + STOP
//      to free a stuck device, skip;
//   2. START, 0x19+W, 0x13, repeated START, 0x19+R, five bytes (the last
//      one NACKed), STOP; any missing ACK -> STOP, error, skip;
//   3. the three high bytes are turned into ULP units and checked against
//      the away and match boxes; watch::poseStep in the program's terms;
//   4. on a wake: copy of the sample, counters, WAKE (only if the chip is
//      ready to be woken, as the earlier INT1 poll did).
// R3 holds the address of the variables block, R2 the return address of
// the two byte subroutines, R1 the byte, R0 scratch and compare results;
// stage_cnt counts bits (loops end on stage_cnt == n, see loopUntilStage).

#include "pose_ulp.h"

#include <string.h>

#include <soc/rtc_cntl_reg.h>
#include <soc/rtc_io_reg.h>
#include <esp32s3/ulp.h>

#include "wrist_wake.h"

namespace poseulp {
namespace {

// The ulp.h macros are designated brace initializers, which this GCC only
// takes in a copy-initialization; INSN() turns one into a value.
#define INSN(...) ([&]() -> ulp_insn_t { const ulp_insn_t insn_ = __VA_ARGS__; return insn_; }())

// A tiny assembler: the IDF's label resolution lives in
// ulp_process_macros_and_load(), which refuses to load anything when no
// ULP memory is reserved (the Arduino core reserves none), so branches to
// labels are fixed up here against where the code actually sits.
class Asm {
 public:
  Asm(uint32_t *out, size_t capacity, uint32_t baseWord) : out_(out), cap_(capacity), base_(baseWord) {
    for (int &l : labels_) l = -1;
  }

  void op(const ulp_insn_t &insn) {
    if (n_ < cap_) memcpy(&out_[n_], &insn, sizeof(uint32_t));
    n_++;
  }

  int newLabel() {
    if (nextLabel_ < kMaxLabels) return nextLabel_++;
    overflow_ = true;
    return 0;
  }
  void label(int id) { labels_[id] = (int)n_; }

  // Relative branches: R0 compared with an immediate.
  void jumpLess(int id, uint16_t imm) { rel(id, INSN(I_BL(0, imm))); }
  void jumpGreater(int id, uint16_t imm) { rel(id, INSN(I_BG(0, imm))); }
  // Loop tail: back to `id` until stage_cnt == count. On the S3 only the
  // EQ condition of the stage branch (JUMPS) works: measured on the watch,
  // the codes the IDF's ulp.h uses for I_BSL (0) and I_BSGE (1) are never
  // taken, so a "stage_cnt < 8" loop ran once (one address bit sent, then
  // a NACK on every run). So: skip the jump back once the count is reached.
  void loopUntilStage(int id, uint8_t count) {
    ulp_insn_t skip = INSN(I_BSL(2, count));
    skip.b.cmp = kStageCmpEq;
    op(skip);
    jump(id);
  }
  // Absolute unconditional jump.
  void jump(int id) { fix(id, FIX_ABS); op(INSN(I_BXI(0))); }
  // reg = absolute address of a label (a return address).
  void movLabel(uint32_t reg, int id) { fix(id, FIX_MOVI); op(INSN(I_MOVI(reg, 0))); }

  // Resolves the fixups; false if anything does not fit.
  bool finish() {
    if (n_ > cap_ || overflow_) return false;
    for (size_t i = 0; i < nFix_; i++) {
      const Fixup &f = fixups_[i];
      if (labels_[f.label] < 0) return false;
      ulp_insn_t insn;
      memcpy(&insn, &out_[f.at], sizeof(insn));
      const int target = labels_[f.label];
      switch (f.kind) {
        case FIX_REL: {
          const int offset = target - (int)f.at;
          if (offset > 127 || offset < -127) return false;
          insn.b.offset = offset < 0 ? -offset : offset;
          insn.b.sign = offset < 0 ? 1 : 0;
          break;
        }
        case FIX_ABS: {
          const uint32_t addr = base_ + (uint32_t)target;
          if (addr > 0x7FF) return false;   // 11-bit word address
          insn.bx.addr = addr;
          break;
        }
        case FIX_MOVI:
          insn.alu_imm.imm = base_ + (uint32_t)target;
          break;
      }
      memcpy(&out_[f.at], &insn, sizeof(insn));
    }
    return true;
  }

  size_t size() const { return n_; }

 private:
  static constexpr uint32_t kStageCmpEq = 2;   // stage branch condition: stage_cnt == imm
  enum FixKind : uint8_t { FIX_REL, FIX_ABS, FIX_MOVI };
  struct Fixup {          // small: the assembler lives on the caller's stack
    uint16_t at;
    uint8_t label;
    FixKind kind;
  };
  static constexpr int kMaxLabels = 64;
  static constexpr int kMaxFixups = 96;

  void fix(int id, FixKind kind) {
    if (nFix_ >= kMaxFixups) {
      overflow_ = true;
      return;
    }
    fixups_[nFix_++] = {(uint16_t)n_, (uint8_t)id, kind};
  }
  void rel(int id, const ulp_insn_t &insn) {
    fix(id, FIX_REL);
    op(insn);
  }

  uint32_t *out_;
  size_t cap_;
  uint32_t base_;
  size_t n_ = 0;
  int labels_[kMaxLabels];
  int nextLabel_ = 0;
  Fixup fixups_[kMaxFixups];
  size_t nFix_ = 0;
  bool overflow_ = false;
};

ulp_insn_t wrBit(uint32_t reg, uint32_t bit, uint32_t value) { return INSN(I_WR_REG(reg, bit, bit, value)); }
ulp_insn_t rdBit(uint32_t reg, uint32_t bit) { return INSN(I_RD_REG(reg, bit, bit)); }
ulp_insn_t delay(uint16_t cycles) { return INSN(I_DELAY(cycles)); }

class Program {
 public:
  Program(Asm &a, int sda, int scl) : a_(a), sda_(sda), scl_(scl) {}

  void sdaLow() { a_.op(wrBit(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + sda_, 0)); }
  void sdaRelease() { a_.op(wrBit(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + sda_, 1)); }
  void sclLow() { a_.op(wrBit(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + scl_, 0)); }
  void sclRelease() { a_.op(wrBit(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + scl_, 1)); }
  void readSda() { a_.op(rdBit(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + sda_)); }    // R0 = SDA
  void readScl() { a_.op(rdBit(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + scl_)); }    // R0 = SCL
  void half() { a_.op(delay(kHalfBitCycles)); }
  void hold() { a_.op(delay(kHoldCycles)); }

  // Variables: both half-words written, see pose_ulp.h.
  void store(uint32_t reg, uint32_t var) {
    a_.op(INSN(I_STL(reg, R3, var)));
    a_.op(INSN(I_STH(reg, R3, var)));
  }
  void load(uint32_t reg, uint32_t var) { a_.op(INSN(I_LD(reg, R3, var))); }
  void storeImm(uint32_t var, uint16_t value) {
    a_.op(INSN(I_MOVI(R0, value)));
    store(R0, var);
  }
  void increment(uint32_t var) {
    load(R0, var);
    a_.op(INSN(I_ADDI(R0, R0, 1)));
    store(R0, var);
  }

  // SCL low on entry (START, previous byte); SCL low on return.
  void call(int sub) {
    const int ret = a_.newLabel();
    a_.movLabel(R2, ret);
    a_.jump(sub);
    a_.label(ret);
  }

  void start() {          // bus idle -> SDA falls while SCL is high
    sdaLow();
    half();
    sclLow();
    hold();
  }
  void repeatedStart() {  // SCL low
    sdaRelease();
    half();
    sclRelease();
    half();
    sdaLow();
    half();
    sclLow();
    hold();
  }
  void stop() {           // SCL low -> SDA rises while SCL is high
    sdaLow();
    half();
    sclRelease();
    half();
    sdaRelease();
    half();
  }

  // R1 = byte to send. Returns R0 = the ACK bit (0 = acknowledged).
  void writeByteSub(int entry) {
    const int bit = a_.newLabel(), one = a_.newLabel(), clock = a_.newLabel();
    a_.label(entry);
    a_.op(INSN(I_STAGE_RST(0, 0, 0)));
    a_.label(bit);
    a_.op(INSN(I_ANDI(R0, R1, 0x80)));
    a_.jumpGreater(one, 0);
    sdaLow();
    a_.jump(clock);
    a_.label(one);
    sdaRelease();
    a_.label(clock);
    half();                 // data setup, SCL low
    sclRelease();
    half();
    sclLow();
    a_.op(INSN(I_LSHI(R1, R1, 1)));
    a_.op(INSN(I_STAGE_INC(0, 0, 1)));
    a_.loopUntilStage(bit, 8);
    sdaRelease();           // the device drives the ACK
    half();
    sclRelease();
    half();
    readSda();
    sclLow();
    hold();
    a_.op(INSN(I_BXR(R2)));
  }

  // Returns R1 = the byte read; acknowledges it unless V_ACK is 1.
  void readByteSub(int entry) {
    const int bit = a_.newLabel(), nack = a_.newLabel();
    a_.label(entry);
    sdaRelease();
    a_.op(INSN(I_MOVI(R1, 0)));
    a_.op(INSN(I_STAGE_RST(0, 0, 0)));
    a_.label(bit);
    half();
    sclRelease();
    half();
    readSda();
    a_.op(INSN(I_LSHI(R1, R1, 1)));
    a_.op(INSN(I_ORR(R1, R1, R0)));
    sclLow();
    a_.op(INSN(I_STAGE_INC(0, 0, 1)));
    a_.loopUntilStage(bit, 8);
    load(R0, V_ACK);
    a_.jumpGreater(nack, 0);
    sdaLow();               // ACK: more bytes wanted
    a_.label(nack);         // NACK: SDA stays released
    half();
    sclRelease();
    half();
    sclLow();
    sdaRelease();           // the device drives the next bit
    hold();
    a_.op(INSN(I_BXR(R2)));
  }

  // R1 = raw high byte -> R1 = (R1 + 128) & 0xFF, stored to var.
  void storeUnits(uint32_t var) {
    a_.op(INSN(I_ADDI(R1, R1, 128)));
    a_.op(INSN(I_ANDI(R1, R1, 0xFF)));
    store(R1, var);
  }

  // Jumps to `outside` unless lo <= sample < lo + width on this axis
  // (watch::ulpInBox).
  void boxAxis(int axis, int loVar, int widthVar, int outside) {
    load(R1, V_SAMPLE + axis);
    load(R2, loVar + axis);
    a_.op(INSN(I_SUBR(R0, R1, R2)));      // u - lo + bias
    a_.jumpLess(outside, watch::kUlpBoxBias);
    load(R2, widthVar + axis);
    a_.op(INSN(I_SUBR(R0, R0, R2)));      // u - lo - width + bias
    a_.jumpGreater(outside, watch::kUlpBoxBias - 1);
  }

 private:
  Asm &a_;
  int sda_, scl_;
};

}  // namespace

size_t build(uint32_t *code, size_t capacity, uint32_t codeWord, uint32_t varsWord, int sdaRtcIo,
             int sclRtcIo) {
  if (varsWord + V_COUNT > 0x800 || sdaRtcIo < 0 || sdaRtcIo > 21 || sclRtcIo < 0 || sclRtcIo > 21) return 0;
  Asm a(code, capacity, codeWord);
  Program p(a, sdaRtcIo, sclRtcIo);

  const int writeByte = a.newLabel(), readByte = a.newLabel();
  const int nackAddr = a.newLabel(), nackReg = a.newLabel(), nackRead = a.newLabel();
  const int nack = a.newLabel(), sclStuck = a.newLabel(), recover = a.newLabel();
  const int recoverLoop = a.newLabel(), error = a.newLabel(), pose = a.newLabel();
  const int away = a.newLabel(), awayStore = a.newLabel(), between = a.newLabel();
  const int resetMatch = a.newLabel(), done = a.newLabel();

  // --- entry ---
  a.op(INSN(I_MOVI(R3, varsWord)));
  p.storeImm(V_BUSY, 1);
  p.increment(V_RUNS);
  p.readScl();
  a.jumpLess(sclStuck, 1);
  p.readSda();
  a.jumpLess(recover, 1);

  // --- read X_MSB .. Z_MSB ---
  p.start();
  a.op(INSN(I_MOVI(R1, kBmaAddr << 1)));
  p.call(writeByte);
  a.jumpGreater(nackAddr, 0);
  a.op(INSN(I_MOVI(R1, kFirstReg)));
  p.call(writeByte);
  a.jumpGreater(nackReg, 0);
  p.repeatedStart();
  a.op(INSN(I_MOVI(R1, (kBmaAddr << 1) | 1)));
  p.call(writeByte);
  a.jumpGreater(nackRead, 0);
  p.storeImm(V_ACK, 0);
  p.call(readByte);         // X_MSB
  p.storeUnits(V_SAMPLE + 0);
  p.call(readByte);         // Y_LSB, unused
  p.call(readByte);         // Y_MSB
  p.storeUnits(V_SAMPLE + 1);
  p.call(readByte);         // Z_LSB, unused
  p.storeImm(V_ACK, 1);     // the last byte is NACKed
  p.call(readByte);         // Z_MSB
  p.storeUnits(V_SAMPLE + 2);
  p.stop();
  p.storeImm(V_LAST_ERROR, ERR_NONE);
  a.jump(pose);

  // --- bus errors: no sample this time ---
  a.label(nackAddr);
  a.op(INSN(I_MOVI(R1, ERR_NACK_ADDR)));
  a.jump(nack);
  a.label(nackReg);
  a.op(INSN(I_MOVI(R1, ERR_NACK_REG)));
  a.jump(nack);
  a.label(nackRead);
  a.op(INSN(I_MOVI(R1, ERR_NACK_READ)));
  a.label(nack);
  p.stop();
  a.jump(error);

  a.label(sclStuck);
  a.op(INSN(I_MOVI(R1, ERR_SCL_LOW)));
  a.jump(error);

  // SDA low with SCL high: a device stopped mid-byte. Nine clocks let it
  // finish and see a NACK, then a STOP returns the bus to idle.
  a.label(recover);
  a.op(INSN(I_STAGE_RST(0, 0, 0)));
  a.label(recoverLoop);
  p.sclLow();
  p.half();
  p.sclRelease();
  p.half();
  a.op(INSN(I_STAGE_INC(0, 0, 1)));
  a.loopUntilStage(recoverLoop, 9);
  p.sclLow();
  p.half();
  p.stop();
  a.op(INSN(I_MOVI(R1, ERR_SDA_LOW)));

  a.label(error);
  p.store(R1, V_LAST_ERROR);
  p.increment(V_ERRORS);
  a.jump(done);

  // --- the detector (watch::poseZone + watch::poseStep) ---
  a.label(pose);
  for (int axis = 0; axis < 3; axis++) p.boxAxis(axis, V_AWAY_LO, V_AWAY_W, away);
  for (int axis = 0; axis < 3; axis++) p.boxAxis(axis, V_MATCH_LO, V_MATCH_W, between);
  // Match.
  p.storeImm(V_ZONE, (uint16_t)watch::PoseZone::Match);
  p.storeImm(V_AWAY_RUN, 0);
  p.load(R0, V_ARMED);
  a.jumpLess(resetMatch, 1);
  p.increment(V_MATCH_RUN);                   // leaves R0 = matchRun
  a.jumpLess(done, watch::kMatchSamples);
  a.op(rdBit(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP_S));
  a.jumpLess(done, 1);                        // still going to sleep: try again next run
  p.storeImm(V_ARMED, 0);
  p.store(R0, V_MATCH_RUN);                   // R0 is still 0
  p.increment(V_WAKES);
  for (int axis = 0; axis < 3; axis++) {
    p.load(R0, V_SAMPLE + axis);
    p.store(R0, V_WAKE_SAMPLE + axis);
  }
  a.op(INSN(I_WAKE()));
  a.jump(done);

  a.label(away);
  p.storeImm(V_ZONE, (uint16_t)watch::PoseZone::Away);
  p.storeImm(V_MATCH_RUN, 0);
  p.load(R0, V_AWAY_RUN);
  a.op(INSN(I_ADDI(R0, R0, 1)));
  a.jumpLess(awayStore, watch::kAwaySamples);
  a.op(INSN(I_MOVI(R0, watch::kAwaySamples)));   // saturate, and arm
  a.op(INSN(I_MOVI(R1, 1)));
  p.store(R1, V_ARMED);
  a.label(awayStore);
  p.store(R0, V_AWAY_RUN);
  a.jump(done);

  a.label(between);
  p.storeImm(V_ZONE, (uint16_t)watch::PoseZone::Between);
  p.store(R0, V_AWAY_RUN);                    // R0 = 0
  a.label(resetMatch);
  p.storeImm(V_MATCH_RUN, 0);

  a.label(done);
  p.storeImm(V_BUSY, 0);
  a.op(INSN(I_HALT()));

  // The ULP's subroutines.
  p.writeByteSub(writeByte);
  p.readByteSub(readByte);

  if (!a.finish()) return 0;
  return a.size();
}

}  // namespace poseulp
