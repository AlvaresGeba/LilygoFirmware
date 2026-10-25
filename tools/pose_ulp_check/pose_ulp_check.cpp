// Host-side check of the raise-to-wake ULP program (src/pose_ulp.cpp),
// built against the ESP-IDF headers of the PlatformIO install: decodes the
// encoded words by raw bit positions (ulp.h layouts), executes them against
// a simulated open-drain I2C bus with a BMA423 slave, checks the I2C
// protocol and timing, and compares every wake decision with
// watch::poseStep. Run it with ./check.sh (add -l for a listing).
//
// Instruction timings are estimates from the ESP32 TRM (ALU 6 cycles,
// LD/ST 8, jumps 4, REG_RD 8, REG_WR 12, WAIT 2+n) at 17.5 MHz; the
// half-word ambiguity of LD on the S3 is run both ways. Stage branches
// (JUMPS) follow the S3 as measured on the watch: only EQ is taken.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <random>
#include <string>
#include <vector>

#include "pose_ulp.h"
#include "wrist_wake.h"

using namespace poseulp;

static const int SDA = 10, SCL = 11;
static const uint32_t CODE_WORD = 28, VARS_WORD = 0;   // as linked: ulpVars at 0x50000000, ulpCode at 0x50000070
static const double MHZ = 17.5;

static int failures = 0;
#define CHECK(c, ...)                                   \
  do {                                                  \
    if (!(c)) {                                         \
      failures++;                                       \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);       \
      printf(__VA_ARGS__);                              \
      printf("\n");                                     \
    }                                                   \
  } while (0)

// ---------------------------------------------------------------------
// I2C slave (BMA423 at 0x19) and a bus monitor
// ---------------------------------------------------------------------
struct Slave {
  bool present = true;
  uint8_t addr = 0x19;
  uint8_t regs[256] = {};
  enum St { IDLE, RX, ACK_RX, TX, ACK_TX, IGNORE } st = IDLE;
  int bit = 0;
  uint8_t sh = 0, txByte = 0, ptr = 0;
  bool isAddr = false, readMode = false, gotReg = false, masterAck = false;
  bool sdaLow = false;
  int starts = 0, stops = 0, bytesRx = 0, bytesTx = 0;

  void onStart() { st = RX; isAddr = true; bit = 0; sh = 0; sdaLow = false; gotReg = false; starts++; }
  void onStop() { st = IDLE; sdaLow = false; stops++; }
  void onRise(bool sda) {
    if (st == RX) { sh = (uint8_t)((sh << 1) | (sda ? 1 : 0)); bit++; }
    else if (st == ACK_TX) masterAck = !sda;
  }
  void drive() { sdaLow = !((txByte >> (7 - bit)) & 1); }
  void onFall() {
    switch (st) {
      case RX:
        if (bit < 8) break;
        bytesRx++;
        if (isAddr) {
          if (present && (sh >> 1) == addr) {
            readMode = sh & 1;
            sdaLow = true;
            st = ACK_RX;
          } else {
            st = IGNORE;
          }
        } else {
          if (!gotReg) { ptr = sh; gotReg = true; }
          sdaLow = true;
          st = ACK_RX;
        }
        break;
      case ACK_RX:
        sdaLow = false;
        if (readMode) { st = TX; bit = 0; txByte = regs[ptr++]; bytesTx++; drive(); }
        else { st = RX; bit = 0; sh = 0; isAddr = false; }
        break;
      case TX:
        bit++;
        if (bit < 8) drive();
        else { sdaLow = false; st = ACK_TX; }
        break;
      case ACK_TX:
        if (masterAck) { st = TX; bit = 0; txByte = regs[ptr++]; bytesTx++; drive(); }
        else st = IGNORE;
        break;
      default: break;
    }
  }
};

// Raw 12-bit sample (milli-g, +-2 g) into the six data registers.
static void setSample(Slave &s, int xMg, int yMg, int zMg) {
  const int mg[3] = {xMg, yMg, zMg};
  for (int i = 0; i < 3; i++) {
    int raw = (int)lround(mg[i] * 1024.0 / 1000.0);
    if (raw > 2047) raw = 2047;
    if (raw < -2048) raw = -2048;
    const uint16_t reg = (uint16_t)(raw << 4);
    s.regs[0x12 + 2 * i] = reg & 0xFF;
    s.regs[0x13 + 2 * i] = reg >> 8;
  }
}

// ---------------------------------------------------------------------
// The ULP FSM
// ---------------------------------------------------------------------
struct Ulp {
  uint32_t mem[2048] = {};
  uint16_t r[4] = {};
  uint32_t stage = 0;
  uint32_t pc = 0;
  uint64_t cycles = 0;
  bool ldReadsUpper = false;   // which half LD with rd_upper=0 reads, to show both work
  bool rdyForWakeup = true;
  bool woke = false;
  uint32_t outReg = 0;         // RTC_GPIO_OUT data bits [31:10]
  Slave *slave = nullptr;

  // Bus monitor
  bool prevSda = true, prevScl = true;
  uint64_t lastSclRise = 0, lastSclFall = 0, lastSdaChange = 0;
  double minHigh = 1e9, minLow = 1e9, minSetup = 1e9, minStartHold = 1e9, minStopSetup = 1e9;
  bool inRun = false;
  int unexpectedStartStop = 0;
  std::vector<char> events;    // 'S' start, 'P' stop

  bool line(int pin) const {
    const bool master = (outReg >> (10 + pin)) & 1;
    if (pin == SDA) return master && !slave->sdaLow;
    return master;
  }
  double us(uint64_t c) const { return c / MHZ; }

  void busUpdate() {
    for (int pass = 0; pass < 2; pass++) {   // the slave may react to an edge
      const bool sda = line(SDA), scl = line(SCL);
      if (scl != prevScl) {
        if (scl) {
          if (inRun) {
            minLow = fmin(minLow, us(cycles - lastSclFall));
            minSetup = fmin(minSetup, us(cycles - lastSdaChange));
          }
          lastSclRise = cycles;
          prevScl = scl;
          slave->onRise(sda);
        } else {
          if (inRun) minHigh = fmin(minHigh, us(cycles - lastSclRise));
          lastSclFall = cycles;
          prevScl = scl;
          slave->onFall();
        }
        continue;
      }
      if (sda != prevSda) {
        if (scl) {   // SDA changed with SCL high: START or STOP
          if (!sda) {
            events.push_back('S');
            slave->onStart();
          } else {
            minStopSetup = fmin(minStopSetup, us(cycles - lastSclRise));
            events.push_back('P');
            slave->onStop();
          }
        }
        lastSdaChange = cycles;
        prevSda = sda;
        continue;
      }
      break;
    }
    // START hold: from the START to SCL falling.
  }

  uint32_t field(uint32_t v, int low, int high) const {
    return (v >> low) & ((high - low >= 31) ? 0xFFFFFFFFu : ((1u << (high - low + 1)) - 1));
  }

  void store(uint32_t addr, uint16_t v, int upper, int way, int label) {
    assert(addr < 2048);
    CHECK(addr >= VARS_WORD && addr < VARS_WORD + V_COUNT, "ST outside the variables block: %u", addr);
    CHECK(way == 3 && label == 0, "unexpected ST mode way=%d", way);
    if (upper) mem[addr] = (mem[addr] & 0x0000FFFF) | ((uint32_t)v << 16);
    else mem[addr] = (mem[addr] & 0xFFFF0000) | v;
  }

  // Runs the program from `entry` until HALT. Returns false on a runaway.
  bool run(uint32_t entry) {
    pc = entry;
    woke = false;
    inRun = true;
    uint64_t startCycle = cycles;
    for (int steps = 0; steps < 200000; steps++) {
      CHECK(pc < 2048, "pc out of range");
      const uint32_t w = mem[pc];
      const uint32_t opcode = w >> 28;
      uint32_t next = pc + 1;
      switch (opcode) {
        case 1: {  // REG_WR
          const uint32_t addr = w & 0xFF, periph = (w >> 8) & 3, data = (w >> 10) & 0xFF;
          const uint32_t low = (w >> 18) & 0x1F, high = (w >> 23) & 0x1F;
          CHECK(periph == 1 && addr == 0, "REG_WR to unexpected register periph=%u addr=%u", periph, addr);
          const uint32_t mask = ((1u << (high - low + 1)) - 1) << low;
          outReg = (outReg & ~mask) | ((data << low) & mask);
          CHECK(low == high && (low == 10 + SDA || low == 10 + SCL), "REG_WR touches bit %u", low);
          cycles += 12;
          busUpdate();
          break;
        }
        case 2: {  // REG_RD
          const uint32_t addr = w & 0xFF, periph = (w >> 8) & 3;
          const uint32_t low = (w >> 18) & 0x1F, high = (w >> 23) & 0x1F;
          uint32_t value = 0;
          if (periph == 1 && addr == 0x24 / 4) {
            value = ((uint32_t)line(SDA) << (10 + SDA)) | ((uint32_t)line(SCL) << (10 + SCL));
          } else if (periph == 0 && addr == 0xD0 / 4) {
            value = (uint32_t)rdyForWakeup << 19;
          } else {
            CHECK(false, "REG_RD from unexpected register periph=%u addr=%u", periph, addr);
          }
          r[0] = (uint16_t)field(value, low, high);
          cycles += 8;
          break;
        }
        case 4:  // WAIT
          cycles += 2 + (w & 0xFFFF);
          break;
        case 6: {  // ST
          const uint32_t sub = (w >> 25) & 7;
          CHECK(sub == 4, "ST sub-opcode %u", sub);
          const uint32_t val = w & 3, areg = (w >> 2) & 3, label = (w >> 4) & 3, upper = (w >> 6) & 1;
          const uint32_t way = (w >> 7) & 3, offset = (w >> 10) & 0x7FF;
          store(r[areg] + offset, r[val], upper, way, label);
          cycles += 8;
          break;
        }
        case 7: {  // ALU
          const uint32_t sub = (w >> 26) & 3, sel = (w >> 21) & 0xF;
          if (sub == 2) {
            const uint32_t imm = (w >> 4) & 0xFF;
            if (sel == 0) stage += imm;
            else if (sel == 1) stage -= imm;
            else if (sel == 2) stage = 0;
            else CHECK(false, "stage sel %u", sel);
          } else {
            const uint32_t d = w & 3, s = (w >> 2) & 3;
            const uint32_t b = sub == 0 ? r[(w >> 4) & 3] : (w >> 4) & 0xFFFF;
            const uint32_t a = r[s];
            uint32_t res = 0;
            switch (sel) {
              case 0: res = a + b; break;
              case 1: res = a - b; break;
              case 2: res = a & b; break;
              case 3: res = a | b; break;
              case 4: res = b; break;
              case 5: res = a << b; break;
              case 6: res = a >> b; break;
              default: CHECK(false, "ALU sel %u", sel);
            }
            if (sub == 0 && sel == 4) res = a;   // MOVR: dest = src
            r[d] = (uint16_t)res;
          }
          cycles += 6;
          break;
        }
        case 8: {  // branches
          const uint32_t sub = (w >> 26) & 3;
          if (sub == 1) {
            const uint32_t d = w & 3, addr = (w >> 2) & 0x7FF, reg = (w >> 21) & 1, type = (w >> 22) & 7;
            CHECK(type == 0, "conditional BX not used");
            next = reg ? r[d] : addr;
          } else {
            const uint32_t imm = w & 0xFFFF, cmp = (w >> 16) & 3, off = (w >> 18) & 0x7F, sign = (w >> 25) & 1;
            bool take;
            if (sub == 0) {
              take = cmp == 0 ? r[0] < imm : cmp == 1 ? r[0] > imm : r[0] == imm;
            } else {
              CHECK(sub == 2, "branch sub %u", sub);
              // Measured on the watch (console U8): only EQ (cmp 2, 3)
              // works; the LT/GE codes of the IDF's ulp.h are never taken.
              CHECK(cmp >= 2, "stage branch cmp %u at %u: never taken on the S3", cmp, pc);
              take = cmp >= 2 && stage == imm;
            }
            if (take) next = sign ? pc - off : pc + off;
          }
          cycles += 4;
          break;
        }
        case 9: {  // END / WAKE
          CHECK(((w >> 26) & 3) == 0 && (w & 1), "END without wakeup");
          if (rdyForWakeup) woke = true;
          cycles += 4;
          break;
        }
        case 11:  // HALT
          cycles += 2;
          inRun = false;
          lastRunCycles = cycles - startCycle;
          return true;
        case 13: {  // LD
          const uint32_t d = w & 3, s = (w >> 2) & 3, offset = (w >> 10) & 0x7FF, upperBit = (w >> 27) & 1;
          const uint32_t addr = r[s] + offset;
          CHECK(addr < 2048, "LD out of range");
          const bool upper = upperBit ? !ldReadsUpper : ldReadsUpper;
          r[d] = upper ? (uint16_t)(mem[addr] >> 16) : (uint16_t)mem[addr];
          cycles += 8;
          break;
        }
        default:
          CHECK(false, "unknown opcode %u at %u (0x%08X)", opcode, pc, w);
          return false;
      }
      pc = next;
    }
    CHECK(false, "program did not halt");
    return false;
  }
  uint64_t lastRunCycles = 0;

  uint16_t var(int v) const { return (uint16_t)mem[VARS_WORD + v]; }
  bool halvesEqual(int v) const { return (mem[VARS_WORD + v] >> 16) == (mem[VARS_WORD + v] & 0xFFFF); }
  void setVar(int v, uint16_t value) { mem[VARS_WORD + v] = value | ((uint32_t)value << 16); }
};

// What the firmware writes before sleeping.
static void arm(Ulp &u, const watch::Accel &center, watch::WakeSensitivity s) {
  const watch::PoseThresholds t = watch::poseThresholds(s);
  const watch::UlpBox match = watch::ulpBox(center, t.matchMg), away = watch::ulpBox(center, t.awayMg);
  for (int i = 0; i < 3; i++) {
    u.setVar(V_MATCH_LO + i, match.loAdj[i]);
    u.setVar(V_MATCH_W + i, match.width[i]);
    u.setVar(V_AWAY_LO + i, away.loAdj[i]);
    u.setVar(V_AWAY_W + i, away.width[i]);
  }
  u.setVar(V_AWAY_RUN, 0);
  u.setVar(V_ARMED, 0);
  u.setVar(V_MATCH_RUN, 0);
  u.setVar(V_BUSY, 0);
}

static void loadProgram(Ulp &u, size_t &words) {
  words = build(&u.mem[CODE_WORD], kCodeWords, CODE_WORD, VARS_WORD, SDA, SCL);
  CHECK(words > 0 && words <= kCodeWords, "build failed: %zu", words);
  u.outReg = (1u << (10 + SDA)) | (1u << (10 + SCL));   // CPU: latches at 1 = released
}

static void disassemble(const Ulp &u, size_t words) {
  static const char *alu[] = {"ADD", "SUB", "AND", "OR", "MOVE", "LSH", "RSH"};
  for (size_t i = 0; i < words; i++) {
    const uint32_t a = CODE_WORD + i, w = u.mem[a];
    printf("%4u  %08X  ", a, w);
    switch (w >> 28) {
      case 1: printf("REG_WR periph=%u addr=0x%02X bit[%u:%u]=%u", (w >> 8) & 3, (w & 0xFF) * 4, (w >> 23) & 31, (w >> 18) & 31, (w >> 10) & 0xFF); break;
      case 2: printf("REG_RD periph=%u addr=0x%02X bit[%u:%u]", (w >> 8) & 3, (w & 0xFF) * 4, (w >> 23) & 31, (w >> 18) & 31); break;
      case 4: printf("WAIT %u", w & 0xFFFF); break;
      case 6: printf("ST%s R%u, R%u, %u", ((w >> 6) & 1) ? "H" : "L", w & 3, (w >> 2) & 3, (w >> 10) & 0x7FF); break;
      case 7: {
        const uint32_t sub = (w >> 26) & 3, sel = (w >> 21) & 0xF;
        if (sub == 2) printf("STAGE_%s %u", sel == 0 ? "INC" : sel == 1 ? "DEC" : "RST", (w >> 4) & 0xFF);
        else if (sub == 0) printf("%s R%u, R%u, R%u", alu[sel], w & 3, (w >> 2) & 3, (w >> 4) & 3);
        else printf("%s R%u, R%u, %u", alu[sel], w & 3, (w >> 2) & 3, (w >> 4) & 0xFFFF);
        break;
      }
      case 8: {
        const uint32_t sub = (w >> 26) & 3;
        if (sub == 1) {
          if ((w >> 21) & 1) printf("JUMP R%u", w & 3);
          else printf("JUMP %u", (w >> 2) & 0x7FF);
        } else {
          const int off = ((w >> 25) & 1) ? -(int)((w >> 18) & 0x7F) : (int)((w >> 18) & 0x7F);
          const uint32_t cmp = (w >> 16) & 3;
          printf("%s %u (%+d), %u, %s", sub == 0 ? "JUMPR" : "JUMPS", a + off, off, w & 0xFFFF,
                 sub == 0 ? (cmp == 0 ? "LT" : cmp == 1 ? "GT" : "EQ") : (cmp == 0 ? "LT(dead)" : cmp == 1 ? "GE(dead)" : "EQ"));
        }
        break;
      }
      case 9: printf("WAKE"); break;
      case 11: printf("HALT"); break;
      case 13: printf("LD%s R%u, R%u, %u", ((w >> 27) & 1) ? "H" : "", w & 3, (w >> 2) & 3, (w >> 10) & 0x7FF); break;
      default: printf("???"); break;
    }
    printf("\n");
  }
}

// One ULP run with the watch at the given vector (milli-g).
static bool sampleRun(Ulp &u, Slave &s, const watch::Accel &a) {
  setSample(s, a.x, a.y, a.z);
  u.events.clear();
  u.run(CODE_WORD);
  return u.woke;
}

static watch::Accel randomUnit(std::mt19937 &rng) {
  std::normal_distribution<double> n(0, 1);
  double x = n(rng), y = n(rng), z = n(rng);
  const double m = sqrt(x * x + y * y + z * z);
  return {(int)lround(1000 * x / m), (int)lround(1000 * y / m), (int)lround(1000 * z / m)};
}

int main(int argc, char **argv) {
  const bool listing = argc > 1 && !strcmp(argv[1], "-l");
  for (int semantics = 0; semantics < 2; semantics++) {
    Slave slave;
    Ulp u;
    u.slave = &slave;
    u.ldReadsUpper = semantics == 1;
    size_t words = 0;
    loadProgram(u, words);
    if (semantics == 0) {
      printf("program: %zu words (%zu bytes), capacity %zu words\n", words, words * 4, kCodeWords);
      if (listing) disassemble(u, words);
    }
    printf("--- LD half-word semantics: %s ---\n", semantics ? "rd_upper=0 reads the HIGH half" : "rd_upper=0 reads the low half");

    const watch::Accel reading = {0, -643, -766};
    arm(u, reading, watch::WakeSensitivity::Medium);

    // 1. A clean read: START, repeated START, STOP; the sample lands.
    setSample(slave, 100, -640, -770);
    u.events.clear();
    u.run(CODE_WORD);
    CHECK(std::string(u.events.begin(), u.events.end()) == "SSP", "bus events %s", std::string(u.events.begin(), u.events.end()).c_str());
    CHECK(u.var(V_LAST_ERROR) == 0, "last error %u", u.var(V_LAST_ERROR));
    CHECK(slave.bytesRx == 3 && slave.bytesTx == kReadBytes, "bytes rx %d tx %d", slave.bytesRx, slave.bytesTx);
    for (int i = 0; i < 3; i++) {
      const uint8_t hb = slave.regs[0x13 + 2 * i];
      CHECK(u.var(V_SAMPLE + i) == watch::ulpUnitsFromHighByte(hb), "sample %d: %u vs %u", i, u.var(V_SAMPLE + i), watch::ulpUnitsFromHighByte(hb));
    }
    CHECK(u.line(SDA) && u.line(SCL) && slave.st == Slave::IDLE, "bus not idle after run");
    CHECK(u.var(V_BUSY) == 0 && u.var(V_RUNS) == 1, "busy %u runs %u", u.var(V_BUSY), u.var(V_RUNS));
    CHECK(u.halvesEqual(V_RUNS) && u.halvesEqual(V_SAMPLE), "half-words differ");
    if (semantics == 0) {
      printf("run: %llu cycles = %.0f us at %.1f MHz\n", (unsigned long long)u.lastRunCycles, u.us(u.lastRunCycles), MHZ);
      printf("timing (us): SCL high >= %.2f, SCL low >= %.2f, data setup >= %.2f, STOP setup >= %.2f  (fast mode: 0.6 / 1.3 / 0.1 / 0.6)\n",
             u.minHigh, u.minLow, u.minSetup, u.minStopSetup);
      printf("SCL period ~ %.1f us (%.0f kHz)\n", u.minHigh + u.minLow, 1000.0 / (u.minHigh + u.minLow));
      CHECK(u.minHigh >= 0.6 * 17.5 / 20 * 1.5 && u.minLow >= 1.3 * 1.5 && u.minSetup >= 0.25 && u.minStopSetup >= 0.6 * 1.5, "timing too tight");
    }

    // 2. Random walks between poses, compared sample by sample with the reference.
    std::mt19937 rng(1234 + semantics);
    int wakes = 0, refWakes = 0, mismatches = 0;
    for (int sens = 0; sens < 3; sens++) {
      const watch::Accel center = sens == 0 ? reading : randomUnit(rng);
      arm(u, center, (watch::WakeSensitivity)sens);
      const watch::PoseThresholds t = watch::poseThresholds((watch::WakeSensitivity)sens);
      const watch::UlpBox mb = watch::ulpBox(center, t.matchMg), ab = watch::ulpBox(center, t.awayMg);
      watch::PoseDetector ref;
      std::uniform_int_distribution<int> pick(0, 9);
      watch::Accel cur = randomUnit(rng);
      for (int i = 0; i < 4000; i++) {
        const int p = pick(rng);
        if (p < 3) cur = center;                       // in the pose
        else if (p < 5) cur = randomUnit(rng);         // anywhere
        else if (p < 8) {                              // near the pose
          std::normal_distribution<double> d(0, t.awayMg * 0.8);
          cur = {center.x + (int)d(rng), center.y + (int)d(rng), center.z + (int)d(rng)};
        }                                              // else: stay
        const bool w = sampleRun(u, slave, cur);
        uint16_t units[3];
        for (int k = 0; k < 3; k++) units[k] = watch::ulpUnitsFromHighByte(slave.regs[0x13 + 2 * k]);
        const bool rw = watch::poseStep(ref, watch::poseZone(units, mb, ab));
        wakes += w;
        refWakes += rw;
        if (w != rw || u.var(V_AWAY_RUN) != ref.awayRun || u.var(V_ARMED) != ref.armed || u.var(V_MATCH_RUN) != ref.matchRun) {
          if (mismatches++ < 5)
            printf("mismatch at %d: ulp wake=%d away=%u armed=%u match=%u; ref wake=%d away=%u armed=%u match=%u\n", i, w,
                   u.var(V_AWAY_RUN), u.var(V_ARMED), u.var(V_MATCH_RUN), rw, ref.awayRun, ref.armed, ref.matchRun);
        }
        CHECK(std::string(u.events.begin(), u.events.end()) == "SSP", "bus events");
      }
    }
    CHECK(mismatches == 0, "%d mismatches with watch::poseStep", mismatches);
    CHECK(u.var(V_WAKES) == wakes, "wake counter %u vs %d", u.var(V_WAKES), wakes);
    printf("random walk: %d wakes (reference %d), %d mismatches\n", wakes, refWakes, mismatches);

    // 3. Still in the pose for a long time: never wakes.
    arm(u, reading, watch::WakeSensitivity::High);
    int stillWakes = 0;
    for (int i = 0; i < 600; i++) stillWakes += sampleRun(u, slave, reading);
    CHECK(stillWakes == 0, "still watch woke %d times", stillWakes);

    // 4. A raise: away, then in the pose -> exactly one wake on the 2nd pose sample.
    arm(u, reading, watch::WakeSensitivity::Medium);
    for (int i = 0; i < 6; i++) sampleRun(u, slave, {0, -1000, 0});   // arm hanging
    sampleRun(u, slave, {0, -900, -400});                            // on the way
    CHECK(!sampleRun(u, slave, reading), "woke on the first pose sample");
    CHECK(sampleRun(u, slave, reading), "no wake on the second pose sample");
    CHECK(u.var(V_WAKE_SAMPLE + 2) == watch::ulpUnitsFromHighByte(slave.regs[0x17]), "wake sample not kept");
    CHECK(!sampleRun(u, slave, reading), "woke twice");

    // 5. Not ready for a wake-up (CPU still going to sleep): retried next run.
    for (int i = 0; i < 6; i++) sampleRun(u, slave, {0, -1000, 0});
    u.rdyForWakeup = false;
    sampleRun(u, slave, reading);
    CHECK(!sampleRun(u, slave, reading), "woke while not ready");
    u.rdyForWakeup = true;
    CHECK(sampleRun(u, slave, reading), "wake not retried");

    // 6. No device: NACK, STOP, error counted, no wake, bus idle.
    slave.present = false;
    const uint16_t errs = u.var(V_ERRORS);
    for (int i = 0; i < 6; i++) sampleRun(u, slave, {0, -1000, 0});
    CHECK(u.var(V_LAST_ERROR) == ERR_NACK_ADDR && u.var(V_ERRORS) == errs + 6, "nack: err %u count %u", u.var(V_LAST_ERROR), u.var(V_ERRORS));
    CHECK(std::string(u.events.begin(), u.events.end()) == "SP", "nack bus events %s", std::string(u.events.begin(), u.events.end()).c_str());
    CHECK(u.line(SDA) && u.line(SCL), "bus not released after NACK");
    slave.present = true;

    // 7. A device stuck mid-byte holding SDA low: freed, then reads work again.
    slave.st = Slave::TX;
    slave.bit = 2;
    slave.txByte = 0x00;
    slave.sdaLow = true;
    u.prevSda = false;
    sampleRun(u, slave, reading);
    CHECK(u.var(V_LAST_ERROR) == ERR_SDA_LOW, "stuck SDA: err %u", u.var(V_LAST_ERROR));
    CHECK(u.line(SDA) && u.line(SCL) && slave.st == Slave::IDLE, "bus not recovered");
    sampleRun(u, slave, reading);
    CHECK(u.var(V_LAST_ERROR) == 0, "read after recovery failed: %u", u.var(V_LAST_ERROR));

    // 8. SCL held low by someone: nothing is driven.
    u.outReg &= ~(1u << (10 + SCL));
    const uint32_t before = u.outReg;
    sampleRun(u, slave, reading);
    CHECK(u.var(V_LAST_ERROR) == ERR_SCL_LOW && u.outReg == before, "SCL low handling");
    u.outReg |= 1u << (10 + SCL);

  }
  printf(failures ? "FAILED: %d\n" : "ALL CHECKS PASSED\n", failures);
  return failures ? 1 : 0;
}
