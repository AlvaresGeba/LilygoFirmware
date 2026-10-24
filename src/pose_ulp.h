// Raise to wake in deep sleep: the ULP (FSM) program that reads the BMA423
// over a bit-banged I2C bus and wakes the CPU when the watch is turned into
// the calibrated pose. The detector logic mirrors watch::poseStep in
// lib/WatchCore/src/wrist_wake.h.
//
// Kept apart from the .ino so the encoder can be compiled on a PC against
// the same ESP-IDF headers and checked there (see the README).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace poseulp {

// Shared variables: word offsets in the variables block. The ULP uses the
// low 16 bits of each word. Values are written to both half-words (by the
// CPU and by the program), so a load reads the right value whichever half
// the hardware takes (the IDF's documentation of LD on the S3 contradicts
// itself on that point).
enum Var : uint8_t {
  V_BUSY = 0,          // 1 while the program runs; the CPU waits for 0 before using the bus
  V_MATCH_LO = 1,      // 3 words: match box per axis, lo - kUlpBoxBias
  V_MATCH_W = 4,       // 3 words: match box widths
  V_AWAY_LO = 7,       // 3 words: away box
  V_AWAY_W = 10,       // 3 words
  V_AWAY_RUN = 13,     // detector state (watch::PoseDetector)
  V_ARMED = 14,
  V_MATCH_RUN = 15,
  V_SAMPLE = 16,       // 3 words: last sample, ULP units (0..255, 128 = 0 g)
  V_WAKE_SAMPLE = 19,  // 3 words: the sample that woke the CPU
  V_ZONE = 22,         // zone of the last sample: 0 between, 1 away, 2 match
  V_RUNS = 23,         // program runs (wraps at 65536)
  V_ERRORS = 24,       // runs that did not get a sample
  V_LAST_ERROR = 25,   // Error of the last run
  V_WAKES = 26,        // wake-ups the program issued
  V_ACK = 27,          // scratch: 0 = ACK the next byte read, 1 = NACK it
  V_COUNT = 28
};

enum Error : uint8_t {
  ERR_NONE = 0,
  ERR_SCL_LOW = 1,      // the bus was not idle: SCL held low; nothing sent
  ERR_SDA_LOW = 2,      // SDA held low: nine clocks and a STOP sent to free it
  ERR_NACK_ADDR = 3,    // no ACK to the address (write)
  ERR_NACK_REG = 4,     // no ACK to the register number
  ERR_NACK_READ = 5,    // no ACK to the address (read)
};

constexpr uint8_t kBmaAddr = 0x19;
// One burst read from ACC_X_MSB: X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB. Only the
// high bytes are used (64 counts per g in the +-2 g range).
constexpr uint8_t kFirstReg = 0x13;
constexpr uint8_t kReadBytes = 5;

// Half a bit: the WAIT between SCL edges, in ULP clock cycles. The ULP runs
// from RTC_FAST_CLK (RC_FAST, ~17.5 MHz). With the instructions around it,
// SCL is high >= 4.0 us and low >= 4.6 us: ~115 kHz, well inside the
// fast-mode (400 kHz) timing of every device on the bus even with a faster
// oscillator, with time for slow rising edges (tools/pose_ulp_check).
constexpr uint16_t kHalfBitCycles = 56;
constexpr uint16_t kHoldCycles = 8;

constexpr size_t kCodeWords = 320;

// Encodes the program into code[0..capacity). codeWord / varsWord: where
// code[] and the variables block sit in RTC slow memory, in 32-bit words
// from its start (the ULP's address space). sdaRtcIo / sclRtcIo: RTC GPIO
// numbers of the bus lines, set up by the CPU as open-drain outputs with
// the output latch at 1 (released). Returns the program length in words,
// 0 if it does not fit or a branch is out of range.
size_t build(uint32_t *code, size_t capacity, uint32_t codeWord, uint32_t varsWord, int sdaRtcIo,
             int sclRtcIo);

}  // namespace poseulp
