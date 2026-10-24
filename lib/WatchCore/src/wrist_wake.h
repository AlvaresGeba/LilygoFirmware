// Raise to wake: the watch wakes when it is turned into a calibrated pose
// (the way the user holds it to read it). The ULP coprocessor samples the
// BMA423 in deep sleep and runs the detector below; this header holds the
// same logic for the main CPU and for the unit tests:
//
//  - thresholds per sensitivity level, as per-axis boxes around the
//    calibrated gravity vector (the ULP has no multiply, so no angles);
//  - the box test in the exact 16-bit arithmetic of the ULP program;
//  - the transition detector: away from the pose for a while, then in it
//    for a couple of samples;
//  - calibration: averaging a still pose and rejecting a moving one;
//  - what happens after the wake (short clock, wake-loop guard).
#pragma once

#include <stdint.h>

namespace watch {

// ---------------------------------------------------------------------
// After a raise wake
// ---------------------------------------------------------------------

// A raise wake only shows the clock briefly: walking, cycling and the like
// produce false raises, and each one lights the screen. A touch or a press
// of the button makes it a normal wake with the normal timeouts.
constexpr uint32_t kTiltWakeShowMs = 4000;

// How long the clock stays up. untouchedTiltWake: the watch woke by a
// raised wrist and nobody has touched it since.
inline uint32_t clockTimeoutMs(bool untouchedTiltWake, uint32_t normalMs) {
  if (!untouchedTiltWake) return normalMs;
  return normalMs < kTiltWakeShowMs ? normalMs : kTiltWakeShowMs;
}

// Guard against wake loops: a raise wake the main CPU does not confirm
// (the pose it reads is far from the calibrated one) is a phantom. After
// kMaxPhantomWakes of them in a row, raise to wake is not armed again until
// the user wakes the watch by touch or button.
enum class WakeSource : uint8_t { User, Raise, Timer };
constexpr uint8_t kMaxPhantomWakes = 3;

inline uint8_t nextPhantomWakes(uint8_t streak, WakeSource source, bool confirmed) {
  switch (source) {
    case WakeSource::User:  return 0;
    case WakeSource::Timer: return streak;   // a background sync is not the user
    case WakeSource::Raise: break;
  }
  if (confirmed) return 0;
  return streak < 255 ? streak + 1 : streak;
}

inline bool raiseToWakeMayArm(uint8_t phantomStreak) { return phantomStreak < kMaxPhantomWakes; }

// ---------------------------------------------------------------------
// Poses and thresholds
// ---------------------------------------------------------------------

// Acceleration in milli-g, in the BMA423's own axes (no remap): z is normal
// to the board. The calibration is stored in these axes, so the screen
// rotation setting does not affect it.
struct Accel {
  int x, y, z;
};

// Without a calibration: screen facing up. The BMA423 sits on the bottom
// layer of the board (LilyGO remaps it as BOTTOM_LAYER for the T-Watch S3),
// so its z axis points away from the screen and reads -1 g with the screen
// up. Which board axis points to 12 o'clock is not known, so there is no
// tilt toward the user; calibrating replaces this guess.
constexpr Accel kDefaultWakePose = {0, 0, -1000};

enum class WakeSensitivity : uint8_t { Low = 0, Medium = 1, High = 2 };
constexpr uint8_t kWakeSensitivityLevels = 3;

inline const char *wakeSensitivityName(WakeSensitivity s) {
  switch (s) {
    case WakeSensitivity::Low:    return "Low";
    case WakeSensitivity::Medium: return "Med";
    case WakeSensitivity::High:   return "High";
  }
  return "?";
}

inline WakeSensitivity wakeSensitivityFrom(uint8_t v) {
  return v < kWakeSensitivityLevels ? (WakeSensitivity)v : WakeSensitivity::Medium;
}

// Per-axis half-widths of the two boxes around the calibrated vector:
//  - match: the pose counts as "held as when reading";
//  - away: leaving this wider box arms the detector.
// Between the two is a dead band, so a pose wobbling at the edge of the
// match box cannot re-arm itself. Turning the watch by an angle a moves the
// gravity vector by the chord 2 sin(a/2) g; the half-widths are that chord
// for the angles noted. A box accepts every turn up to that angle, and a
// bit more (up to ~1.4x) in diagonal directions.
struct PoseThresholds {
  int matchMg;
  int awayMg;
};

constexpr PoseThresholds kPoseThresholds[kWakeSensitivityLevels] = {
    {209, 467},   // Low:    match within ~12 deg, re-arm beyond ~27 deg
    {313, 568},   // Medium: ~18 / ~33 deg
    {433, 684},   // High:   ~25 / ~40 deg
};

inline PoseThresholds poseThresholds(WakeSensitivity s) { return kPoseThresholds[(uint8_t)s]; }

// ---------------------------------------------------------------------
// ULP units and the box test
// ---------------------------------------------------------------------

// The ULP reads only the high byte of each 12-bit sample. In the +-2 g
// range that is a signed byte with 64 counts per g (15.6 mg). The ULP adds
// 128 to get an unsigned 0..255 value, 128 meaning 0 g ("units" below).
constexpr int kUlpCountsPerG = 64;
constexpr int kUlpZero = 128;

inline int roundDiv(int num, int den) {   // den > 0
  return num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den);
}

inline int ulpCounts(int mg) { return roundDiv(mg * kUlpCountsPerG, 1000); }

inline int ulpUnitsFromMg(int mg) {
  int c = ulpCounts(mg);
  if (c < -128) c = -128;
  if (c > 127) c = 127;
  return c + kUlpZero;
}

// What the ULP computes from the raw high byte: (byte + 128) & 0xFF.
inline uint16_t ulpUnitsFromHighByte(uint8_t highByte) { return (uint16_t)((highByte + 128) & 0xFF); }

inline int mgFromUlpUnits(int units) { return (units - kUlpZero) * 1000 / kUlpCountsPerG; }

// One box as the ULP program uses it, per axis: the sample u is inside if
// lo <= u < lo + width. lo is stored biased by -kUlpBoxBias so the ULP's
// unsigned 16-bit subtraction never wraps: R0 = u - loAdj = u - lo + bias.
constexpr uint16_t kUlpBoxBias = 1024;

struct UlpBox {
  uint16_t loAdj[3];
  uint16_t width[3];
};

inline UlpBox ulpBox(const Accel &center, int halfWidthMg) {
  const int c[3] = {ulpUnitsFromMg(center.x), ulpUnitsFromMg(center.y), ulpUnitsFromMg(center.z)};
  int t = ulpCounts(halfWidthMg);
  if (t < 1) t = 1;
  UlpBox box;
  for (int i = 0; i < 3; i++) {
    box.loAdj[i] = (uint16_t)(c[i] - t - kUlpBoxBias);
    box.width[i] = (uint16_t)(2 * t + 1);
  }
  return box;
}

// The ULP program's test, step by step (two subtractions and two
// compares against immediates per axis).
inline bool ulpInBox(const uint16_t units[3], const UlpBox &box) {
  for (int i = 0; i < 3; i++) {
    uint16_t r0 = (uint16_t)(units[i] - box.loAdj[i]);
    if (r0 < kUlpBoxBias) return false;            // u < lo
    r0 = (uint16_t)(r0 - box.width[i]);
    if (r0 > kUlpBoxBias - 1) return false;        // u >= lo + width
  }
  return true;
}

inline void ulpUnitsFromAccel(const Accel &a, uint16_t out[3]) {
  out[0] = (uint16_t)ulpUnitsFromMg(a.x);
  out[1] = (uint16_t)ulpUnitsFromMg(a.y);
  out[2] = (uint16_t)ulpUnitsFromMg(a.z);
}

// ---------------------------------------------------------------------
// The transition detector (the ULP program implements exactly this)
// ---------------------------------------------------------------------

// At 10 samples a second: away for 0.4 s arms it, 0.2 s in the pose wakes.
constexpr uint16_t kAwaySamples = 4;
constexpr uint16_t kMatchSamples = 2;

enum class PoseZone : uint8_t { Between = 0, Away = 1, Match = 2 };

inline PoseZone poseZone(const uint16_t units[3], const UlpBox &match, const UlpBox &away) {
  if (!ulpInBox(units, away)) return PoseZone::Away;
  if (ulpInBox(units, match)) return PoseZone::Match;
  return PoseZone::Between;
}

// Disarmed on every sleep: a watch lying still in the pose (on a desk, say)
// has to leave it before it can wake the CPU, and after each wake the
// detector disarms again.
struct PoseDetector {
  uint16_t awayRun = 0;    // consecutive samples outside the away box, up to kAwaySamples
  uint16_t armed = 0;      // 1 once awayRun reached kAwaySamples
  uint16_t matchRun = 0;   // consecutive samples in the match box while armed
};

// One sample. True when the CPU is to be woken (the ULP also checks that
// the chip is ready for a wake-up; if not, matchRun stays up and the next
// sample tries again).
inline bool poseStep(PoseDetector &d, PoseZone zone) {
  if (zone == PoseZone::Away) {
    d.matchRun = 0;
    if (d.awayRun + 1 < kAwaySamples) {
      d.awayRun++;
    } else {
      d.awayRun = kAwaySamples;
      d.armed = 1;
    }
    return false;
  }
  d.awayRun = 0;
  if (zone != PoseZone::Match || !d.armed) {
    d.matchRun = 0;
    return false;
  }
  d.matchRun++;
  if (d.matchRun < kMatchSamples) return false;
  d.armed = 0;
  d.matchRun = 0;
  return true;
}

// After a raise wake the CPU reads the accelerometer itself (some 300 ms
// later). The wake is confirmed unless the watch has clearly left the pose,
// i.e. is outside the away box again.
inline bool poseConfirmsWake(const Accel &now, const Accel &center, const PoseThresholds &t) {
  uint16_t u[3];
  ulpUnitsFromAccel(now, u);
  return ulpInBox(u, ulpBox(center, t.awayMg));
}

// ---------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------

// The user holds the watch as when reading it for a few seconds; the mean
// of the samples is the pose. Rejected when the watch moved: too few
// samples, a mean far from 1 g (accelerating), or a wide spread.
constexpr int kCalMinSamples = 20;
constexpr int kCalMinMg = 850;
constexpr int kCalMaxMg = 1150;
constexpr int kCalMaxSpreadMg = 120;   // peak to peak on any axis, ~7 deg

enum class CalStatus : uint8_t { Ok, TooFewSamples, NotStill, Moving };

inline const char *calStatusText(CalStatus s) {
  switch (s) {
    case CalStatus::Ok:            return "Saved";
    case CalStatus::TooFewSamples: return "Sensor not responding";
    case CalStatus::NotStill:      return "Watch was accelerating";
    case CalStatus::Moving:        return "Watch was moving";
  }
  return "?";
}

struct CalResult {
  CalStatus status;
  Accel mean;
  int spreadMg;      // largest peak-to-peak of the three axes
  int magnitudeMg;   // of the mean, rounded down
};

class CalAccumulator {
 public:
  void add(const Accel &a) {
    const int v[3] = {a.x, a.y, a.z};
    for (int i = 0; i < 3; i++) {
      sum_[i] += v[i];
      if (n_ == 0 || v[i] < min_[i]) min_[i] = v[i];
      if (n_ == 0 || v[i] > max_[i]) max_[i] = v[i];
    }
    n_++;
  }

  int count() const { return n_; }

  CalResult result() const {
    CalResult r{CalStatus::TooFewSamples, {0, 0, 0}, 0, 0};
    if (n_ < kCalMinSamples) return r;
    int m[3];
    for (int i = 0; i < 3; i++) {
      m[i] = (int)(sum_[i] >= 0 ? (sum_[i] + n_ / 2) / n_ : -((-sum_[i] + n_ / 2) / n_));
      const int spread = max_[i] - min_[i];
      if (spread > r.spreadMg) r.spreadMg = spread;
    }
    r.mean = {m[0], m[1], m[2]};
    const int64_t sq = (int64_t)m[0] * m[0] + (int64_t)m[1] * m[1] + (int64_t)m[2] * m[2];
    int mag = 0;
    while ((int64_t)(mag + 1) * (mag + 1) <= sq) mag++;
    r.magnitudeMg = mag;
    if (r.spreadMg > kCalMaxSpreadMg) r.status = CalStatus::Moving;
    else if (mag < kCalMinMg || mag > kCalMaxMg) r.status = CalStatus::NotStill;
    else r.status = CalStatus::Ok;
    return r;
  }

 private:
  int64_t sum_[3] = {0, 0, 0};
  int min_[3] = {0, 0, 0};
  int max_[3] = {0, 0, 0};
  int n_ = 0;
};

}  // namespace watch
