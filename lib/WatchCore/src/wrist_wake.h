// Raise to wake: what the watch does after the accelerometer's wrist-tilt
// gesture woke it from deep sleep.
#pragma once

#include <stdint.h>

namespace watch {

// A tilt wake only shows the clock briefly: walking, cycling and the like
// raise false gestures, and each one lights the screen. A touch or a press
// of the button makes it a normal wake with the normal timeouts.
constexpr uint32_t kTiltWakeShowMs = 4000;

// How long the clock stays up. untouchedTiltWake: the watch woke by a wrist
// tilt and nobody has touched it since.
inline uint32_t clockTimeoutMs(bool untouchedTiltWake, uint32_t normalMs) {
  if (!untouchedTiltWake) return normalMs;
  return normalMs < kTiltWakeShowMs ? normalMs : kTiltWakeShowMs;
}

// Whether the watch, right after a tilt wake, could be facing the user.
// x, y, z: acceleration in milli-g, z along the screen normal. Looking at
// the watch, the screen is turned well out of the vertical, so a good share
// of gravity lies along z; with the arm hanging or swinging at the side,
// gravity lies in the screen plane and z is small. Only |z| is checked: the
// sign depends on which way the chip faces on the board, and a wrong guess
// there would throw away every wake. A reading far from 1 g means the watch
// is moving and says nothing about where it points: the wake is kept.
constexpr int kFacingMinZPct = 40;    // |z| >= 40 % of g: screen >= ~24 deg out of vertical
constexpr int kStillMinMg = 700;
constexpr int kStillMaxMg = 1300;

inline bool tiltWakeFacesUser(int xMg, int yMg, int zMg) {
  const int64_t x = xMg, y = yMg, z = zMg;
  const int64_t sq = x * x + y * y + z * z;
  if (sq < (int64_t)kStillMinMg * kStillMinMg || sq > (int64_t)kStillMaxMg * kStillMaxMg) return true;
  return z * z * 100 * 100 >= sq * kFacingMinZPct * kFacingMinZPct;
}

// Guard against wake loops: a wake by the accelerometer line that finds no
// wrist-tilt event in the chip's status is a phantom (line stuck or noisy).
// After kMaxPhantomTiltWakes of them in a row, raise to wake is not armed
// again until the user wakes the watch by touch or button.
enum class WakeSource : uint8_t { User, Tilt, Timer };
constexpr uint8_t kMaxPhantomTiltWakes = 3;

inline uint8_t nextPhantomTiltWakes(uint8_t streak, WakeSource source, bool tiltEventSeen) {
  switch (source) {
    case WakeSource::User:  return 0;
    case WakeSource::Timer: return streak;   // a background sync is not the user
    case WakeSource::Tilt:  break;
  }
  if (tiltEventSeen) return 0;
  return streak < 255 ? streak + 1 : streak;
}

inline bool raiseToWakeMayArm(uint8_t phantomStreak) { return phantomStreak < kMaxPhantomTiltWakes; }

}  // namespace watch
