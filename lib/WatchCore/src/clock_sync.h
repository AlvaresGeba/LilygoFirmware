// Time keeping rules that do not touch hardware: parsing the time the phone
// (or the USB console) sends, keeping the system clock consistent with the
// PCF8563 RTC, and deciding when to ask the phone for the time again.
//
// Background: in deep sleep the ESP32-S3 keeps time with its RTC timer, which
// runs from an internal RC oscillator. That oscillator drifts with temperature
// by a few percent - minutes per day - so after every wake the system clock is
// pulled back to the PCF8563, which has its own 32.768 kHz quartz crystal.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace watch {

// Anything earlier means "never set": after a reset the system clock starts
// from zero. 1700000000 is 2023-11-14.
constexpr int64_t kMinValidEpoch = 1700000000;
constexpr int64_t kMaxValidEpoch = 4102444800;  // 2100-01-01, the PCF8563 stores 2 year digits

constexpr int32_t kMinTzMinutes = -12 * 60;
constexpr int32_t kMaxTzMinutes = 14 * 60;

inline bool isValidEpoch(int64_t seconds) {
  return seconds >= kMinValidEpoch && seconds < kMaxValidEpoch;
}

struct TimeMessage {
  int64_t epoch;      // UTC, whole seconds
  uint32_t micros;    // fraction of the second, 0..999999
  int32_t tzMinutes;  // local time = UTC + tzMinutes; valid only if hasTz
  bool hasTz;
};

// Parses a time message. Accepted forms (fields separated by ',' or ' '):
//   <unix seconds>
//   <unix seconds>,<utc offset minutes>
//   <unix seconds>,<utc offset minutes>,<milliseconds>    (BLE, from the app)
//   <unix seconds>.<fraction>[,<utc offset minutes>]      (USB console)
// Surrounding whitespace and a trailing CR/LF are ignored. Returns false for
// anything else, including out-of-range values; `out` is then left untouched.
bool parseTimeMessage(const char *text, size_t len, TimeMessage &out);

// The PCF8563 counts whole seconds: when it reads `rtcSeconds`, the true time
// lies somewhere in [rtcSeconds, rtcSeconds + 1). Returns how many
// microseconds to add to the system clock (`systemUs`, UTC) so that it agrees:
// 0 if it already lies in that window, otherwise the shift that moves it to
// the middle of the window, which halves the worst-case error.
int64_t rtcCorrectionUs(int64_t systemUs, int64_t rtcSeconds);

struct SyncSchedule {
  uint32_t intervalS;    // a successful sync is trusted for this long
  uint32_t firstRetryS;  // after a failed attempt; doubled per failure, capped at intervalS
  uint32_t minDelayS;    // never wake sooner than this (no wake loops)
};

// Seconds from `now` until the watch should wake up by itself and ask the
// phone for the time. `lastSync` is the UTC time of the last successful sync
// (0 if there never was one), `failedAttempts` counts background attempts
// since then that found no phone.
uint32_t secondsUntilBackgroundSync(int64_t now, int64_t lastSync, uint8_t failedAttempts,
                                    const SyncSchedule &schedule);

}  // namespace watch
