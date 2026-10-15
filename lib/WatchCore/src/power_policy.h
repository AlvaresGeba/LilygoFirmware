// When the watch may give up and go to sleep on its own.
#pragma once

namespace watch {

struct Battery {
  int percent;    // 0..100, negative if unknown (no battery, gauge not ready)
  bool charging;  // on USB power
};

// While a voice command is being delivered (waiting for the phone, sending,
// waiting for the server's answer) the watch never times out into sleep, so
// an answer or an error always reaches the screen. The one exception is a
// battery running low: below this charge the usual timeouts apply.
constexpr int kSendKeepAwakeMinBatteryPct = 30;

inline bool sendMayTimeOut(const Battery &battery) {
  return !battery.charging && battery.percent >= 0 &&
         battery.percent < kSendKeepAwakeMinBatteryPct;
}

// Waking up by itself to fetch the time from the phone costs a few seconds
// of radio time; skip it when the battery is nearly empty.
constexpr int kBackgroundSyncMinBatteryPct = 15;

inline bool backgroundSyncAllowed(const Battery &battery) {
  return battery.charging || battery.percent < 0 ||
         battery.percent >= kBackgroundSyncMinBatteryPct;
}

}  // namespace watch
