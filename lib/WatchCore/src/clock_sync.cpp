#include "clock_sync.h"

namespace watch {

namespace {

constexpr int64_t kUsPerSecond = 1000000;

// A cursor over a length-bounded, not necessarily NUL-terminated buffer.
struct Cursor {
  const char *p;
  const char *end;

  bool atEnd() const { return p >= end; }
  char peek() const { return atEnd() ? '\0' : *p; }

  // Reads 1..maxDigits decimal digits.
  bool digits(int maxDigits, int64_t &value, int &count) {
    value = 0;
    count = 0;
    while (!atEnd() && *p >= '0' && *p <= '9') {
      if (count == maxDigits) return false;
      value = value * 10 + (*p - '0');
      ++p;
      ++count;
    }
    return count > 0;
  }

  bool separator() {
    if (peek() != ',' && peek() != ' ') return false;
    ++p;
    return true;
  }
};

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\0'; }

}  // namespace

bool parseTimeMessage(const char *text, size_t len, TimeMessage &out) {
  if (text == nullptr) return false;
  const char *begin = text;
  const char *end = text + len;
  while (begin < end && isSpace(*begin)) ++begin;
  while (end > begin && isSpace(end[-1])) --end;

  Cursor c{begin, end};
  TimeMessage msg{0, 0, 0, false};
  int count = 0;

  if (!c.digits(12, msg.epoch, count) || !isValidEpoch(msg.epoch)) return false;

  bool hasFraction = false;
  if (c.peek() == '.') {
    ++c.p;
    int64_t fraction = 0;
    if (!c.digits(6, fraction, count)) return false;
    for (int i = count; i < 6; i++) fraction *= 10;  // ".25" is 250000 us
    msg.micros = static_cast<uint32_t>(fraction);
    hasFraction = true;
  }

  if (c.separator()) {
    bool negative = false;
    if (c.peek() == '-' || c.peek() == '+') {
      negative = c.peek() == '-';
      ++c.p;
    }
    int64_t tz = 0;
    if (!c.digits(4, tz, count)) return false;
    if (negative) tz = -tz;
    if (tz < kMinTzMinutes || tz > kMaxTzMinutes) return false;
    msg.tzMinutes = static_cast<int32_t>(tz);
    msg.hasTz = true;

    if (c.separator()) {
      int64_t millis = 0;
      // Either a fraction or a milliseconds field, never both.
      if (hasFraction || !c.digits(3, millis, count)) return false;
      msg.micros = static_cast<uint32_t>(millis * 1000);
    }
  }

  if (!c.atEnd()) return false;
  out = msg;
  return true;
}

int64_t rtcCorrectionUs(int64_t systemUs, int64_t rtcSeconds) {
  const int64_t windowStart = rtcSeconds * kUsPerSecond;
  const int64_t windowEnd = windowStart + kUsPerSecond;  // exclusive
  if (systemUs >= windowStart && systemUs < windowEnd) return 0;
  return windowStart + kUsPerSecond / 2 - systemUs;
}

int64_t rtcTickCorrectionUs(int64_t systemUsAtTick, int64_t rtcSecondsAfterTick) {
  return rtcSecondsAfterTick * kUsPerSecond - systemUsAtTick;
}

uint32_t secondsUntilBackgroundSync(int64_t now, int64_t lastSync, uint8_t failedAttempts,
                                    const SyncSchedule &schedule) {
  int64_t wait;
  if (failedAttempts > 0) {
    // Back off: the phone was not around, do not keep the radio up for nothing.
    int64_t backoff = schedule.firstRetryS;
    for (uint8_t i = 1; i < failedAttempts && backoff < schedule.intervalS; i++) backoff *= 2;
    wait = backoff;
  } else if (!isValidEpoch(lastSync) || lastSync > now) {
    // Never synced, or the clock went backwards past the last sync: soon.
    wait = schedule.firstRetryS;
  } else {
    wait = lastSync + schedule.intervalS - now;
  }
  if (wait > schedule.intervalS) wait = schedule.intervalS;
  if (wait < schedule.minDelayS) wait = schedule.minDelayS;
  return static_cast<uint32_t>(wait);
}

}  // namespace watch
