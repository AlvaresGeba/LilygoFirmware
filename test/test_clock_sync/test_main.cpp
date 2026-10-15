#include <string.h>
#include <unity.h>

#include "clock_sync.h"

using namespace watch;

void setUp() {}
void tearDown() {}

static bool parse(const char *s, TimeMessage &m) { return parseTimeMessage(s, strlen(s), m); }

// --- parseTimeMessage ------------------------------------------------------

void test_parse_epoch_only_keeps_timezone_unset() {
  TimeMessage m;
  TEST_ASSERT_TRUE(parse("1760000000", m));
  TEST_ASSERT_EQUAL_INT64(1760000000, m.epoch);
  TEST_ASSERT_EQUAL_UINT32(0, m.micros);
  TEST_ASSERT_FALSE(m.hasTz);  // must not reset the stored offset to UTC
}

void test_parse_ble_format_with_offset() {
  TimeMessage m;
  TEST_ASSERT_TRUE(parse("1760000000,120", m));
  TEST_ASSERT_EQUAL_INT64(1760000000, m.epoch);
  TEST_ASSERT_TRUE(m.hasTz);
  TEST_ASSERT_EQUAL_INT32(120, m.tzMinutes);
}

void test_parse_ble_format_with_millis() {
  TimeMessage m;
  TEST_ASSERT_TRUE(parse("1760000000,-300,250", m));
  TEST_ASSERT_EQUAL_INT32(-300, m.tzMinutes);
  TEST_ASSERT_EQUAL_UINT32(250000, m.micros);
}

void test_parse_console_format_with_fraction_and_space() {
  TimeMessage m;
  TEST_ASSERT_TRUE(parse("1760000000.123456 180\r\n", m));
  TEST_ASSERT_EQUAL_UINT32(123456, m.micros);
  TEST_ASSERT_EQUAL_INT32(180, m.tzMinutes);

  TEST_ASSERT_TRUE(parse("1760000000.5", m));
  TEST_ASSERT_EQUAL_UINT32(500000, m.micros);
}

void test_parse_accepts_explicit_plus_sign() {
  TimeMessage m;
  TEST_ASSERT_TRUE(parse("1760000000,+330", m));
  TEST_ASSERT_EQUAL_INT32(330, m.tzMinutes);
}

void test_parse_rejects_malformed_input() {
  TimeMessage m{42, 7, 9, true};
  const char *bad[] = {
      "",                          // empty
      "abc",                       // not a number
      "1760000000x",               // trailing garbage
      "1760000000,",               // dangling separator
      "1760000000,120,",           // dangling separator
      "1760000000.,120",           // empty fraction
      "1760000000.1234567",        // fraction longer than microseconds
      "1760000000.5,120,250",      // fraction and millis together
      "1760000000,120,1000",       // millis out of range
      "1760000000,900",            // offset beyond +14:00
      "1760000000,-721",           // offset beyond -12:00
      "1699999999",                // before kMinValidEpoch
      "4102444800",                // 2100: the RTC cannot hold it
      "1234567890123",             // more digits than any valid epoch
      "-1760000000",               // negative
  };
  for (const char *s : bad) {
    TEST_ASSERT_FALSE_MESSAGE(parse(s, m), s);
  }
  // A rejected message leaves the output alone.
  TEST_ASSERT_EQUAL_INT64(42, m.epoch);
  TEST_ASSERT_EQUAL_UINT32(7, m.micros);
}

void test_parse_respects_length_not_nul() {
  // BLE payloads are not NUL-terminated; only `len` bytes may be read.
  const char buf[] = {'1', '7', '6', '0', '0', '0', '0', '0', '0', '0', '9', '9'};
  TimeMessage m;
  TEST_ASSERT_TRUE(parseTimeMessage(buf, 10, m));
  TEST_ASSERT_EQUAL_INT64(1760000000, m.epoch);
}

void test_parse_null_text() {
  TimeMessage m;
  TEST_ASSERT_FALSE(parseTimeMessage(nullptr, 0, m));
}

// --- rtcCorrectionUs -------------------------------------------------------

static const int64_t T = 1760000000;
static const int64_t US = 1000000;

void test_correction_zero_inside_rtc_second() {
  TEST_ASSERT_EQUAL_INT64(0, rtcCorrectionUs(T * US, T));
  TEST_ASSERT_EQUAL_INT64(0, rtcCorrectionUs(T * US + 999999, T));
  TEST_ASSERT_EQUAL_INT64(0, rtcCorrectionUs(T * US + 400000, T));
}

void test_correction_pulls_fast_clock_back_to_middle() {
  // System clock 72 s ahead (2 % fast over an hour of sleep).
  const int64_t sys = (T + 72) * US + 300000;
  const int64_t c = rtcCorrectionUs(sys, T);
  TEST_ASSERT_EQUAL_INT64(T * US + 500000, sys + c);
}

void test_correction_pushes_slow_clock_forward_to_middle() {
  const int64_t sys = (T - 5) * US;
  TEST_ASSERT_EQUAL_INT64(T * US + 500000, sys + rtcCorrectionUs(sys, T));
}

void test_correction_at_window_edge() {
  // Exactly one second later is outside the half-open window.
  const int64_t sys = (T + 1) * US;
  TEST_ASSERT_EQUAL_INT64(-500000, rtcCorrectionUs(sys, T));
  // One microsecond early, too.
  TEST_ASSERT_EQUAL_INT64(500001, rtcCorrectionUs(T * US - 1, T));
}

void test_correction_from_unset_clock() {
  // After a reset the system clock starts near zero.
  const int64_t c = rtcCorrectionUs(1234567, T);
  TEST_ASSERT_EQUAL_INT64(T * US + 500000, 1234567 + c);
}

// --- secondsUntilBackgroundSync -------------------------------------------

static const SyncSchedule kDaily = {24 * 3600, 3600, 300};

void test_schedule_waits_for_interval_after_success() {
  TEST_ASSERT_EQUAL_UINT32(24 * 3600, secondsUntilBackgroundSync(T, T, 0, kDaily));
  TEST_ASSERT_EQUAL_UINT32(14 * 3600, secondsUntilBackgroundSync(T + 10 * 3600, T, 0, kDaily));
}

void test_schedule_overdue_sync_uses_minimum_delay() {
  TEST_ASSERT_EQUAL_UINT32(300, secondsUntilBackgroundSync(T + 30 * 3600, T, 0, kDaily));
}

void test_schedule_backs_off_after_failures() {
  TEST_ASSERT_EQUAL_UINT32(3600, secondsUntilBackgroundSync(T, T - 90000, 1, kDaily));
  TEST_ASSERT_EQUAL_UINT32(7200, secondsUntilBackgroundSync(T, T - 90000, 2, kDaily));
  TEST_ASSERT_EQUAL_UINT32(4 * 3600, secondsUntilBackgroundSync(T, T - 90000, 3, kDaily));
  // Capped at the interval, also for absurd counts (no overflow).
  TEST_ASSERT_EQUAL_UINT32(24 * 3600, secondsUntilBackgroundSync(T, T - 90000, 10, kDaily));
  TEST_ASSERT_EQUAL_UINT32(24 * 3600, secondsUntilBackgroundSync(T, T - 90000, 255, kDaily));
}

void test_schedule_never_synced_tries_soon() {
  TEST_ASSERT_EQUAL_UINT32(3600, secondsUntilBackgroundSync(T, 0, 0, kDaily));
}

void test_schedule_clock_moved_before_last_sync() {
  TEST_ASSERT_EQUAL_UINT32(3600, secondsUntilBackgroundSync(T, T + 5000, 0, kDaily));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_parse_epoch_only_keeps_timezone_unset);
  RUN_TEST(test_parse_ble_format_with_offset);
  RUN_TEST(test_parse_ble_format_with_millis);
  RUN_TEST(test_parse_console_format_with_fraction_and_space);
  RUN_TEST(test_parse_accepts_explicit_plus_sign);
  RUN_TEST(test_parse_rejects_malformed_input);
  RUN_TEST(test_parse_respects_length_not_nul);
  RUN_TEST(test_parse_null_text);
  RUN_TEST(test_correction_zero_inside_rtc_second);
  RUN_TEST(test_correction_pulls_fast_clock_back_to_middle);
  RUN_TEST(test_correction_pushes_slow_clock_forward_to_middle);
  RUN_TEST(test_correction_at_window_edge);
  RUN_TEST(test_correction_from_unset_clock);
  RUN_TEST(test_schedule_waits_for_interval_after_success);
  RUN_TEST(test_schedule_overdue_sync_uses_minimum_delay);
  RUN_TEST(test_schedule_backs_off_after_failures);
  RUN_TEST(test_schedule_never_synced_tries_soon);
  RUN_TEST(test_schedule_clock_moved_before_last_sync);
  return UNITY_END();
}
