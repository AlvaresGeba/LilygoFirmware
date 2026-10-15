#include <unity.h>

#include "time_format.h"

using namespace watch;

void setUp() {}
void tearDown() {}

static const char *timer(uint32_t ms) {
  static char buf[16];
  formatTimer(ms, buf, sizeof(buf));
  return buf;
}

static const char *duration(uint32_t ms) {
  static char buf[16];
  formatDuration(ms, buf, sizeof(buf));
  return buf;
}

void test_timer_minutes_and_seconds() {
  TEST_ASSERT_EQUAL_STRING("0:00", timer(0));
  TEST_ASSERT_EQUAL_STRING("0:00", timer(999));
  TEST_ASSERT_EQUAL_STRING("0:07", timer(7400));
  TEST_ASSERT_EQUAL_STRING("1:00", timer(60000));
  TEST_ASSERT_EQUAL_STRING("12:34", timer((12 * 60 + 34) * 1000));
  TEST_ASSERT_EQUAL_STRING("59:59", timer(3599999));
}

void test_timer_hours() {
  TEST_ASSERT_EQUAL_STRING("1:00:00", timer(3600000));
  TEST_ASSERT_EQUAL_STRING("1:02:03", timer((3600 + 2 * 60 + 3) * 1000));
}

void test_duration_tenths_under_a_minute() {
  TEST_ASSERT_EQUAL_STRING("0.0 s", duration(0));
  TEST_ASSERT_EQUAL_STRING("0.8 s", duration(850));
  TEST_ASSERT_EQUAL_STRING("12.4 s", duration(12499));
  TEST_ASSERT_EQUAL_STRING("59.9 s", duration(59999));
}

void test_duration_agrees_with_timer() {
  // The timer showed 0:12 at 12.99 s; the result must not say 13.0 s.
  TEST_ASSERT_EQUAL_STRING("0:12", timer(12990));
  TEST_ASSERT_EQUAL_STRING("12.9 s", duration(12990));
}

void test_duration_minutes_and_hours() {
  TEST_ASSERT_EQUAL_STRING("1:00", duration(60000));
  TEST_ASSERT_EQUAL_STRING("1:05", duration(65000));
  TEST_ASSERT_EQUAL_STRING("1:02:03", duration((3600 + 2 * 60 + 3) * 1000));
}

void test_small_buffer_is_terminated() {
  char buf[4];
  formatDuration(12400, buf, sizeof(buf));
  TEST_ASSERT_EQUAL_STRING("12.", buf);
  formatTimer(0, buf, 0);  // no write at all
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_timer_minutes_and_seconds);
  RUN_TEST(test_timer_hours);
  RUN_TEST(test_duration_tenths_under_a_minute);
  RUN_TEST(test_duration_agrees_with_timer);
  RUN_TEST(test_duration_minutes_and_hours);
  RUN_TEST(test_small_buffer_is_terminated);
  return UNITY_END();
}
