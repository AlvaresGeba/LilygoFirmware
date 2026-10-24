#include <unity.h>

#include "wrist_wake.h"

using namespace watch;

void setUp() {}
void tearDown() {}

void test_tilt_wake_shortens_the_clock() {
  TEST_ASSERT_EQUAL_UINT32(kTiltWakeShowMs, clockTimeoutMs(true, 10000));
  TEST_ASSERT_EQUAL_UINT32(10000, clockTimeoutMs(false, 10000));
}

void test_tilt_wake_never_lengthens_the_clock() {
  TEST_ASSERT_EQUAL_UINT32(3000, clockTimeoutMs(true, 3000));
}

void test_facing_when_screen_tilted_toward_the_face() {
  TEST_ASSERT_TRUE(tiltWakeFacesUser(0, 0, 1000));      // flat, screen up
  TEST_ASSERT_TRUE(tiltWakeFacesUser(0, 0, -1000));     // flat, either chip side
  TEST_ASSERT_TRUE(tiltWakeFacesUser(0, -707, 707));    // 45 degrees, the usual glance
  TEST_ASSERT_TRUE(tiltWakeFacesUser(300, -800, 520));  // screen ~31 deg out of vertical
}

void test_not_facing_with_the_arm_hanging() {
  TEST_ASSERT_FALSE(tiltWakeFacesUser(0, -1000, 0));    // vertical, arm down
  TEST_ASSERT_FALSE(tiltWakeFacesUser(-980, 0, 150));   // vertical, crown down
  TEST_ASSERT_FALSE(tiltWakeFacesUser(600, -760, 200)); // ~12 deg out of vertical
}

void test_moving_readings_keep_the_wake() {
  TEST_ASSERT_TRUE(tiltWakeFacesUser(0, -1800, 0));     // swinging: well over 1 g
  TEST_ASSERT_TRUE(tiltWakeFacesUser(0, -300, 0));      // falling / bad reading
  TEST_ASSERT_TRUE(tiltWakeFacesUser(0, 0, 0));
}

void test_phantom_streak() {
  uint8_t s = 0;
  s = nextPhantomTiltWakes(s, WakeSource::Tilt, false);
  s = nextPhantomTiltWakes(s, WakeSource::Tilt, false);
  TEST_ASSERT_TRUE(raiseToWakeMayArm(s));
  s = nextPhantomTiltWakes(s, WakeSource::Timer, false);  // a background sync changes nothing
  TEST_ASSERT_EQUAL_UINT8(2, s);
  s = nextPhantomTiltWakes(s, WakeSource::Tilt, false);
  TEST_ASSERT_FALSE(raiseToWakeMayArm(s));
  TEST_ASSERT_EQUAL_UINT8(0, nextPhantomTiltWakes(s, WakeSource::User, false));
  TEST_ASSERT_EQUAL_UINT8(0, nextPhantomTiltWakes(s, WakeSource::Tilt, true));
  TEST_ASSERT_EQUAL_UINT8(255, nextPhantomTiltWakes(255, WakeSource::Tilt, false));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_tilt_wake_shortens_the_clock);
  RUN_TEST(test_tilt_wake_never_lengthens_the_clock);
  RUN_TEST(test_facing_when_screen_tilted_toward_the_face);
  RUN_TEST(test_not_facing_with_the_arm_hanging);
  RUN_TEST(test_moving_readings_keep_the_wake);
  RUN_TEST(test_phantom_streak);
  return UNITY_END();
}
