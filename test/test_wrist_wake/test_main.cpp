#include <unity.h>

#include <math.h>

#include "wrist_wake.h"

using namespace watch;

void setUp() {}
void tearDown() {}

// The watch turned from `from` by `deg` degrees about the unit axis
// (ax, ay, az) (Rodrigues' formula), as a vector in milli-g.
static Accel rotated(const Accel &from, double ax, double ay, double az, double deg) {
  const double n = sqrt(ax * ax + ay * ay + az * az);
  ax /= n; ay /= n; az /= n;
  const double r = deg * M_PI / 180.0, c = cos(r), s = sin(r);
  const double x = from.x, y = from.y, z = from.z;
  const double dot = ax * x + ay * y + az * z;
  const double cx = ay * z - az * y, cy = az * x - ax * z, cz = ax * y - ay * x;
  return {(int)lround(x * c + cx * s + ax * dot * (1 - c)),
          (int)lround(y * c + cy * s + ay * dot * (1 - c)),
          (int)lround(z * c + cz * s + az * dot * (1 - c))};
}

// Angle whose chord is `mg` (milli-g on a 1 g circle).
static double chordDeg(double mg) { return 2 * asin(mg / 2000.0) * 180.0 / M_PI; }

static PoseZone zoneOf(const Accel &a, const Accel &center, WakeSensitivity s) {
  uint16_t u[3];
  ulpUnitsFromAccel(a, u);
  const PoseThresholds t = poseThresholds(s);
  return poseZone(u, ulpBox(center, t.matchMg), ulpBox(center, t.awayMg));
}

// A typical reading pose: screen up, tilted ~40 deg toward the face.
static const Accel kReading = {0, -643, -766};

// ---------------------------------------------------------------------

void test_tilt_wake_shortens_the_clock() {
  TEST_ASSERT_EQUAL_UINT32(kTiltWakeShowMs, clockTimeoutMs(true, 10000));
  TEST_ASSERT_EQUAL_UINT32(10000, clockTimeoutMs(false, 10000));
  TEST_ASSERT_EQUAL_UINT32(3000, clockTimeoutMs(true, 3000));
}

void test_phantom_streak() {
  uint8_t s = 0;
  s = nextPhantomWakes(s, WakeSource::Raise, false);
  s = nextPhantomWakes(s, WakeSource::Raise, false);
  TEST_ASSERT_TRUE(raiseToWakeMayArm(s));
  s = nextPhantomWakes(s, WakeSource::Timer, false);  // a background sync changes nothing
  TEST_ASSERT_EQUAL_UINT8(2, s);
  s = nextPhantomWakes(s, WakeSource::Raise, false);
  TEST_ASSERT_FALSE(raiseToWakeMayArm(s));
  TEST_ASSERT_EQUAL_UINT8(0, nextPhantomWakes(s, WakeSource::User, false));
  TEST_ASSERT_EQUAL_UINT8(0, nextPhantomWakes(s, WakeSource::Raise, true));
  TEST_ASSERT_EQUAL_UINT8(255, nextPhantomWakes(255, WakeSource::Raise, false));
}

void test_ulp_units() {
  TEST_ASSERT_EQUAL_INT(128, ulpUnitsFromMg(0));
  TEST_ASSERT_EQUAL_INT(192, ulpUnitsFromMg(1000));
  TEST_ASSERT_EQUAL_INT(64, ulpUnitsFromMg(-1000));
  TEST_ASSERT_EQUAL_INT(255, ulpUnitsFromMg(5000));   // clamped
  TEST_ASSERT_EQUAL_INT(0, ulpUnitsFromMg(-5000));
  // The ULP's view of the raw high byte (12-bit sample >> 4, signed).
  TEST_ASSERT_EQUAL_UINT16(192, ulpUnitsFromHighByte(0x40));   // +64 = +1 g
  TEST_ASSERT_EQUAL_UINT16(64, ulpUnitsFromHighByte(0xC0));    // -64 = -1 g
  TEST_ASSERT_EQUAL_UINT16(128, ulpUnitsFromHighByte(0x00));
  TEST_ASSERT_EQUAL_INT(1000, mgFromUlpUnits(192));
}

void test_box_edges_match_the_definition() {
  const Accel c = {0, 0, -1000};
  const UlpBox box = ulpBox(c, 313);    // 20 counts
  const int cu[3] = {128, 128, 64};
  for (int axis = 0; axis < 3; axis++) {
    for (int d = -25; d <= 25; d++) {
      uint16_t u[3] = {(uint16_t)cu[0], (uint16_t)cu[1], (uint16_t)cu[2]};
      const int v = cu[axis] + d;
      if (v < 0 || v > 255) continue;
      u[axis] = (uint16_t)v;
      TEST_ASSERT_EQUAL_MESSAGE(d >= -20 && d <= 20, ulpInBox(u, box), "box edge");
    }
  }
}

void test_box_near_the_range_limits_does_not_wrap() {
  // Center at -2 g: lo is negative, the biased subtraction must not wrap.
  const UlpBox box = ulpBox({-2000, 2000, 0}, 684);
  uint16_t inside[3] = {0, 255, 128};
  TEST_ASSERT_TRUE(ulpInBox(inside, box));
  uint16_t outside[3] = {200, 255, 128};
  TEST_ASSERT_FALSE(ulpInBox(outside, box));
}

void test_zones_by_angle() {
  // Axes perpendicular to kReading: x, the one across it, and diagonals.
  const double axes[][3] = {{1, 0, 0}, {0, -0.766, 0.643}, {1, -0.766, 0.643}, {1, 0.766, -0.643}};
  for (int s = 0; s < kWakeSensitivityLevels; s++) {
    const WakeSensitivity sens = (WakeSensitivity)s;
    const PoseThresholds t = kPoseThresholds[s];
    for (const auto &a : axes) {
      for (int sign = -1; sign <= 1; sign += 2) {
        // Every turn whose chord is inside the match box matches...
        const Accel near = rotated(kReading, a[0], a[1], a[2], sign * (chordDeg(t.matchMg) - 1.5));
        TEST_ASSERT_EQUAL_MESSAGE((int)PoseZone::Match, (int)zoneOf(near, kReading, sens), "near");
        // ...and a turn well beyond the away box, in any direction, is away.
        const Accel far = rotated(kReading, a[0], a[1], a[2], sign * chordDeg(1.8 * t.awayMg));
        TEST_ASSERT_EQUAL_MESSAGE((int)PoseZone::Away, (int)zoneOf(far, kReading, sens), "far");
      }
    }
    // Arm hanging down: screen vertical, gravity along the board.
    TEST_ASSERT_EQUAL((int)PoseZone::Away, (int)zoneOf({0, -1000, 0}, kReading, sens));
    TEST_ASSERT_EQUAL((int)PoseZone::Away, (int)zoneOf({1000, 0, 0}, kReading, sens));
    TEST_ASSERT_EQUAL((int)PoseZone::Match, (int)zoneOf(kReading, kReading, sens));
  }
}

void test_sensitivity_widens_the_match_box() {
  // Lying flat, then turned 20 deg about y: only x changes noticeably.
  const Accel flat = {0, 0, -1000};
  const Accel tilted = rotated(flat, 0, 1, 0, 20);
  TEST_ASSERT_NOT_EQUAL((int)PoseZone::Match, (int)zoneOf(tilted, flat, WakeSensitivity::Low));
  TEST_ASSERT_NOT_EQUAL((int)PoseZone::Match, (int)zoneOf(tilted, flat, WakeSensitivity::Medium));
  TEST_ASSERT_EQUAL((int)PoseZone::Match, (int)zoneOf(tilted, flat, WakeSensitivity::High));
  for (int s = 0; s < kWakeSensitivityLevels; s++) {
    TEST_ASSERT_TRUE(kPoseThresholds[s].matchMg < kPoseThresholds[s].awayMg);
    if (s) TEST_ASSERT_TRUE(kPoseThresholds[s].matchMg > kPoseThresholds[s - 1].matchMg);
  }
}

static int runDetector(PoseDetector &d, PoseZone zone, int samples) {
  int wakes = 0;
  for (int i = 0; i < samples; i++) wakes += poseStep(d, zone);
  return wakes;
}

void test_raise_wakes_once() {
  PoseDetector d;
  TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Away, kAwaySamples));
  TEST_ASSERT_EQUAL_UINT16(1, d.armed);
  TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Between, 3));   // on the way up
  TEST_ASSERT_FALSE(poseStep(d, PoseZone::Match));              // one sample is not enough
  TEST_ASSERT_TRUE(poseStep(d, PoseZone::Match));
  // Still held: no second wake until the watch has been away again.
  TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Match, 100));
  TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Between, 10));
  TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Match, 10));
  TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Away, kAwaySamples));
  TEST_ASSERT_EQUAL(1, runDetector(d, PoseZone::Match, 10));
}

void test_still_in_the_pose_never_wakes() {
  PoseDetector d;     // as armed before sleep: disarmed
  TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Match, 10000));
}

void test_short_excursions_do_not_arm() {
  PoseDetector d;
  for (int i = 0; i < 50; i++) {
    runDetector(d, PoseZone::Away, kAwaySamples - 1);
    TEST_ASSERT_EQUAL(0, runDetector(d, PoseZone::Match, 3));
  }
  TEST_ASSERT_EQUAL_UINT16(0, d.armed);
}

void test_a_single_match_sample_does_not_wake() {
  PoseDetector d;
  runDetector(d, PoseZone::Away, 10);
  for (int i = 0; i < 20; i++) {
    TEST_ASSERT_FALSE(poseStep(d, PoseZone::Match));
    TEST_ASSERT_FALSE(poseStep(d, PoseZone::Between));
  }
  TEST_ASSERT_TRUE(d.armed == 1);   // still armed, waiting for a held pose
}

void test_confirmation_after_wake() {
  const PoseThresholds t = poseThresholds(WakeSensitivity::Medium);
  TEST_ASSERT_TRUE(poseConfirmsWake(kReading, kReading, t));
  TEST_ASSERT_TRUE(poseConfirmsWake(rotated(kReading, 1, 0, 0, 25), kReading, t));  // between the boxes
  TEST_ASSERT_FALSE(poseConfirmsWake({0, -1000, 0}, kReading, t));           // arm dropped again
}

void test_calibration_accepts_a_still_pose() {
  CalAccumulator acc;
  for (int i = 0; i < 50; i++) acc.add({10 + (i % 5) * 5, -640 - (i % 3) * 10, -770 + (i % 7) * 4});
  const CalResult r = acc.result();
  TEST_ASSERT_EQUAL((int)CalStatus::Ok, (int)r.status);
  TEST_ASSERT_INT_WITHIN(5, 20, r.mean.x);
  TEST_ASSERT_INT_WITHIN(5, -650, r.mean.y);
  TEST_ASSERT_INT_WITHIN(5, -758, r.mean.z);
  TEST_ASSERT_INT_WITHIN(10, 1000, r.magnitudeMg);
}

void test_calibration_rejects_movement() {
  CalAccumulator moving;
  for (int i = 0; i < 50; i++) moving.add(rotated(kReading, 1, 0, 0, (i % 10) * 2.0));  // swaying 18 deg
  TEST_ASSERT_EQUAL((int)CalStatus::Moving, (int)moving.result().status);

  CalAccumulator accelerating;   // steady but 1.4 g: in a car taking a turn, a swing
  for (int i = 0; i < 50; i++) accelerating.add({0, -900, -1100});
  TEST_ASSERT_EQUAL((int)CalStatus::NotStill, (int)accelerating.result().status);

  CalAccumulator few;
  for (int i = 0; i < kCalMinSamples - 1; i++) few.add(kReading);
  TEST_ASSERT_EQUAL((int)CalStatus::TooFewSamples, (int)few.result().status);
}

void test_sensitivity_names() {
  TEST_ASSERT_EQUAL_STRING("Low", wakeSensitivityName(WakeSensitivity::Low));
  TEST_ASSERT_EQUAL_STRING("Med", wakeSensitivityName(WakeSensitivity::Medium));
  TEST_ASSERT_EQUAL_STRING("High", wakeSensitivityName(WakeSensitivity::High));
  TEST_ASSERT_EQUAL((int)WakeSensitivity::Medium, (int)wakeSensitivityFrom(7));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_tilt_wake_shortens_the_clock);
  RUN_TEST(test_phantom_streak);
  RUN_TEST(test_ulp_units);
  RUN_TEST(test_box_edges_match_the_definition);
  RUN_TEST(test_box_near_the_range_limits_does_not_wrap);
  RUN_TEST(test_zones_by_angle);
  RUN_TEST(test_sensitivity_widens_the_match_box);
  RUN_TEST(test_raise_wakes_once);
  RUN_TEST(test_still_in_the_pose_never_wakes);
  RUN_TEST(test_short_excursions_do_not_arm);
  RUN_TEST(test_a_single_match_sample_does_not_wake);
  RUN_TEST(test_confirmation_after_wake);
  RUN_TEST(test_calibration_accepts_a_still_pose);
  RUN_TEST(test_calibration_rejects_movement);
  RUN_TEST(test_sensitivity_names);
  return UNITY_END();
}
