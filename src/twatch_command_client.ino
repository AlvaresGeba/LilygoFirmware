/*
  twatch_command_client.ino

  Firmware for LILYGO T-Watch S3.

  The watch spends its time in deep sleep. A tap on the touch screen or a
  press of the side button wakes it and shows the clock for the "Sleep
  after" time from the settings. A tap while the clock is up opens the
  status menu (phone link, last server response) and starts BLE so the
  phone can reconnect and sync the time.

  Side button:
    - single press: sleep (or wake, from sleep)
    - double press: start recording, also straight from sleep
    - single press while recording: stop (or wait MAX_RECORD_SECONDS);
      the audio is streamed to the phone app over BLE and the server's
      response is shown on screen. Then the watch goes back to sleep.

  While a command is on its way (waiting for the phone, sending, waiting
  for the answer) the watch does not time out into sleep, so the answer or
  the error always reaches the screen; only on battery below 30 % the old
  timeouts apply. A single press still cancels and sleeps.

  Raise to wake: the BMA423's wrist-tilt gesture wakes the watch too. Both
  ext0 (touch) and ext1 (button) are taken, so the ULP coprocessor polls the
  accelerometer's INT1 line during deep sleep and wakes the CPU. A tilt wake
  shows the clock for only 4 s unless the watch is touched or the button is
  pressed; wakes with the arm hanging down go straight back to sleep.

  The status menu has a Settings button at the bottom: brightness (5-100%),
  interface color and sleep timeout are set with a touch slider; vibration,
  screen rotation and raise to wake toggle on tap. (The crown only clicks: on T-Watch S3
  it has no rotary encoder - LilyGoLib marks hasRotary = false.)
  Settings live in NVS. Every screen has a status bar on top with the
  time, the phone link and the battery level.

  Time keeping: the PCF8563 RTC (32.768 kHz quartz) is the reference. The
  ESP32-S3's own deep-sleep timer runs from an internal RC oscillator that
  drifts by percents, so the system clock is checked against the PCF8563 on
  every wake. The phone sets both on every connection, and once a day the
  watch wakes up by itself (screen off) to fetch the time if no connection
  happened in between. See lib/WatchCore/src/clock_sync.h.

  Wire protocol (must match BleWatchService.kt exactly):
    COMMAND_CHAR   notify, watch -> phone, chunked payload:
                   [u16 chunk_index][u16 total_chunks][payload bytes]
    STATUS_CHAR    notify, watch -> phone, ASCII "DONE" once last chunk sent
    RESPONSE_CHAR  write, phone -> watch, UTF-8 text to display. Lines
                   starting with OK / REJECTED / ERROR are final answers;
                   "Processing..." and "STATUS: ..." are progress only.
    TIME_CHAR      write, phone -> watch, ASCII "<unix seconds>,<utc offset
                   minutes>[,<milliseconds>]" - sets the clock and the RTC.
                   read: the watch's current "<unix seconds>.<ms>,<offset>".

  USB serial console (115200): "H" lists the commands - setting the time,
  clock diagnostics for drift tests, and a hardware/sensor probe.

  Build with PlatformIO: `pio run -t upload` in the repository root
  (platformio.ini pins the platform, libraries and the TFT_eSPI setup).
  Libraries: NimBLE-Arduino 1.4.x, TFT_eSPI, XPowersLib, SensorLib (BMA423
  feature engine); lib/WatchCore holds
  the hardware-independent logic (unit tests: `pio test -e native`).

  Pin numbers follow LilyGO's TTGO_TWatch_Library src/utilities.h,
  section USING_TWATCH_S3 (t-watch-s3 branch).
*/

#include <Wire.h>
#include <NimBLEDevice.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <driver/rtc_io.h>
#include <esp_private/esp_clk.h>
#include <esp_sleep.h>
#include <esp32s3/ulp.h>
#include <soc/rtc.h>
#include <soc/rtc_cntl_reg.h>
#include <soc/rtc_io_reg.h>
#include <sys/time.h>

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>
#include <SensorBMA423.hpp>

#include "clock_sync.h"
#include "power_policy.h"
#include "server_reply.h"
#include "text_wrap.h"
#include "time_format.h"
#include "wrist_wake.h"

// ---------------------------------------------------------------------
// Hardware pins (TTGO_TWatch_Library utilities.h, USING_TWATCH_S3)
// ---------------------------------------------------------------------
// The side button is not a GPIO: it is the AXP2101 PWRKEY, reported
// through the PMU interrupt line.
#define PMU_I2C_SDA       10    // shared with the PCF8563 RTC, BMA423 and DRV2605
#define PMU_I2C_SCL       11
#define PMU_IRQ_PIN       21
#define TOUCH_I2C_SDA     39
#define TOUCH_I2C_SCL     40
#define TOUCH_INT_PIN     16
#define TFT_BL_PIN        45    // backlight, powered from AXP2101 ALDO2
#define MIC_CLOCK_PIN     44    // PDM clock
#define MIC_DATA_PIN      47    // PDM data
#define RTC_INT_PIN       17    // PCF8563 alarm/timer output (unused)
#define BMA423_INT_PIN    14    // accelerometer INT1: raise to wake, polled by the ULP in deep sleep

// SX1262 LoRa radio (powered from ALDO4) and the GPS of the Plus model
// (DC3): both are off; the hardware probe switches them on briefly.
#define RADIO_SCK_PIN     3
#define RADIO_MISO_PIN    4
#define RADIO_MOSI_PIN    1
#define RADIO_CS_PIN      5
#define RADIO_RST_PIN     8
#define RADIO_BUSY_PIN    7
#define GPS_RX_PIN        41
#define GPS_TX_PIN        42

#define PCF8563_ADDR      0x51
#define FT6336_ADDR       0x38
#define DRV2605_ADDR      0x5A
#define BMA423_ADDR       0x19
#define AXP2101_ADDR      0x34

#define BL_LEDC_CHANNEL   0

// ---------------------------------------------------------------------
// BLE UUIDs - must match BleWatchService.kt on the phone
// ---------------------------------------------------------------------
#define SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define COMMAND_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define STATUS_CHAR_UUID  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define RESPONSE_CHAR_UUID "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
#define TIME_CHAR_UUID    "6e400005-b5a3-f393-e0a9-e50e24dcca9e"

// ---------------------------------------------------------------------
// Audio recording config
// ---------------------------------------------------------------------
#define SAMPLE_RATE       16000
#define MAX_RECORD_SECONDS 8
#define CHUNK_PAYLOAD_SIZE 508   // 512 - 4 header bytes, keeps under typical MTU
#define SEND_ATTEMPTS     3      // the link dropped mid-transfer: resend this often

// ---------------------------------------------------------------------
// Screen timeouts (ms); the clock timeout is a setting
// ---------------------------------------------------------------------
#define MENU_IDLE_MS      15000  // menu stays up this long after the last tap
#define SETTINGS_IDLE_MS  20000
#define RESPONSE_SHOW_MS  10000  // answer on screen before sleeping
#define WAIT_DIM_MS       30000  // waiting on a command: dim the backlight after this
// Only on battery below watch::kSendKeepAwakeMinBatteryPct; otherwise the
// watch waits as long as it takes.
#define PHONE_WAIT_LOWBAT_MS    20000  // recording done, waiting for the phone
#define RESPONSE_WAIT_LOWBAT_MS 60000  // audio sent, waiting for the server's answer

// ---------------------------------------------------------------------
// Time sync
// ---------------------------------------------------------------------
// The PCF8563 drifts by seconds per day at most, so a daily sync keeps the
// clock right; the phone also sets it on every connection. A failed
// background attempt (no phone around) is retried after 1 h, 2 h, 4 h...
static const watch::SyncSchedule kSyncSchedule = {
    24 * 3600,   // intervalS
    3600,        // firstRetryS
    300,         // minDelayS
};
#define BG_SYNC_WINDOW_MS 20000  // background sync: advertise this long, screen off
#define SYNC_PERSIST_S    3600   // write the last sync time to flash at most hourly

// RTC_SLOW_CLK during deep sleep, selectable from the console ("C").
// Measured on this watch with tools/clock_drift_test.py: the 136 kHz RC
// runs ~+2600 ppm fast in deep sleep; RC_FAST/256 (~68 kHz from the 17.5 MHz
// RC) runs ~23 % slow in deep sleep against its awake calibration (the
// RTC domain's lower sleep voltage, probably), and costs ~5 uA more. So
// the 136 kHz RC is the default, and the PCF8563 corrects either on wake.
enum SleepClock : uint8_t { SLEEP_CLOCK_RC136K = 0, SLEEP_CLOCK_RCFAST_D256 = 1 };
#define SLEEP_CLOCK_DEFAULT  SLEEP_CLOCK_RC136K
#define SLEEP_CLOCK_CAL_CYCLES 3000   // IDF's default for RC_FAST/256: ~44 ms

// ---------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------
#define SCREEN_W          240
#define SCREEN_H          240
#define BAR_H             24     // status bar
#define ROW_Y0            BAR_H  // settings rows
#define ROW_H             30
#define SETTINGS_ROWS     6
#define BACK_Y            (ROW_Y0 + SETTINGS_ROWS * ROW_H)
#define TEXT_MARGIN       10

static const size_t RECORD_BUFFER_BYTES = SAMPLE_RATE * 2 * MAX_RECORD_SECONDS; // 16-bit mono
uint8_t *recordBuffer = nullptr;    // allocated in PSRAM in setup()
size_t recordedBytes = 0;
volatile bool isRecording = false;
uint8_t sendAttempts = 0;

TFT_eSPI tft = TFT_eSPI();
bool backlightDimmed = false;

XPowersPMU pmu;
volatile bool pmuIrq = false;

Preferences prefs;
int tzOffsetMin = 0;                // local time = UTC + tzOffsetMin

NimBLEServer *bleServer = nullptr;
NimBLECharacteristic *commandChar = nullptr;
NimBLECharacteristic *statusChar = nullptr;
NimBLECharacteristic *responseChar = nullptr;
bool bleStarted = false;
volatile bool phoneConnected = false;

// Mailbox from the BLE callbacks (NimBLE host task) to loop(): the display
// and the I2C buses are only touched from the main task. bleMux guards the
// buffers; the flags tell loop() there is something to collect.
#define RESPONSE_MAX      512    // the app writes at most 500 bytes
portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool linkChanged = false;
volatile bool responseArrived = false;
volatile bool timeArrived = false;
char pendingResponse[RESPONSE_MAX + 1];
watch::TimeMessage pendingTime;

// Survive deep sleep (reset on power-on).
RTC_DATA_ATTR char lastResponse[RESPONSE_MAX + 1] = "";
RTC_DATA_ATTR int64_t lastSyncEpoch = 0;       // last time set by the phone/console, UTC
RTC_DATA_ATTR uint8_t bgSyncFailures = 0;      // background attempts that found no phone
RTC_DATA_ATTR int64_t lastWakeCorrectionUs = 0;  // diagnostics: RTC correction at the last boot
RTC_DATA_ATTR bool rtcResyncDisabled = false;  // diagnostics: measure the raw sleep clock
int64_t lastTickCorrectionUs = 0;   // diagnostics: sub-second step from the RTC tick, this boot

enum WakePlan : uint8_t { WAKE_PLAN_NONE, WAKE_PLAN_SYNC, WAKE_PLAN_TEST };
RTC_DATA_ATTR uint8_t wakePlan = WAKE_PLAN_NONE;  // why the sleep timer was armed
uint32_t testSleepSeconds = 0;      // console "Z": next sleep wakes by timer

enum Screen : uint8_t {
  SCREEN_CLOCK,
  SCREEN_MENU,
  SCREEN_SETTINGS,
  SCREEN_EDIT,                      // one setting with a slider
  SCREEN_RECORDING,
  SCREEN_WAIT_PHONE,
  SCREEN_WAIT_RESPONSE,
  SCREEN_RESPONSE,
  SCREEN_BG_SYNC,                   // woken by the timer to sync the time, screen off
};
Screen screen = SCREEN_CLOCK;
uint32_t screenSince = 0;           // millis() when the screen was entered or last tapped
uint32_t litSince = 0;              // millis() of the last activity that lit the backlight
char waitDetail[96] = "";           // progress line from the phone while waiting

// Response timer: runs from the moment a command is sent off (the recording
// stops) to its final answer or error. The watch does not sleep meanwhile,
// so millis() is continuous.
bool commandPending = false;
uint32_t commandStartedAt = 0;      // millis()
RTC_DATA_ATTR uint32_t lastResponseMs = 0;  // how long lastResponse took; 0 = not timed

enum Press : uint8_t { PRESS_NONE, PRESS_SINGLE, PRESS_DOUBLE };

// Declared up here because the Arduino build puts its generated function
// prototypes above the first function, and some of them use these types.
enum ReplyIcon : uint8_t { ICON_CHECK, ICON_CROSS, ICON_BANG, ICON_DOTS };
struct ReplyStyle { uint16_t color; const char *title; uint8_t haptic; ReplyIcon icon; };

void goToSleep();
void startBle();
void enterScreen(Screen s);
void enterScreenKeepTimer();

// ---------------------------------------------------------------------
// Settings (NVS)
// ---------------------------------------------------------------------

#define BRIGHTNESS_MIN_PCT 5     // below this the screen is unreadable outdoors

struct AccentColor { const char *name; uint16_t rgb565; };
static const AccentColor ACCENTS[] = {
    {"Cyan", TFT_CYAN},  {"Green", TFT_GREEN},   {"Orange", TFT_ORANGE},
    {"Violet", TFT_VIOLET}, {"Red", TFT_RED},    {"Yellow", TFT_YELLOW},
    {"White", TFT_WHITE},
};
static const uint8_t ACCENT_COUNT = sizeof(ACCENTS) / sizeof(ACCENTS[0]);

static const uint8_t SLEEP_SECONDS[] = {5, 10, 15, 30, 60};
static const uint8_t SLEEP_OPTIONS = sizeof(SLEEP_SECONDS);

struct Settings {
  uint8_t brightness;   // percent, BRIGHTNESS_MIN_PCT..100
  uint8_t accent;       // index into ACCENTS
  uint8_t sleepAfter;   // index into SLEEP_SECONDS
  bool vibration;
  bool flipped;         // screen rotated 180 degrees
  bool raiseToWake;     // wrist tilt wakes the watch (BMA423 + ULP)
  SleepClock sleepClock;
} cfg;

uint16_t accent() { return ACCENTS[cfg.accent].rgb565; }
uint32_t clockShowMs() { return SLEEP_SECONDS[cfg.sleepAfter] * 1000UL; }

void loadSettings() {
  cfg.brightness = constrain(prefs.getUChar("brightpct", 60), BRIGHTNESS_MIN_PCT, 100);
  cfg.accent = min<uint8_t>(prefs.getUChar("accent", 0), ACCENT_COUNT - 1);
  cfg.sleepAfter = min<uint8_t>(prefs.getUChar("sleep", 1), SLEEP_OPTIONS - 1);
  cfg.vibration = prefs.getBool("vibro", true);
  cfg.flipped = prefs.getBool("flip", false);
  cfg.raiseToWake = prefs.getBool("raise", true);
  cfg.sleepClock = prefs.getUChar("slowclk", SLEEP_CLOCK_DEFAULT) == SLEEP_CLOCK_RC136K
                       ? SLEEP_CLOCK_RC136K : SLEEP_CLOCK_RCFAST_D256;
}

void saveSettings() {
  prefs.putUChar("brightpct", cfg.brightness);
  prefs.putUChar("accent", cfg.accent);
  prefs.putUChar("sleep", cfg.sleepAfter);
  prefs.putBool("vibro", cfg.vibration);
  prefs.putBool("flip", cfg.flipped);
  prefs.putBool("raise", cfg.raiseToWake);
}

void applyBrightness() {
  ledcWrite(BL_LEDC_CHANNEL, cfg.brightness * 255 / 100);
  backlightDimmed = false;
  litSince = millis();
}

void dimBacklight() {
  ledcWrite(BL_LEDC_CHANNEL, BRIGHTNESS_MIN_PCT * 255 / 100);
  backlightDimmed = true;
}

// ST7789 sleep in/out, with the backlight.
void displayOff() {
  ledcWrite(BL_LEDC_CHANNEL, 0);
  tft.writecommand(0x28);           // DISPOFF
  tft.writecommand(0x10);           // SLPIN
}

void displayOn() {
  tft.writecommand(0x11);           // SLPOUT
  delay(120);                       // ST7789: 120 ms before the next command
  tft.writecommand(0x29);           // DISPON
  applyBrightness();
}

watch::Battery readBattery() {
  return {pmu.getBatteryPercent(), pmu.isVbusIn() || pmu.isCharging()};
}

// ---------------------------------------------------------------------
// Vibration (DRV2605, register sequence as in SensorLib's SensorDRV2605)
// ---------------------------------------------------------------------

#define HAPTIC_CLICK      1      // ROM effect "strong click 100%"
#define HAPTIC_DOUBLE     10     // ROM effect "double click 100%"
#define HAPTIC_TICK       3      // ROM effect "strong click 30%": slider steps
#define HAPTIC_ALERT      14     // ROM effect "strong buzz 100%": errors

bool hapticReady = false;

void drvWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DRV2605_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t i2cRead8(TwoWire &bus, uint8_t addr, uint8_t reg) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.endTransmission(false);
  bus.requestFrom(addr, (uint8_t)1);
  return bus.available() ? bus.read() : 0;
}

uint8_t drvRead(uint8_t reg) { return i2cRead8(Wire, DRV2605_ADDR, reg); }

// The DRV2605 is powered from BLDO2: off unless vibration is enabled.
void applyHaptics() {
  if (!cfg.vibration) {
    pmu.disableBLDO2();
    hapticReady = false;
    return;
  }
  if (hapticReady) return;
  pmu.setBLDO2Voltage(3300);
  pmu.enableBLDO2();
  delay(5);
  drvWrite(0x01, 0x00);             // MODE: out of standby, internal trigger
  drvWrite(0x02, 0x00);             // no real-time playback
  drvWrite(0x0D, 0x00);             // overdrive / sustain / brake offsets
  drvWrite(0x0E, 0x00);
  drvWrite(0x0F, 0x00);
  drvWrite(0x10, 0x00);
  drvWrite(0x13, 0x64);             // audio-to-vibe max
  drvWrite(0x1A, drvRead(0x1A) & 0x7F);  // FEEDBACK: ERM motor
  drvWrite(0x1D, drvRead(0x1D) | 0x20);  // CONTROL3: ERM open loop
  drvWrite(0x03, 0x01);             // ROM library A (ERM)
  hapticReady = true;
}

void buzz(uint8_t effect) {
  if (!cfg.vibration || !hapticReady) return;
  drvWrite(0x04, effect);           // waveform sequence slot 1
  drvWrite(0x05, 0x00);             // end of sequence
  drvWrite(0x0C, 0x01);             // GO
}

// ---------------------------------------------------------------------
// Clock: system time, PCF8563 RTC, sleep clock
// ---------------------------------------------------------------------

static uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

int64_t nowUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

void setNowUs(int64_t us) {
  struct timeval tv = {(time_t)(us / 1000000), (suseconds_t)(us % 1000000)};
  settimeofday(&tv, nullptr);
}

// The RTC holds UTC. Returns false if it never got a valid time.
bool rtcRead(time_t &utc) {
  Wire.beginTransmission(PCF8563_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(PCF8563_ADDR, 7) != 7) return false;
  uint8_t r[7];
  for (int i = 0; i < 7; i++) r[i] = Wire.read();
  if (r[0] & 0x80) return false;    // VL bit: oscillator stopped, time invalid

  struct tm t = {};
  t.tm_sec = bcd2dec(r[0] & 0x7F);
  t.tm_min = bcd2dec(r[1] & 0x7F);
  t.tm_hour = bcd2dec(r[2] & 0x3F);
  t.tm_mday = bcd2dec(r[3] & 0x3F);
  t.tm_mon = bcd2dec(r[5] & 0x1F) - 1;
  t.tm_year = bcd2dec(r[6]) + 100;  // 20xx
  utc = mktime(&t);                 // TZ is unset, so mktime works in UTC
  return watch::isValidEpoch(utc);
}

void rtcWrite(time_t utc) {
  struct tm t;
  gmtime_r(&utc, &t);
  Wire.beginTransmission(PCF8563_ADDR);
  Wire.write(0x02);
  Wire.write(dec2bcd(t.tm_sec));    // also clears VL
  Wire.write(dec2bcd(t.tm_min));
  Wire.write(dec2bcd(t.tm_hour));
  Wire.write(dec2bcd(t.tm_mday));
  Wire.write(t.tm_wday);
  Wire.write(dec2bcd(t.tm_mon + 1));
  Wire.write(dec2bcd(t.tm_year - 100));
  Wire.endTransmission();
}

// The seconds register alone, for timing the RTC's second boundary.
int rtcSecondsRegister() {
  uint8_t v = i2cRead8(Wire, PCF8563_ADDR, 0x02);
  return bcd2dec(v & 0x7F);
}

bool clockValid() { return watch::isValidEpoch(time(nullptr)); }

// STOP bit (Control_1 bit 5): stops the PCF8563 and clears its divider chain.
void rtcSetStop(bool stop) {
  Wire.beginTransmission(PCF8563_ADDR);
  Wire.write(0x00);
  Wire.write(stop ? 0x20 : 0x00);
  Wire.endTransmission();
}

// After a new time is set, the RTC is loaded so that its seconds tick in
// step with the system clock. Writing the time registers alone does not
// reset the PCF8563's 1 Hz divider, so the new second would tick at an
// arbitrary phase, up to a second off. Instead the clock is stopped (which
// clears the divider), loaded with second B-1 and released so that its
// first tick, which comes a fixed RTC_RELEASE_TO_TICK_US after the release,
// lands on the system clock's second B. The delay was measured with the
// console's "R" on this watch (tick 493 ms late with 507874 us).
bool rtcWritePending = false;
static const int64_t RTC_RELEASE_TO_TICK_US = 1000928;

void serviceRtcWrite(bool now) {
  if (!rtcWritePending) return;
  const int64_t us = nowUs();
  int64_t tickAt = (us / 1000000 + 1) * 1000000;
  int64_t releaseAt = tickAt - RTC_RELEASE_TO_TICK_US;
  // The stop and the time write take ~1 ms on the bus: leave room for them.
  while (releaseAt - us < 5000) {
    tickAt += 1000000;
    releaseAt += 1000000;
  }
  // loop() runs every <= 20 ms, so this window catches every second; the
  // rest is a short busy wait. Before sleeping it waits up to a second.
  if (!now && releaseAt - us > 30000) return;
  rtcSetStop(true);
  rtcWrite((time_t)(tickAt / 1000000 - 1));
  while (nowUs() < releaseAt) {
  }
  rtcSetStop(false);
  rtcWritePending = false;
}

void rememberSync(int64_t epoch) {
  lastSyncEpoch = epoch;
  bgSyncFailures = 0;
  if (llabs(epoch - prefs.getLong64("synced", 0)) >= SYNC_PERSIST_S) prefs.putLong64("synced", epoch);
}

// A new time from the phone or the console.
void applyTime(const watch::TimeMessage &m, const char *source) {
  const int64_t newUs = m.epoch * 1000000 + m.micros;
  const int64_t offsetMs = clockValid() ? (nowUs() - newUs) / 1000 : 0;
  const int64_t sinceLast = watch::isValidEpoch(lastSyncEpoch) ? m.epoch - lastSyncEpoch : 0;
  setNowUs(newUs);
  rtcWritePending = true;
  if (m.hasTz && m.tzMinutes != tzOffsetMin) {
    tzOffsetMin = m.tzMinutes;
    prefs.putInt("tz", tzOffsetMin);
  }
  rememberSync(m.epoch);
  // How far off the clock was: the drift since the last sync.
  Serial.printf("time set from %s: %lld.%06u tz %d, clock was off by %+lld ms after %lld s\n",
                source, (long long)m.epoch, (unsigned)m.micros, tzOffsetMin,
                (long long)offsetMs, (long long)sinceLast);
}

// The wake correction only knows the RTC's whole second (+-0.5 s). The
// moment its seconds register changes is an exact second boundary, so while
// awake loop() watches for it once, which takes at most a second, and steps
// the system clock onto it, to within one loop pass (loop() runs fast meanwhile).
bool rtcTickLockPending = false;
int rtcTickLockFrom = -1;           // seconds register at the first poll

void serviceRtcTickLock() {
  if (!rtcTickLockPending) return;
  // A new time from the phone or console supersedes it (and is written to
  // the RTC in phase anyway).
  if (rtcWritePending || rtcResyncDisabled) {
    rtcTickLockPending = false;
    return;
  }
  const int sec = rtcSecondsRegister();
  if (rtcTickLockFrom < 0) {
    rtcTickLockFrom = sec;
    return;
  }
  if (sec == rtcTickLockFrom) return;
  const int64_t us = nowUs();
  rtcTickLockPending = false;
  time_t rtc;
  if (!rtcRead(rtc)) return;
  lastTickCorrectionUs = watch::rtcTickCorrectionUs(us, rtc);
  setNowUs(nowUs() + lastTickCorrectionUs);
}

// Every boot, also after a deep-sleep wake. The system clock kept running
// in deep sleep on the drifting RC oscillator; the PCF8563 is the
// reference. After a power-on the system clock starts from zero and is
// simply loaded from the RTC.
void loadClock() {
  tzOffsetMin = prefs.getInt("tz", 0);
  if (!watch::isValidEpoch(lastSyncEpoch)) lastSyncEpoch = prefs.getLong64("synced", 0);

  time_t rtc;
  if (rtcRead(rtc)) {
    const int64_t correction = watch::rtcCorrectionUs(nowUs(), rtc);
    lastWakeCorrectionUs = correction;
    if (correction != 0 && !rtcResyncDisabled) setNowUs(nowUs() + correction);
    rtcTickLockPending = true;
  } else if (clockValid()) {
    // The RTC lost its time (VL set, e.g. battery ran flat) but the system
    // clock survived: put it back.
    rtcWritePending = true;     // written in phase by serviceRtcWrite()
  }
}

void localNow(struct tm &out) {
  time_t local = time(nullptr) + (time_t)tzOffsetMin * 60;
  gmtime_r(&local, &out);
}

const char *sleepClockName(SleepClock c) { return c == SLEEP_CLOCK_RC136K ? "rc136k" : "rcfast256"; }

// Switches RTC_SLOW_CLK, which times deep sleep, to the configured source
// and calibrates it against the 40 MHz crystal right before sleeping, so
// the calibration reflects the current temperature. The IDF startup code
// switches back to the 136 kHz RC on the next boot; the time slept is
// accounted with the calibration set here.
void prepareSleepClock() {
  uint32_t cal;
  if (cfg.sleepClock == SLEEP_CLOCK_RCFAST_D256) {
    rtc_clk_8m_enable(true, true);
    // Measured while still running from the RC, then a quick switch:
    // esp_clk_slowclk_cal_set() first books the ticks so far at the old
    // period, so the time base does not jump.
    cal = rtc_clk_cal(RTC_CAL_8MD256, SLEEP_CLOCK_CAL_CYCLES);
    if (cal == 0) {
      Serial.println("sleep clock: RC_FAST/256 calibration failed, staying on RC");
      return;
    }
    esp_clk_slowclk_cal_set(esp_clk_slowclk_cal_get());
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_8MD256);
    esp_clk_slowclk_cal_set(cal);
    // The 17.5 MHz oscillator must keep running in deep sleep, or the RTC
    // timer stops along with the clock.
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC8M, ESP_PD_OPTION_ON);
  } else {
    cal = rtc_clk_cal(RTC_CAL_RTC_MUX, 1024);
    if (cal) esp_clk_slowclk_cal_set(cal);
  }
}

void printClockReport() {
  const int64_t us = nowUs();
  time_t rtc = 0;
  const bool rtcOk = rtcRead(rtc);
  Serial.printf("CLK sys=%lld.%06lld rtc=%lld rtc_ok=%d tz=%d src=%s cal=%u corr_us=%lld "
                "resync=%d synced=%lld bgfail=%u wake=%d tick_us=%lld\n",
                (long long)(us / 1000000), (long long)(us % 1000000), (long long)rtc, rtcOk,
                tzOffsetMin, sleepClockName(cfg.sleepClock), (unsigned)esp_clk_slowclk_cal_get(),
                (long long)lastWakeCorrectionUs, !rtcResyncDisabled, (long long)lastSyncEpoch,
                bgSyncFailures, (int)esp_sleep_get_wakeup_cause(), (long long)lastTickCorrectionUs);
}

// Waits for the PCF8563 seconds register to change and reports that
// moment: on the host, comparing "rtc" with the arrival time of the line
// gives the RTC's error to a millisecond, despite its 1 s resolution.
void printRtcTick() {
  const int start = rtcSecondsRegister();
  const uint32_t since = millis();
  while (rtcSecondsRegister() == start) {
    if (millis() - since > 1500) {
      Serial.println("TICK error: RTC seconds do not advance");
      return;
    }
  }
  const int64_t us = nowUs();
  time_t rtc = 0;
  rtcRead(rtc);
  Serial.printf("TICK rtc=%lld sys=%lld.%06lld\n", (long long)rtc, (long long)(us / 1000000),
                (long long)(us % 1000000));
}

// ---------------------------------------------------------------------
// Touch (FT6336)
// ---------------------------------------------------------------------

void setupTouch() {
  Wire1.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL);
  // G_MODE = polling: INT stays low while a finger is down. Deep sleep
  // wakes on that level, so a short trigger pulse could be missed.
  Wire1.beginTransmission(FT6336_ADDR);
  Wire1.write(0xA4);
  Wire1.write(0x00);
  Wire1.endTransmission();
}

// Reads the first touch point in screen coordinates. The panel reports
// coordinates for LilyGO's default rotation 2 (LV_Helper.cpp
// touchpad_read), so rotation 0 mirrors both axes.
bool readTouch(int16_t &x, int16_t &y) {
  Wire1.beginTransmission(FT6336_ADDR);
  Wire1.write(0x02);                // TD_STATUS, then P1_XH, P1_XL, P1_YH, P1_YL
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(FT6336_ADDR, 5) != 5) return false;
  uint8_t r[5];
  for (int i = 0; i < 5; i++) r[i] = Wire1.read();
  uint8_t n = r[0] & 0x0F;
  if (n == 0 || n > 2) return false;
  int16_t rx = ((r[1] & 0x0F) << 8) | r[2];
  int16_t ry = ((r[3] & 0x0F) << 8) | r[4];
  if (cfg.flipped) {                // rotation 2
    x = rx;
    y = ry;
  } else {                          // rotation 0
    x = SCREEN_W - rx;
    y = SCREEN_H - ry;
  }
  return true;
}

bool touchDown() {
  int16_t x, y;
  return readTouch(x, y);
}

bool fingerDown = false;            // as of the last tapped() call; x/y valid while true

// True once per new finger-down; x/y is where it landed, and keeps
// tracking the finger while fingerDown is set (for sliders).
bool tapped(int16_t &x, int16_t &y) {
  static bool wasDown = true;       // ignore the finger that woke us until it lifts
  bool down = readTouch(x, y);
  bool tap = down && !wasDown;
  wasDown = down;
  fingerDown = down;
  if (tap) Serial.printf("tap %d,%d\n", x, y);
  return tap;
}

// ---------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------

int lastBarMinute = -1;

void drawStatusBar() {
  tft.fillRect(0, 0, SCREEN_W, BAR_H, TFT_BLACK);
  tft.drawFastHLine(0, BAR_H - 1, SCREEN_W, accent());
  tft.setTextSize(1);

  // phone link
  uint16_t linkColor = phoneConnected ? accent() : TFT_DARKGREY;
  if (bleStarted) tft.fillCircle(12, BAR_H / 2 - 1, 4, linkColor);
  else tft.drawCircle(12, BAR_H / 2 - 1, 4, TFT_DARKGREY);

  // time
  char buf[12];
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(accent(), TFT_BLACK);
  if (clockValid()) {
    struct tm t;
    localNow(t);
    snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
    lastBarMinute = t.tm_min;
  } else {
    strcpy(buf, "--:--");
  }
  tft.drawString(buf, SCREEN_W / 2, BAR_H / 2 - 1, 2);

  // battery
  int batt = pmu.getBatteryPercent();
  if (batt >= 0) {
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(batt <= 15 ? TFT_ORANGE : TFT_LIGHTGREY, TFT_BLACK);
    snprintf(buf, sizeof(buf), "%s%d%%", pmu.isCharging() ? "+" : "", batt);
    tft.drawString(buf, SCREEN_W - 6, BAR_H / 2 - 1, 2);
  }
}

// Clears everything below the status bar.
void clearBody() { tft.fillRect(0, BAR_H, SCREEN_W, SCREEN_H - BAR_H, TFT_BLACK); }

void showStatus(const char *line1, const char *line2 = "") {
  clearBody();
  drawStatusBar();
  int cy = BAR_H + (SCREEN_H - BAR_H) / 2;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString(line1, SCREEN_W / 2, cy - 12);
  if (strlen(line2) > 0) {
    tft.setTextSize(1);
    tft.drawString(line2, SCREEN_W / 2, cy + 14);
  }
}

// Width of s[0..len) in the current font and text size (for watch::wrapText).
int measureText(const char *s, size_t len, void *) {
  char buf[256];
  len = min(len, sizeof(buf) - 1);
  memcpy(buf, s, len);
  buf[len] = '\0';
  return tft.textWidth(buf);
}

// Word-wraps text into the box x..x+w, y..maxY in the built-in 6x8 font
// scaled by `size`; text that does not fit ends in "...". Returns the y
// below the last line.
int drawTextBox(uint8_t size, const char *text, int x, int y, int w, int maxY, uint16_t color) {
  tft.setTextFont(1);
  tft.setTextSize(size);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  const int lineH = tft.fontHeight() + 4 * size;

  watch::TextSpan lines[12];
  size_t maxLines = lineH > 0 ? max(0, (maxY - y) / lineH) : 0;
  maxLines = min(maxLines, sizeof(lines) / sizeof(lines[0]));
  bool truncated = false;
  const size_t n = watch::wrapText(text, w, measureText, nullptr, lines, maxLines, &truncated);

  static const char kEllipsis[] = "...";
  char buf[256];
  for (size_t i = 0; i < n; i++) {
    const char *start = text + lines[i].start;
    size_t len = min(lines[i].len, sizeof(buf) - sizeof(kEllipsis));
    const bool last = i + 1 == n;
    if (last && truncated) len = watch::fitWithSuffix(start, len, kEllipsis, w, measureText, nullptr);
    memcpy(buf, start, len);
    buf[len] = '\0';
    if (last && truncated) strcat(buf, kEllipsis);
    tft.drawString(buf, x, y);
    y += lineH;
  }
  tft.setTextSize(1);
  return y;
}

// How each kind of answer looks and feels (ReplyStyle is declared at the top).

ReplyStyle replyStyle(watch::ReplyKind kind) {
  switch (kind) {
    case watch::ReplyKind::Ok:       return {TFT_GREEN, "OK", HAPTIC_DOUBLE, ICON_CHECK};
    case watch::ReplyKind::Rejected: return {TFT_ORANGE, "REJECTED", HAPTIC_ALERT, ICON_BANG};
    case watch::ReplyKind::Error:    return {TFT_RED, "ERROR", HAPTIC_ALERT, ICON_CROSS};
    case watch::ReplyKind::Status:   return {TFT_LIGHTGREY, "...", 0, ICON_DOTS};
    case watch::ReplyKind::Info:     break;
  }
  return {accent(), "ANSWER", HAPTIC_DOUBLE, ICON_CHECK};
}

// A filled, anti-aliased disc in the style's color with the glyph cut out
// in black: a check mark, a cross, "!" or "...".
void drawReplyIcon(int cx, int cy, int r, const ReplyStyle &style) {
  tft.fillSmoothCircle(cx, cy, r, style.color, TFT_BLACK);
  const float w = max(2.0f, r * 0.24f);   // stroke width
  const uint16_t ink = TFT_BLACK;
  switch (style.icon) {
    case ICON_CHECK:
      tft.drawWideLine(cx - 0.45f * r, cy + 0.02f * r, cx - 0.12f * r, cy + 0.36f * r, w, ink, style.color);
      tft.drawWideLine(cx - 0.12f * r, cy + 0.36f * r, cx + 0.46f * r, cy - 0.32f * r, w, ink, style.color);
      break;
    case ICON_CROSS:
      tft.drawWideLine(cx - 0.36f * r, cy - 0.36f * r, cx + 0.36f * r, cy + 0.36f * r, w, ink, style.color);
      tft.drawWideLine(cx + 0.36f * r, cy - 0.36f * r, cx - 0.36f * r, cy + 0.36f * r, w, ink, style.color);
      break;
    case ICON_BANG:
      tft.drawWideLine(cx, cy - 0.52f * r, cx, cy + 0.10f * r, w, ink, style.color);
      tft.drawSpot(cx, cy + 0.44f * r, w * 0.6f, ink, style.color);
      break;
    case ICON_DOTS:
      for (int i = -1; i <= 1; i++) tft.drawSpot(cx + i * 0.42f * r, cy, w * 0.55f, ink, style.color);
      break;
  }
}

#define REPLY_HEADER_H    50     // icon, label, response time, divider

// The head of the answer screen: icon and label in the answer's color on
// the left, how long the answer took on the right, a thin rule below.
// Returns the y below it.
int drawReplyHeader(int y, const ReplyStyle &style, uint32_t tookMs) {
  const int cy = y + 22;
  const int r = 17;
  drawReplyIcon(TEXT_MARGIN + r, cy, r, style);

  tft.setTextSize(1);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(style.color, TFT_BLACK);
  tft.drawString(style.title, TEXT_MARGIN + 2 * r + 10, cy + 1, 4);

  if (tookMs) {
    char took[16];
    watch::formatDuration(tookMs, took, sizeof(took));
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString(took, SCREEN_W - TEXT_MARGIN, cy + 1, 2);
  }
  tft.drawFastHLine(TEXT_MARGIN, y + REPLY_HEADER_H - 6, SCREEN_W - 2 * TEXT_MARGIN, style.color);
  return y + REPLY_HEADER_H;
}

// The answer screen: what kind of answer it is (OK, REJECTED, ERROR) and
// how long it took, then the text, wrapped by word.
void showServerResponse(const char *response) {
  clearBody();
  drawStatusBar();
  const watch::Reply reply = watch::parseReply(response);
  const ReplyStyle style = replyStyle(reply.kind);
  const int y = drawReplyHeader(BAR_H + 4, style, lastResponseMs);

  const char *body = reply.body[0] ? reply.body : "(no details)";
  drawTextBox(2, body, TEXT_MARGIN, y, SCREEN_W - 2 * TEXT_MARGIN, SCREEN_H - 4, TFT_WHITE);
}

void drawClock() {
  clearBody();
  drawStatusBar();
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  int cx = SCREEN_W / 2;

  if (!clockValid()) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("--:--", cx, 110, 7);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("time not set", cx, 160, 2);
    tft.drawString("tap to open the menu and sync", cx, 180, 2);
    return;
  }

  struct tm t;
  localNow(t);
  char buf[24];
  tft.setTextColor(accent(), TFT_BLACK);
  snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
  tft.drawString(buf, cx, 110, 7);

  static const char *days[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  snprintf(buf, sizeof(buf), "%s %02d.%02d.%04d", days[t.tm_wday], t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
  tft.drawString(buf, cx, 165, 4);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("tap: info   2x button: record", cx, 222, 1);
}

// Settings button: full width so a slightly-off tap still lands.
#define MENU_BTN_Y        (SCREEN_H - 44)
#define MENU_BTN_H        40

// The status screen: phone link, the last server answer, Settings button.
void drawMenu() {
  clearBody();
  drawStatusBar();
  tft.setTextDatum(MC_DATUM);
  int cx = SCREEN_W / 2;

  tft.setTextColor(phoneConnected ? accent() : TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString(phoneConnected ? "Connected" : "Waiting for phone...", cx, 54);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Double-press button to record", cx, 78);

  if (lastResponse[0]) {
    const watch::Reply reply = watch::parseReply(lastResponse);
    const ReplyStyle style = replyStyle(reply.kind);
    char title[32];
    snprintf(title, sizeof(title), "Last answer: %s", style.title);
    drawReplyIcon(TEXT_MARGIN + 7, 105, 7, style);
    tft.setTextDatum(ML_DATUM);
    tft.setTextColor(style.color, TFT_BLACK);
    tft.drawString(title, TEXT_MARGIN + 20, 106, 2);
    if (lastResponseMs) {
      char took[16];
      watch::formatDuration(lastResponseMs, took, sizeof(took));
      tft.setTextDatum(MR_DATUM);
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      tft.drawString(took, SCREEN_W - TEXT_MARGIN, 106, 2);
    }
    drawTextBox(1, reply.body, TEXT_MARGIN, 116, SCREEN_W - 2 * TEXT_MARGIN,
                MENU_BTN_Y - 4, TFT_LIGHTGREY);
  }

  tft.drawRoundRect(8, MENU_BTN_Y, SCREEN_W - 16, MENU_BTN_H, 8, accent());
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(accent(), TFT_BLACK);
  tft.drawString("Settings", cx, MENU_BTN_Y + MENU_BTN_H / 2, 4);
}

#define WAIT_TIMER_Y      100    // center of the response timer on the waiting screens

// The running response timer, big, in the 7-segment font. Redrawn in place
// every second (the padding erases the previous value).
void drawWaitTimer() {
  if (!commandPending) return;
  char buf[12];
  watch::formatTimer(millis() - commandStartedAt, buf, sizeof(buf));
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("8:88:88", 7));
  tft.drawString(buf, SCREEN_W / 2, WAIT_TIMER_Y, 7);
  tft.setTextPadding(0);
}

// The two waiting screens of a command on its way.
void drawWait(const char *title) {
  clearBody();
  drawStatusBar();
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(title, SCREEN_W / 2, BAR_H + 22);
  tft.setTextSize(1);
  drawWaitTimer();
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(phoneConnected ? accent() : TFT_ORANGE, TFT_BLACK);
  tft.drawString(phoneConnected ? "phone connected" : "phone not connected", SCREEN_W / 2, 146, 2);
  if (waitDetail[0]) {
    drawTextBox(1, waitDetail, TEXT_MARGIN, 162, SCREEN_W - 2 * TEXT_MARGIN, SCREEN_H - 24,
                TFT_LIGHTGREY);
  }
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("press button to cancel", SCREEN_W / 2, SCREEN_H - 12, 1);
}

// ---------------------------------------------------------------------
// Settings screen: a list of settings, Back at the bottom (returns to the
// status menu). Tapping Vibration, Rotate or Raise to wake flips it; tapping Brightness,
// Color or Sleep after opens SCREEN_EDIT, where the value follows a finger
// dragged along a slider.
// ---------------------------------------------------------------------

#define ROW_BRIGHTNESS    0
#define ROW_COLOR         1
#define ROW_SLEEP         2
#define ROW_VIBRATION     3
#define ROW_ROTATE        4
#define ROW_RAISE         5

static const char *SETTING_LABELS[SETTINGS_ROWS] = {"Brightness", "Color", "Sleep after", "Vibration",
                                                    "Rotate", "Raise to wake"};

bool rowHasSlider(int row) { return row == ROW_BRIGHTNESS || row == ROW_COLOR || row == ROW_SLEEP; }

void settingValue(int row, char *buf, size_t len) {
  switch (row) {
    case ROW_BRIGHTNESS: snprintf(buf, len, "%d%%", cfg.brightness); break;
    case ROW_COLOR:      snprintf(buf, len, "%s", ACCENTS[cfg.accent].name); break;
    case ROW_SLEEP:      snprintf(buf, len, "%ds", SLEEP_SECONDS[cfg.sleepAfter]); break;
    case ROW_VIBRATION:  snprintf(buf, len, "%s", cfg.vibration ? "On" : "Off"); break;
    case ROW_ROTATE:     snprintf(buf, len, "%s", cfg.flipped ? "180" : "0"); break;
    case ROW_RAISE:      snprintf(buf, len, "%s", cfg.raiseToWake ? "On" : "Off"); break;
  }
}

void drawSettingsRow(int row) {
  int y = ROW_Y0 + row * ROW_H;
  int cy = y + ROW_H / 2;
  tft.fillRect(0, y, SCREEN_W, ROW_H, TFT_BLACK);
  tft.drawFastHLine(8, y + ROW_H - 1, SCREEN_W - 16, TFT_DARKGREY);

  tft.setTextSize(1);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(SETTING_LABELS[row], 8, cy, 2);

  char buf[16];
  settingValue(row, buf, sizeof(buf));
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(accent(), TFT_BLACK);
  tft.drawString(buf, SCREEN_W - 26, cy, 2);
  if (rowHasSlider(row)) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString(">", SCREEN_W - 8, cy, 2);
  }
}

void drawSettings() {
  clearBody();
  drawStatusBar();
  for (int row = 0; row < SETTINGS_ROWS; row++) drawSettingsRow(row);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(accent(), TFT_BLACK);
  tft.drawString("Back", SCREEN_W / 2, BACK_Y + (SCREEN_H - BACK_Y) / 2, 2);
}

int editRow = ROW_BRIGHTNESS;       // the setting SCREEN_EDIT is showing

// Returns the row to edit, -1 for nothing, SETTINGS_ROWS for Back.
int settingsTap(int16_t x, int16_t y) {
  if (y >= BACK_Y) return SETTINGS_ROWS;
  if (y < ROW_Y0) return -1;
  int row = (y - ROW_Y0) / ROW_H;
  buzz(HAPTIC_CLICK);
  if (rowHasSlider(row)) return row;

  if (row == ROW_VIBRATION) {
    cfg.vibration = !cfg.vibration;
    applyHaptics();
    drawSettingsRow(row);
  } else if (row == ROW_ROTATE) {
    cfg.flipped = !cfg.flipped;
    tft.setRotation(cfg.flipped ? 2 : 0);
    drawSettings();                 // rotation changes everything
    if (cfg.raiseToWake) setupRaiseToWake();  // the wrist gesture follows the rotation
  } else if (row == ROW_RAISE) {
    cfg.raiseToWake = !cfg.raiseToWake;
    if (cfg.raiseToWake) setupRaiseToWake();
    else suspendAccel();
    drawSettingsRow(row);
  }
  saveSettings();
  return -1;
}

// ---------------------------------------------------------------------
// Edit screen: title, big value, slider, Done button.
// ---------------------------------------------------------------------

#define SLIDER_X0         24
#define SLIDER_X1         (SCREEN_W - 24)
#define SLIDER_Y          150
#define SLIDER_TOUCH_Y0   110      // finger anywhere in this band moves the slider
#define SLIDER_TOUCH_Y1   188
#define DONE_Y            (SCREEN_H - 44)

// Slider position 0..1000 for the current value of editRow.
int sliderPos() {
  switch (editRow) {
    case ROW_BRIGHTNESS: return (cfg.brightness - BRIGHTNESS_MIN_PCT) * 1000 / (100 - BRIGHTNESS_MIN_PCT);
    case ROW_COLOR:      return ACCENT_COUNT > 1 ? cfg.accent * 1000 / (ACCENT_COUNT - 1) : 0;
    case ROW_SLEEP:      return SLEEP_OPTIONS > 1 ? cfg.sleepAfter * 1000 / (SLEEP_OPTIONS - 1) : 0;
  }
  return 0;
}

void drawEditValue() {
  tft.fillRect(0, 70, SCREEN_W, SLIDER_TOUCH_Y1 - 70, TFT_BLACK);

  char buf[16];
  settingValue(editRow, buf, sizeof(buf));
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(buf, SCREEN_W / 2, 100, 4);

  int knobX = SLIDER_X0 + (SLIDER_X1 - SLIDER_X0) * sliderPos() / 1000;
  tft.fillRoundRect(SLIDER_X0, SLIDER_Y - 3, SLIDER_X1 - SLIDER_X0, 6, 3, TFT_DARKGREY);
  tft.fillRoundRect(SLIDER_X0, SLIDER_Y - 3, knobX - SLIDER_X0, 6, 3, accent());
  tft.fillCircle(knobX, SLIDER_Y, 11, accent());
}

void drawEdit() {
  clearBody();
  drawStatusBar();
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString(SETTING_LABELS[editRow], SCREEN_W / 2, 50, 2);
  drawEditValue();
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("slide to change", SCREEN_W / 2, SLIDER_Y + 26, 1);

  tft.drawRoundRect(8, DONE_Y, SCREEN_W - 16, 40, 8, accent());
  tft.setTextColor(accent(), TFT_BLACK);
  tft.drawString("Done", SCREEN_W / 2, DONE_Y + 20, 4);
}

// Finger at x on the slider band: set the value, redraw if it changed.
void editSlide(int16_t x) {
  int pos = constrain((x - SLIDER_X0) * 1000 / (SLIDER_X1 - SLIDER_X0), 0, 1000);
  bool changed = false;
  switch (editRow) {
    case ROW_BRIGHTNESS: {
      uint8_t pct = BRIGHTNESS_MIN_PCT + pos * (100 - BRIGHTNESS_MIN_PCT) / 1000;
      if (pct != cfg.brightness) {
        // a tick every 10% so the steps can be felt
        if (pct / 10 != cfg.brightness / 10) buzz(HAPTIC_TICK);
        cfg.brightness = pct;
        applyBrightness();
        changed = true;
      }
      break;
    }
    case ROW_COLOR: {
      uint8_t idx = min<int>(pos * ACCENT_COUNT / 1001, ACCENT_COUNT - 1);
      if (idx != cfg.accent) {
        cfg.accent = idx;
        buzz(HAPTIC_TICK);
        drawEdit();                 // the whole screen is drawn in the accent color
        return;
      }
      break;
    }
    case ROW_SLEEP: {
      uint8_t idx = min<int>(pos * SLEEP_OPTIONS / 1001, SLEEP_OPTIONS - 1);
      if (idx != cfg.sleepAfter) {
        cfg.sleepAfter = idx;
        buzz(HAPTIC_TICK);
        changed = true;
      }
      break;
    }
  }
  if (changed) drawEditValue();
}

void enterScreen(Screen s) {
  screen = s;
  screenSince = millis();
  if (backlightDimmed) applyBrightness();
  switch (s) {
    case SCREEN_CLOCK:         drawClock(); break;
    case SCREEN_MENU:          drawMenu(); break;
    case SCREEN_SETTINGS:      drawSettings(); break;
    case SCREEN_EDIT:          drawEdit(); break;
    case SCREEN_RECORDING:     showStatus("Recording...", "press button to send"); break;
    case SCREEN_WAIT_PHONE:    drawWait(sendAttempts > 0 ? "Resending..." : "Waiting for phone"); break;
    case SCREEN_WAIT_RESPONSE: drawWait("Sent"); break;
    case SCREEN_RESPONSE:      showServerResponse(lastResponse); break;
    case SCREEN_BG_SYNC:       break;  // screen stays off
  }
}

// Redraws the current screen after the phone link or the time changed.
void refreshScreen() {
  switch (screen) {
    case SCREEN_MENU:          drawMenu(); break;
    case SCREEN_CLOCK:         drawClock(); break;
    case SCREEN_WAIT_PHONE:
    case SCREEN_WAIT_RESPONSE: enterScreenKeepTimer(); break;
    case SCREEN_BG_SYNC:       break;
    default:                   drawStatusBar(); break;
  }
}

// Stops the response timer. Returns how long the command took, or 0 if
// none was on its way (e.g. a late answer to an abandoned command).
uint32_t finishCommand() {
  if (!commandPending) return 0;
  commandPending = false;
  const uint32_t ms = millis() - commandStartedAt;
  Serial.printf("response after %lu ms\n", (unsigned long)ms);
  return ms;
}

// An error the watch itself detected, shown and remembered like one from
// the server.
void showLocalError(const char *what) {
  snprintf(lastResponse, sizeof(lastResponse), "ERROR: %s", what);
  lastResponseMs = finishCommand();
  Serial.printf("error: %s\n", what);
  buzz(HAPTIC_ALERT);
  enterScreen(SCREEN_RESPONSE);
}

// ---------------------------------------------------------------------
// BLE server callbacks (run on the NimBLE host task: set flags only)
// ---------------------------------------------------------------------

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
    phoneConnected = true;
    linkChanged = true;
    // Audio throughput: by default the link runs 1M PHY with 27-byte LL
    // packets, ~20 KB/s - slower than the 32 KB/s the mic produces.
    // 251-byte packets and 2M PHY are asked for here; the phone may refuse
    // either, in which case the link just stays as it was.
    server->setDataLen(desc->conn_handle, 251);
    ble_gap_set_prefered_le_phy(desc->conn_handle, BLE_GAP_LE_PHY_2M_MASK,
                                BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_CODED_ANY);
  }
  void onDisconnect(NimBLEServer *server) override {
    phoneConnected = false;
    linkChanged = true;
    NimBLEDevice::startAdvertising();
  }
};

// Fires when the phone writes the server's response back to the watch.
class ResponseCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic) override {
    std::string value = characteristic->getValue();
    portENTER_CRITICAL(&bleMux);
    watch::copyUtf8(pendingResponse, sizeof(pendingResponse), value.data(), value.size());
    responseArrived = true;
    portEXIT_CRITICAL(&bleMux);
  }
};

class TimeCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic) override {
    std::string value = characteristic->getValue();
    watch::TimeMessage msg;
    if (!watch::parseTimeMessage(value.data(), value.size(), msg)) return;
    portENTER_CRITICAL(&bleMux);
    pendingTime = msg;
    timeArrived = true;
    portEXIT_CRITICAL(&bleMux);
  }
  // Lets the phone see how far off the watch is before it sets the time.
  void onRead(NimBLECharacteristic *characteristic) override {
    char buf[40];
    const int64_t us = nowUs();
    snprintf(buf, sizeof(buf), "%lld.%03d,%d", (long long)(us / 1000000),
             (int)(us % 1000000 / 1000), tzOffsetMin);
    characteristic->setValue(buf);
  }
};

// Collects what the BLE callbacks left; false if nothing arrived.
bool takeResponse(char *out, size_t size) {
  if (!responseArrived) return false;
  portENTER_CRITICAL(&bleMux);
  strlcpy(out, pendingResponse, size);
  responseArrived = false;
  portEXIT_CRITICAL(&bleMux);
  return true;
}

bool takeTime(watch::TimeMessage &out) {
  if (!timeArrived) return false;
  portENTER_CRITICAL(&bleMux);
  out = pendingTime;
  timeArrived = false;
  portEXIT_CRITICAL(&bleMux);
  return true;
}

// The phone has subscribed to both notify characteristics.
bool phoneReady() {
  return phoneConnected && commandChar->getSubscribedCount() > 0 &&
         statusChar->getSubscribedCount() > 0;
}

// ---------------------------------------------------------------------
// Audio recording (PDM microphone, config from LilyGoLib::initMicrophone)
// ---------------------------------------------------------------------

void setupI2SMic() {
  i2s_config_t config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 6,
      .dma_buf_len = 512,
      .use_apll = true,
  };
  i2s_pin_config_t pins = {
      .mck_io_num = I2S_PIN_NO_CHANGE,
      .bck_io_num = I2S_PIN_NO_CHANGE,
      .ws_io_num = MIC_CLOCK_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = MIC_DATA_PIN,
  };
  i2s_driver_install(I2S_NUM_0, &config, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &pins);
  // Keep the mic clock off until a recording starts.
  i2s_stop(I2S_NUM_0);
}

void startRecording() {
  if (isRecording) return;
  startBle();                       // the phone reconnects while we record
  isRecording = true;
  recordedBytes = 0;
  sendAttempts = 0;
  waitDetail[0] = '\0';
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_start(I2S_NUM_0);
  buzz(HAPTIC_CLICK);
  enterScreen(SCREEN_RECORDING);
}

void stopRecording() {
  if (!isRecording) return;
  isRecording = false;
  i2s_stop(I2S_NUM_0);
  buzz(HAPTIC_CLICK);

  if (recordedBytes == 0) {
    showStatus("Nothing recorded");
    delay(1500);
    goToSleep();
  }
  commandStartedAt = millis();      // the response timer starts here
  commandPending = true;
  enterScreen(SCREEN_WAIT_PHONE);   // loop() sends as soon as the phone is ready
}

bool sendBufferOverBle(const uint8_t *data, size_t length);

void sendRecording() {
  showStatus("Sending...", "");
  if (sendBufferOverBle(recordBuffer, recordedBytes)) {
    recordedBytes = 0;
    waitDetail[0] = '\0';
    enterScreen(SCREEN_WAIT_RESPONSE);
    return;
  }
  // The link dropped mid-transfer. The recording is still in the buffer:
  // send it again once the phone is back.
  if (++sendAttempts < SEND_ATTEMPTS) {
    snprintf(waitDetail, sizeof(waitDetail), "Link lost, attempt %u of %u", sendAttempts + 1,
             SEND_ATTEMPTS);
    enterScreen(SCREEN_WAIT_PHONE);
    return;
  }
  recordedBytes = 0;
  showLocalError("Could not send the command: Bluetooth link lost");
}

// Called continuously from loop() while a recording is running.
void recordChunk() {
  if (!isRecording) return;
  if (recordedBytes >= RECORD_BUFFER_BYTES) {
    stopRecording();                // hit the cap, send what we have
    return;
  }

  // Read in small slices: with a steady mic stream, asking for all of
  // spaceLeft blocks until the buffer is full and the stop press is missed.
  size_t bytesRead = 0;
  size_t spaceLeft = RECORD_BUFFER_BYTES - recordedBytes;
  size_t toRead = min(spaceLeft, (size_t)2048);
  i2s_read(I2S_NUM_0, recordBuffer + recordedBytes, toRead, &bytesRead, 100 / portTICK_PERIOD_MS);
  recordedBytes += bytesRead;
}

// ---------------------------------------------------------------------
// BLE chunked send
// ---------------------------------------------------------------------

// NimBLECharacteristic::notify() ignores the host's return code, so a
// notification that finds the mbuf pool full is silently dropped. This
// sends to every connected peer and waits out ENOMEM instead.
bool notifyReliable(NimBLECharacteristic *chr, const uint8_t *data, size_t length) {
  for (uint16_t conn : bleServer->getPeerDevices()) {
    for (int attempt = 0;; attempt++) {
      os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
      // A NULL mbuf must not reach notify_custom: it would send the
      // characteristic's stored value instead. notify_custom consumes om.
      int rc = om ? ble_gattc_notify_custom(conn, chr->getHandle(), om) : BLE_HS_ENOMEM;
      if (rc == 0) break;
      if (rc != BLE_HS_ENOMEM || attempt >= 2000 || !phoneConnected) {
        Serial.printf("notify failed: rc=%d after %d attempts\n", rc, attempt);
        return false;
      }
      delay(1);
    }
  }
  return true;
}

// True once every chunk and the DONE marker are queued for the phone.
bool sendBufferOverBle(const uint8_t *data, size_t length) {
  uint16_t totalChunks = (length + CHUNK_PAYLOAD_SIZE - 1) / CHUNK_PAYLOAD_SIZE;
  uint32_t startedAt = millis();
  std::vector<uint16_t> peers = bleServer->getPeerDevices();
  Serial.printf("send: %u bytes, %u chunks, peers %u, mtu %u, subscribed %u/%u\n",
                (unsigned)length, totalChunks, (unsigned)peers.size(),
                peers.empty() ? 0 : bleServer->getPeerMTU(peers[0]),
                (unsigned)commandChar->getSubscribedCount(),
                (unsigned)statusChar->getSubscribedCount());
  if (peers.empty()) return false;

  for (uint16_t i = 0; i < totalChunks; i++) {
    size_t offset = (size_t)i * CHUNK_PAYLOAD_SIZE;
    size_t thisChunkSize = min((size_t)CHUNK_PAYLOAD_SIZE, length - offset);

    uint8_t packet[4 + CHUNK_PAYLOAD_SIZE];
    packet[0] = i & 0xFF;
    packet[1] = (i >> 8) & 0xFF;
    packet[2] = totalChunks & 0xFF;
    packet[3] = (totalChunks >> 8) & 0xFF;
    memcpy(packet + 4, data + offset, thisChunkSize);

    if (!notifyReliable(commandChar, packet, 4 + thisChunkSize)) {
      Serial.printf("send aborted at chunk %u/%u\n", i, totalChunks);
      return false;
    }
  }

  if (!notifyReliable(statusChar, (const uint8_t *)"DONE", 4)) return false;
  Serial.printf("send: done in %lu ms\n", (unsigned long)(millis() - startedAt));
  return true;
}

// ---------------------------------------------------------------------
// Raise to wake: BMA423 wrist tilt, watched by the ULP in deep sleep
// ---------------------------------------------------------------------
// The BMA423's feature engine recognizes the wrist-tilt gesture and raises
// INT1 (GPIO 14). Deep sleep has no pin wake left for it: ext0 is the
// touch line, and ext1 on the S3 only knows "all pins low" or "any pin
// high", so GPIO 14 cannot join the PMU line there. Instead the ULP
// coprocessor (the FSM) polls GPIO 14 every ULP_POLL_MS and wakes the CPU;
// setup() sees ESP_SLEEP_WAKEUP_ULP.
//
// The BMA423 sits on the always-on 3.3 V rail and keeps running through
// deep sleep. After a wake it is only checked; its ~6 KB feature config is
// uploaded again (SensorLib's begin(), ~200 ms) only if the chip lost it.

#define BMA_REG_INT_STAT_0     0x1C   // feature interrupts, then 0x1D; reading clears the latch
#define BMA_REG_ACC_X_LSB      0x12
#define BMA_REG_INTERNAL_STAT  0x2A   // bits 0-4 = 0x01: feature config loaded and running
#define BMA_REG_ACC_RANGE      0x41
#define BMA_REG_INT_LATCH      0x55
#define BMA_REG_INT1_MAP       0x56
#define BMA_REG_PWR_CONF       0x7C   // bit 0: advanced power save
#define BMA_REG_PWR_CTRL       0x7D   // bit 2: accelerometer on
#define BMA_INT_WRIST_TILT     0x08   // INT_STAT_0 bit of the wrist-tilt feature

#define ULP_POLL_MS            100    // INT1 is latched, so a slow poll misses nothing

SensorBMA423 accel;

// How the BMA423 is set up; survives deep sleep so a wake can skip the
// setup. NONE also after power-on: the chip may still run an old setup.
enum AccelConfig : uint8_t { ACCEL_CFG_NONE, ACCEL_CFG_TILT, ACCEL_CFG_TILT_FLIPPED };
RTC_DATA_ATTR uint8_t accelConfig = ACCEL_CFG_NONE;
RTC_DATA_ATTR uint8_t phantomTiltWakes = 0;     // see watch::nextPhantomTiltWakes

// Diagnostics (console "G"), reset on power-on: how tilt wakes ended.
struct TiltStats { uint16_t wakes, shown, armDown, phantom, expired, used; };
RTC_DATA_ATTR TiltStats tiltStats = {};

bool tiltWakeUntouched = false;     // woke by a wrist tilt, nobody touched the watch since

// The ULP program lives in RTC slow memory, where the linker puts every
// RTC_DATA_ATTR variable. The Arduino core reserves no ULP area there
// (CONFIG_ULP_COPROC_RESERVE_MEM = 0), so ulp_process_macros_and_load()
// rejects any program: it is copied in by hand, and its branches are plain
// relative offsets instead of labels.
RTC_DATA_ATTR uint32_t ulpProgram[8];

void bmaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMA423_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t bmaRead(uint8_t reg) { return i2cRead8(Wire, BMA423_ADDR, reg); }

// Reads, and so clears, the latched feature interrupts; INT1 drops.
uint8_t takeAccelIrq() {
  Wire.beginTransmission(BMA423_ADDR);
  Wire.write(BMA_REG_INT_STAT_0);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom((uint8_t)BMA423_ADDR, (uint8_t)2) != 2) return 0;
  const uint8_t features = Wire.read();
  Wire.read();                      // INT_STAT_1: data ready etc., unused
  return features;
}

// Sets the BMA423 up for wrist-tilt detection, or checks that it still is.
// INT1 is push-pull, active high and latched (high until INT_STAT is read):
// what the ULP program polls for, and a short gesture pulse cannot slip
// between two polls. The wrist gesture depends on how the watch is worn,
// so the axis remap follows the screen rotation: rotation 2 is LilyGO's
// default orientation, with the chip on the bottom layer, bottom-left
// corner (LilyGoLib); rotation 0 turns it by 180 degrees.
bool setupRaiseToWake() {
  const uint8_t want = cfg.flipped ? ACCEL_CFG_TILT_FLIPPED : ACCEL_CFG_TILT;
  if (accelConfig == want && (bmaRead(BMA_REG_INTERNAL_STAT) & 0x1F) == 0x01) return true;

  const uint32_t started = millis();
  accelConfig = ACCEL_CFG_NONE;
  if (!accel.begin(Wire, BMA423_ADDR)) {   // uploads the feature config unless loaded
    Serial.println("raise to wake: BMA423 not ready");
    return false;
  }
  // Advanced power save off while configuring: with it on, writes need
  // 450 us gaps, and SensorLib writes back to back.
  bmaWrite(BMA_REG_PWR_CONF, 0x00);
  delay(1);
  // Low-power mode, 50 Hz: the slowest rate the feature engine accepts.
  accel.configAccelerometer(SensorBMA423::RANGE_4G, SensorBMA423::ODR_50HZ,
                            SensorBMA423::BW_NORMAL_AVG4, SensorBMA423::PERF_CIC_AVG_MODE);
  accel.enableAccelerometer();
  accel.enableFeature(SensorBMA423::FEATURE_TILT, true);
  accel.setRemapAxes(cfg.flipped ? SensorBMA423::REMAP_BOTTOM_LAYER_BOTTOM_LEFT_CORNER
                                 : SensorBMA423::REMAP_BOTTOM_LAYER_TOP_RIGHT_CORNER);
  // INT1: output, push-pull, active high (SensorLib's defaults; the
  // constants themselves are protected in SensorBMA423).
  accel.configInterrupt();
  bmaWrite(BMA_REG_INT_LATCH, 0x01);
  delay(1);
  accel.enableTiltIRQ();
  takeAccelIrq();
  delay(1);
  bmaWrite(BMA_REG_PWR_CONF, 0x01);  // advanced power save: sleeps between samples

  const uint8_t status = bmaRead(BMA_REG_INTERNAL_STAT);
  Serial.printf("raise to wake: BMA423 set up in %lu ms, internal status 0x%02X\n",
                (unsigned long)(millis() - started), status);
  if ((status & 0x1F) != 0x01) return false;
  accelConfig = want;
  return true;
}

// Raise to wake off: interrupt unmapped, accelerometer off, chip in
// suspend. The feature config stays loaded for when it is switched on.
void suspendAccel() {
  bmaWrite(BMA_REG_PWR_CONF, 0x00);
  delay(1);
  bmaWrite(BMA_REG_INT1_MAP, 0x00);
  delay(1);
  bmaWrite(BMA_REG_PWR_CTRL, 0x00);
  delay(1);
  takeAccelIrq();
  bmaWrite(BMA_REG_PWR_CONF, 0x03);  // advanced power save + FIFO self wake-up, as after reset
  accelConfig = ACCEL_CFG_NONE;
}

// Acceleration in milli-g, the mean of three samples (50 Hz: 20 ms apart).
// Chip axes: z is normal to the board, so to the screen.
bool readAccelMg(int &x, int &y, int &z) {
  const int lsbPerG = 1024 >> (bmaRead(BMA_REG_ACC_RANGE) & 0x03);  // 12 bit over +-2/4/8/16 g
  long sum[3] = {0, 0, 0};
  for (int n = 0; n < 3; n++) {
    if (n) delay(20);
    Wire.beginTransmission(BMA423_ADDR);
    Wire.write(BMA_REG_ACC_X_LSB);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)BMA423_ADDR, (uint8_t)6) != 6) return false;
    for (int i = 0; i < 3; i++) {
      const uint8_t lo = Wire.read(), hi = Wire.read();
      sum[i] += (int16_t)((hi << 8) | lo) >> 4;  // 12 bit, left-justified
    }
  }
  x = sum[0] * 1000 / (3 * lsbPerG);
  y = sum[1] * 1000 / (3 * lsbPerG);
  z = sum[2] * 1000 / (3 * lsbPerG);
  return true;
}

// GPIO 14 as an RTC input, the way the ULP reads it. The pull-down holds
// the line low should the chip ever release it; INT1 is push-pull, so it
// costs nothing while the line is low.
int accelIntLevel() {
  const gpio_num_t pin = (gpio_num_t)BMA423_INT_PIN;
  rtc_gpio_init(pin);
  rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis(pin);
  rtc_gpio_pulldown_en(pin);
  return rtc_gpio_get_level(pin);
}

// Once per boot, before the backlight comes on. Clears the interrupt that
// woke us and sets the chip up (or checks it, or suspends it). Returns
// false if a tilt wake found the arm hanging down: back to sleep, dark.
bool raiseToWakeAtBoot(esp_sleep_wakeup_cause_t wake) {
  const bool tiltWake = wake == ESP_SLEEP_WAKEUP_ULP;
  const uint8_t irq = takeAccelIrq();
  const watch::WakeSource source = tiltWake ? watch::WakeSource::Tilt
                                   : wake == ESP_SLEEP_WAKEUP_TIMER ? watch::WakeSource::Timer
                                                                    : watch::WakeSource::User;
  phantomTiltWakes = watch::nextPhantomTiltWakes(phantomTiltWakes, source, irq & BMA_INT_WRIST_TILT);

  if (!cfg.raiseToWake) {
    if (accelConfig != ACCEL_CFG_NONE || wake == ESP_SLEEP_WAKEUP_UNDEFINED) suspendAccel();
    return true;
  }
  setupRaiseToWake();
  if (!tiltWake) return true;

  tiltStats.wakes++;
  if (!(irq & BMA_INT_WRIST_TILT)) tiltStats.phantom++;
  int x = 0, y = 0, z = 0;
  const bool facing = !readAccelMg(x, y, z) || watch::tiltWakeFacesUser(x, y, z);
  if (facing) tiltStats.shown++;
  else tiltStats.armDown++;
  Serial.printf("raise to wake: INT 0x%02X, accel %d %d %d mg, %s\n", irq, x, y, z,
                facing ? "clock shown" : "arm down, back to sleep");
  return facing;
}

// Before sleeping: whether to arm the ULP. Releases a latch left by a
// tilt while awake, and refuses a line that stays high (it would wake the
// watch at once, over and over).
bool raiseToWakeReady() {
  if (!cfg.raiseToWake || accelConfig == ACCEL_CFG_NONE) return false;
  if (!watch::raiseToWakeMayArm(phantomTiltWakes)) {
    Serial.printf("sleep: raise to wake paused, %u wakes without a tilt\n", phantomTiltWakes);
    return false;
  }
  takeAccelIrq();
  if (accelIntLevel()) {
    Serial.println("sleep: raise to wake off, INT1 stuck high");
    return false;
  }
  return true;
}

// Loads the ULP program and starts it. Called after prepareSleepClock():
// ulp_set_wakeup_period() converts the period with the current sleep clock.
// The ULP keeps running after it woke the CPU; setup() stops it.
bool startRaiseToWakeUlp() {
  const ulp_insn_t program[] = {
      // R0 = INT1
      I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + BMA423_INT_PIN, RTC_GPIO_IN_NEXT_S + BMA423_INT_PIN),
      I_BL(4, 1),                   // low: nothing happened, to HALT
      // R0 = the RTC is in sleep and can take a wake-up
      I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP_S, RTC_CNTL_RDY_FOR_WAKEUP_S),
      I_BL(2, 1),                   // not yet (still going to sleep): next poll, to HALT
      I_WAKE(),
      I_HALT(),                     // the ULP timer starts the program again
  };
  static_assert(sizeof(program) <= sizeof(ulpProgram), "ULP program does not fit");
  const uintptr_t at = (uintptr_t)ulpProgram;
  if (at < SOC_RTC_DATA_LOW || at + sizeof(program) > SOC_RTC_DATA_HIGH) {
    Serial.println("sleep: raise to wake off, ULP program not in RTC slow memory");
    return false;
  }
  memcpy(ulpProgram, program, sizeof(program));
  if (ulp_set_wakeup_period(0, ULP_POLL_MS * 1000) != ESP_OK || esp_sleep_enable_ulp_wakeup() != ESP_OK ||
      ulp_run((at - SOC_RTC_DATA_LOW) / sizeof(uint32_t)) != ESP_OK) {
    Serial.println("sleep: raise to wake off, ULP did not start");
    return false;
  }
  return true;
}

void printTiltReport() {
  Serial.printf("RAISE setting=%d config=%u internal=0x%02X int1=%d phantom_streak=%u "
                "wakes=%u shown=%u arm_down=%u no_tilt=%u expired=%u used=%u\n",
                cfg.raiseToWake, accelConfig, bmaRead(BMA_REG_INTERNAL_STAT), accelIntLevel(),
                phantomTiltWakes, tiltStats.wakes, tiltStats.shown, tiltStats.armDown,
                tiltStats.phantom, tiltStats.expired, tiltStats.used);
}

// ---------------------------------------------------------------------
// Hardware probe (console "S"): what is fitted and alive on this watch
// ---------------------------------------------------------------------

const char *knownI2cDevice(int bus, uint8_t addr) {
  if (bus == 0) {
    switch (addr) {
      case BMA423_ADDR:  return "BMA423 accelerometer";
      case AXP2101_ADDR: return "AXP2101 PMU";
      case PCF8563_ADDR: return "PCF8563 RTC";
      case DRV2605_ADDR: return "DRV2605 haptic driver";
    }
  } else if (addr == FT6336_ADDR) {
    return "FT6336 touch";
  }
  return "unknown";
}

void scanI2c(TwoWire &bus, int index, const char *pins) {
  Serial.printf("I2C bus %d (%s):\n", index, pins);
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.printf("  0x%02X  %s\n", addr, knownI2cDevice(index, addr));
    }
  }
}

void probeBma423() {
  const uint8_t id = i2cRead8(Wire, BMA423_ADDR, 0x00);
  Serial.printf("BMA423: chip id 0x%02X (%s)\n", id, id == 0x13 ? "ok" : "unexpected");
  if (id != 0x13) return;
  // Accelerometer on, read one sample, back to suspend.
  Wire.beginTransmission(BMA423_ADDR); Wire.write(0x7C); Wire.write(0x00); Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(BMA423_ADDR); Wire.write(0x7D); Wire.write(0x04); Wire.endTransmission();
  delay(50);
  Wire.beginTransmission(BMA423_ADDR);
  Wire.write(0x12);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMA423_ADDR, (uint8_t)6);
  int16_t a[3];
  for (int i = 0; i < 3; i++) {
    uint8_t lo = Wire.read(), hi = Wire.read();
    a[i] = (int16_t)((hi << 8) | lo) >> 4;  // 12 bit, left-justified
  }
  const uint8_t range = i2cRead8(Wire, BMA423_ADDR, 0x41) & 0x03;  // 0:2g 1:4g 2:8g 3:16g
  const float lsbPerG = 2048.0f / (2 << range);
  const int8_t temp = (int8_t)i2cRead8(Wire, BMA423_ADDR, 0x22);
  Serial.printf("  accel x=%.2f y=%.2f z=%.2f g, temperature %d C\n", a[0] / lsbPerG,
                a[1] / lsbPerG, a[2] / lsbPerG, temp + 23);
  // Back to suspend, or to the raise-to-wake setup this probe disturbed.
  accelConfig = ACCEL_CFG_NONE;
  if (cfg.raiseToWake) {
    setupRaiseToWake();
  } else {
    Wire.beginTransmission(BMA423_ADDR); Wire.write(0x7D); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(BMA423_ADDR); Wire.write(0x7C); Wire.write(0x03); Wire.endTransmission();
  }
}

void probePmu() {
  const uint8_t id = pmu.getChipID();
  Serial.printf("AXP2101: chip id 0x%02X (%s)\n", id, id == XPOWERS_AXP2101_CHIP_ID ? "ok" : "unexpected");
  pmu.enableVbusVoltageMeasure();
  pmu.enableSystemVoltageMeasure();
  pmu.enableTemperatureMeasure();
  delay(20);
  Serial.printf("  battery %u mV, %d %%, %s; VBUS %u mV; system %u mV; die %.1f C\n",
                pmu.getBattVoltage(), pmu.getBatteryPercent(),
                pmu.isCharging() ? "charging" : (pmu.isVbusIn() ? "on USB" : "discharging"),
                pmu.getVbusVoltage(), pmu.getSystemVoltage(), pmu.getTemperature());
}

void probeRtc() {
  const uint8_t sec = i2cRead8(Wire, PCF8563_ADDR, 0x02);
  time_t utc = 0;
  const bool ok = rtcRead(utc);
  Serial.printf("PCF8563: %s, VL bit %d, time %lld; alarm/timer INT on GPIO %d\n",
                ok ? "running" : "time invalid", sec >> 7, (long long)utc, RTC_INT_PIN);
}

void probeHaptics() {
  const bool wasOn = hapticReady;
  if (!wasOn) {
    pmu.setBLDO2Voltage(3300);
    pmu.enableBLDO2();
    delay(5);
  }
  const uint8_t id = drvRead(0x00) >> 5;
  const char *name = id == 3 ? "DRV2605" : id == 7 ? "DRV2605L" : id == 4 ? "DRV2604" :
                     id == 6 ? "DRV2604L" : "unexpected";
  Serial.printf("DRV2605: device id %u (%s)\n", id, name);
  if (!wasOn) pmu.disableBLDO2();
}

void probeTouch() {
  Serial.printf("FT6336: vendor 0x%02X, chip 0x%02X, firmware 0x%02X\n",
                i2cRead8(Wire1, FT6336_ADDR, 0xA8), i2cRead8(Wire1, FT6336_ADDR, 0xA3),
                i2cRead8(Wire1, FT6336_ADDR, 0xA6));
}

// Bit-banged SPI: a few bytes, without claiming a SPI host the display uses.
uint8_t radioTransfer(uint8_t out) {
  uint8_t in = 0;
  for (int bit = 7; bit >= 0; bit--) {
    digitalWrite(RADIO_MOSI_PIN, (out >> bit) & 1);
    delayMicroseconds(2);
    digitalWrite(RADIO_SCK_PIN, HIGH);
    in = (in << 1) | digitalRead(RADIO_MISO_PIN);
    delayMicroseconds(2);
    digitalWrite(RADIO_SCK_PIN, LOW);
  }
  return in;
}

void probeRadio() {
  pmu.setALDO4Voltage(3300);
  pmu.enableALDO4();
  pinMode(RADIO_CS_PIN, OUTPUT);
  digitalWrite(RADIO_CS_PIN, HIGH);
  pinMode(RADIO_SCK_PIN, OUTPUT);
  digitalWrite(RADIO_SCK_PIN, LOW);
  pinMode(RADIO_MOSI_PIN, OUTPUT);
  pinMode(RADIO_MISO_PIN, INPUT);
  pinMode(RADIO_BUSY_PIN, INPUT);
  pinMode(RADIO_RST_PIN, OUTPUT);
  delay(10);
  digitalWrite(RADIO_RST_PIN, LOW);
  delay(2);
  digitalWrite(RADIO_RST_PIN, HIGH);
  const uint32_t start = millis();
  while (digitalRead(RADIO_BUSY_PIN) && millis() - start < 100) delay(1);

  // ReadRegister(0x0740): the LoRa sync word, 0x1424 after reset.
  digitalWrite(RADIO_CS_PIN, LOW);
  radioTransfer(0x1D);
  radioTransfer(0x07);
  radioTransfer(0x40);
  radioTransfer(0x00);
  const uint8_t hi = radioTransfer(0x00), lo = radioTransfer(0x00);
  digitalWrite(RADIO_CS_PIN, HIGH);
  Serial.printf("SX1262: sync word 0x%02X%02X (%s)\n", hi, lo,
                hi == 0x14 && lo == 0x24 ? "ok" : "no answer");

  for (int pin : {RADIO_CS_PIN, RADIO_SCK_PIN, RADIO_MOSI_PIN, RADIO_RST_PIN}) pinMode(pin, INPUT);
  pmu.disableALDO4();
}

void probeGps() {
  pmu.setDC3Voltage(3300);
  pmu.enableDC3();
  bool found = false;
  for (uint32_t baud : {38400UL, 9600UL}) {
    Serial1.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    const uint32_t start = millis();
    char prev = 0;
    while (!found && millis() - start < 1500) {
      while (Serial1.available()) {
        char c = Serial1.read();
        if (prev == '$' && c == 'G') found = true;
        prev = c;
      }
      delay(5);
    }
    Serial1.end();
    if (found) {
      Serial.printf("GPS: NMEA at %lu baud (T-Watch S3 Plus)\n", (unsigned long)baud);
      break;
    }
  }
  if (!found) Serial.println("GPS: no NMEA data (not fitted - Plus model only)");
  pmu.disableDC3();
}

void probeMic() {
  static int16_t samples[1600];     // 100 ms
  size_t got = 0;
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_start(I2S_NUM_0);
  i2s_read(I2S_NUM_0, samples, sizeof(samples), &got, 200 / portTICK_PERIOD_MS);  // settle
  i2s_read(I2S_NUM_0, samples, sizeof(samples), &got, 200 / portTICK_PERIOD_MS);
  i2s_stop(I2S_NUM_0);
  const size_t n = got / 2;
  double sum = 0, sumSq = 0;
  int peak = 0;
  for (size_t i = 0; i < n; i++) sum += samples[i];
  const double mean = n ? sum / n : 0;
  for (size_t i = 0; i < n; i++) {
    const double v = samples[i] - mean;
    sumSq += v * v;
    peak = max(peak, abs((int)v));
  }
  Serial.printf("PDM microphone: %u samples, rms %.0f, peak %d (%s)\n", (unsigned)n,
                n ? sqrt(sumSq / n) : 0.0, peak, n && peak > 0 ? "signal" : "no signal");
}

void probeHardware() {
  Serial.println("--- hardware probe ---");
  scanI2c(Wire, 0, "SDA 10 / SCL 11");
  scanI2c(Wire1, 1, "SDA 39 / SCL 40");
  probePmu();
  probeRtc();
  probeBma423();
  probeHaptics();
  probeTouch();
  probeMic();
  probeRadio();
  probeGps();
  Serial.printf("ESP32-S3 internal temperature: %.1f C\n", temperatureRead());
  Serial.println("--- end of probe ---");
}

// ---------------------------------------------------------------------
// USB serial console
// ---------------------------------------------------------------------

// Raw FT6336 registers for <seconds>, 10 per second, for diagnosing
// unexpected touch wakes.
void dumpTouch(uint32_t seconds) {
  Serial.printf("touch dump: %lu s, INT pin %d\n", (unsigned long)seconds, digitalRead(TOUCH_INT_PIN));
  const uint32_t start = millis();
  while (millis() - start < seconds * 1000) {
    Wire1.beginTransmission(FT6336_ADDR);
    Wire1.write(0x00);              // DEV_MODE, GEST_ID, TD_STATUS, P1 XH XL YH YL WEIGHT MISC
    uint8_t r[9] = {};
    if (Wire1.endTransmission(false) == 0 && Wire1.requestFrom(FT6336_ADDR, 9) == 9) {
      for (int i = 0; i < 9; i++) r[i] = Wire1.read();
    }
    Serial.printf("TOUCH t=%lu int=%d n=%u ev=%u x=%u y=%u weight=%u area=%u\n",
                  (unsigned long)(millis() - start), digitalRead(TOUCH_INT_PIN), r[2] & 0x0F,
                  r[3] >> 6, ((r[3] & 0x0F) << 8) | r[4], ((r[5] & 0x0F) << 8) | r[6], r[7],
                  r[8] >> 4);
    delay(100);
  }
  Serial.println("touch dump done");
}

void printHelp() {
  Serial.println(
      "Commands:\n"
      "  T<unix>[.<frac>] [<utc offset min>]   set the clock, e.g. T1760000000.25 120\n"
      "  Q        clock report (system clock, RTC, sleep clock, last correction)\n"
      "  R        wait for the RTC's next second and report it (sub-second phase)\n"
      "  Z<sec>   deep sleep now, wake by timer after <sec> (drift tests)\n"
      "  C<0|1>   sleep clock: 0 = RC 136 kHz, 1 = RC_FAST/256 (saved)\n"
      "  N<0|1>   RTC correction on wake: 0 = off (tests, until reset), 1 = on\n"
      "  S        hardware probe: I2C devices, sensors, radio, microphone\n"
      "  P<sec>   raw touch controller registers for <sec>\n"
      "  G        raise to wake: setting, BMA423 state, INT1 level, tilt wake counters\n"
      "  W        UI test: show the waiting screen with the response timer\n"
      "  A<text>  UI test: an answer from the server, e.g. AOK: lights on\n"
      "  H        this help");
}

void handleCommand(const String &line) {
  if (line.isEmpty()) return;
  screenSince = millis();           // a console session keeps the watch awake
  const char cmd = toupper(line[0]);
  const char *arg = line.c_str() + 1;
  switch (cmd) {
    case 'T': {
      watch::TimeMessage m;
      if (watch::parseTimeMessage(arg, strlen(arg), m)) {
        applyTime(m, "console");
        linkChanged = true;         // redraw
      } else {
        Serial.println("bad time, expected T<unix>[.<frac>] [<offset min>]");
      }
      break;
    }
    case 'Q': printClockReport(); break;
    case 'R':
      serviceRtcWrite(true);        // measure the RTC as set, not as it was
      printRtcTick();
      break;
    case 'Z': {
      const long seconds = atol(arg);
      if (seconds < 1 || seconds > 86400) {
        Serial.println("Z<seconds>, 1..86400");
        break;
      }
      testSleepSeconds = seconds;
      Serial.printf("sleeping %ld s\n", seconds);
      goToSleep();
      break;
    }
    case 'C':
      cfg.sleepClock = atoi(arg) == 0 ? SLEEP_CLOCK_RC136K : SLEEP_CLOCK_RCFAST_D256;
      prefs.putUChar("slowclk", cfg.sleepClock);
      Serial.printf("sleep clock: %s\n", sleepClockName(cfg.sleepClock));
      break;
    case 'N':
      rtcResyncDisabled = atoi(arg) == 0;
      Serial.printf("RTC correction on wake: %s\n", rtcResyncDisabled ? "off" : "on");
      break;
    case 'S': probeHardware(); break;
    case 'W':                       // UI test: a command on its way
      waitDetail[0] = '\0';
      commandStartedAt = millis();
      commandPending = true;
      enterScreen(SCREEN_WAIT_RESPONSE);
      Serial.println("waiting screen (send A<text> to answer)");
      break;
    case 'A':                       // UI test: an answer from the server
      handleReply(arg);
      break;
    case 'P': dumpTouch(constrain(atol(arg), 1, 60)); break;
    case 'G': printTiltReport(); break;
    case 'H': printHelp(); break;
    default: Serial.println("unknown command, H for help"); break;
  }
}

void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (line.length() < 60) line += c;
      continue;
    }
    handleCommand(line);
    line = "";
  }
}

// ---------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------

// Started only when needed (menu, recording, time sync): BLE is the biggest
// power draw while awake, and a sleeping watch has no use for it.
void startBle() {
  if (bleStarted) return;
  bleStarted = true;
  linkChanged = true;               // status bar shows the link dot

  NimBLEDevice::init("T-Watch-Command");
  // v3: allow a 517-byte ATT MTU. CHUNK_PAYLOAD_SIZE (508) + 4 header bytes
  // only fit in one notification if both sides agree on MTU >= 515; the
  // phone requests 517. NimBLE's default (255) silently truncated packets.
  NimBLEDevice::setMTU(517);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  NimBLEService *service = bleServer->createService(SERVICE_UUID);

  commandChar = service->createCharacteristic(
      COMMAND_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  statusChar = service->createCharacteristic(
      STATUS_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);

  responseChar = service->createCharacteristic(
      RESPONSE_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  responseChar->setCallbacks(new ResponseCallback());

  NimBLECharacteristic *timeChar = service->createCharacteristic(
      TIME_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  timeChar->setCallbacks(new TimeCallback());

  service->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();
}

// Power rails per LilyGoLib::initPMU for T-Watch S3. Display and backlight
// stay dark until ALDO2/ALDO3 are on; the radio and GPS rails are not used
// here and are switched off. BLDO2 (vibration) follows the setting.
void setupPmu() {
  if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, PMU_I2C_SDA, PMU_I2C_SCL)) {
    Serial.println("AXP2101 not found");
    return;
  }
  pmu.setALDO2Voltage(3300);   // backlight VDD
  pmu.enableALDO2();
  pmu.setALDO3Voltage(3300);   // display + touch VDD, stays on in sleep for touch wake
  pmu.enableALDO3();
  pmu.disableALDO4();          // SX1262
  pmu.disableDC3();            // GPS (Plus only)

  pmu.disableTSPinMeasure();
  pmu.enableBattDetection();
  pmu.enableBattVoltageMeasure();
  pmu.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);

  pmu.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
  pmu.clearIrqStatus();
  pmu.enableIRQ(XPOWERS_AXP2101_PKEY_SHORT_IRQ);

  pinMode(PMU_IRQ_PIN, INPUT_PULLUP);
  attachInterrupt(PMU_IRQ_PIN, [] { pmuIrq = true; }, FALLING);
}

// True once per short press of the side button.
bool buttonPressed() {
  if (!pmuIrq) return false;
  pmuIrq = false;
  pmu.getIrqStatus();
  bool pressed = pmu.isPekeyShortPressIrq();
  pmu.clearIrqStatus();
  return pressed;
}

#define DOUBLE_PRESS_MS   600    // second press within this window = double press

bool pressPending = false;          // a first press is waiting for a possible second
uint32_t pressPendingAt = 0;
bool pressIsWake = false;           // the pending press is the one that woke the watch

// Single presses are reported DOUBLE_PRESS_MS late, once no second press
// followed. While recording there is nothing to disambiguate, so a press
// stops the recording at once.
Press readButton() {
  if (buttonPressed()) {
    if (isRecording) return PRESS_SINGLE;
    if (pressPending && millis() - pressPendingAt <= DOUBLE_PRESS_MS) {
      pressPending = false;
      pressIsWake = false;
      return PRESS_DOUBLE;
    }
    pressPending = true;
    pressPendingAt = millis();
    return PRESS_NONE;
  }
  if (pressPending && millis() - pressPendingAt > DOUBLE_PRESS_MS) {
    pressPending = false;
    // A lone wake press has already done its job: the watch is awake.
    if (pressIsWake) {
      pressIsWake = false;
      return PRESS_NONE;
    }
    return PRESS_SINGLE;
  }
  return PRESS_NONE;
}

// Arms the timer for the next wake without the user: a drift test asked
// for by the console, or the next background time sync.
void armTimerWake() {
  uint32_t seconds = 0;
  if (testSleepSeconds) {
    wakePlan = WAKE_PLAN_TEST;
    seconds = testSleepSeconds;
  } else if (watch::backgroundSyncAllowed(readBattery())) {
    wakePlan = WAKE_PLAN_SYNC;
    seconds = watch::secondsUntilBackgroundSync(time(nullptr), lastSyncEpoch, bgSyncFailures,
                                                kSyncSchedule);
  } else {
    wakePlan = WAKE_PLAN_NONE;
  }
  if (seconds) esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  Serial.printf("sleep: timer %lu s (%s), sleep clock %s\n", (unsigned long)seconds,
                wakePlan == WAKE_PLAN_TEST ? "test" : wakePlan == WAKE_PLAN_SYNC ? "time sync" : "off",
                sleepClockName(cfg.sleepClock));
}

void goToSleep() {
  if (isRecording) i2s_stop(I2S_NUM_0);
  if (screen == SCREEN_EDIT) saveSettings();   // slider values apply live, unsaved
  serviceRtcWrite(true);

  // In polling mode the touch INT is held low while a finger is down;
  // sleeping now would wake immediately.
  uint32_t start = millis();
  while (touchDown() && millis() - start < 3000) delay(20);

  displayOff();
  pmu.disableALDO2();               // backlight VDD
  pmu.disableBLDO2();               // vibration motor driver
  pmu.clearIrqStatus();             // release the IRQ line before arming it

  armTimerWake();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_INT_PIN, 0);
  esp_sleep_enable_ext1_wakeup(1ULL << PMU_IRQ_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
  rtc_gpio_pullup_en((gpio_num_t)TOUCH_INT_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)TOUCH_INT_PIN);
  rtc_gpio_pullup_en((gpio_num_t)PMU_IRQ_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)PMU_IRQ_PIN);
  const bool raiseToWake = raiseToWakeReady();
  prepareSleepClock();              // the calibration should be fresh
  // Third wake source: the ULP polling the accelerometer (needs the
  // calibration above).
  if (raiseToWake && startRaiseToWakeUlp()) {
    Serial.printf("sleep: raise to wake armed, INT1 polled every %d ms\n", ULP_POLL_MS);
  }

  // No Serial.flush() here: with USB plugged in but no terminal reading,
  // it never returns and the watch hangs with the screen already off.
  esp_deep_sleep_start();
}

// A background sync wake turns into a normal one when the user touches
// the watch.
void wakeFromBackground() {
  displayOn();
  enterScreen(SCREEN_CLOCK);
}

void setup() {
  esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
  const bool backgroundSync = wake == ESP_SLEEP_WAKEUP_TIMER && wakePlan == WAKE_PLAN_SYNC;
  ulp_timer_stop();                 // raise to wake: the ULP keeps polling after it woke us

  Serial.begin(115200);
  // USB CDC log is best-effort: drop output instead of blocking when no
  // terminal on the host is reading it.
  Serial.setTxTimeoutMs(0);
  prefs.begin("watch", false);
  loadSettings();
  setupPmu();
  applyHaptics();
  loadClock();
  setupTouch();

  ledcSetup(BL_LEDC_CHANNEL, 1000, 8);
  ledcAttachPin(TFT_BL_PIN, BL_LEDC_CHANNEL);
  ledcWrite(BL_LEDC_CHANNEL, 0);    // no flash of the old frame buffer
  tft.init();
  tft.setRotation(cfg.flipped ? 2 : 0);
  tft.fillScreen(TFT_BLACK);
  // Before the backlight comes on: a tilt wake with the arm hanging down
  // goes back to sleep without lighting the screen.
  if (!raiseToWakeAtBoot(wake)) goToSleep();
  if (backgroundSync) displayOff();
  else applyBrightness();

  // PSRAM allocation - RECORD_BUFFER_BYTES is ~256KB at 16kHz/16-bit/8s,
  // won't fit reliably in internal RAM alongside BLE + display buffers.
  recordBuffer = (uint8_t *)ps_malloc(RECORD_BUFFER_BYTES);
  if (!recordBuffer) {
    if (backgroundSync) displayOn();
    showStatus("PSRAM alloc failed!");
    while (true) delay(1000);
  }

  setupI2SMic();

  if (backgroundSync) {
    // Woken by the timer to fetch the time: screen off, advertise for
    // BG_SYNC_WINDOW_MS, sleep as soon as the phone has written TIME.
    startBle();
    enterScreen(SCREEN_BG_SYNC);
  } else if (wake == ESP_SLEEP_WAKEUP_EXT1) {
    // Woken by the side button. That press was cleared in setupPmu(); if a
    // second one follows, it is a double press and starts a recording.
    pressPending = true;
    pressPendingAt = 0;             // the wake press happened at boot
    pressIsWake = true;
    enterScreen(SCREEN_CLOCK);
  } else if (wake == ESP_SLEEP_WAKEUP_ULP) {
    // Raised wrist: the clock, briefly, unless the watch gets touched.
    tiltWakeUntouched = true;
    enterScreen(SCREEN_CLOCK);
  } else if (wake == ESP_SLEEP_WAKEUP_EXT0 || wake == ESP_SLEEP_WAKEUP_TIMER) {
    enterScreen(SCREEN_CLOCK);      // woken by a tap, or a console drift test
  } else {
    // Power-on, reset or a fresh flash: open the menu so the phone can
    // reconnect and sync the time.
    startBle();
    enterScreen(SCREEN_MENU);
  }
  wakePlan = WAKE_PLAN_NONE;
  Serial.printf("boot: wake cause %d, clock corrected by %lld us\n", (int)wake,
                (long long)lastWakeCorrectionUs);
}

// The battery is read over I2C; while waiting on a command, every few
// seconds is plenty.
bool sendTimeoutApplies() {
  static uint32_t checkedAt = 0;
  static bool applies = false;
  static bool checked = false;
  if (!checked || millis() - checkedAt > 5000) {
    applies = watch::sendMayTimeOut(readBattery());
    checkedAt = millis();
    checked = true;
  }
  return applies;
}

// Redraws a waiting screen without restarting its timeout.
void enterScreenKeepTimer() {
  const uint32_t since = screenSince;
  enterScreen(screen);
  screenSince = since;
}

void handleReply(const char *text) {
  const watch::Reply reply = watch::parseReply(text);
  Serial.printf("reply (%d): %s\n", (int)reply.kind, text);
  if (!watch::isFinalReply(reply.kind)) {
    // Progress from the phone: the command arrived and is being worked on.
    if (screen == SCREEN_WAIT_RESPONSE) {
      watch::copyUtf8(waitDetail, sizeof(waitDetail), reply.body, strlen(reply.body));
      enterScreen(SCREEN_WAIT_RESPONSE);  // restarts the low-battery timeout too
    }
    return;
  }
  strlcpy(lastResponse, text, sizeof(lastResponse));
  lastResponseMs = finishCommand();
  // Shown wherever the watch is, except mid-recording and with the screen off.
  if (screen == SCREEN_RECORDING || screen == SCREEN_BG_SYNC) return;
  buzz(replyStyle(reply.kind).haptic);
  enterScreen(SCREEN_RESPONSE);
}

void loop() {
  handleSerial();
  serviceRtcWrite(false);
  serviceRtcTickLock();

  watch::TimeMessage newTime;
  bool timeSynced = false;
  if (takeTime(newTime)) {
    applyTime(newTime, "phone");
    timeSynced = true;
    linkChanged = true;             // redraw with the new time
  }

  char reply[RESPONSE_MAX + 1];
  if (takeResponse(reply, sizeof(reply))) handleReply(reply);

  // Side button: single press sleeps (the wake is handled by deep sleep),
  // double press records, a press while recording sends.
  Press press = readButton();
  if (screen == SCREEN_BG_SYNC && (press != PRESS_NONE || pressPending)) {
    pressIsWake = true;             // this press only wakes the screen
    wakeFromBackground();
    press = PRESS_NONE;
  }
  switch (press) {
    case PRESS_SINGLE:
      if (isRecording) stopRecording();
      else goToSleep();
      break;
    case PRESS_DOUBLE:
      if (screen != SCREEN_WAIT_PHONE && screen != SCREEN_WAIT_RESPONSE) startRecording();
      break;
    case PRESS_NONE:
      break;
  }

  int16_t tx = 0, ty = 0;
  bool tap = tapped(tx, ty);

  // A touch or a press turns a tilt wake into a normal one, with the
  // normal timeouts from now on.
  if (tiltWakeUntouched && (fingerDown || pressPending || press != PRESS_NONE)) {
    tiltWakeUntouched = false;
    tiltStats.used++;
    screenSince = millis();
  }

  // Keep the status bar current: new minute, phone link change.
  if (linkChanged) {
    linkChanged = false;
    refreshScreen();
  } else if (clockValid() && screen != SCREEN_BG_SYNC) {
    struct tm t;
    localNow(t);
    if (t.tm_min != lastBarMinute) {
      if (screen == SCREEN_CLOCK) drawClock();
      else drawStatusBar();
    }
  }

  uint32_t onScreen = millis() - screenSince;

  switch (screen) {
    case SCREEN_CLOCK:
      if (tap) {
        startBle();
        enterScreen(SCREEN_MENU);
      } else if (onScreen > watch::clockTimeoutMs(tiltWakeUntouched, clockShowMs())) {
        if (tiltWakeUntouched) tiltStats.expired++;
        goToSleep();
      }
      break;

    case SCREEN_MENU:
      if (tap) {
        screenSince = millis();
        if (ty >= MENU_BTN_Y - 8) {
          buzz(HAPTIC_CLICK);
          enterScreen(SCREEN_SETTINGS);
          break;
        }
      }
      if (millis() - screenSince > MENU_IDLE_MS) goToSleep();
      break;

    case SCREEN_SETTINGS:
      if (tap) {
        screenSince = millis();
        int target = settingsTap(tx, ty);
        if (target == SETTINGS_ROWS) {
          enterScreen(SCREEN_MENU);
        } else if (target >= 0) {
          editRow = target;
          enterScreen(SCREEN_EDIT);
        }
      }
      if (millis() - screenSince > SETTINGS_IDLE_MS) goToSleep();
      break;

    case SCREEN_EDIT:
      if (fingerDown) screenSince = millis();
      if (tap && ty >= DONE_Y - 8) {
        saveSettings();
        buzz(HAPTIC_CLICK);
        enterScreen(SCREEN_SETTINGS);
      } else if (fingerDown && ty >= SLIDER_TOUCH_Y0 && ty <= SLIDER_TOUCH_Y1) {
        editSlide(tx);
      }
      if (millis() - screenSince > SETTINGS_IDLE_MS) {
        saveSettings();
        goToSleep();
      }
      break;

    case SCREEN_RECORDING:
      recordChunk();
      break;

    // A command on its way: no timeout into sleep unless the battery is
    // low. The backlight dims while nothing happens; a tap lights it.
    case SCREEN_WAIT_PHONE:
    case SCREEN_WAIT_RESPONSE: {
      static uint32_t shownSecond = UINT32_MAX;
      const uint32_t second = (millis() - commandStartedAt) / 1000;
      if (second != shownSecond) {
        shownSecond = second;
        drawWaitTimer();
      }
      if (tap) applyBrightness();
      else if (!backlightDimmed && millis() - litSince > WAIT_DIM_MS) dimBacklight();

      if (screen == SCREEN_WAIT_PHONE) {
        if (phoneReady()) {
          sendRecording();
        } else if (sendTimeoutApplies() && onScreen > PHONE_WAIT_LOWBAT_MS) {
          recordedBytes = 0;
          showLocalError("No phone connected, command discarded (battery low)");
        }
      } else if (sendTimeoutApplies() && onScreen > RESPONSE_WAIT_LOWBAT_MS) {
        showLocalError("No answer yet (battery low). The phone app keeps the result.");
      }
      break;
    }

    case SCREEN_RESPONSE:
      if (tap) screenSince = millis();
      if (millis() - screenSince > RESPONSE_SHOW_MS) goToSleep();
      break;

    case SCREEN_BG_SYNC:
      if (tap) {
        wakeFromBackground();
      } else if (timeSynced) {
        goToSleep();                // bgSyncFailures was reset by rememberSync()
      } else if (onScreen > BG_SYNC_WINDOW_MS) {
        if (bgSyncFailures < 255) bgSyncFailures++;
        Serial.printf("background sync: no phone (%u in a row)\n", bgSyncFailures);
        goToSleep();
      }
      break;
  }

  // Fast passes while recording, and while waiting for the RTC tick (at
  // most a second after waking): its moment is caught to within a pass.
  delay(isRecording || rtcTickLockPending ? 1 : 20);
}
