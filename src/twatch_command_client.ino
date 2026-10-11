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

  The status menu has a Settings button at the bottom: brightness (5-100%),
  interface color and sleep timeout are set with a touch slider; vibration
  and screen rotation toggle on tap. (The crown only clicks: on T-Watch S3
  it has no rotary encoder - LilyGoLib marks hasRotary = false.)
  Settings live in NVS. Every screen has a status bar on top with the
  time, the phone link and the battery level.

  Wire protocol (must match BleWatchService.kt exactly):
    COMMAND_CHAR   notify, watch -> phone, chunked payload:
                   [u16 chunk_index][u16 total_chunks][payload bytes]
    STATUS_CHAR    notify, watch -> phone, ASCII "DONE" once last chunk sent
    RESPONSE_CHAR  write, phone -> watch, plain UTF-8 text to display
    TIME_CHAR      write, phone -> watch, ASCII "<unix seconds>,<utc offset
                   minutes>" - sets the clock and the PCF8563 RTC

  The clock can also be set over USB serial: "T<unix seconds> <offset min>".

  Build with PlatformIO: `pio run -t upload` in the repository root
  (platformio.ini pins the platform, libraries and the TFT_eSPI setup).
  Libraries: NimBLE-Arduino 1.4.x, TFT_eSPI, XPowersLib.

  Pin numbers follow LilyGO's TTGO_TWatch_Library src/utilities.h,
  section USING_TWATCH_S3 (t-watch-s3 branch).
*/

#include <Wire.h>
#include <NimBLEDevice.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <sys/time.h>

#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// ---------------------------------------------------------------------
// Hardware pins (TTGO_TWatch_Library utilities.h, USING_TWATCH_S3)
// ---------------------------------------------------------------------
// The side button is not a GPIO: it is the AXP2101 PWRKEY, reported
// through the PMU interrupt line.
#define PMU_I2C_SDA       10    // shared with the PCF8563 RTC and DRV2605
#define PMU_I2C_SCL       11
#define PMU_IRQ_PIN       21
#define TOUCH_I2C_SDA     39
#define TOUCH_I2C_SCL     40
#define TOUCH_INT_PIN     16
#define TFT_BL_PIN        45    // backlight, powered from AXP2101 ALDO2
#define MIC_CLOCK_PIN     44    // PDM clock
#define MIC_DATA_PIN      47    // PDM data

#define PCF8563_ADDR      0x51
#define FT6336_ADDR       0x38
#define DRV2605_ADDR      0x5A

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

// ---------------------------------------------------------------------
// Screen timeouts (ms); the clock timeout is a setting
// ---------------------------------------------------------------------
#define MENU_IDLE_MS      15000  // menu stays up this long after the last tap
#define SETTINGS_IDLE_MS  20000
#define PHONE_WAIT_MS     20000  // recording done, waiting for the phone to reconnect
#define RESPONSE_WAIT_MS  30000  // audio sent, waiting for the server's answer
#define RESPONSE_SHOW_MS  10000  // answer on screen before sleeping

// ---------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------
#define SCREEN_W          240
#define SCREEN_H          240
#define BAR_H             24     // status bar
#define ROW_Y0            BAR_H  // settings rows
#define ROW_H             36
#define SETTINGS_ROWS     5
#define BACK_Y            (ROW_Y0 + SETTINGS_ROWS * ROW_H)

static const size_t RECORD_BUFFER_BYTES = SAMPLE_RATE * 2 * MAX_RECORD_SECONDS; // 16-bit mono
uint8_t *recordBuffer = nullptr;    // allocated in PSRAM in setup()
size_t recordedBytes = 0;
volatile bool isRecording = false;

TFT_eSPI tft = TFT_eSPI();

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

// Set from BLE callbacks (NimBLE host task), consumed in loop(): the
// display and the I2C buses are only touched from the main task.
volatile bool linkChanged = false;
volatile bool responseArrived = false;
volatile bool timeArrived = false;
volatile int64_t pendingEpoch = 0;
volatile int pendingTzMin = 0;

// Survives deep sleep, so the menu can show the last answer.
RTC_DATA_ATTR char lastResponse[200] = "";

enum Screen : uint8_t {
  SCREEN_CLOCK,
  SCREEN_MENU,
  SCREEN_SETTINGS,
  SCREEN_EDIT,                      // one setting with a slider
  SCREEN_RECORDING,
  SCREEN_WAIT_PHONE,
  SCREEN_WAIT_RESPONSE,
  SCREEN_RESPONSE,
};
Screen screen = SCREEN_CLOCK;
uint32_t screenSince = 0;           // millis() when the screen was entered or last tapped

enum Press : uint8_t { PRESS_NONE, PRESS_SINGLE, PRESS_DOUBLE };

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
} cfg;

uint16_t accent() { return ACCENTS[cfg.accent].rgb565; }
uint32_t clockShowMs() { return SLEEP_SECONDS[cfg.sleepAfter] * 1000UL; }

void loadSettings() {
  cfg.brightness = constrain(prefs.getUChar("brightpct", 60), BRIGHTNESS_MIN_PCT, 100);
  cfg.accent = min<uint8_t>(prefs.getUChar("accent", 0), ACCENT_COUNT - 1);
  cfg.sleepAfter = min<uint8_t>(prefs.getUChar("sleep", 1), SLEEP_OPTIONS - 1);
  cfg.vibration = prefs.getBool("vibro", true);
  cfg.flipped = prefs.getBool("flip", false);
}

void saveSettings() {
  prefs.putUChar("brightpct", cfg.brightness);
  prefs.putUChar("accent", cfg.accent);
  prefs.putUChar("sleep", cfg.sleepAfter);
  prefs.putBool("vibro", cfg.vibration);
  prefs.putBool("flip", cfg.flipped);
}

void applyBrightness() { ledcWrite(BL_LEDC_CHANNEL, cfg.brightness * 255 / 100); }

// ---------------------------------------------------------------------
// Vibration (DRV2605, register sequence as in SensorLib's SensorDRV2605)
// ---------------------------------------------------------------------

#define HAPTIC_CLICK      1      // ROM effect "strong click 100%"
#define HAPTIC_DOUBLE     10     // ROM effect "double click 100%"
#define HAPTIC_TICK       3      // ROM effect "strong click 30%": slider steps

bool hapticReady = false;

void drvWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DRV2605_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t drvRead(uint8_t reg) {
  Wire.beginTransmission(DRV2605_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(DRV2605_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

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
// Clock: system time, PCF8563 RTC
// ---------------------------------------------------------------------

static uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

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
  return utc > 1700000000;
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

bool clockValid() { return time(nullptr) > 1700000000; }

void setClock(time_t utc, int tzMin) {
  struct timeval tv = {utc, 0};
  settimeofday(&tv, nullptr);
  rtcWrite(utc);
  if (tzMin != tzOffsetMin) {
    tzOffsetMin = tzMin;
    prefs.putInt("tz", tzMin);
  }
}

// After a deep-sleep wake the system clock is still running; after a
// power-on or reset it starts from zero and is reloaded from the RTC.
void loadClock() {
  tzOffsetMin = prefs.getInt("tz", 0);
  if (clockValid()) return;
  time_t utc;
  if (rtcRead(utc)) {
    struct timeval tv = {utc, 0};
    settimeofday(&tv, nullptr);
  }
}

void localNow(struct tm &out) {
  time_t local = time(nullptr) + (time_t)tzOffsetMin * 60;
  gmtime_r(&local, &out);
}

// "T<unix seconds> <offset min>" over USB serial.
void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') {
      if (line.length() < 40) line += c;
      continue;
    }
    if (line.startsWith("T")) {
      long long epoch = 0;
      int tz = 0;
      if (sscanf(line.c_str() + 1, "%lld %d", &epoch, &tz) >= 1 && epoch > 1700000000) {
        setClock((time_t)epoch, tz);
        Serial.printf("time set: %lld, tz %d min\n", epoch, tz);
        linkChanged = true;         // redraw
      }
    }
    line = "";
  }
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

// Naive word-wrap into lines of charsPerLine at the current text size.
void drawWrapped(const char *text, int y, int charsPerLine, int lineHeight, int maxY = SCREEN_H) {
  String s(text);
  int start = 0;
  while (start < (int)s.length() && y + lineHeight <= maxY) {
    int end = min(start + charsPerLine, (int)s.length());
    tft.drawString(s.substring(start, end), 10, y);
    y += lineHeight;
    start = end;
  }
}

// Server response text is expected to be short JSON-ish or plain text,
// e.g. "OK: systemctl status nginx" or "REJECTED: needs confirm=true".
void showServerResponse(const char *response) {
  clearBody();
  drawStatusBar();
  tft.setTextColor(strncmp(response, "OK", 2) == 0 ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(2);
  drawWrapped(response, BAR_H + 12, 18, 24);
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
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(strncmp(lastResponse, "OK", 2) == 0 ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
    tft.drawString("Last answer:", 10, 104);
    drawWrapped(lastResponse, 120, 36, 12, MENU_BTN_Y - 4);
  }

  tft.drawRoundRect(8, MENU_BTN_Y, SCREEN_W - 16, MENU_BTN_H, 8, accent());
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(accent(), TFT_BLACK);
  tft.drawString("Settings", cx, MENU_BTN_Y + MENU_BTN_H / 2, 4);
}

// ---------------------------------------------------------------------
// Settings screen: a list of settings, Back at the bottom (returns to the
// status menu). Tapping Vibration or Rotate flips it; tapping Brightness,
// Color or Sleep after opens SCREEN_EDIT, where the value follows a finger
// dragged along a slider.
// ---------------------------------------------------------------------

#define ROW_BRIGHTNESS    0
#define ROW_COLOR         1
#define ROW_SLEEP         2
#define ROW_VIBRATION     3
#define ROW_ROTATE        4

static const char *SETTING_LABELS[SETTINGS_ROWS] = {"Brightness", "Color", "Sleep after", "Vibration", "Rotate"};

bool rowHasSlider(int row) { return row == ROW_BRIGHTNESS || row == ROW_COLOR || row == ROW_SLEEP; }

void settingValue(int row, char *buf, size_t len) {
  switch (row) {
    case ROW_BRIGHTNESS: snprintf(buf, len, "%d%%", cfg.brightness); break;
    case ROW_COLOR:      snprintf(buf, len, "%s", ACCENTS[cfg.accent].name); break;
    case ROW_SLEEP:      snprintf(buf, len, "%ds", SLEEP_SECONDS[cfg.sleepAfter]); break;
    case ROW_VIBRATION:  snprintf(buf, len, "%s", cfg.vibration ? "On" : "Off"); break;
    case ROW_ROTATE:     snprintf(buf, len, "%s", cfg.flipped ? "180" : "0"); break;
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
  switch (s) {
    case SCREEN_CLOCK:         drawClock(); break;
    case SCREEN_MENU:          drawMenu(); break;
    case SCREEN_SETTINGS:      drawSettings(); break;
    case SCREEN_EDIT:          drawEdit(); break;
    case SCREEN_RECORDING:     showStatus("Recording...", "press button to send"); break;
    case SCREEN_WAIT_PHONE:    showStatus("Waiting for phone...", "audio will be sent"); break;
    case SCREEN_WAIT_RESPONSE: showStatus("Sent", "Waiting for server..."); break;
    case SCREEN_RESPONSE:      showServerResponse(lastResponse); break;
  }
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
    strlcpy(lastResponse, value.c_str(), sizeof(lastResponse));
    responseArrived = true;
  }
};

class TimeCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic) override {
    std::string value = characteristic->getValue();
    long long epoch = 0;
    int tz = 0;
    if (sscanf(value.c_str(), "%lld,%d", &epoch, &tz) >= 1 && epoch > 1700000000) {
      pendingEpoch = epoch;
      pendingTzMin = tz;
      timeArrived = true;
    }
  }
};

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
  enterScreen(SCREEN_WAIT_PHONE);   // loop() sends as soon as the phone is ready
}

void sendRecording() {
  showStatus("Sending...", "");
  responseArrived = false;
  sendBufferOverBle(recordBuffer, recordedBytes);
  recordedBytes = 0;
  enterScreen(SCREEN_WAIT_RESPONSE);
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

void sendBufferOverBle(const uint8_t *data, size_t length) {
  uint16_t totalChunks = (length + CHUNK_PAYLOAD_SIZE - 1) / CHUNK_PAYLOAD_SIZE;
  uint32_t startedAt = millis();
  std::vector<uint16_t> peers = bleServer->getPeerDevices();
  Serial.printf("send: %u bytes, %u chunks, peers %u, mtu %u, subscribed %u/%u\n",
                (unsigned)length, totalChunks, (unsigned)peers.size(),
                peers.empty() ? 0 : bleServer->getPeerMTU(peers[0]),
                (unsigned)commandChar->getSubscribedCount(),
                (unsigned)statusChar->getSubscribedCount());

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
      return;
    }
  }

  notifyReliable(statusChar, (const uint8_t *)"DONE", 4);
  Serial.printf("send: done in %lu ms\n", (unsigned long)(millis() - startedAt));
}

// ---------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------

// Started only when needed (menu or recording): BLE is the biggest
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
      TIME_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
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

void goToSleep() {
  if (isRecording) i2s_stop(I2S_NUM_0);
  if (screen == SCREEN_EDIT) saveSettings();   // slider values apply live, unsaved

  // In polling mode the touch INT is held low while a finger is down;
  // sleeping now would wake immediately.
  uint32_t start = millis();
  while (touchDown() && millis() - start < 3000) delay(20);

  tft.writecommand(0x28);           // ST7789 DISPOFF
  tft.writecommand(0x10);           // ST7789 SLPIN
  ledcWrite(BL_LEDC_CHANNEL, 0);
  pmu.disableALDO2();               // backlight VDD
  pmu.disableBLDO2();               // vibration motor driver
  pmu.clearIrqStatus();             // release the IRQ line before arming it

  esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_INT_PIN, 0);
  esp_sleep_enable_ext1_wakeup(1ULL << PMU_IRQ_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
  rtc_gpio_pullup_en((gpio_num_t)TOUCH_INT_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)TOUCH_INT_PIN);
  rtc_gpio_pullup_en((gpio_num_t)PMU_IRQ_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)PMU_IRQ_PIN);

  // No Serial.flush() here: with USB plugged in but no terminal reading,
  // it never returns and the watch hangs with the screen already off.
  esp_deep_sleep_start();
}

void setup() {
  esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();

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
  applyBrightness();

  // PSRAM allocation - RECORD_BUFFER_BYTES is ~256KB at 16kHz/16-bit/8s,
  // won't fit reliably in internal RAM alongside BLE + display buffers.
  recordBuffer = (uint8_t *)ps_malloc(RECORD_BUFFER_BYTES);
  if (!recordBuffer) {
    showStatus("PSRAM alloc failed!");
    while (true) delay(1000);
  }

  setupI2SMic();

  if (wake == ESP_SLEEP_WAKEUP_EXT1) {
    // Woken by the side button. That press was cleared in setupPmu(); if a
    // second one follows, it is a double press and starts a recording.
    pressPending = true;
    pressPendingAt = 0;             // the wake press happened at boot
    pressIsWake = true;
    enterScreen(SCREEN_CLOCK);
  } else if (wake == ESP_SLEEP_WAKEUP_EXT0) {
    enterScreen(SCREEN_CLOCK);      // woken by a tap
  } else {
    // Power-on, reset or a fresh flash: open the menu so the phone can
    // reconnect and sync the time.
    startBle();
    enterScreen(SCREEN_MENU);
  }
}

void loop() {
  handleSerial();

  if (timeArrived) {
    timeArrived = false;
    setClock((time_t)pendingEpoch, pendingTzMin);
    linkChanged = true;             // redraw with the new time
  }

  // Side button: single press sleeps (the wake is handled by deep sleep),
  // double press records, a press while recording sends.
  switch (readButton()) {
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

  // Keep the status bar current: new minute, phone link change.
  if (linkChanged) {
    linkChanged = false;
    if (screen == SCREEN_MENU) drawMenu();
    else if (screen == SCREEN_CLOCK) drawClock();
    else drawStatusBar();
  } else if (clockValid()) {
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
      } else if (onScreen > clockShowMs()) {
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

    case SCREEN_WAIT_PHONE:
      if (phoneReady()) {
        sendRecording();
      } else if (onScreen > PHONE_WAIT_MS) {
        showStatus("No phone connected", "Command discarded");
        delay(2000);
        goToSleep();
      }
      break;

    case SCREEN_WAIT_RESPONSE:
      if (responseArrived) {
        responseArrived = false;
        buzz(HAPTIC_DOUBLE);
        enterScreen(SCREEN_RESPONSE);
      } else if (onScreen > RESPONSE_WAIT_MS) {
        showStatus("No answer", "from the server");
        delay(2000);
        goToSleep();
      }
      break;

    case SCREEN_RESPONSE:
      if (tap) screenSince = millis();
      if (millis() - screenSince > RESPONSE_SHOW_MS) goToSleep();
      break;
  }

  delay(isRecording ? 1 : 20);
}
