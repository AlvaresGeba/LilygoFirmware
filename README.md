# LilygoFirmware

Firmware for the **LILYGO T-Watch S3** that turns the watch into a voice-command
remote for an Android companion app. The watch records a short voice command,
streams it to the phone over Bluetooth Low Energy, and shows the server's answer
on its screen. Between uses it stays in deep sleep and works as a plain watch.

```
T-Watch S3 ──BLE──▶ Android app ──▶ your server (speech-to-text, command router)
     ▲                                   │
     └──────── answer text ◀─────────────┘
```

The watch never talks to the network itself; everything beyond Bluetooth is the
phone app's job.

## Features

- **Watch face** with date and battery level; a status bar with time, phone
  link and battery on every screen.
- **Voice commands**: double-press the side button, speak, press once to send.
  Audio is 16 kHz / 16-bit mono PCM, up to 8 seconds.
- **Server answers and errors on screen**: an icon and a colored label show
  whether the answer is `OK` (check), `REJECTED` (!) or `ERROR` (cross),
  with its own vibration for failures.
  Text wraps by word (built-in 6x8 font, Latin only). Errors the watch
  detects itself (no phone, link lost) look the same.
- **Response timer**: the waiting screens count up from the moment the
  command is sent; the answer screen and the menu show how long it took.
- **Never gives up on a command**: while a command is on its way the watch
  stays awake until the answer or an error arrives (the backlight dims after
  30 s). Only on battery below 30 % does it fall back to timeouts.
- **Deep sleep** between uses; wakes on a tap, the side button or a raised
  wrist.
- **Raise to wake**: turning the watch into the pose you read it in (you
  calibrate it once) shows the clock for 4 s; touching the watch or pressing
  the button keeps it on as usual. Three sensitivity levels.
  See [Raise to wake](#raise-to-wake).
- **Settings** stored in flash: brightness (5–100 %), interface color, screen
  timeout, vibration, 180° screen rotation, raise to wake (on/off,
  sensitivity, calibration).
- **Haptic feedback** through the built-in DRV2605 motor driver.
- **Accurate clock**: the PCF8563 RTC (quartz) is the reference and is
  checked on every wake; the phone sets it on every connect, and once a day
  the watch fetches the time by itself with the screen off. See
  [Time keeping](#time-keeping).

## Hardware

Tested on the **T-Watch S3** (ESP32-S3R8, 8 MB PSRAM, 16 MB flash, SX1262).
Pins follow LILYGO's
[`utilities.h`](https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library/blob/t-watch-s3/src/utilities.h)
(`USING_TWATCH_S3`).

| Part | Used for | Connection |
|---|---|---|
| ST7789 240×240 display | UI | SPI: MOSI 13, SCLK 18, CS 12, DC 38, backlight 45 |
| FT6336 touch | taps, sliders, wake from sleep | I2C on 39/40, INT 16 |
| AXP2101 PMU | power rails, side button, battery | I2C on 10/11, IRQ 21 |
| PCF8563 RTC | keeps time across power loss | I2C on 10/11 |
| BMA423 accelerometer | raise to wake (pose, read by the ULP in deep sleep) | I2C on 10/11 (INT1 on 14, unused) |
| DRV2605 | vibration | I2C on 10/11, powered from BLDO2 |
| PDM microphone | voice recording | clock 44, data 47 |

The SX1262 LoRa radio and the GPS rail are switched off. The side button is the
AXP2101 power key, not a GPIO. The crown turns, but only mechanically: on the
T-Watch S3 it is a plain push button with no rotary encoder (LilyGoLib reports
`hasRotary = false`).

## Controls

| Action | Result |
|---|---|
| Tap the screen (asleep) | Wake and show the clock |
| Raise the wrist into the calibrated pose (asleep) | Show the clock for 4 s; a touch or a press keeps the watch awake |
| Tap while the clock is shown | Info screen: phone link, last server answer, **Settings** button |
| Side button, single press | Sleep (or wake, when asleep) |
| Side button, double press | Start recording, also straight from sleep |
| Side button, single press while recording | Stop and send |
| Side button, held 4 s or longer | Power off (the PMU does this, whatever the firmware is doing) |

The watch goes back to sleep on its own: after the screen timeout on the clock
(4 s after a raised wrist, unless touched), 15 s on the info screen, 20 s in settings, and 10 s after showing an answer.
It does not time out while a command is being delivered (waiting for the
phone, sending, waiting for the server), unless the battery is below 30 %
and not charging; then it waits 20 s for the phone and 60 s for the answer.
A single press cancels the wait at any time.

In **Settings**, tap *Brightness*, *Color* or *Sleep after* to open a slider and
drag a finger along it; tap *Vibration* or *Rotate* to toggle them. *Raise to
wake* opens its own page: tap *Raise to wake* to switch it on or off, *Wake
sens* to cycle Low / Med / High, and *Calibrate wake* to store the pose (see
[Raise to wake](#raise-to-wake)).

## BLE protocol

The watch is a GATT server advertising as `T-Watch-Command`. The phone app is
the client. This protocol is the only contract between the firmware and the app,
so any app version that implements it can talk to the watch.

Service `6e400001-b5a3-f393-e0a9-e50e24dcca9e`:

| Characteristic | UUID | Direction | Payload |
|---|---|---|---|
| COMMAND | `6e400002-…` | notify, watch → phone | `[u16 chunk index LE][u16 total chunks LE][up to 508 bytes of PCM]` |
| STATUS | `6e400003-…` | notify, watch → phone | ASCII `DONE` after the last chunk |
| RESPONSE | `6e400004-…` | write, phone → watch | UTF-8 text to show (up to 500 bytes) |
| TIME | `6e400005-…` | write, phone → watch | ASCII `<unix seconds>,<UTC offset in minutes>[,<milliseconds>]` |
| TIME | `6e400005-…` | read | the watch's clock: `<unix seconds>.<ms>,<UTC offset>` |

RESPONSE texts starting with `OK`, `REJECTED` or `ERROR` (the gateway's
contract) are final answers and are shown with that status. `Processing...`
and `STATUS: …` are progress: the watch shows them on the waiting screen and
keeps waiting for the final answer. Anything else is shown as a plain answer.

The app should:

1. Request an ATT MTU of 517: chunks are 512 bytes and do not fit a smaller MTU.
2. Subscribe to COMMAND and STATUS. The watch only sends once both
   subscriptions are in place.
3. Write TIME after subscribing, on every connection. The optional
   milliseconds field makes the clock exact to a few tens of milliseconds.
   The watch's daily background sync relies on this: it wakes, advertises
   for 20 s and goes back to sleep as soon as TIME is written.
4. On `DONE`, join the chunks in index order and check that the count matches
   `total chunks`.

The watch drops the link whenever it sleeps. Reconnecting with
`connectGatt(autoConnect = true)` to the remembered device is cheaper for the
phone than scanning continuously.

Audio is raw PCM, 16 kHz, signed 16-bit little-endian, mono, without a WAV
header.

## Building and flashing

You need [PlatformIO Core](https://platformio.org/install/cli) (for example
`pipx install platformio`). Versions of the platform and libraries are pinned in
`platformio.ini`, and the TFT_eSPI display setup is passed as build flags, so no
library files need editing.

```sh
pio run                  # build
pio run -t upload        # build and flash over USB
pio device monitor       # serial log
```

The watch only shows up as a USB serial port (`/dev/ttyACM*`) while it is
awake. Tap the screen just before uploading, or start the upload and wake the
watch while PlatformIO waits for the port.

### USB console

While awake, the watch takes commands on the serial port (115200 baud, one
per line); `H` lists them:

| Command | Does |
|---|---|
| `T<unix>[.<frac>] [<offset min>]` | set the clock, e.g. `T1760000000.25 120` |
| `Q` | clock report: system clock, RTC, sleep clock, last correction |
| `R` | wait for the RTC's next second and report it (sub-second phase) |
| `Z<sec>` | deep sleep now, wake by timer (drift tests) |
| `C<0\|1>` | sleep clock: 0 = RC 136 kHz (default), 1 = RC_FAST/256 (saved) |
| `N<0\|1>` | RTC correction on wake off/on (tests; off lasts until reset) |
| `S` | hardware probe: I2C devices, sensors, radio, GPS, microphone |
| `P<sec>` | raw touch controller registers for <sec> seconds (diagnosing touch wakes) |
| `G` | raise to wake: pose, thresholds, current reading, ULP state and counters, how raise wakes ended |
| `U[n]` | raise to wake: the sleep-time ULP program run awake on the handed-over bus for n runs (default 5; n not 1, 8 or 9), then `G` |
| `U1` | raise to wake: RTC pad check and address probes bit-banged by the CPU over the RTC pads |
| `U8` | raise to wake: the ULP's branch conditions on this chip (R0 and stage_cnt), see below |
| `U9` | raise to wake: logic capture of one ULP run (every SDA/SCL change, in µs) |
| `Y` | raise to wake: sleep now with the detector armed; held in the pose, the watch wakes at once (tests the wake path without moving it) |
| `K<sec>` | stay awake for <sec> seconds (debugging over USB; `K0` ends it) |
| `W` | UI test: the waiting screen with the response timer running |
| `A<text>` | UI test: an answer from the server, e.g. `AOK: lights on`, `AERROR: timeout` |

`tools/watch_console.py` sends them and prints the answers:

```sh
tools/watch_console.py --set-time      # set the clock from this computer
tools/watch_console.py S               # list and check the sensors
```

## Time keeping

In deep sleep the ESP32-S3 keeps time with its RTC timer, clocked by an
internal RC oscillator whose frequency is far from its awake calibration.
The old firmware trusted it across every sleep, so the clock ran ahead by
minutes a day. Measured on this watch with `tools/clock_drift_test.py`:

| clock | drift in deep sleep |
|---|---|
| ESP32-S3 RC 136 kHz (IDF default) | +2800 ppm over 1-minute sleeps (+4 min/day); +1.7 % over a 36-minute sleep |
| ESP32-S3 RC_FAST/256 (17.5 MHz RC / 256) | −23 %, about −5 h/day |
| PCF8563 (32.768 kHz quartz) | +7.7 ppm, about +0.7 s/day |

Now:

1. **The PCF8563 is the reference.** On every wake the system clock is
   pulled into the RTC's current second if it has wandered off
   (`rtcCorrectionUs` in `lib/WatchCore`); within the next second, while the
   screen is on anyway, the watch catches the moment the RTC's seconds
   register changes and steps the clock onto that exact boundary
   (`rtcTickCorrectionUs`). Measured: within 16 ms of NTP time after every test
   wake (sleeps of 20-60 s; the error no longer grows with sleep length).
2. **The RTC is written in phase.** The PCF8563's 1 Hz divider is not reset
   by writing the time, so the watch stops it (STOP bit, which clears the
   divider), loads the time and releases it so that its first tick, 1.0009 s
   later (measured), lands on the system clock's second boundary.
3. **The sleep clock is the 136 kHz RC**, calibrated against the 40 MHz
   crystal right before each sleep. RC_FAST/256 is selectable with `C1` for
   experiments, but it is much worse in deep sleep and costs ~5 µA more.
4. **The phone corrects the RTC** on every connection (TIME, with
   milliseconds). If the phone has not connected for 24 hours the watch
   wakes by itself with the screen off, advertises for 20 s and sleeps again
   as soon as it has the time. Failed attempts are retried after 1, 2, 4, …
   hours, and skipped below 15 % battery. Daily is plenty: the PCF8563 gains
   under a second a day, and each background wake costs radio time.

`tools/clock_drift_test.py` measures all of this on the watch against the
computer's NTP-synced clock: the drift of both sleep clocks with the
correction off, the PCF8563's drift, and the end-to-end error with the
correction on.

## Raise to wake

The watch wakes when it is turned into a stored pose: the way you hold it to
read it. In deep sleep the ULP coprocessor (the FSM) reads the BMA423 every
100 ms and wakes the CPU (wake cause `ESP_SLEEP_WAKEUP_ULP`) on a
*transition into* that pose.

**Calibrating.** Settings > Raise to wake > *Calibrate wake*. Hold the watch
as when reading it and keep it still during the 3-2-1 countdown. The
samples of the last two seconds are averaged into a gravity vector, stored
in flash in the accelerometer's own axes (so the *Rotate* setting does not
matter). It is rejected if the watch moved (more than 120 mg peak to peak on
an axis, about 7°) or the mean is far from 1 g (0.85–1.15 g); the screen
then says why. Without a calibration a default applies: screen facing up
(the chip's z axis at -1 g; the chip sits on the bottom layer of the board).

**Sensitivity.** How close to the stored pose the watch has to be, as a box
per axis around the stored vector (the ULP has no multiply, so no angles):

| Wake sens | wakes within | re-arms beyond |
|---|---|---|
| Low  | ~12° (±209 mg per axis) | ~27° (±467 mg) |
| Med  | ~18° (±313 mg) | ~33° (±568 mg) |
| High | ~25° (±433 mg) | ~40° (±684 mg) |

A turn up to the first angle always counts as the pose, whatever its
direction; turned past the second angle the watch counts as away. Turns that
move two or three axes at once (diagonal to the boxes) get up to ~1.4 times
more room on both.

**Detector** (`watch::poseStep` in `lib/WatchCore/src/wrist_wake.h`; the ULP
program implements the same steps):

- the watch has to be outside the wider "re-arm" box for 4 samples in a row
  (0.4 s), which arms the detector;
- then 2 samples in a row (0.2 s) inside the "wake" box wake the CPU and
  disarm the detector;
- it is also disarmed every time the watch goes to sleep. A watch lying
  still in the pose (on a desk, say) or still being looked at never wakes
  again until it has left the pose.

**After the wake** the CPU reads the accelerometer itself (about 0.3 s
later). If the watch is already outside the re-arm box again, it goes back
to sleep without lighting the screen. Otherwise the clock shows for 4 s
unless the watch is touched or the button is pressed. Three unconfirmed
wakes in a row pause raise to wake until the next wake by touch or button
(guard against a wake loop).

**The ULP program** (`src/pose_ulp.*`). The BMA423 is on the shared I2C bus
(GPIO 10/11, with the PMU, the PCF8563 and the DRV2605). Those are RTC GPIOs
on the S3 but not the RTC I2C pins, so the program bit-bangs I2C through the
RTC GPIO registers:

- before sleeping the CPU finishes its own I2C traffic, then switches SDA and
  SCL to the RTC IO mux as **open-drain** pads with the output latch at 1
  (released) and pull-ups on. The program pulls a line low by writing its
  latch to 0, so it can never drive a line high against a device;
- each run (~0.8 ms) checks that the bus is idle, sends START, `0x19`+W,
  register `0x13`, repeated START, `0x19`+R, reads five bytes (X_MSB, Y_LSB,
  Y_MSB, Z_LSB, Z_MSB; the last one NACKed) and a STOP. Only the high bytes
  are used: 64 counts per g in the ±2 g range. SCL runs at ~115 kHz (high
  ≥ 4 µs, low ≥ 4.6 µs). Only address 0x19 is ever sent;
- a missing ACK ends the transfer with a STOP and counts an error; SDA held
  low by a device is freed with nine clocks and a STOP; SCL held low makes
  the run skip the bus entirely;
- on every boot the CPU stops the ULP timer, waits for a run in progress to
  finish (a busy flag the program sets and clears) and only then hands the
  pins back to the digital I2C peripheral.
- the bit loops end on `stage_cnt == n`: on the S3 only the EQ condition of
  the stage branch (JUMPS) works. Measured with `U8`: the codes the IDF's
  `ulp.h` emits for `I_BSL` (stage_cnt < n) and `I_BSGE` are never taken,
  which made every byte loop run once and every run end in a NACK. R0
  branches (`I_BL`, `I_BG`, EQ) work as documented.

The Arduino core reserves no RTC memory for the ULP, so the program (262
words, 320 reserved) and its variables (28 words) are `RTC_DATA_ATTR` arrays;
the program is encoded at run time against wherever the linker put them and
started with `ulp_run()`. Shared variables are written to both half-words,
since the IDF documents the S3's LD instruction both ways. To check the
program on the PC after changing it:

```sh
tools/pose_ulp_check/check.sh       # add -l for a listing of the encoded words
```

It builds the encoder against the ESP-IDF headers from the PlatformIO
install, decodes and executes the words against a simulated BMA423 on an
open-drain bus, checks the I2C sequence and timing, the error paths, and
compares every wake decision with `watch::poseStep`.

**The accelerometer** runs without its feature engine (no config upload):
low-power mode, 25 Hz, 4-sample averaging, ±2 g, interrupt outputs off. It
stays powered and configured through deep sleep; after power-on it is
soft-reset once (this also clears the previous firmware's wrist-tilt setup).
With raise to wake off it is suspended and the ULP is not started.

**Cost** in deep sleep, estimated: the BMA423 in low-power mode ~10–15 µA,
the ULP runs ~2 µA, the pull-up current while the bus is driven low ~3–5 µA
(with 10 kΩ pull-ups): roughly 15–25 µA in all. Plus about 4 s of screen
time per false raise.

`G` on the USB console shows: the setting, sensitivity, stored pose and
thresholds; the current reading and its zone; the boxes in ULP units; the
ULP's counters (`runs`, `errors`, `last_error`, `wakes`), its last sample,
zone and detector state, the sample that woke the CPU; and how raise wakes
ended (`shown`, `moved`: went back to sleep dark, `expired`, `used`).
`last_error`: 1 SCL held low, 2 SDA held low (freed), 3/4/5 no ACK to the
address, register, or read address.

## Tests

```sh
pio test -e native                          # unit tests of lib/WatchCore, on the PC
tools/pose_ulp_check/check.sh               # the raise-to-wake ULP program, emulated on the PC
tools/clock_drift_test.py --cycles 6 --sleep 60   # on the watch (awake, plugged in)
```

## Troubleshooting

- **Screen stays black and the watch ignores taps and the button.** Hold the
  side button for more than 4 seconds to power off, then hold it for about a
  second to power on again.
- **Upload cannot find the port.** The watch is asleep. Wake it and try again.
- **Build hangs at the linking step.** Some on-access antivirus scanners (for
  example ClamAV's `clamonacc`) block every file access in the folders they
  watch. Keep the project outside those folders.
- **Phone receives nothing.** Check that the app requests MTU 517 and
  subscribes to both COMMAND and STATUS. The info screen shows whether the
  phone is connected.

## Project layout

```
.
├── platformio.ini               # board, flash/PSRAM, libraries, display setup; native test env
├── src/
│   ├── twatch_command_client.ino   # the firmware
│   └── pose_ulp.*               # raise to wake: the ULP program (bit-banged I2C + detector)
├── lib/WatchCore/src/           # hardware-independent logic, unit-tested on the PC
│   ├── clock_sync.*             # TIME parsing, RTC correction, background sync schedule
│   ├── server_reply.*           # OK/REJECTED/ERROR/progress, safe UTF-8 copy
│   ├── power_policy.h           # when the watch may time out / sync in the background
│   ├── wrist_wake.h             # raise to wake: pose boxes, detector, calibration, loop guard
│   ├── text_wrap.*              # word wrap by pixel width, UTF-8 aware
│   └── time_format.*            # response timer and duration text
├── test/                        # Unity tests for lib/WatchCore (pio test -e native)
└── tools/
    ├── clock_drift_test.py      # drift measurement on the watch
    ├── pose_ulp_check/          # PC check of the encoded ULP program (emulated)
    └── watch_console.py         # USB console client (sensor probe, set time)
```
