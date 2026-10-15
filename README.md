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
- **Server answers and errors on screen**: a colored band shows whether the
  answer is `OK`, `REJECTED` or `ERROR`, with its own vibration for failures.
  Text wraps by word and renders Cyrillic (an anti-aliased DejaVu Sans font
  in flash). Errors the watch detects itself (no phone, link lost) look the
  same.
- **Never gives up on a command**: while a command is on its way the watch
  stays awake until the answer or an error arrives (the backlight dims after
  30 s). Only on battery below 30 % does it fall back to timeouts.
- **Deep sleep** between uses; wakes on a tap or the side button.
- **Settings** stored in flash: brightness (5–100 %), interface color, screen
  timeout, vibration, 180° screen rotation.
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
| Tap while the clock is shown | Info screen: phone link, last server answer, **Settings** button |
| Side button, single press | Sleep (or wake, when asleep) |
| Side button, double press | Start recording, also straight from sleep |
| Side button, single press while recording | Stop and send |
| Side button, held 4 s or longer | Power off (the PMU does this, whatever the firmware is doing) |

The watch goes back to sleep on its own: after the screen timeout on the clock,
15 s on the info screen, 20 s in settings, and 10 s after showing an answer.
It does not time out while a command is being delivered (waiting for the
phone, sending, waiting for the server), unless the battery is below 30 %
and not charging; then it waits 20 s for the phone and 60 s for the answer.
A single press cancels the wait at any time.

In **Settings**, tap *Brightness*, *Color* or *Sleep after* to open a slider and
drag a finger along it; tap *Vibration* or *Rotate* to toggle them.

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
| `C<0\|1>` | sleep clock: 0 = RC 136 kHz, 1 = RC_FAST/256 (saved) |
| `N<0\|1>` | RTC correction on wake off/on (tests; off lasts until reset) |
| `S` | hardware probe: I2C devices, sensors, radio, GPS, microphone |

`tools/watch_console.py` sends them and prints the answers:

```sh
tools/watch_console.py --set-time      # set the clock from this computer
tools/watch_console.py S               # list and check the sensors
```

## Time keeping

In deep sleep the ESP32-S3 keeps time with its RTC timer, clocked by an
internal RC oscillator. By default that is the 136 kHz RC, whose frequency
moves with temperature; the old firmware trusted it across every sleep, so
the clock ran ahead by minutes. Now:

1. **The PCF8563 is the reference.** It runs from its own 32.768 kHz quartz.
   On every wake the system clock is compared with it and pulled back into
   the RTC's current second if it has wandered off (`rtcCorrectionUs` in
   `lib/WatchCore`), so the error stays below one second however long the
   watch sleeps.
2. **The sleep clock is the more stable RC_FAST/256** (17.5 MHz RC / 256,
   about 68 kHz, ~5 µA more in deep sleep), calibrated against the 40 MHz
   crystal right before each sleep. This keeps the sub-second part tight
   and the timer wakes punctual. `C0` switches back to the 136 kHz RC.
3. **The phone corrects the RTC** on every connection (TIME, with
   milliseconds; the RTC is written on a second boundary). If the phone has
   not connected for 24 hours the watch wakes by itself with the screen off,
   advertises for 20 s and sleeps again as soon as it has the time. Failed
   attempts are retried after 1, 2, 4, … hours, and skipped below 15 %
   battery.

`tools/clock_drift_test.py` measures all of this on the watch against the
computer's NTP-synced clock: the drift of both sleep clocks with the
correction off, the PCF8563's drift, and the end-to-end error with the
correction on.

## Tests

```sh
pio test -e native                          # unit tests of lib/WatchCore, on the PC
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
│   └── fonts/                      # generated Latin + Cyrillic smooth fonts
├── lib/WatchCore/src/           # hardware-independent logic, unit-tested on the PC
│   ├── clock_sync.*             # TIME parsing, RTC correction, background sync schedule
│   ├── server_reply.*           # OK/REJECTED/ERROR/progress, safe UTF-8 copy
│   ├── power_policy.h           # when the watch may time out / sync in the background
│   └── text_wrap.*              # word wrap by pixel width, UTF-8 aware
├── test/                        # Unity tests for lib/WatchCore (pio test -e native)
└── tools/
    ├── clock_drift_test.py      # drift measurement on the watch
    ├── watch_console.py         # USB console client (sensor probe, set time)
    └── make_vlw_font.py         # TTF -> TFT_eSPI smooth font header
```

Regenerate the fonts with:

```sh
tools/make_vlw_font.py --ttf /usr/share/fonts/truetype/dejavu/DejaVuSans.ttf \
    --size 20 --name ui_font_20 --out src/fonts/ui_font_20.h
```
