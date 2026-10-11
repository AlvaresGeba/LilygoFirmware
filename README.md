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
- **Deep sleep** between uses; wakes on a tap or the side button.
- **Settings** stored in flash: brightness (5–100 %), interface color, screen
  timeout, vibration, 180° screen rotation.
- **Haptic feedback** through the built-in DRV2605 motor driver.
- **Clock sync** from the phone on every connect, kept in the PCF8563 RTC.

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
| TIME | `6e400005-…` | write, phone → watch | ASCII `<unix seconds>,<UTC offset in minutes>` |

The app should:

1. Request an ATT MTU of 517: chunks are 512 bytes and do not fit a smaller MTU.
2. Subscribe to COMMAND and STATUS. The watch only sends once both
   subscriptions are in place.
3. Write TIME after subscribing. This is optional: without it the clock keeps
   whatever time the RTC has.
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

### Setting the time over USB

Until the phone app has synced it, the clock can be set over the serial port by
sending `T<unix seconds> <UTC offset in minutes>` and a newline, for example:

```sh
printf 'T%d %d\n' "$(date +%s)" 120 > /dev/ttyACM0
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
├── platformio.ini               # board, flash/PSRAM, libraries, display setup
└── src/
    └── twatch_command_client.ino
```
