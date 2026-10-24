#!/bin/bash
# Builds and runs the ULP program check on the PC. Needs g++ and the
# PlatformIO ESP32 framework (installed by the first `pio run`).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
sdk="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}/packages/framework-arduinoespressif32/tools/sdk/esp32s3"
inc=(-I"$sdk/qio_opi/include" -I"$sdk/include/ulp/include" -I"$sdk/include/soc/esp32s3/include"
     -I"$sdk/include/soc/include" -I"$sdk/include/esp_common/include" -I"$sdk/include/esp_hw_support/include"
     -I"$sdk/include/hal/esp32s3/include" -I"$sdk/include/hal/include" -I"$sdk/include/esp_rom/include"
     -I"$repo/lib/WatchCore/src" -I"$repo/src")
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
# -Wno-narrowing: ulp.h divides by sizeof() (64-bit on the PC) in a brace initializer.
g++ -std=gnu++17 -O1 -Wall -Wextra -Wno-narrowing "${inc[@]}" "$here/pose_ulp_check.cpp" "$repo/src/pose_ulp.cpp" \
    -o "$out/pose_ulp_check"
"$out/pose_ulp_check" "$@"
