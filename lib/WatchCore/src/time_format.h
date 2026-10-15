// How long a command took, as text for the watch's small screen.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace watch {

// A running timer, whole seconds: "0:07", "12:34", "1:02:03".
void formatTimer(uint32_t ms, char *out, size_t size);

// A finished duration: "0.8 s", "12.4 s" under a minute (tenths, rounded
// down so a timer that showed 0:12 never reads 13.0 s), then "1:05" or
// "1:02:03".
void formatDuration(uint32_t ms, char *out, size_t size);

}  // namespace watch
