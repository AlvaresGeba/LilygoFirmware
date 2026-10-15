#include "time_format.h"

#include <stdio.h>

namespace watch {

void formatTimer(uint32_t ms, char *out, size_t size) {
  if (out == nullptr || size == 0) return;
  const uint32_t s = ms / 1000;
  const uint32_t h = s / 3600;
  if (h > 0) {
    snprintf(out, size, "%lu:%02lu:%02lu", (unsigned long)h, (unsigned long)(s / 60 % 60),
             (unsigned long)(s % 60));
  } else {
    snprintf(out, size, "%lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
  }
}

void formatDuration(uint32_t ms, char *out, size_t size) {
  if (out == nullptr || size == 0) return;
  if (ms < 60000) {
    const uint32_t tenths = ms / 100;
    snprintf(out, size, "%lu.%lu s", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10));
  } else {
    formatTimer(ms, out, size);
  }
}

}  // namespace watch
