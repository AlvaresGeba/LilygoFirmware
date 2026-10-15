// Word wrap by rendered width, for proportional fonts and UTF-8 text
// (server answers are often Russian, two bytes per letter).
#pragma once

#include <stddef.h>

namespace watch {

// Returns the rendered width in pixels of s[0..len).
using MeasureFn = int (*)(const char *s, size_t len, void *ctx);

struct TextSpan {
  size_t start;  // byte offset into the text
  size_t len;    // bytes
};

// Breaks NUL-terminated `text` into at most `maxLines` lines, each no wider
// than `maxWidth`. Lines break at spaces and at '\n'; a word wider than a
// whole line is split between characters, never inside a UTF-8 sequence.
// Writes the lines to `lines` and returns how many there are. `*truncated`
// (if given) is set when text was left over.
size_t wrapText(const char *text, int maxWidth, MeasureFn measure, void *ctx, TextSpan *lines,
                size_t maxLines, bool *truncated);

// The longest prefix of s[0..len), in bytes and on a character boundary,
// that still fits in maxWidth with `suffix` (e.g. an ellipsis) after it.
size_t fitWithSuffix(const char *s, size_t len, const char *suffix, int maxWidth,
                     MeasureFn measure, void *ctx);

}  // namespace watch
