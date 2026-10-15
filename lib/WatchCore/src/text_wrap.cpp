#include "text_wrap.h"

#include <string.h>

#include "server_reply.h"  // utf8SeqLen

namespace watch {

namespace {

bool isBlank(char c) { return c == ' ' || c == '\t' || c == '\r'; }

}  // namespace

size_t wrapText(const char *text, int maxWidth, MeasureFn measure, void *ctx, TextSpan *lines,
                size_t maxLines, bool *truncated) {
  const size_t n = text ? strlen(text) : 0;
  size_t count = 0;
  size_t pos = 0;
  while (pos < n && isBlank(text[pos])) ++pos;

  while (pos < n && count < maxLines) {
    const size_t begin = pos;
    if (text[begin] == '\n') {  // an empty line
      lines[count++] = {begin, 0};
      pos = begin + 1;
      continue;
    }

    // Grow the line one character at a time until it no longer fits.
    size_t i = begin;
    size_t lastSpace = begin;  // == begin: no break opportunity yet
    bool overflow = false;
    while (i < n && text[i] != '\n') {
      const size_t next = i + utf8SeqLen(text + i, n - i);
      if (measure(text + begin, next - begin, ctx) > maxWidth) {
        overflow = true;
        break;
      }
      if (text[i] == ' ') lastSpace = i;
      i = next;
    }

    size_t end;
    if (!overflow) {
      end = i;
      pos = (i < n) ? i + 1 : i;  // consume the '\n'
    } else if (lastSpace > begin) {
      end = lastSpace;  // break at the last space that fitted
      pos = lastSpace + 1;
    } else if (i > begin) {
      end = i;  // a single word wider than the line: split it
      pos = i;
    } else {
      // Not even one character fits; take it anyway so the loop advances.
      end = begin + utf8SeqLen(text + begin, n - begin);
      pos = end;
    }

    while (end > begin && isBlank(text[end - 1])) --end;
    lines[count++] = {begin, end - begin};
    // A wrapped line's continuation does not start with blanks.
    if (overflow) {
      while (pos < n && isBlank(text[pos])) ++pos;
    }
  }

  if (truncated) {
    size_t rest = pos;
    while (rest < n && (isBlank(text[rest]) || text[rest] == '\n')) ++rest;
    *truncated = rest < n;
  }
  return count;
}

size_t fitWithSuffix(const char *s, size_t len, const char *suffix, int maxWidth,
                     MeasureFn measure, void *ctx) {
  char buf[256];
  const size_t suffixLen = strlen(suffix);
  size_t best = 0;
  size_t i = 0;
  while (i < len) {
    const size_t next = i + utf8SeqLen(s + i, len - i);
    if (next + suffixLen >= sizeof(buf)) break;
    memcpy(buf, s, next);
    memcpy(buf + next, suffix, suffixLen);
    if (measure(buf, next + suffixLen, ctx) > maxWidth) break;
    best = next;
    i = next;
  }
  while (best > 0 && isBlank(s[best - 1])) --best;
  return best;
}

}  // namespace watch
