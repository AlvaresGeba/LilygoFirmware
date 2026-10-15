#include "server_reply.h"

namespace watch {

namespace {

char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

bool isWordChar(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
}

// True if `text` starts with the keyword (case-insensitive) as a whole word:
// "ERROR: x" and "error x" match, "ERRORS" and "OKAY" do not.
bool startsWithWord(const char *text, const char *word, const char **after) {
  const char *p = text;
  for (; *word; ++word, ++p) {
    if (lower(*p) != *word) return false;
  }
  if (isWordChar(*p)) return false;
  *after = p;
  return true;
}

const char *skipSeparators(const char *p) {
  while (*p == ':' || *p == ' ' || *p == '\t' || *p == '-') ++p;
  return p;
}

bool isContinuation(unsigned char c) { return (c & 0xC0) == 0x80; }

}  // namespace

Reply parseReply(const char *text) {
  if (text == nullptr) return {ReplyKind::Info, ""};
  const char *p = text;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;

  struct Prefix {
    const char *word;
    ReplyKind kind;
  };
  static const Prefix kPrefixes[] = {
      {"ok", ReplyKind::Ok},
      {"rejected", ReplyKind::Rejected},
      {"error", ReplyKind::Error},
      {"status", ReplyKind::Status},
  };
  for (const Prefix &prefix : kPrefixes) {
    const char *after = nullptr;
    if (startsWithWord(p, prefix.word, &after)) return {prefix.kind, skipSeparators(after)};
  }

  // The app's progress line, sent before the gateway answers.
  const char *after = nullptr;
  if (startsWithWord(p, "processing", &after)) return {ReplyKind::Status, p};

  return {ReplyKind::Info, p};
}

size_t utf8SeqLen(const char *s, size_t avail) {
  if (avail == 0) return 0;
  const unsigned char lead = static_cast<unsigned char>(s[0]);
  size_t len;
  if (lead < 0x80) return 1;
  if (lead >= 0xC2 && lead <= 0xDF) len = 2;
  else if (lead >= 0xE0 && lead <= 0xEF) len = 3;
  else if (lead >= 0xF0 && lead <= 0xF4) len = 4;
  else return 1;  // stray continuation byte or invalid lead
  if (len > avail) return 1;
  for (size_t i = 1; i < len; i++) {
    if (!isContinuation(static_cast<unsigned char>(s[i]))) return 1;
  }
  return len;
}

size_t copyUtf8(char *dst, size_t dstSize, const char *src, size_t srcLen) {
  if (dst == nullptr || dstSize == 0) return 0;
  size_t out = 0;
  size_t in = 0;
  while (in < srcLen && src[in] != '\0') {
    const size_t seq = utf8SeqLen(src + in, srcLen - in);
    const bool malformed = seq == 1 && static_cast<unsigned char>(src[in]) >= 0x80;
    const size_t need = malformed ? 1 : seq;
    if (out + need > dstSize - 1) break;
    if (malformed) {
      dst[out++] = '?';
    } else {
      for (size_t i = 0; i < seq; i++) dst[out++] = src[in + i];
    }
    in += seq;
  }
  dst[out] = '\0';
  return out;
}

}  // namespace watch
