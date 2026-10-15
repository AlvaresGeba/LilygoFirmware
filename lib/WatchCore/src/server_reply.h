// The text the phone writes to RESPONSE_CHAR. The gateway's contract
// (AlvaresGeba command_router.py) is one short line starting with
//   OK: <summary> | REJECTED: <reason> | ERROR: <what went wrong>
// The app also writes "Processing..." as soon as it has the audio, and its own
// "ERROR: ..." lines when the gateway cannot be reached. Anything without a
// known prefix is shown as a plain answer.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace watch {

enum class ReplyKind : uint8_t {
  Ok,
  Rejected,
  Error,
  Status,  // progress ("Processing...", "STATUS: ..."): not the answer yet
  Info,    // no known prefix
};

struct Reply {
  ReplyKind kind;
  const char *body;  // the text after the prefix, leading ':' and spaces skipped
};

// `text` must be NUL-terminated; `body` points into it.
Reply parseReply(const char *text);

inline bool isFinalReply(ReplyKind kind) { return kind != ReplyKind::Status; }
inline bool isFailureReply(ReplyKind kind) {
  return kind == ReplyKind::Error || kind == ReplyKind::Rejected;
}

// Copies src[0..srcLen) into dst as NUL-terminated UTF-8 of at most
// dstSize - 1 bytes. A multi-byte character that does not fit is dropped
// whole instead of being cut in half; malformed bytes become '?'. Stops at a
// NUL in src. Returns the number of bytes written (without the NUL).
size_t copyUtf8(char *dst, size_t dstSize, const char *src, size_t srcLen);

// Length in bytes of the UTF-8 sequence starting at s[0] (1 for malformed
// input, so callers always make progress). `avail` bytes are readable.
size_t utf8SeqLen(const char *s, size_t avail);

}  // namespace watch
