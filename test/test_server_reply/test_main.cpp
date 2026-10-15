#include <string.h>
#include <unity.h>

#include "power_policy.h"
#include "server_reply.h"

using namespace watch;

void setUp() {}
void tearDown() {}

// --- parseReply --------------------------------------------------------------

static void expectReply(const char *text, ReplyKind kind, const char *body) {
  Reply r = parseReply(text);
  TEST_ASSERT_EQUAL_INT_MESSAGE((int)kind, (int)r.kind, text);
  TEST_ASSERT_EQUAL_STRING_MESSAGE(body, r.body, text);
}

void test_reply_prefixes_from_gateway_contract() {
  expectReply("OK: nginx restarted", ReplyKind::Ok, "nginx restarted");
  expectReply("REJECTED: needs confirm=true", ReplyKind::Rejected, "needs confirm=true");
  expectReply("ERROR: speech not recognized", ReplyKind::Error, "speech not recognized");
}

void test_reply_prefix_case_and_spacing() {
  expectReply("error - Gateway 500", ReplyKind::Error, "Gateway 500");
  expectReply("  ok:done", ReplyKind::Ok, "done");
  expectReply("OK", ReplyKind::Ok, "");
  expectReply("Rejected:", ReplyKind::Rejected, "");
}

void test_reply_prefix_must_be_whole_word() {
  expectReply("OKAY then", ReplyKind::Info, "OKAY then");
  expectReply("ERRORS happen", ReplyKind::Info, "ERRORS happen");
}

void test_reply_app_progress_line_is_not_final() {
  Reply r = parseReply("Processing...");
  TEST_ASSERT_EQUAL_INT((int)ReplyKind::Status, (int)r.kind);
  TEST_ASSERT_FALSE(isFinalReply(r.kind));
  expectReply("STATUS: transcribing", ReplyKind::Status, "transcribing");
}

void test_reply_without_prefix_is_info() {
  expectReply("Сейчас 21:05", ReplyKind::Info, "Сейчас 21:05");
  expectReply("", ReplyKind::Info, "");
  TEST_ASSERT_TRUE(isFinalReply(ReplyKind::Info));
}

void test_reply_failure_kinds() {
  TEST_ASSERT_TRUE(isFailureReply(ReplyKind::Error));
  TEST_ASSERT_TRUE(isFailureReply(ReplyKind::Rejected));
  TEST_ASSERT_FALSE(isFailureReply(ReplyKind::Ok));
  TEST_ASSERT_FALSE(isFailureReply(ReplyKind::Info));
}

void test_reply_null() { TEST_ASSERT_EQUAL_INT((int)ReplyKind::Info, (int)parseReply(nullptr).kind); }

// --- copyUtf8 ----------------------------------------------------------------

void test_copy_ascii_fits() {
  char dst[16];
  TEST_ASSERT_EQUAL_UINT(5, copyUtf8(dst, sizeof(dst), "hello", 5));
  TEST_ASSERT_EQUAL_STRING("hello", dst);
}

void test_copy_never_splits_a_character() {
  // "Привет" is 12 bytes; 8 bytes of room (7 + NUL) hold 3 letters, not 3.5.
  const char *src = "Привет";
  char dst[8];
  TEST_ASSERT_EQUAL_UINT(6, copyUtf8(dst, sizeof(dst), src, strlen(src)));
  TEST_ASSERT_EQUAL_STRING("При", dst);
}

void test_copy_replaces_malformed_bytes() {
  const char src[] = {'a', (char)0xFF, 'b', (char)0xD0, 'c', 0};  // 0xD0 needs a continuation
  char dst[16];
  copyUtf8(dst, sizeof(dst), src, strlen(src));
  TEST_ASSERT_EQUAL_STRING("a?b?c", dst);
}

void test_copy_drops_sequence_cut_off_at_source_end() {
  // The app truncates at 500 bytes, possibly inside a letter.
  const char src[] = {'x', (char)0xD0};
  char dst[16];
  copyUtf8(dst, sizeof(dst), src, sizeof(src));
  TEST_ASSERT_EQUAL_STRING("x?", dst);
}

void test_copy_stops_at_nul_and_handles_tiny_buffers() {
  char dst[4];
  TEST_ASSERT_EQUAL_UINT(2, copyUtf8(dst, sizeof(dst), "ab\0cd", 5));
  TEST_ASSERT_EQUAL_STRING("ab", dst);
  char one[1];
  TEST_ASSERT_EQUAL_UINT(0, copyUtf8(one, 1, "abc", 3));
  TEST_ASSERT_EQUAL_STRING("", one);
  TEST_ASSERT_EQUAL_UINT(0, copyUtf8(nullptr, 0, "abc", 3));
}

void test_copy_four_byte_sequences() {
  const char *src = "a\xF0\x9F\x98\x80" "b";  // U+1F600
  char dst[16];
  TEST_ASSERT_EQUAL_UINT(6, copyUtf8(dst, sizeof(dst), src, strlen(src)));
  char small[5];  // room for 'a' + 3 bytes: the emoji does not fit
  copyUtf8(small, sizeof(small), src, strlen(src));
  TEST_ASSERT_EQUAL_STRING("a", small);
}

// --- power policy ------------------------------------------------------------

void test_send_never_times_out_at_or_above_30_percent() {
  TEST_ASSERT_FALSE(sendMayTimeOut({100, false}));
  TEST_ASSERT_FALSE(sendMayTimeOut({30, false}));
  TEST_ASSERT_TRUE(sendMayTimeOut({29, false}));
  TEST_ASSERT_TRUE(sendMayTimeOut({0, false}));
}

void test_send_never_times_out_while_charging_or_unknown() {
  TEST_ASSERT_FALSE(sendMayTimeOut({5, true}));
  TEST_ASSERT_FALSE(sendMayTimeOut({-1, false}));  // no battery: USB powered
}

void test_background_sync_needs_some_charge() {
  TEST_ASSERT_TRUE(backgroundSyncAllowed({15, false}));
  TEST_ASSERT_FALSE(backgroundSyncAllowed({14, false}));
  TEST_ASSERT_TRUE(backgroundSyncAllowed({3, true}));
  TEST_ASSERT_TRUE(backgroundSyncAllowed({-1, false}));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_reply_prefixes_from_gateway_contract);
  RUN_TEST(test_reply_prefix_case_and_spacing);
  RUN_TEST(test_reply_prefix_must_be_whole_word);
  RUN_TEST(test_reply_app_progress_line_is_not_final);
  RUN_TEST(test_reply_without_prefix_is_info);
  RUN_TEST(test_reply_failure_kinds);
  RUN_TEST(test_reply_null);
  RUN_TEST(test_copy_ascii_fits);
  RUN_TEST(test_copy_never_splits_a_character);
  RUN_TEST(test_copy_replaces_malformed_bytes);
  RUN_TEST(test_copy_drops_sequence_cut_off_at_source_end);
  RUN_TEST(test_copy_stops_at_nul_and_handles_tiny_buffers);
  RUN_TEST(test_copy_four_byte_sequences);
  RUN_TEST(test_send_never_times_out_at_or_above_30_percent);
  RUN_TEST(test_send_never_times_out_while_charging_or_unknown);
  RUN_TEST(test_background_sync_needs_some_charge);
  return UNITY_END();
}
