#include <string.h>
#include <string>
#include <unity.h>

#include "server_reply.h"
#include "text_wrap.h"

using namespace watch;

void setUp() {}
void tearDown() {}

// A monospace "font": every character, ASCII or Cyrillic, is 10 px wide.
static int measure10(const char *s, size_t len, void *) {
  int chars = 0;
  for (size_t i = 0; i < len; i += utf8SeqLen(s + i, len - i)) chars++;
  return chars * 10;
}

static std::string line(const char *text, const TextSpan &span) {
  return std::string(text + span.start, span.len);
}

void test_wrap_breaks_at_spaces() {
  const char *t = "the quick brown fox";
  TextSpan lines[8];
  bool truncated = true;
  size_t n = wrapText(t, 100, measure10, nullptr, lines, 8, &truncated);  // 10 chars/line
  TEST_ASSERT_EQUAL_UINT(2, n);
  TEST_ASSERT_EQUAL_STRING("the quick", line(t, lines[0]).c_str());
  TEST_ASSERT_EQUAL_STRING("brown fox", line(t, lines[1]).c_str());
  TEST_ASSERT_FALSE(truncated);
}

void test_wrap_counts_characters_not_bytes() {
  // 16 Cyrillic letters + 2 spaces = 34 bytes. The old firmware wrapped every
  // 18 *bytes*, i.e. after 9 letters and sometimes inside one.
  const char *t = "Сервер не отвечает";
  TextSpan lines[4];
  size_t n = wrapText(t, 100, measure10, nullptr, lines, 4, nullptr);
  TEST_ASSERT_EQUAL_UINT(2, n);
  TEST_ASSERT_EQUAL_STRING("Сервер не", line(t, lines[0]).c_str());
  TEST_ASSERT_EQUAL_STRING("отвечает", line(t, lines[1]).c_str());
}

void test_wrap_splits_overlong_word_on_char_boundary() {
  const char *t = "Превышениелимита";  // 16 letters, no spaces
  TextSpan lines[4];
  size_t n = wrapText(t, 50, measure10, nullptr, lines, 4, nullptr);  // 5 letters/line
  TEST_ASSERT_EQUAL_UINT(4, n);
  TEST_ASSERT_EQUAL_STRING("Превы", line(t, lines[0]).c_str());
  TEST_ASSERT_EQUAL_STRING("шение", line(t, lines[1]).c_str());
  TEST_ASSERT_EQUAL_STRING("лимит", line(t, lines[2]).c_str());
  TEST_ASSERT_EQUAL_STRING("а", line(t, lines[3]).c_str());
}

void test_wrap_honours_newlines_and_blank_lines() {
  const char *t = "one\n\ntwo";
  TextSpan lines[4];
  size_t n = wrapText(t, 100, measure10, nullptr, lines, 4, nullptr);
  TEST_ASSERT_EQUAL_UINT(3, n);
  TEST_ASSERT_EQUAL_STRING("one", line(t, lines[0]).c_str());
  TEST_ASSERT_EQUAL_STRING("", line(t, lines[1]).c_str());
  TEST_ASSERT_EQUAL_STRING("two", line(t, lines[2]).c_str());
}

void test_wrap_reports_truncation() {
  const char *t = "aaaa bbbb cccc dddd";
  TextSpan lines[2];
  bool truncated = false;
  size_t n = wrapText(t, 40, measure10, nullptr, lines, 2, &truncated);
  TEST_ASSERT_EQUAL_UINT(2, n);
  TEST_ASSERT_TRUE(truncated);
  TEST_ASSERT_EQUAL_STRING("bbbb", line(t, lines[1]).c_str());
}

void test_wrap_trailing_spaces_are_not_truncation() {
  const char *t = "abc   \n  ";
  TextSpan lines[1];
  bool truncated = true;
  size_t n = wrapText(t, 100, measure10, nullptr, lines, 1, &truncated);
  TEST_ASSERT_EQUAL_UINT(1, n);
  TEST_ASSERT_EQUAL_STRING("abc", line(t, lines[0]).c_str());
  TEST_ASSERT_FALSE(truncated);
}

void test_wrap_char_wider_than_line_still_progresses() {
  const char *t = "abc";
  TextSpan lines[8];
  size_t n = wrapText(t, 5, measure10, nullptr, lines, 8, nullptr);
  TEST_ASSERT_EQUAL_UINT(3, n);
}

void test_wrap_empty_and_null() {
  TextSpan lines[2];
  bool truncated = true;
  TEST_ASSERT_EQUAL_UINT(0, wrapText("", 100, measure10, nullptr, lines, 2, &truncated));
  TEST_ASSERT_FALSE(truncated);
  TEST_ASSERT_EQUAL_UINT(0, wrapText(nullptr, 100, measure10, nullptr, lines, 2, nullptr));
}

void test_fit_with_suffix() {
  const char *s = "Привет мир";
  // 60 px = 6 chars; with a 1-char suffix 5 letters remain: "Приве".
  size_t keep = fitWithSuffix(s, strlen(s), "…", 60, measure10, nullptr);
  TEST_ASSERT_EQUAL_STRING("Приве", std::string(s, keep).c_str());
  // Trailing blanks before the suffix are dropped: "Привет …" -> "Привет…".
  keep = fitWithSuffix(s, strlen(s), "…", 80, measure10, nullptr);
  TEST_ASSERT_EQUAL_STRING("Привет", std::string(s, keep).c_str());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_wrap_breaks_at_spaces);
  RUN_TEST(test_wrap_counts_characters_not_bytes);
  RUN_TEST(test_wrap_splits_overlong_word_on_char_boundary);
  RUN_TEST(test_wrap_honours_newlines_and_blank_lines);
  RUN_TEST(test_wrap_reports_truncation);
  RUN_TEST(test_wrap_trailing_spaces_are_not_truncation);
  RUN_TEST(test_wrap_char_wider_than_line_still_progresses);
  RUN_TEST(test_wrap_empty_and_null);
  RUN_TEST(test_fit_with_suffix);
  return UNITY_END();
}
