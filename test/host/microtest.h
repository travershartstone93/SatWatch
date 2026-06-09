#pragma once
#include <cstdio>
#include <cstdlib>
static int _tests = 0, _fails = 0;
#define TEST(name) static void test_##name(); \
  static struct _reg_##name { _reg_##name() { test_##name(); } } _r_##name; \
  static void test_##name()
#define EXPECT(cond) do { _tests++; if (!(cond)) { \
  fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); _fails++; } } while(0)
#define EXPECT_EQ(a, b) EXPECT((a) == (b))
#define TEST_MAIN() int main() { \
  fprintf(stderr, "%d tests, %d failures\n", _tests, _fails); \
  return _fails ? 1 : 0; }
