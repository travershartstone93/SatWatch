#pragma once
#include <cstdio>
#include <cstdlib>
static int _tests = 0, _fails = 0;
#define TEST(name) static void name()
#define EXPECT(cond) do { _tests++; if (!(cond)) { _fails++; fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while(0)
#define EXPECT_EQ(a, b) do { _tests++; auto _a=(a); auto _b=(b); if (_a != _b) { _fails++; fprintf(stderr, "FAIL %s:%d: %s == %s (%d vs %d)\n", __FILE__, __LINE__, #a, #b, (int)_a, (int)_b); } } while(0)
#define RUN(fn) do { fn(); } while(0)
#define REPORT() do { printf("%d tests, %d failures\n", _tests, _fails); return _fails ? 1 : 0; } while(0)
