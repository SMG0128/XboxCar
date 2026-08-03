#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

/*
 * Minimal assertion harness for the host test build.
 *
 * Deliberately tiny: no allocation, no registration macros, no dependency
 * beyond stdio. Each test file exposes one run_*_tests() entry point and
 * test_main.c calls them in turn.
 */

#include <stdio.h>

extern int g_test_checks;
extern int g_test_failures;
extern int g_test_cases;
extern const char *g_test_current;

void Test_Begin(const char *name);
int Test_Summary(void);

#define TEST_CASE(name)  \
  do {                   \
    Test_Begin(name);    \
  } while (0)

#define TEST_CHECK(condition)                                            \
  do {                                                                   \
    ++g_test_checks;                                                     \
    if (!(condition)) {                                                  \
      ++g_test_failures;                                                 \
      printf("  FAIL [%s] %s:%d: %s\n", g_test_current, __FILE__,        \
             __LINE__, #condition);                                      \
    }                                                                    \
  } while (0)

#define TEST_EQ(actual, expected)                                             \
  do {                                                                        \
    long long actual_value_ = (long long)(actual);                            \
    long long expected_value_ = (long long)(expected);                        \
    ++g_test_checks;                                                          \
    if (actual_value_ != expected_value_) {                                   \
      ++g_test_failures;                                                      \
      printf("  FAIL [%s] %s:%d: %s == %s (got %lld, want %lld)\n",           \
             g_test_current, __FILE__, __LINE__, #actual, #expected,          \
             actual_value_, expected_value_);                                 \
    }                                                                         \
  } while (0)

/* Inclusive range check, useful for ramp positions that need not be exact. */
#define TEST_IN_RANGE(actual, low, high)                                      \
  do {                                                                        \
    long long actual_value_ = (long long)(actual);                            \
    long long low_value_ = (long long)(low);                                  \
    long long high_value_ = (long long)(high);                                \
    ++g_test_checks;                                                          \
    if (actual_value_ < low_value_ || actual_value_ > high_value_) {          \
      ++g_test_failures;                                                      \
      printf("  FAIL [%s] %s:%d: %s in [%lld,%lld] (got %lld)\n",             \
             g_test_current, __FILE__, __LINE__, #actual, low_value_,         \
             high_value_, actual_value_);                                     \
    }                                                                         \
  } while (0)

int run_protocol_tests(void);
int run_motor_tests(void);
int run_control_tests(void);
int run_safety_tests(void);
int run_ultrasonic_tests(void);
int run_system_tests(void);

#endif /* TEST_FRAMEWORK_H */
