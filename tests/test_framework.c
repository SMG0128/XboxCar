#include "test_framework.h"

int g_test_checks = 0;
int g_test_failures = 0;
int g_test_cases = 0;
const char *g_test_current = "(none)";

void Test_Begin(const char *name)
{
  g_test_current = name;
  ++g_test_cases;
}

int Test_Summary(void)
{
  printf("\n");
  printf("test cases : %d\n", g_test_cases);
  printf("checks     : %d\n", g_test_checks);
  printf("failures   : %d\n", g_test_failures);
  printf("%s\n", (g_test_failures == 0) ? "RESULT: PASS" : "RESULT: FAIL");
  return (g_test_failures == 0) ? 0 : 1;
}
