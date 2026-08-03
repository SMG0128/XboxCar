#include "test_framework.h"

/*
 * Host test entry point.
 *
 * Every suite runs even if an earlier one fails, so a single run reports the
 * full picture rather than stopping at the first problem.
 */
int main(void)
{
  printf("XboxCar STM32 host tests\n");
  printf("========================\n\n");

  (void)run_protocol_tests();
  (void)run_motor_tests();
  (void)run_ultrasonic_tests();
  (void)run_safety_tests();
  (void)run_control_tests();
  (void)run_system_tests();

  return Test_Summary();
}
