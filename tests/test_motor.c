/*
 * Motor mapping, direction and ramp tests.
 *
 * The ramp is exercised through the real MotorController, so a change to the
 * step sizes in app_config.h moves these expectations rather than breaking
 * them: the tests assert on the configured constants, not on literals.
 */

#include <string.h>

#include "motor.h"
#include "test_framework.h"

static void RunPeriods(MotorController *motor, int periods)
{
  int index;
  for (index = 0; index < periods; ++index)
  {
    Motor_Update(motor);
  }
}

static void TestSideMappingFansOut(void)
{
  MotorController motor;

  TEST_CASE("left and right targets fan out to two motors each");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 400, -600);

  TEST_EQ(Motor_GetTarget(&motor, MOTOR_FRONT_LEFT), 400);
  TEST_EQ(Motor_GetTarget(&motor, MOTOR_REAR_LEFT), 400);
  TEST_EQ(Motor_GetTarget(&motor, MOTOR_FRONT_RIGHT), -600);
  TEST_EQ(Motor_GetTarget(&motor, MOTOR_REAR_RIGHT), -600);
}

static void TestOutputDisabledUntilEnabled(void)
{
  MotorController motor;
  MotorOutput output;

  TEST_CASE("a freshly initialised controller drives nothing");

  Motor_Init(&motor);
  TEST_CHECK(!Motor_IsOutputEnabled(&motor));

  Motor_SetTargets(&motor, 1000, 1000);
  RunPeriods(&motor, 100);

  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 0);
  TEST_CHECK(Motor_GetOutput(&motor, MOTOR_FRONT_LEFT, &output));
  TEST_EQ(output.duty, 0);
  TEST_CHECK(!output.in1);
  TEST_CHECK(!output.in2);
}

static void TestZeroSpeedGivesDefinedStop(void)
{
  MotorController motor;
  MotorOutput output;
  uint8_t index;

  TEST_CASE("zero speed puts both direction pins in a defined low state");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 0, 0);
  Motor_Update(&motor);

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    TEST_CHECK(Motor_GetOutput(&motor, index, &output));
    TEST_EQ(output.duty, 0);
    TEST_CHECK(!output.in1);
    TEST_CHECK(!output.in2);
  }
}

static void TestDirectionPinsNeverBothHigh(void)
{
  MotorController motor;
  MotorOutput output;
  int16_t speed;
  uint8_t index;

  TEST_CASE("no speed produces an illegal direction combination");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);

  for (speed = -MOTOR_SPEED_MAX; speed <= MOTOR_SPEED_MAX; speed = (int16_t)(speed + 50))
  {
    Motor_SetTargets(&motor, speed, speed);
    /* Reach the target regardless of ramp length. */
    RunPeriods(&motor, 200);

    for (index = 0U; index < MOTOR_COUNT; ++index)
    {
      TEST_CHECK(Motor_GetOutput(&motor, index, &output));
      TEST_CHECK(!(output.in1 && output.in2));
    }
  }
}

static void TestFullScaleDuty(void)
{
  TEST_CASE("duty conversion covers zero, mid and full scale");

  TEST_EQ(Motor_SpeedToDuty(0), 0);
  TEST_EQ(Motor_SpeedToDuty(MOTOR_SPEED_MAX), SOFT_PWM_RESOLUTION);
  TEST_EQ(Motor_SpeedToDuty(-MOTOR_SPEED_MAX), SOFT_PWM_RESOLUTION);
  TEST_EQ(Motor_SpeedToDuty(MOTOR_SPEED_MAX / 2), SOFT_PWM_RESOLUTION / 2);
  /* Magnitude only: direction lives in the pins, not the duty. */
  TEST_EQ(Motor_SpeedToDuty(-400), Motor_SpeedToDuty(400));
}

static void TestAccelerationFromZero(void)
{
  MotorController motor;

  TEST_CASE("acceleration advances by one configured step per period");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 1000, 1000);

  Motor_Update(&motor);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), MOTOR_ACCEL_STEP);
  Motor_Update(&motor);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 2 * MOTOR_ACCEL_STEP);

  /* And it converges exactly on the target rather than overshooting. */
  RunPeriods(&motor, 200);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 1000);
}

static void TestDecelerationToZero(void)
{
  MotorController motor;

  TEST_CASE("deceleration uses the deceleration step");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 1000, 1000);
  RunPeriods(&motor, 200);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 1000);

  Motor_SetTargets(&motor, 0, 0);
  Motor_Update(&motor);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 1000 - MOTOR_DECEL_STEP);

  RunPeriods(&motor, 200);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 0);
}

static void TestReverseGoesThroughZero(void)
{
  MotorController motor;
  int period;
  bool crossed_directly = false;
  int16_t previous;

  TEST_CASE("a direction change always passes through zero");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 600, 600);
  RunPeriods(&motor, 200);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 600);

  /* Ask for the opposite direction and watch every intermediate value. */
  Motor_SetTargets(&motor, -600, -600);
  previous = Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT);
  for (period = 0; period < 200; ++period)
  {
    int16_t current;
    Motor_Update(&motor);
    current = Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT);
    if ((previous > 0 && current < 0) || (previous < 0 && current > 0))
    {
      crossed_directly = true;
    }
    previous = current;
  }

  TEST_CHECK(!crossed_directly);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), -600);
}

static void TestReverseZeroHoldObserved(void)
{
  MotorController motor;
  int period;
  int zero_periods = 0;

  TEST_CASE("the reverse hold keeps the motor at zero before reversing");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  /* Creep forward at a speed low enough that one decel step reaches zero. */
  Motor_SetTargets(&motor, 30, 30);
  RunPeriods(&motor, 10);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 30);

  Motor_SetTargets(&motor, -500, -500);
  for (period = 0; period < 20; ++period)
  {
    int16_t current;
    Motor_Update(&motor);
    current = Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT);
    if (current == 0)
    {
      ++zero_periods;
    }
    else if (current < 0)
    {
      break;
    }
  }

  /* The period that reaches zero, plus the configured hold. */
  TEST_CHECK(zero_periods >= (int)MOTOR_REVERSE_ZERO_HOLD);
}

static void TestForceStopBypassesRamp(void)
{
  MotorController motor;

  TEST_CASE("a forced stop bypasses the deceleration ramp");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 1000, 1000);
  RunPeriods(&motor, 200);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 1000);

  Motor_ForceStop(&motor);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 0);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_REAR_RIGHT), 0);
  /* And it does not creep back up on the next period. */
  Motor_Update(&motor);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 0);
}

static void TestDisableForcesImmediateStop(void)
{
  MotorController motor;

  TEST_CASE("disabling the output stops instantly");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, -900, -900);
  RunPeriods(&motor, 200);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), -900);

  Motor_SetOutputEnabled(&motor, false);
  TEST_EQ(Motor_GetSpeed(&motor, MOTOR_FRONT_LEFT), 0);
}

static void TestTargetsAreClamped(void)
{
  MotorController motor;

  TEST_CASE("out of range targets are clamped rather than wrapped");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 32000, -32000);

  TEST_EQ(Motor_GetTarget(&motor, MOTOR_FRONT_LEFT), MOTOR_SPEED_MAX);
  TEST_EQ(Motor_GetTarget(&motor, MOTOR_FRONT_RIGHT), -MOTOR_SPEED_MAX);
  TEST_CHECK(Motor_Validate(&motor));
}

static void TestInvalidIndexIsSafe(void)
{
  MotorController motor;
  MotorOutput output;

  TEST_CASE("an out of range motor index yields the safe stopped state");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 800, 800);
  RunPeriods(&motor, 200);

  TEST_CHECK(!Motor_GetOutput(&motor, MOTOR_COUNT, &output));
  TEST_EQ(output.duty, 0);
  TEST_CHECK(!output.in1);
  TEST_CHECK(!output.in2);
  TEST_EQ(Motor_GetSpeed(&motor, 99), 0);
}

static void TestValidateDetectsCorruption(void)
{
  MotorController motor;

  TEST_CASE("validation rejects a corrupted speed");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  TEST_CHECK(Motor_Validate(&motor));

  /* Simulate memory corruption the control system must notice. */
  motor.speed[MOTOR_REAR_LEFT] = (int16_t)(MOTOR_SPEED_MAX + 1);
  TEST_CHECK(!Motor_Validate(&motor));
}

static void TestPerMotorInversion(void)
{
  MotorController motor;
  MotorOutput output;
  bool expected_forward;

  TEST_CASE("per-motor inversion is applied to the direction pins");

  Motor_Init(&motor);
  Motor_SetOutputEnabled(&motor, true);
  Motor_SetTargets(&motor, 500, 500);
  RunPeriods(&motor, 200);

  /*
   * The inversion flags are build-time configuration. The test asserts the
   * relationship rather than a fixed polarity, so it stays correct after a
   * bring-up engineer calibrates one on real hardware.
   */
  expected_forward = (MOTOR_FRONT_LEFT_INVERTED == 0);
  TEST_CHECK(Motor_GetOutput(&motor, MOTOR_FRONT_LEFT, &output));
  TEST_EQ(output.in1, expected_forward);
  TEST_EQ(output.in2, !expected_forward);

  expected_forward = (MOTOR_REAR_RIGHT_INVERTED == 0);
  TEST_CHECK(Motor_GetOutput(&motor, MOTOR_REAR_RIGHT, &output));
  TEST_EQ(output.in1, expected_forward);
  TEST_EQ(output.in2, !expected_forward);
}

int run_motor_tests(void)
{
  printf("-- motor --\n");
  TestSideMappingFansOut();
  TestOutputDisabledUntilEnabled();
  TestZeroSpeedGivesDefinedStop();
  TestDirectionPinsNeverBothHigh();
  TestFullScaleDuty();
  TestAccelerationFromZero();
  TestDecelerationToZero();
  TestReverseGoesThroughZero();
  TestReverseZeroHoldObserved();
  TestForceStopBypassesRamp();
  TestDisableForcesImmediateStop();
  TestTargetsAreClamped();
  TestInvalidIndexIsSafe();
  TestValidateDetectsCorruption();
  TestPerMotorInversion();
  return 0;
}
