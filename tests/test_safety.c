/*
 * Obstacle limiting tests.
 *
 * Every case drives the real Safety_Apply with a table of distances, so the
 * decomposition into forward and turn components, the hysteresis latches and
 * the fail policy are all the shipping implementation.
 */

#include <string.h>

#include "safety_controller.h"
#include "test_framework.h"

static void MakeInput(SafetyInput *input, int16_t left, int16_t right,
                      uint16_t front, uint16_t rear, uint16_t side_left,
                      uint16_t side_right)
{
  memset(input, 0, sizeof(*input));
  input->requested_left = left;
  input->requested_right = right;
  input->distance_mm[ULTRASONIC_FRONT] = front;
  input->distance_mm[ULTRASONIC_REAR] = rear;
  input->distance_mm[ULTRASONIC_LEFT] = side_left;
  input->distance_mm[ULTRASONIC_RIGHT] = side_right;
  input->sensor_valid_mask = 0x0FU;
  input->sensor_fault_mask = 0x00U;
}

/* A distance comfortably outside every threshold. */
#define CLEAR_MM 3000U

static void TestClearPathIsUnmodified(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("a clear path passes the request through unchanged");

  Safety_Init(&safety);
  MakeInput(&input, 800, 800, CLEAR_MM, CLEAR_MM, CLEAR_MM, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  TEST_EQ(output.limited_left, 800);
  TEST_EQ(output.limited_right, 800);
  TEST_EQ(output.limit_reason, LIMIT_REASON_NONE);
}

static void TestFrontLinearSlowdown(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;
  uint16_t midpoint;

  TEST_CASE("front obstacle scales forward speed linearly");

  Safety_Init(&safety);
  midpoint = (uint16_t)((SAFETY_STOP_DISTANCE_MM + SAFETY_SLOW_DISTANCE_MM) / 2U);
  MakeInput(&input, 1000, 1000, midpoint, CLEAR_MM, CLEAR_MM, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  /* Halfway through the band means roughly half the speed ceiling. */
  TEST_IN_RANGE(output.limited_left, MOTOR_SPEED_MAX / 2 - 30,
                MOTOR_SPEED_MAX / 2 + 30);
  TEST_EQ(output.limited_left, output.limited_right);
  TEST_CHECK((output.limit_reason & LIMIT_REASON_FRONT_SLOW) != 0U);
  TEST_CHECK((output.obstacle_flags & OBSTACLE_FLAG_FRONT) != 0U);
}

static void TestSlowBandUpperEdgeUnlimited(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("at the slow-down distance the request is not yet limited");

  Safety_Init(&safety);
  MakeInput(&input, 900, 900, SAFETY_SLOW_DISTANCE_MM, CLEAR_MM, CLEAR_MM,
            CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  TEST_EQ(output.limited_left, 900);
  TEST_EQ(output.limit_reason, LIMIT_REASON_NONE);
}

static void TestFrontStopBlocksForward(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("front obstacle inside the stop distance blocks forward motion");

  Safety_Init(&safety);
  MakeInput(&input, 900, 900, SAFETY_STOP_DISTANCE_MM - 50U, CLEAR_MM, CLEAR_MM,
            CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  TEST_EQ(output.limited_left, 0);
  TEST_EQ(output.limited_right, 0);
  TEST_CHECK((output.limit_reason & LIMIT_REASON_FRONT_STOP) != 0U);
  TEST_CHECK((output.obstacle_flags & OBSTACLE_FLAG_FRONT_BLOCKED) != 0U);
}

static void TestReverseAllowedWithFrontObstacle(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("a blocking front obstacle still allows reversing away");

  Safety_Init(&safety);
  MakeInput(&input, 900, 900, SAFETY_STOP_DISTANCE_MM - 50U, CLEAR_MM, CLEAR_MM,
            CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_EQ(output.limited_left, 0);

  /* Now ask to reverse: the escape route must stay open. */
  MakeInput(&input, -700, -700, SAFETY_STOP_DISTANCE_MM - 50U, CLEAR_MM,
            CLEAR_MM, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_EQ(output.limited_left, -700);
  TEST_EQ(output.limited_right, -700);
}

static void TestRearStopBlocksReverse(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("rear obstacle blocks reversing but allows driving forward");

  Safety_Init(&safety);
  MakeInput(&input, -800, -800, CLEAR_MM, SAFETY_STOP_DISTANCE_MM - 30U,
            CLEAR_MM, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_EQ(output.limited_left, 0);
  TEST_CHECK((output.limit_reason & LIMIT_REASON_REAR_STOP) != 0U);

  MakeInput(&input, 800, 800, CLEAR_MM, SAFETY_STOP_DISTANCE_MM - 30U, CLEAR_MM,
            CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_EQ(output.limited_left, 800);
}

static void TestHysteresisPreventsChatter(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;
  uint16_t between;

  TEST_CASE("a blocked direction stays blocked until the release distance");

  Safety_Init(&safety);

  /* Enter the block. */
  MakeInput(&input, 800, 800, SAFETY_STOP_DISTANCE_MM - 10U, CLEAR_MM, CLEAR_MM,
            CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_EQ(output.limited_left, 0);

  /*
   * Back off to just above the stop distance but below the release distance.
   * Without hysteresis the vehicle would start moving again here and then
   * immediately re-block, which is the stutter this exists to prevent.
   */
  between = (uint16_t)(SAFETY_STOP_DISTANCE_MM + 10U);
  TEST_CHECK(between < SAFETY_RELEASE_DISTANCE_MM);
  MakeInput(&input, 800, 800, between, CLEAR_MM, CLEAR_MM, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_EQ(output.limited_left, 0);

  /* Clear of the release distance: motion is permitted again. */
  MakeInput(&input, 800, 800, SAFETY_RELEASE_DISTANCE_MM + 10U, CLEAR_MM,
            CLEAR_MM, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_CHECK(output.limited_left > 0);
}

static void TestSideObstacleLimitsTurnToward(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;
  int32_t turn;

  TEST_CASE("a left obstacle caps turning further left");

  Safety_Init(&safety);
  /* left < right means a left turn. */
  MakeInput(&input, 200, 800, CLEAR_MM, CLEAR_MM,
            SAFETY_SIDE_LIMIT_DISTANCE_MM - 50U, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  turn = ((int32_t)output.limited_left - (int32_t)output.limited_right) / 2;
  TEST_CHECK(turn < 0); /* still turning left */
  TEST_CHECK(-turn <= SAFETY_SIDE_MIN_TURN);
  TEST_CHECK((output.limit_reason & LIMIT_REASON_SIDE_LEFT) != 0U);
}

static void TestSideObstacleAllowsTurnAway(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("a left obstacle does not restrict turning right");

  Safety_Init(&safety);
  /* left > right means a right turn, away from the left obstacle. */
  MakeInput(&input, 800, 200, CLEAR_MM, CLEAR_MM,
            SAFETY_SIDE_LIMIT_DISTANCE_MM - 50U, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  TEST_EQ(output.limited_left, 800);
  TEST_EQ(output.limited_right, 200);
}

static void TestSideObstacleNeverStopsVehicle(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("a very close side obstacle does not stop the vehicle");

  Safety_Init(&safety);
  /* Driving straight ahead with a wall alongside. */
  MakeInput(&input, 700, 700, CLEAR_MM, CLEAR_MM, 30U, 30U);
  Safety_Apply(&safety, &input, &output);

  TEST_EQ(output.limited_left, 700);
  TEST_EQ(output.limited_right, 700);
}

static void TestSpinIsLimitedBySide(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;
  int32_t turn;

  TEST_CASE("a stationary spin is limited by the corresponding side sensor");

  Safety_Init(&safety);
  /* Pure spin to the left: forward component is zero. */
  MakeInput(&input, -650, 650, CLEAR_MM, CLEAR_MM,
            SAFETY_SIDE_LIMIT_DISTANCE_MM - 100U, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  turn = ((int32_t)output.limited_left - (int32_t)output.limited_right) / 2;
  TEST_CHECK(-turn <= SAFETY_SIDE_MIN_TURN);
  TEST_CHECK((output.limit_reason & LIMIT_REASON_SIDE_LEFT) != 0U);
}

static void TestSideHysteresis(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;
  int32_t turn;

  TEST_CASE("side limiting has its own release distance");

  Safety_Init(&safety);
  MakeInput(&input, 200, 800, CLEAR_MM, CLEAR_MM,
            SAFETY_SIDE_LIMIT_DISTANCE_MM - 20U, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  turn = ((int32_t)output.limited_left - (int32_t)output.limited_right) / 2;
  TEST_CHECK(-turn <= SAFETY_SIDE_MIN_TURN);

  /* Between the limit and release distances the cap must persist. */
  MakeInput(&input, 200, 800, CLEAR_MM, CLEAR_MM,
            (uint16_t)(SAFETY_SIDE_LIMIT_DISTANCE_MM + 20U), CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  turn = ((int32_t)output.limited_left - (int32_t)output.limited_right) / 2;
  TEST_CHECK(-turn <= SAFETY_SIDE_MIN_TURN);

  /* Past the release distance the full turn returns. */
  MakeInput(&input, 200, 800, CLEAR_MM, CLEAR_MM,
            (uint16_t)(SAFETY_SIDE_RELEASE_DISTANCE_MM + 20U), CLEAR_MM);
  Safety_Apply(&safety, &input, &output);
  TEST_EQ(output.limited_left, 200);
  TEST_EQ(output.limited_right, 800);
}

static void TestFaultedSensorPolicy(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("a faulted front sensor follows the configured policy");

  Safety_Init(&safety);
  MakeInput(&input, 800, 800, 100U, CLEAR_MM, CLEAR_MM, CLEAR_MM);
  /* Mark the front sensor faulty; its distance must not be trusted. */
  input.sensor_fault_mask = (uint8_t)(1U << ULTRASONIC_FRONT);
  Safety_Apply(&safety, &input, &output);

  TEST_CHECK((output.limit_reason & LIMIT_REASON_SENSOR_FAULT) != 0U);
#if SAFETY_SENSOR_FAULT_POLICY == SAFETY_FAIL_SAFE
  TEST_EQ(output.limited_left, 0);
#else
  /* FAIL_OPEN keeps the vehicle drivable; the operator stays responsible. */
  TEST_EQ(output.limited_left, 800);
#endif
}

static void TestNoValidSensorsPassesThrough(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("with ultrasonic disabled the request is not limited");

  Safety_Init(&safety);
  memset(&input, 0, sizeof(input));
  input.requested_left = 750;
  input.requested_right = 750;
  /* This is the shipping default: the feature is off, nothing is valid. */
  input.sensor_valid_mask = 0U;
  input.sensor_fault_mask = 0U;
  Safety_Apply(&safety, &input, &output);

#if SAFETY_SENSOR_FAULT_POLICY == SAFETY_FAIL_SAFE
  TEST_EQ(output.limited_left, 0);
#else
  TEST_EQ(output.limited_left, 750);
  TEST_EQ(output.limited_right, 750);
#endif
}

static void TestRecompositionStaysInRange(void)
{
  SafetyController safety;
  SafetyInput input;
  SafetyOutput output;

  TEST_CASE("recomposed wheel speeds stay inside the speed limit");

  Safety_Init(&safety);
  MakeInput(&input, 1000, -1000, CLEAR_MM, CLEAR_MM, CLEAR_MM, CLEAR_MM);
  Safety_Apply(&safety, &input, &output);

  TEST_IN_RANGE(output.limited_left, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
  TEST_IN_RANGE(output.limited_right, -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX);
}

static void TestNullInputsAreSafe(void)
{
  SafetyController safety;
  SafetyOutput output;

  TEST_CASE("a null input yields a stopped output");

  Safety_Init(&safety);
  Safety_Apply(&safety, NULL, &output);
  TEST_EQ(output.limited_left, 0);
  TEST_EQ(output.limited_right, 0);
}

int run_safety_tests(void)
{
  printf("-- safety --\n");
  TestClearPathIsUnmodified();
  TestFrontLinearSlowdown();
  TestSlowBandUpperEdgeUnlimited();
  TestFrontStopBlocksForward();
  TestReverseAllowedWithFrontObstacle();
  TestRearStopBlocksReverse();
  TestHysteresisPreventsChatter();
  TestSideObstacleLimitsTurnToward();
  TestSideObstacleAllowsTurnAway();
  TestSideObstacleNeverStopsVehicle();
  TestSpinIsLimitedBySide();
  TestSideHysteresis();
  TestFaultedSensorPolicy();
  TestNoValidSensorsPassesThrough();
  TestRecompositionStaysInRange();
  TestNullInputsAreSafe();
  return 0;
}
