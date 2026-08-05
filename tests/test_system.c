/*
 * End-to-end tests over the whole chain:
 *
 *   UART bytes -> parse -> communication state -> emergency -> obstacle limit
 *   -> ramp -> motor outputs
 *
 * These exist to catch problems that only appear when the stages are composed,
 * in particular the priority ordering between safety sources.
 */

#include <string.h>

#include "control_system.h"
#include "test_framework.h"
#include "test_helpers.h"

/* Feeds frames at the emitter's 50 Hz while the control loop runs at 100 Hz. */
static uint32_t DriveFor(ControlSystem *system, uint32_t now_ms,
                         uint32_t duration_ms, uint8_t command, int16_t left,
                         int16_t right, uint16_t *sequence,
                         const SensorSnapshot *sensors)
{
  uint32_t elapsed = 0U;
  uint32_t period_index = 0U;

  while (elapsed < duration_ms)
  {
    if ((period_index % 2U) == 0U)
    {
      TestHelper_PushFrame(system, command, left, right, (*sequence)++);
    }
    now_ms += CONTROL_PERIOD_MS;
    elapsed += CONTROL_PERIOD_MS;
    ++period_index;
    ControlSystem_Update(system, now_ms, sensors);
  }
  return now_ms;
}

static void TestFullChainForward(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  const AppDebugState *debug;

  TEST_CASE("bytes in, four motor outputs out");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);

  now = DriveFor(&system, now, 1000U, 0x1, 700, 700, &sequence, &sensors);
  debug = ControlSystem_GetDebugState(&system);

  TEST_EQ(debug->control_state, CONTROL_STATE_ONLINE);
  TEST_EQ(debug->requested_left, 700);
  TEST_EQ(debug->limited_left, 700);
  TEST_EQ(debug->actual_left, 700);

  /* All four motors carry the same magnitude for a straight line. */
  TEST_EQ(debug->motor_speed[MOTOR_FRONT_LEFT], 700);
  TEST_EQ(debug->motor_speed[MOTOR_REAR_LEFT], 700);
  TEST_EQ(debug->motor_speed[MOTOR_FRONT_RIGHT], 700);
  TEST_EQ(debug->motor_speed[MOTOR_REAR_RIGHT], 700);
  TEST_EQ(debug->motor_pwm[MOTOR_FRONT_LEFT], Motor_SpeedToDuty(700));
  TEST_CHECK(debug->uart_rx_bytes > 0U);
  TEST_CHECK(debug->valid_frames > 0U);
}

static void TestObstacleSlowsThenBlocks(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  int16_t cruising;

  TEST_CASE("approaching an obstacle slows then blocks forward motion");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);

  now = DriveFor(&system, now, 1000U, 0x1, 900, 900, &sequence, &sensors);
  cruising = ControlSystem_GetDebugState(&system)->actual_left;
  TEST_EQ(cruising, 900);

  /* Obstacle enters the slow band. */
  sensors.distance_mm[ULTRASONIC_FRONT] =
      (uint16_t)((SAFETY_STOP_DISTANCE_MM + SAFETY_SLOW_DISTANCE_MM) / 2U);
  now = DriveFor(&system, now, 1000U, 0x1, 900, 900, &sequence, &sensors);
  TEST_CHECK(ControlSystem_GetDebugState(&system)->actual_left < cruising);
  TEST_CHECK(ControlSystem_GetDebugState(&system)->actual_left > 0);

  /* Obstacle enters the stop band. */
  sensors.distance_mm[ULTRASONIC_FRONT] = SAFETY_STOP_DISTANCE_MM - 50U;
  now = DriveFor(&system, now, 1000U, 0x1, 900, 900, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
  TEST_CHECK((ControlSystem_GetDebugState(&system)->obstacle_flags &
              OBSTACLE_FLAG_FRONT_BLOCKED) != 0U);
}

static void TestReverseEscapeFromObstacle(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("reversing away from a blocking obstacle is permitted");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);
  sensors.distance_mm[ULTRASONIC_FRONT] = SAFETY_STOP_DISTANCE_MM - 50U;

  now = DriveFor(&system, now, 500U, 0x1, 800, 800, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);

  now = DriveFor(&system, now, 1000U, 0x2, -600, -600, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, -600);
}

static void TestHysteresisAcrossFullChain(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("the obstacle must clear the release distance to resume");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);

  now = DriveFor(&system, now, 500U, 0x1, 800, 800, &sequence, &sensors);
  sensors.distance_mm[ULTRASONIC_FRONT] = SAFETY_STOP_DISTANCE_MM - 20U;
  now = DriveFor(&system, now, 500U, 0x1, 800, 800, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);

  /* Just outside the stop band but inside the release band: still blocked. */
  sensors.distance_mm[ULTRASONIC_FRONT] = (uint16_t)(SAFETY_STOP_DISTANCE_MM + 20U);
  now = DriveFor(&system, now, 500U, 0x1, 800, 800, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);

  /* Clear of the release distance: motion resumes. */
  sensors.distance_mm[ULTRASONIC_FRONT] = 2000U;
  now = DriveFor(&system, now, 1000U, 0x1, 800, 800, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 800);
}

static void TestEmergencyOutranksObstacleClearance(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("a clear path cannot release the emergency latch");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);

  now = DriveFor(&system, now, 500U, 0x1, 800, 800, &sequence, &sensors);
  now = DriveFor(&system, now, 100U, 0x7, 0, 0, &sequence, &sensors);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));

  /* Wide open road ahead makes no difference whatsoever. */
  TestHelper_UniformSensors(&sensors, 4000U);
  now = DriveFor(&system, now, 40U, 0x7, 0, 0, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_EMERGENCY_LOCKED);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
}

static void TestTimeoutOutranksObstacleClearance(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("a clear path cannot override a communication timeout");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 4000U);

  now = DriveFor(&system, now, 1000U, 0x1, 900, 900, &sequence, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 900);

  now = TestHelper_RunFor(&system, now, COMM_TIMEOUT_MS * 2U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_COMM_TIMEOUT);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
}

static void TestSensorFaultCannotOverrideEmergency(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("a faulted sensor cannot release a latched vehicle");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);

  now = DriveFor(&system, now, 500U, 0x1, 700, 700, &sequence, &sensors);
  now = DriveFor(&system, now, 100U, 0x7, 0, 0, &sequence, &sensors);

  /* Every sensor faults, which under FAIL_OPEN removes all limiting. */
  sensors.valid_mask = 0U;
  sensors.fault_mask = 0x0FU;
  now = DriveFor(&system, now, 100U, 0x7, 0, 0, &sequence, &sensors);

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_EMERGENCY_LOCKED);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
}

static void TestSnapshotStagesAreDistinct(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  const AppDebugState *debug;

  TEST_CASE("the snapshot separates requested, limited and actual speeds");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);
  sensors.distance_mm[ULTRASONIC_FRONT] =
      (uint16_t)((SAFETY_STOP_DISTANCE_MM + SAFETY_SLOW_DISTANCE_MM) / 2U);

  /* One period only, so the ramp has not yet caught up with the limit. */
  TestHelper_PushFrame(&system, 0x1, 1000, 1000, sequence++);
  now += CONTROL_PERIOD_MS;
  ControlSystem_Update(&system, now, &sensors);

  debug = ControlSystem_GetDebugState(&system);
  TEST_EQ(debug->requested_left, 1000);
  TEST_CHECK(debug->limited_left < debug->requested_left);
  TEST_CHECK(debug->actual_left < debug->limited_left);
  TEST_EQ(debug->actual_left, MOTOR_ACCEL_STEP);
}

static void TestSpinInPlaceWorks(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  const AppDebugState *debug;

  TEST_CASE("a stationary spin drives the two sides in opposition");

  ControlSystem_Init(&system);
  TestHelper_UniformSensors(&sensors, 3000U);

  now = DriveFor(&system, now, 1500U, 0x6, 650, -650, &sequence, &sensors);
  debug = ControlSystem_GetDebugState(&system);

  TEST_EQ(debug->actual_left, 650);
  TEST_EQ(debug->actual_right, -650);
  TEST_EQ(debug->motor_speed[MOTOR_FRONT_LEFT], 650);
  TEST_EQ(debug->motor_speed[MOTOR_REAR_RIGHT], -650);
}

static void TestNoSensorSnapshotIsAccepted(void)
{
  ControlSystem system;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("a null sensor snapshot behaves like the disabled default");

  ControlSystem_Init(&system);
  now = DriveFor(&system, now, 1000U, 0x1, 550, 550, &sequence, NULL);

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);
#if SAFETY_SENSOR_FAULT_POLICY == SAFETY_FAIL_OPEN
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 550);
#endif
}

int run_system_tests(void)
{
  printf("-- system --\n");
  TestFullChainForward();
  TestObstacleSlowsThenBlocks();
  TestReverseEscapeFromObstacle();
  TestHysteresisAcrossFullChain();
  TestEmergencyOutranksObstacleClearance();
  TestTimeoutOutranksObstacleClearance();
  TestSensorFaultCannotOverrideEmergency();
  TestSnapshotStagesAreDistinct();
  TestSpinInPlaceWorks();
  TestNoSensorSnapshotIsAccepted();
  return 0;
}
