/*
 * Control state machine tests: startup, communication timeout, emergency latch
 * and recovery, and internal faults.
 *
 * Time is virtual throughout. Nothing sleeps.
 */

#include <string.h>

#include "control_system.h"
#include "test_framework.h"
#include "test_helpers.h"

/* Feeds one frame and runs a single control period. */
static uint32_t PushAndStep(ControlSystem *system, uint32_t now_ms,
                            uint8_t command, int16_t left, int16_t right,
                            uint16_t sequence, const SensorSnapshot *sensors)
{
  TestHelper_PushFrame(system, command, left, right, sequence);
  now_ms += CONTROL_PERIOD_MS;
  ControlSystem_Update(system, now_ms, sensors);
  return now_ms;
}

static void TestStartsSafe(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;

  TEST_CASE("power-on state is safe and drives nothing");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_STARTUP_SAFE);

  /* Time passing without any frame must not enable the output. */
  now = TestHelper_RunFor(&system, now, 500U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_STARTUP_SAFE);
  TEST_EQ(ControlSystem_GetDebugState(&system)->motor_output_enabled, 0);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
  (void)now;
}

static void TestFirstFrameGoesOnline(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;

  TEST_CASE("the first accepted frame brings the vehicle online");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, 1U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);
  TEST_CHECK(ControlSystem_IsOnline(&system));
  TEST_EQ(ControlSystem_GetDebugState(&system)->motor_output_enabled, 1);
  TEST_EQ(ControlSystem_GetDebugState(&system)->requested_left, 500);
}

static void TestRampReachesTarget(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  int period;

  TEST_CASE("a held request ramps up to the commanded speed");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  /* Emitter runs at 50 Hz, control at 100 Hz: a frame every other period. */
  for (period = 0; period < 200; ++period)
  {
    if ((period % 2) == 0)
    {
      TestHelper_PushFrame(&system, 0x1, 600, 600, sequence++);
    }
    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }

  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 600);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_right, 600);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);
}

static void TestCommunicationTimeoutStops(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  int period;

  TEST_CASE("losing the link stops the vehicle immediately");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  for (period = 0; period < 100; ++period)
  {
    TestHelper_PushFrame(&system, 0x1, 800, 800, sequence++);
    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 800);

  /* Emitter goes quiet. */
  now = TestHelper_RunFor(&system, now, COMM_TIMEOUT_MS + CONTROL_PERIOD_MS * 2U,
                          &sensors);

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_COMM_TIMEOUT);
  /* Not a ramp down: zero on the period the timeout is detected. */
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
  TEST_EQ(ControlSystem_GetDebugState(&system)->motor_output_enabled, 0);
  TEST_EQ(ControlSystem_GetDebugState(&system)->communication_timeouts, 1);
}

static void TestTimeoutCountedOncePerOutage(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;

  TEST_CASE("one outage counts once, not once per period");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 400, 400, 1U, &sensors);
  now = TestHelper_RunFor(&system, now, COMM_TIMEOUT_MS * 3U, &sensors);

  TEST_EQ(ControlSystem_GetDebugState(&system)->communication_timeouts, 1);
}

static void TestRecoveryAfterTimeout(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;

  TEST_CASE("the link recovering brings the vehicle back online from zero");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 900, 900, 10U, &sensors);
  now = TestHelper_RunFor(&system, now, COMM_TIMEOUT_MS * 2U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_COMM_TIMEOUT);

  /*
   * The emitter comes back. Its sequence continues from where it left off, but
   * the baseline was dropped at the timeout, so the frame is accepted at once.
   */
  now = PushAndStep(&system, now, 0x1, 900, 900, 200U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);
  /* Restarting from standstill, not from the pre-outage speed. */
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, MOTOR_ACCEL_STEP);
}

static void TestEsp32RestartAfterTimeout(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;

  TEST_CASE("an esp32 restart during an outage reconnects cleanly");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, 4000U, &sensors);
  now = TestHelper_RunFor(&system, now, COMM_TIMEOUT_MS * 2U, &sensors);

  /* Rebooted emitter starts again from 0000. */
  now = PushAndStep(&system, now, 0x0, 0, 0, 0U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);

  now = PushAndStep(&system, now, 0x1, 300, 300, 1U, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->requested_left, 300);
}

static void TestErrorFramesDoNotRefreshWatchdog(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  int period;

  TEST_CASE("corrupt frames do not keep the link alive");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 700, 700, 1U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);

  /*
   * A steady stream of corrupt frames. If any of them refreshed the watchdog
   * the vehicle would keep driving on data it rejected.
   */
  for (period = 0; period < 60; ++period)
  {
    char frame[XBOX_FRAME_LENGTH + 1U];
    TestHelper_BuildFrame(frame, 0x1, 700, 700, (uint16_t)(period + 2));
    frame[XBOX_FRAME_CRC_OFFSET] = 'F';
    frame[XBOX_FRAME_CRC_OFFSET + 1U] = 'F';
    TestHelper_PushBytes(&system, frame, XBOX_FRAME_LENGTH);

    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_COMM_TIMEOUT);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
  TEST_CHECK(ControlSystem_GetDebugState(&system)->crc_errors > 0U);
}

static void TestDuplicateFramesDoNotRefreshWatchdog(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  int period;
  char frame[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("a stuck emitter repeating one frame is treated as a dead link");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 600, 600, 77U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);

  /* The same frame over and over carries no new information. */
  TestHelper_BuildFrame(frame, 0x1, 600, 600, 77U);
  for (period = 0; period < 60; ++period)
  {
    TestHelper_PushBytes(&system, frame, XBOX_FRAME_LENGTH);
    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_COMM_TIMEOUT);
  TEST_CHECK(ControlSystem_GetDebugState(&system)->duplicate_frames > 0U);
}

static void TestEmergencyStopLatches(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  int period;

  TEST_CASE("an emergency frame latches the vehicle stopped");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  for (period = 0; period < 100; ++period)
  {
    TestHelper_PushFrame(&system, 0x1, 800, 800, sequence++);
    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 800);

  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_EMERGENCY_LOCKED);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
  TEST_EQ(ControlSystem_GetDebugState(&system)->last_fault_code,
          FAULT_EMERGENCY_STOP);
}

static void TestOrdinaryFramesCannotClearEmergency(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("fewer than the required frames cannot clear the latch");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));

  /* One short of the requirement. */
  {
    uint8_t index;
    for (index = 0U; index < COMM_RECOVERY_FRAME_COUNT - 1U; ++index)
    {
      now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
      TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));
      TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
    }
  }
}

static void TestEmergencyClearsAfterConsecutiveFrames(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  uint8_t index;

  TEST_CASE("the required run of good frames clears the latch");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));

  for (index = 0U; index < COMM_RECOVERY_FRAME_COUNT; ++index)
  {
    now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  }

  TEST_CHECK(!ControlSystem_IsEmergencyLocked(&system));
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);
  /* And it resumes from standstill. */
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, MOTOR_ACCEL_STEP);
}

static void TestDuplicateDoesNotCountTowardRecovery(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  char frame[XBOX_FRAME_LENGTH + 1U];
  uint8_t index;

  TEST_CASE("repeated frames do not earn recovery credit");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));

  /* One genuine recovery frame, then the same frame repeated many times. */
  now = PushAndStep(&system, now, 0x1, 400, 400, sequence, &sensors);
  TestHelper_BuildFrame(frame, 0x1, 400, 400, sequence);
  for (index = 0U; index < 10U; ++index)
  {
    TestHelper_PushBytes(&system, frame, XBOX_FRAME_LENGTH);
    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }

  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));
}

static void TestErrorFrameBreaksRecoveryRun(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  char frame[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("a corrupt frame restarts the recovery run");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);

  /* Two good frames, one short of clearing with the default of three. */
  now = PushAndStep(&system, now, 0x1, 400, 400, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x1, 400, 400, sequence++, &sensors);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));

  /* A corrupt frame lands, which must reset the count to zero. */
  TestHelper_BuildFrame(frame, 0x1, 400, 400, sequence++);
  frame[XBOX_FRAME_CRC_OFFSET] = 'F';
  frame[XBOX_FRAME_CRC_OFFSET + 1U] = 'F';
  TestHelper_PushBytes(&system, frame, XBOX_FRAME_LENGTH);
  now += CONTROL_PERIOD_MS;
  ControlSystem_Update(&system, now, &sensors);

  /* One more good frame would have cleared it before the corruption. */
  now = PushAndStep(&system, now, 0x1, 400, 400, sequence++, &sensors);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));

  /* A full clean run does clear it. */
  {
    uint8_t index;
    for (index = 1U; index < COMM_RECOVERY_FRAME_COUNT; ++index)
    {
      now = PushAndStep(&system, now, 0x1, 400, 400, sequence++, &sensors);
    }
  }
  TEST_CHECK(!ControlSystem_IsEmergencyLocked(&system));
}

static void TestRepeatedEmergencyResetsRun(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("a further emergency frame discards recovery progress");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x1, 400, 400, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x1, 400, 400, sequence++, &sensors);

  /* Operator presses the button again. */
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x1, 400, 400, sequence++, &sensors);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));
}

static void TestEmergencyOutranksCommunicationTimeout(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("the emergency latch survives a communication outage");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);

  /* Link drops while latched. */
  now = TestHelper_RunFor(&system, now, COMM_TIMEOUT_MS * 2U, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_EMERGENCY_LOCKED);
  TEST_CHECK(ControlSystem_IsEmergencyLocked(&system));
}

static void TestInternalFaultLatches(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  uint8_t index;

  TEST_CASE("an internal fault latches the output off permanently");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_ONLINE);

  ControlSystem_ReportInternalFault(&system, FAULT_INTERNAL_PWM_TIMER,
                                    INTERNAL_FAULT_PWM_TIMER, now);

  /* No amount of good traffic recovers from this. */
  for (index = 0U; index < 20U; ++index)
  {
    now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  }

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_INTERNAL_FAULT);
  TEST_EQ(ControlSystem_GetDebugState(&system)->motor_output_enabled, 0);
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
  TEST_CHECK((ControlSystem_GetDebugState(&system)->internal_faults &
              INTERNAL_FAULT_PWM_TIMER) != 0U);
}

static void TestInternalFaultOutranksEmergency(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;

  TEST_CASE("an internal fault outranks the emergency latch");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);
  now = PushAndStep(&system, now, 0x7, 0, 0, sequence++, &sensors);
  ControlSystem_ReportInternalFault(&system, FAULT_INTERNAL_GPIO_INIT,
                                    INTERNAL_FAULT_GPIO_INIT, now);
  now = PushAndStep(&system, now, 0x1, 500, 500, sequence++, &sensors);

  TEST_EQ(ControlSystem_GetState(&system), CONTROL_STATE_INTERNAL_FAULT);
}

static void TestDisconnectedCommandStopsVehicle(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  int period;

  TEST_CASE("a controller disconnect frame stops the vehicle");

  ControlSystem_Init(&system);
  TestHelper_NoSensors(&sensors);

  for (period = 0; period < 60; ++period)
  {
    TestHelper_PushFrame(&system, 0x1, 700, 700, sequence++);
    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }
  TEST_CHECK(ControlSystem_GetDebugState(&system)->actual_left > 0);

  /* Disconnected carries zeros, so the vehicle ramps down and stops. */
  for (period = 0; period < 100; ++period)
  {
    TestHelper_PushFrame(&system, 0x9, 0, 0, sequence++);
    now += CONTROL_PERIOD_MS;
    ControlSystem_Update(&system, now, &sensors);
  }
  TEST_EQ(ControlSystem_GetDebugState(&system)->actual_left, 0);
}

int run_control_tests(void)
{
  printf("-- control --\n");
  TestStartsSafe();
  TestFirstFrameGoesOnline();
  TestRampReachesTarget();
  TestCommunicationTimeoutStops();
  TestTimeoutCountedOncePerOutage();
  TestRecoveryAfterTimeout();
  TestEsp32RestartAfterTimeout();
  TestErrorFramesDoNotRefreshWatchdog();
  TestDuplicateFramesDoNotRefreshWatchdog();
  TestEmergencyStopLatches();
  TestOrdinaryFramesCannotClearEmergency();
  TestEmergencyClearsAfterConsecutiveFrames();
  TestDuplicateDoesNotCountTowardRecovery();
  TestErrorFrameBreaksRecoveryRun();
  TestRepeatedEmergencyResetsRun();
  TestEmergencyOutranksCommunicationTimeout();
  TestInternalFaultLatches();
  TestInternalFaultOutranksEmergency();
  TestDisconnectedCommandStopsVehicle();
  return 0;
}
