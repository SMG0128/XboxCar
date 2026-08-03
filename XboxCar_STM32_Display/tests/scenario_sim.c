/*
 * System scenario simulation.
 *
 * Runs the real firmware modules through a scripted drive and prints a
 * timeline. This is a diagnostic tool for watching the state machine behave,
 * not a substitute for the firmware or for driving the actual vehicle: no
 * hardware, timing or electrical behaviour is modelled here.
 *
 * Time is virtual. The run completes in milliseconds of wall clock.
 */

#include <stdio.h>
#include <string.h>

#include "control_system.h"
#include "test_helpers.h"

static const char *StateName(uint8_t state)
{
  switch (state)
  {
    case CONTROL_STATE_STARTUP_SAFE:
      return "STARTUP_SAFE";
    case CONTROL_STATE_ONLINE:
      return "ONLINE";
    case CONTROL_STATE_COMM_TIMEOUT:
      return "COMM_TIMEOUT";
    case CONTROL_STATE_EMERGENCY_LOCKED:
      return "EMERG_LOCKED";
    case CONTROL_STATE_INTERNAL_FAULT:
      return "INTERNAL_FAULT";
    default:
      return "UNKNOWN";
  }
}

static void PrintHeader(void)
{
  printf("\n");
  printf("  time  state          req_l req_r  lim_l lim_r  act_l act_r  "
         "m0   m1   m2   m3   pwm0 front  obst fault\n");
  printf("  ----  -------------  ----- -----  ----- -----  ----- -----  "
         "---- ---- ---- ----  ---- -----  ---- -----\n");
}

static void PrintRow(uint32_t now_ms, const ControlSystem *system,
                     const SensorSnapshot *sensors)
{
  const AppDebugState *d = ControlSystem_GetDebugState(system);

  printf("  %4u  %-13s  %5d %5d  %5d %5d  %5d %5d  %4d %4d %4d %4d  %4u %5u  "
         "0x%02X 0x%02X\n",
         (unsigned)now_ms, StateName(d->control_state), d->requested_left,
         d->requested_right, d->limited_left, d->limited_right, d->actual_left,
         d->actual_right, d->motor_speed[0], d->motor_speed[1],
         d->motor_speed[2], d->motor_speed[3], (unsigned)d->motor_pwm[0],
         (unsigned)(sensors != NULL ? sensors->distance_mm[ULTRASONIC_FRONT] : 0U),
         (unsigned)d->obstacle_flags, (unsigned)d->last_fault_code);
}

/* One phase of the script. */
typedef struct {
  const char *label;
  uint32_t duration_ms;
  bool send_frames;
  uint8_t command;
  int16_t left;
  int16_t right;
  uint16_t front_mm;
} Phase;

static uint32_t RunPhase(ControlSystem *system, uint32_t now_ms,
                         const Phase *phase, uint16_t *sequence,
                         SensorSnapshot *sensors, uint32_t report_every_ms)
{
  uint32_t elapsed = 0U;
  uint32_t since_report = report_every_ms;
  uint32_t period_index = 0U;

  printf("\n[%s]\n", phase->label);
  PrintHeader();

  sensors->distance_mm[ULTRASONIC_FRONT] = phase->front_mm;
  sensors->raw_mm[ULTRASONIC_FRONT] = phase->front_mm;

  while (elapsed < phase->duration_ms)
  {
    /* The emitter runs at 50 Hz, the control loop at 100 Hz. */
    if (phase->send_frames && (period_index % 2U) == 0U)
    {
      TestHelper_PushFrame(system, phase->command, phase->left, phase->right,
                           (*sequence)++);
    }

    now_ms += CONTROL_PERIOD_MS;
    elapsed += CONTROL_PERIOD_MS;
    ++period_index;
    ControlSystem_Update(system, now_ms, sensors);

    since_report += CONTROL_PERIOD_MS;
    if (since_report >= report_every_ms)
    {
      since_report = 0U;
      PrintRow(now_ms, system, sensors);
    }
  }

  return now_ms;
}

int main(void)
{
  ControlSystem system;
  SensorSnapshot sensors;
  uint32_t now = 0U;
  uint16_t sequence = 1U;
  const AppDebugState *debug;
  int failures = 0;

  printf("XboxCar STM32 scenario simulation\n");
  printf("=================================\n");
  printf("Virtual time, real firmware modules. No hardware is involved.\n");

  ControlSystem_Init(&system);

  /* Ultrasonic is disabled by default in the firmware; the simulation enables
   * it so the obstacle logic can be observed. */
  TestHelper_UniformSensors(&sensors, 3000U);

  {
    static const Phase kIdle = {"power on, no traffic yet", 200U, false, 0x0, 0,
                                0, 3000U};
    now = RunPhase(&system, now, &kIdle, &sequence, &sensors, 100U);
    if (ControlSystem_GetState(&system) != CONTROL_STATE_STARTUP_SAFE)
    {
      printf("  !! expected STARTUP_SAFE\n");
      ++failures;
    }
  }

  {
    static const Phase kForward = {"forward request, smooth acceleration", 600U,
                                   true, 0x1, 800, 800, 3000U};
    now = RunPhase(&system, now, &kForward, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (debug->actual_left != 800)
    {
      printf("  !! expected to reach 800\n");
      ++failures;
    }
  }

  {
    static const Phase kTurn = {"turning right while driving", 400U, true, 0x4,
                                800, 350, 3000U};
    now = RunPhase(&system, now, &kTurn, &sequence, &sensors, 100U);
  }

  {
    static const Phase kApproach = {"obstacle enters the slow band", 500U, true,
                                    0x1, 900, 900, 500U};
    now = RunPhase(&system, now, &kApproach, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (debug->limited_left >= debug->requested_left)
    {
      printf("  !! expected the request to be limited\n");
      ++failures;
    }
  }

  {
    static const Phase kBlocked = {"obstacle inside the stop band", 400U, true,
                                   0x1, 900, 900, 200U};
    now = RunPhase(&system, now, &kBlocked, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (debug->actual_left != 0)
    {
      printf("  !! expected forward motion to be blocked\n");
      ++failures;
    }
  }

  {
    static const Phase kEscape = {"reversing away is still allowed", 500U, true,
                                  0x2, -600, -600, 200U};
    now = RunPhase(&system, now, &kEscape, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (debug->actual_left >= 0)
    {
      printf("  !! expected reverse motion to be permitted\n");
      ++failures;
    }
  }

  {
    /* Inside the release band: hysteresis must keep forward blocked. */
    static const Phase kStillBlocked = {
        "obstacle backs off but not past the release distance", 300U, true, 0x1,
        800, 800, (uint16_t)(SAFETY_STOP_DISTANCE_MM + 20U)};
    now = RunPhase(&system, now, &kStillBlocked, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (debug->actual_left != 0)
    {
      printf("  !! expected hysteresis to keep forward blocked\n");
      ++failures;
    }
  }

  {
    static const Phase kCleared = {"obstacle gone, motion resumes", 600U, true,
                                   0x1, 800, 800, 2500U};
    now = RunPhase(&system, now, &kCleared, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (debug->actual_left != 800)
    {
      printf("  !! expected motion to resume\n");
      ++failures;
    }
  }

  {
    static const Phase kSilence = {"esp32 stops sending", 500U, false, 0x0, 0, 0,
                                   2500U};
    now = RunPhase(&system, now, &kSilence, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (ControlSystem_GetState(&system) != CONTROL_STATE_COMM_TIMEOUT ||
        debug->actual_left != 0)
    {
      printf("  !! expected a communication timeout stop\n");
      ++failures;
    }
  }

  {
    static const Phase kRestored = {"link restored", 500U, true, 0x1, 500, 500,
                                    2500U};
    now = RunPhase(&system, now, &kRestored, &sequence, &sensors, 100U);
    if (ControlSystem_GetState(&system) != CONTROL_STATE_ONLINE)
    {
      printf("  !! expected the link to recover\n");
      ++failures;
    }
  }

  {
    static const Phase kEmergency = {"emergency stop pressed", 200U, true, 0x7,
                                     0, 0, 2500U};
    now = RunPhase(&system, now, &kEmergency, &sequence, &sensors, 100U);
    if (!ControlSystem_IsEmergencyLocked(&system))
    {
      printf("  !! expected the emergency latch to engage\n");
      ++failures;
    }
  }

  {
    /*
     * Two ordinary frames, one short of the requirement. The latch must hold.
     * Sent by hand so the count is exact rather than time based.
     */
    uint8_t index;
    printf("\n[ordinary frames cannot clear the latch]\n");
    PrintHeader();
    for (index = 0U; index < COMM_RECOVERY_FRAME_COUNT - 1U; ++index)
    {
      TestHelper_PushFrame(&system, 0x1, 500, 500, sequence++);
      now += CONTROL_PERIOD_MS;
      ControlSystem_Update(&system, now, &sensors);
      PrintRow(now, &system, &sensors);
    }
    if (!ControlSystem_IsEmergencyLocked(&system))
    {
      printf("  !! latch cleared too early\n");
      ++failures;
    }
  }

  {
    uint8_t index;
    printf("\n[the full recovery run clears the latch]\n");
    PrintHeader();
    /* One more accepted frame completes the run. */
    for (index = 0U; index < 2U; ++index)
    {
      TestHelper_PushFrame(&system, 0x1, 500, 500, sequence++);
      now += CONTROL_PERIOD_MS;
      ControlSystem_Update(&system, now, &sensors);
      PrintRow(now, &system, &sensors);
    }
    if (ControlSystem_IsEmergencyLocked(&system))
    {
      printf("  !! expected the latch to clear\n");
      ++failures;
    }
  }

  {
    static const Phase kResume = {"driving again from standstill", 600U, true,
                                  0x1, 700, 700, 2500U};
    now = RunPhase(&system, now, &kResume, &sequence, &sensors, 100U);
    debug = ControlSystem_GetDebugState(&system);
    if (debug->actual_left != 700)
    {
      printf("  !! expected the vehicle to drive again\n");
      ++failures;
    }
  }

  debug = ControlSystem_GetDebugState(&system);
  printf("\n");
  printf("final counters\n");
  printf("  uptime_ms             : %u\n", (unsigned)debug->uptime_ms);
  printf("  uart_rx_bytes         : %u\n", (unsigned)debug->uart_rx_bytes);
  printf("  valid_frames          : %u\n", (unsigned)debug->valid_frames);
  printf("  crc_errors            : %u\n", (unsigned)debug->crc_errors);
  printf("  format_errors         : %u\n", (unsigned)debug->format_errors);
  printf("  range_errors          : %u\n", (unsigned)debug->range_errors);
  printf("  sequence_errors       : %u\n", (unsigned)debug->sequence_errors);
  printf("  duplicate_frames      : %u\n", (unsigned)debug->duplicate_frames);
  printf("  rx_overflows          : %u\n", (unsigned)debug->rx_overflows);
  printf("  communication_timeouts: %u\n",
         (unsigned)debug->communication_timeouts);
  printf("  last_fault_code       : 0x%02X\n", (unsigned)debug->last_fault_code);

  printf("\n%s\n", (failures == 0) ? "SCENARIO: PASS" : "SCENARIO: FAIL");
  (void)now;
  return (failures == 0) ? 0 : 1;
}
