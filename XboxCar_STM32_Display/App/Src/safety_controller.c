#include "safety_controller.h"

#include <string.h>

/* Whether a sensor's reading may be trusted this cycle. */
static bool SensorUsable(const SafetyInput *input, uint8_t sensor)
{
  const uint8_t bit = (uint8_t)(1U << sensor);

  if ((input->sensor_fault_mask & bit) != 0U)
  {
    return false;
  }
  return (input->sensor_valid_mask & bit) != 0U;
}

/*
 * Speed ceiling permitted by one distance, in the range 0..MOTOR_SPEED_MAX.
 *
 * Beyond the slow band there is no limit. Inside it the ceiling falls linearly
 * to zero at the stop distance. Below the stop distance the direction is
 * blocked, which the caller latches.
 */
static int16_t SpeedCeilingFor(uint16_t distance_mm)
{
  uint32_t span;
  uint32_t above;
  uint32_t ceiling;

  if (distance_mm >= SAFETY_SLOW_DISTANCE_MM)
  {
    return (int16_t)MOTOR_SPEED_MAX;
  }
  if (distance_mm <= SAFETY_STOP_DISTANCE_MM)
  {
    return 0;
  }

  span = (uint32_t)(SAFETY_SLOW_DISTANCE_MM - SAFETY_STOP_DISTANCE_MM);
  above = (uint32_t)(distance_mm - SAFETY_STOP_DISTANCE_MM);
  ceiling = (above * (uint32_t)MOTOR_SPEED_MAX) / span;

  return (int16_t)ceiling;
}

/*
 * Resolves one longitudinal direction into a ceiling, updating its latch.
 *
 * blocked carries the latch in and out. slow_reason and stop_reason are ORed
 * into reason so the caller can report why the request changed.
 */
static int16_t ResolveDirection(const SafetyInput *input, uint8_t sensor,
                                bool *blocked, uint8_t slow_reason,
                                uint8_t stop_reason, uint8_t *reason)
{
  uint16_t distance;

  if (!SensorUsable(input, sensor))
  {
    *reason |= LIMIT_REASON_SENSOR_FAULT;
#if SAFETY_SENSOR_FAULT_POLICY == SAFETY_FAIL_SAFE
    /* Distrust the direction entirely. */
    *blocked = true;
    *reason |= stop_reason;
    return 0;
#else
    /*
     * FAIL_OPEN. Without a sensor there is nothing to limit against, so the
     * request passes through and the operator remains responsible. The latch is
     * released so a recovering sensor does not inherit a stale block.
     */
    *blocked = false;
    return (int16_t)MOTOR_SPEED_MAX;
#endif
  }

  distance = input->distance_mm[sensor];

  if (*blocked)
  {
    /* Hysteresis: only a clear margin releases the latch. */
    if (distance < SAFETY_RELEASE_DISTANCE_MM)
    {
      *reason |= stop_reason;
      return 0;
    }
    *blocked = false;
  }

  if (distance <= SAFETY_STOP_DISTANCE_MM)
  {
    *blocked = true;
    *reason |= stop_reason;
    return 0;
  }

  if (distance < SAFETY_SLOW_DISTANCE_MM)
  {
    *reason |= slow_reason;
  }

  return SpeedCeilingFor(distance);
}

/*
 * Resolves one side sensor into a turn-authority ceiling toward that side.
 *
 * A side obstacle never stops the vehicle. Full turn authority away from the
 * obstacle is preserved; only turning further into it is capped, and never
 * below SAFETY_SIDE_MIN_TURN so the vehicle stays steerable.
 */
static int16_t ResolveSide(const SafetyInput *input, uint8_t sensor,
                           bool *limited, uint8_t side_reason, uint8_t *reason)
{
  uint16_t distance;

  if (!SensorUsable(input, sensor))
  {
    *reason |= LIMIT_REASON_SENSOR_FAULT;
#if SAFETY_SENSOR_FAULT_POLICY == SAFETY_FAIL_SAFE
    *limited = true;
    *reason |= side_reason;
    return (int16_t)SAFETY_SIDE_MIN_TURN;
#else
    *limited = false;
    return (int16_t)MOTOR_SPEED_MAX;
#endif
  }

  distance = input->distance_mm[sensor];

  if (*limited)
  {
    if (distance < SAFETY_SIDE_RELEASE_DISTANCE_MM)
    {
      *reason |= side_reason;
      return (int16_t)SAFETY_SIDE_MIN_TURN;
    }
    *limited = false;
  }

  if (distance <= SAFETY_SIDE_LIMIT_DISTANCE_MM)
  {
    *limited = true;
    *reason |= side_reason;
    return (int16_t)SAFETY_SIDE_MIN_TURN;
  }

  return (int16_t)MOTOR_SPEED_MAX;
}

void Safety_Init(SafetyController *safety)
{
  if (safety == NULL)
  {
    return;
  }
  memset(safety, 0, sizeof(*safety));
}

bool Safety_IsLimiting(const SafetyController *safety)
{
  if (safety == NULL)
  {
    return false;
  }
  return safety->front_blocked || safety->rear_blocked ||
         safety->left_limited || safety->right_limited;
}

void Safety_Apply(SafetyController *safety, const SafetyInput *input,
                  SafetyOutput *output)
{
  int32_t forward;
  int32_t turn;
  int32_t left;
  int32_t right;
  int32_t magnitude;
  uint8_t reason = LIMIT_REASON_NONE;
  uint8_t flags = 0U;

  if (output == NULL)
  {
    return;
  }

  if (safety == NULL || input == NULL)
  {
    output->limited_left = 0;
    output->limited_right = 0;
    output->obstacle_flags = 0U;
    output->limit_reason = LIMIT_REASON_NONE;
    return;
  }

  forward = ((int32_t)input->requested_left + (int32_t)input->requested_right) / 2;
  turn = ((int32_t)input->requested_left - (int32_t)input->requested_right) / 2;

  /* Longitudinal limiting applies only to the direction actually requested. */
  if (forward > 0)
  {
    int16_t ceiling = ResolveDirection(input, ULTRASONIC_FRONT,
                                       &safety->front_blocked,
                                       LIMIT_REASON_FRONT_SLOW,
                                       LIMIT_REASON_FRONT_STOP, &reason);
    if (forward > ceiling)
    {
      forward = ceiling;
    }
  }
  else if (forward < 0)
  {
    int16_t ceiling = ResolveDirection(input, ULTRASONIC_REAR,
                                       &safety->rear_blocked,
                                       LIMIT_REASON_REAR_SLOW,
                                       LIMIT_REASON_REAR_STOP, &reason);
    if (-forward > ceiling)
    {
      forward = -(int32_t)ceiling;
    }
  }

  /*
   * Side limiting applies whenever the vehicle is turning toward that side,
   * including a stationary spin where the forward component is zero.
   */
  if (turn < 0)
  {
    /* Turning left. */
    int16_t ceiling = ResolveSide(input, ULTRASONIC_LEFT, &safety->left_limited,
                                  LIMIT_REASON_SIDE_LEFT, &reason);
    if (-turn > ceiling)
    {
      turn = -(int32_t)ceiling;
    }
  }
  else if (turn > 0)
  {
    /* Turning right. */
    int16_t ceiling = ResolveSide(input, ULTRASONIC_RIGHT,
                                  &safety->right_limited,
                                  LIMIT_REASON_SIDE_RIGHT, &reason);
    if (turn > ceiling)
    {
      turn = ceiling;
    }
  }

  left = forward + turn;
  right = forward - turn;

  /*
   * Recomposition can exceed the speed limit. Scale both wheels by the same
   * factor rather than clipping them independently, which would change the
   * turn radius the operator asked for.
   */
  magnitude = (left < 0) ? -left : left;
  {
    int32_t other = (right < 0) ? -right : right;
    if (other > magnitude)
    {
      magnitude = other;
    }
  }
  if (magnitude > MOTOR_SPEED_MAX)
  {
    left = (left * MOTOR_SPEED_MAX) / magnitude;
    right = (right * MOTOR_SPEED_MAX) / magnitude;
  }

  /* Report what is near, independently of whether it changed the request. */
  if (SensorUsable(input, ULTRASONIC_FRONT) &&
      input->distance_mm[ULTRASONIC_FRONT] < SAFETY_SLOW_DISTANCE_MM)
  {
    flags |= OBSTACLE_FLAG_FRONT;
  }
  if (SensorUsable(input, ULTRASONIC_REAR) &&
      input->distance_mm[ULTRASONIC_REAR] < SAFETY_SLOW_DISTANCE_MM)
  {
    flags |= OBSTACLE_FLAG_REAR;
  }
  if (SensorUsable(input, ULTRASONIC_LEFT) &&
      input->distance_mm[ULTRASONIC_LEFT] <= SAFETY_SIDE_LIMIT_DISTANCE_MM)
  {
    flags |= OBSTACLE_FLAG_LEFT;
  }
  if (SensorUsable(input, ULTRASONIC_RIGHT) &&
      input->distance_mm[ULTRASONIC_RIGHT] <= SAFETY_SIDE_LIMIT_DISTANCE_MM)
  {
    flags |= OBSTACLE_FLAG_RIGHT;
  }
  if (safety->front_blocked)
  {
    flags |= OBSTACLE_FLAG_FRONT_BLOCKED;
  }
  if (safety->rear_blocked)
  {
    flags |= OBSTACLE_FLAG_REAR_BLOCKED;
  }

  output->limited_left = (int16_t)left;
  output->limited_right = (int16_t)right;
  output->obstacle_flags = flags;
  output->limit_reason = reason;
}
