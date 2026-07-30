#include "control_mixer.h"

#include "app_config.h"

#include <stddef.h>

static int32_t Abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int16_t ClampCommand(int32_t value)
{
  if (value > MOTOR_COMMAND_MAX)
  {
    return MOTOR_COMMAND_MAX;
  }
  if (value < -MOTOR_COMMAND_MAX)
  {
    return -MOTOR_COMMAND_MAX;
  }
  return (int16_t)value;
}

int16_t ControlMixer_ApplyDeadzone(int16_t value)
{
  int32_t magnitude;
  int32_t mapped;

  value = ClampCommand(value);
  magnitude = Abs32(value);
  if (magnitude <= CONTROL_DEADZONE)
  {
    return 0;
  }

  mapped = (magnitude - CONTROL_DEADZONE) * MOTOR_COMMAND_MAX /
           (MOTOR_COMMAND_MAX - CONTROL_DEADZONE);
  return (value < 0) ? (int16_t)-mapped : (int16_t)mapped;
}

void ControlMixer_Mix(int16_t throttle, int16_t steering,
                      int16_t *left, int16_t *right)
{
  int32_t left_raw;
  int32_t right_raw;
  int32_t maximum;

  if (left == NULL || right == NULL)
  {
    return;
  }

  throttle = ClampCommand(throttle);
  steering = ClampCommand(steering);
  left_raw = (int32_t)throttle + steering;
  right_raw = (int32_t)throttle - steering;
  maximum = Abs32(left_raw);
  if (Abs32(right_raw) > maximum)
  {
    maximum = Abs32(right_raw);
  }

  if (maximum > MOTOR_COMMAND_MAX)
  {
    left_raw = left_raw * MOTOR_COMMAND_MAX / maximum;
    right_raw = right_raw * MOTOR_COMMAND_MAX / maximum;
  }
  *left = ClampCommand(left_raw);
  *right = ClampCommand(right_raw);
}

int16_t ControlMixer_Approach(int16_t current, int16_t target, int16_t step)
{
  int32_t delta;

  current = ClampCommand(current);
  target = ClampCommand(target);
  if (step <= 0)
  {
    return current;
  }

  delta = (int32_t)target - current;
  if (delta > step)
  {
    return (int16_t)(current + step);
  }
  if (delta < -step)
  {
    return (int16_t)(current - step);
  }
  return target;
}
