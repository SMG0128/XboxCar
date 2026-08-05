#include "motor.h"

#include <string.h>

/* Per-motor direction inversion, indexed by MotorIndex. */
static const bool kMotorInverted[MOTOR_COUNT] = {
    (bool)MOTOR_FRONT_LEFT_INVERTED,
    (bool)MOTOR_REAR_LEFT_INVERTED,
    (bool)MOTOR_FRONT_RIGHT_INVERTED,
    (bool)MOTOR_REAR_RIGHT_INVERTED,
};

static int16_t ClampSpeed(int32_t value)
{
  if (value > MOTOR_SPEED_MAX)
  {
    return (int16_t)MOTOR_SPEED_MAX;
  }
  if (value < -MOTOR_SPEED_MAX)
  {
    return (int16_t)(-MOTOR_SPEED_MAX);
  }
  return (int16_t)value;
}

static int16_t AbsSpeed(int16_t value)
{
  return (value < 0) ? (int16_t)(-(int32_t)value) : value;
}

/* Zero counts as neither direction, so a move away from zero is not a reversal. */
static bool SignsOppose(int16_t a, int16_t b)
{
  return (a > 0 && b < 0) || (a < 0 && b > 0);
}

void Motor_Init(MotorController *motor)
{
  if (motor == NULL)
  {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  motor->output_enabled = false;
}

void Motor_SetTargets(MotorController *motor, int16_t left, int16_t right)
{
  int16_t clamped_left;
  int16_t clamped_right;

  if (motor == NULL)
  {
    return;
  }

  clamped_left = ClampSpeed(left);
  clamped_right = ClampSpeed(right);

  motor->target[MOTOR_FRONT_LEFT] = clamped_left;
  motor->target[MOTOR_REAR_LEFT] = clamped_left;
  motor->target[MOTOR_FRONT_RIGHT] = clamped_right;
  motor->target[MOTOR_REAR_RIGHT] = clamped_right;
}

void Motor_ForceStop(MotorController *motor)
{
  uint8_t index;

  if (motor == NULL)
  {
    return;
  }

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    motor->speed[index] = 0;
    motor->target[index] = 0;
    /*
     * Clearing the hold is deliberate. The vehicle is already stationary, so
     * the protection the hold provides has been satisfied by the stop itself.
     */
    motor->reverse_hold[index] = 0U;
  }
}

void Motor_SetOutputEnabled(MotorController *motor, bool enabled)
{
  if (motor == NULL)
  {
    return;
  }

  if (!enabled)
  {
    Motor_ForceStop(motor);
  }
  motor->output_enabled = enabled;
}

bool Motor_IsOutputEnabled(const MotorController *motor)
{
  return (motor != NULL) && motor->output_enabled;
}

/* Advances one motor by a single control period. */
static void RampOne(MotorController *motor, uint8_t index)
{
  int16_t current = motor->speed[index];
  int16_t target = motor->target[index];
  int16_t step;
  int32_t next;

  /* Holding at zero after a reversal request. */
  if (motor->reverse_hold[index] > 0U)
  {
    motor->speed[index] = 0;
    --motor->reverse_hold[index];
    return;
  }

  if (current == target)
  {
    return;
  }

  /*
   * A direction change is never applied directly. The motor decelerates to
   * zero, then spends MOTOR_REVERSE_ZERO_HOLD periods there before the new
   * direction is allowed.
   */
  if (SignsOppose(current, target))
  {
    step = (int16_t)MOTOR_DECEL_STEP;
    if (AbsSpeed(current) <= step)
    {
      motor->speed[index] = 0;
      motor->reverse_hold[index] = (uint8_t)MOTOR_REVERSE_ZERO_HOLD;
      return;
    }
    next = (current > 0) ? ((int32_t)current - step) : ((int32_t)current + step);
    motor->speed[index] = ClampSpeed(next);
    return;
  }

  /* Same direction, or one of the two is zero. */
  if (AbsSpeed(target) > AbsSpeed(current))
  {
    step = (int16_t)MOTOR_ACCEL_STEP;
  }
  else
  {
    step = (int16_t)MOTOR_DECEL_STEP;
  }

  if (target > current)
  {
    next = (int32_t)current + step;
    if (next > target)
    {
      next = target;
    }
  }
  else
  {
    next = (int32_t)current - step;
    if (next < target)
    {
      next = target;
    }
  }

  motor->speed[index] = ClampSpeed(next);
}

void Motor_Update(MotorController *motor)
{
  uint8_t index;

  if (motor == NULL)
  {
    return;
  }

  if (!motor->output_enabled)
  {
    /* Disabled means stopped, and the ramp must not creep away from zero. */
    for (index = 0U; index < MOTOR_COUNT; ++index)
    {
      motor->speed[index] = 0;
    }
    return;
  }

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    RampOne(motor, index);
  }
}

int16_t Motor_GetSpeed(const MotorController *motor, uint8_t index)
{
  if (motor == NULL || index >= MOTOR_COUNT)
  {
    return 0;
  }
  return motor->speed[index];
}

int16_t Motor_GetTarget(const MotorController *motor, uint8_t index)
{
  if (motor == NULL || index >= MOTOR_COUNT)
  {
    return 0;
  }
  return motor->target[index];
}

uint16_t Motor_SpeedToDuty(int16_t speed)
{
  uint32_t magnitude = (uint32_t)AbsSpeed(speed);
  uint32_t duty;

  if (magnitude == 0U)
  {
    return 0U;
  }

  duty = (magnitude * SOFT_PWM_RESOLUTION) / (uint32_t)MOTOR_SPEED_MAX;

  /*
   * Below the stall threshold a motor draws current without turning. Lifting
   * the duty to the measured minimum keeps low stick deflections useful. Left
   * at zero until the value has been measured on real hardware, in which case
   * the comparison is compiled out rather than left as a no-op.
   */
#if MOTOR_MIN_START_PWM > 0U
  if (duty < MOTOR_MIN_START_PWM)
  {
    duty = MOTOR_MIN_START_PWM;
  }
#endif

  if (duty > SOFT_PWM_RESOLUTION)
  {
    duty = SOFT_PWM_RESOLUTION;
  }

  return (uint16_t)duty;
}

bool Motor_GetOutput(const MotorController *motor, uint8_t index,
                     MotorOutput *output)
{
  int16_t speed;
  bool forward;

  if (output == NULL)
  {
    return false;
  }

  /* Any failure path must still yield the safe stopped state. */
  output->duty = 0U;
  output->in1 = false;
  output->in2 = false;

  if (motor == NULL || index >= MOTOR_COUNT)
  {
    return false;
  }

  if (!motor->output_enabled)
  {
    return true;
  }

  speed = motor->speed[index];
  if (speed == 0)
  {
    /*
     * Both direction pins low is the TB6612 coast state. It is the same
     * combination the pins hold out of reset, so a stopped motor and an
     * uninitialised one look identical to the driver.
     */
    return true;
  }

  forward = (speed > 0);
  if (kMotorInverted[index])
  {
    forward = !forward;
  }

  output->duty = Motor_SpeedToDuty(speed);
  output->in1 = forward;
  output->in2 = !forward;
  return true;
}

bool Motor_Validate(const MotorController *motor)
{
  uint8_t index;

  if (motor == NULL)
  {
    return false;
  }

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    if (motor->speed[index] > MOTOR_SPEED_MAX ||
        motor->speed[index] < -MOTOR_SPEED_MAX)
    {
      return false;
    }
    if (motor->target[index] > MOTOR_SPEED_MAX ||
        motor->target[index] < -MOTOR_SPEED_MAX)
    {
      return false;
    }
    if (motor->reverse_hold[index] > MOTOR_REVERSE_ZERO_HOLD)
    {
      return false;
    }
    /* A held motor must be at rest. */
    if (motor->reverse_hold[index] > 0U && motor->speed[index] != 0)
    {
      return false;
    }
  }

  return true;
}
