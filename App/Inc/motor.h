#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Four fixed DC motors driven by two TB6612 bridges.
 *
 * The ESP32 has already done the joystick mixing and differential steering, so
 * this module never re-derives them. It receives a left-side and a right-side
 * target speed and fans them out:
 *
 *     LEFT  -> front left, rear left
 *     RIGHT -> front right, rear right
 *
 * Responsibilities: per-motor direction inversion, the acceleration and
 * deceleration ramp, the reverse-through-zero rule, and turning a signed speed
 * into a duty plus a direction pin pair. It owns the only copy of the current
 * motor speeds.
 *
 * No STM32 HAL dependency: the ramp and the mapping are exercised by the host
 * tests. Writing the pins is the board layer's job.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_types.h"

typedef struct {
  /* Signed speed after ramping, -MOTOR_SPEED_MAX..+MOTOR_SPEED_MAX. */
  int16_t speed[MOTOR_COUNT];
  /* Signed target requested by the control system. */
  int16_t target[MOTOR_COUNT];
  /* Control periods still to spend at zero before reversing. */
  uint8_t reverse_hold[MOTOR_COUNT];

  /* When false every output is forced to the safe stopped state. */
  bool output_enabled;
} MotorController;

/* One motor's electrical output. */
typedef struct {
  uint16_t duty; /* 0..SOFT_PWM_RESOLUTION */
  bool in1;
  bool in2;
} MotorOutput;

/*
 * Clears every speed and target, releases the holds, and leaves the output
 * stage disabled. Callers must enable it explicitly once the vehicle is known
 * to be in a controllable state.
 */
void Motor_Init(MotorController *motor);

/*
 * Sets the side targets. Values outside the speed range are clamped rather
 * than rejected: the protocol layer has already refused untrusted numbers, so
 * anything arriving here is a programming error that should still fail safe.
 */
void Motor_SetTargets(MotorController *motor, int16_t left, int16_t right);

/*
 * Immediately zeroes both the targets and the current speeds, bypassing the
 * ramp. Used for the emergency stop, the communication timeout and internal
 * faults, all of which must not wait for a deceleration curve.
 */
void Motor_ForceStop(MotorController *motor);

/* Enables or disables the output stage. Disabling also forces a stop. */
void Motor_SetOutputEnabled(MotorController *motor, bool enabled);

/* True when the output stage is permitted to drive. */
bool Motor_IsOutputEnabled(const MotorController *motor);

/*
 * Advances the ramp by one control period. Must be called at a steady
 * CONTROL_PERIOD_MS cadence.
 */
void Motor_Update(MotorController *motor);

/* Signed speed of one motor, or 0 for an out-of-range index. */
int16_t Motor_GetSpeed(const MotorController *motor, uint8_t index);

/* Signed target of one motor, or 0 for an out-of-range index. */
int16_t Motor_GetTarget(const MotorController *motor, uint8_t index);

/*
 * Electrical output for one motor, with the per-motor inversion applied.
 * Returns false for an out-of-range index, in which case output is set to the
 * safe stopped state.
 */
bool Motor_GetOutput(const MotorController *motor, uint8_t index,
                     MotorOutput *output);

/*
 * Checks the invariants that the rest of the firmware relies on: speeds and
 * targets inside range, holds inside range, and no motor able to produce an
 * illegal direction pin combination. Returns false if the state has been
 * corrupted, which the control system escalates to an internal fault.
 */
bool Motor_Validate(const MotorController *motor);

/* Converts a signed speed to a duty, applying the stall compensation. */
uint16_t Motor_SpeedToDuty(int16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
