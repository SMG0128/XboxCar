#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Obstacle-based limiting of an already-mixed drive request.
 *
 * This module sits between the protocol target and the speed ramp:
 *
 *     protocol target -> emergency / comm checks -> THIS -> ramp -> motors
 *
 * It is deliberately the weakest link in that chain. The emergency stop, the
 * communication timeout and internal faults are all decided before this module
 * is consulted, and none of them can be overridden by a sensor reading. A
 * failed sensor can only change how much the request is limited; it can never
 * un-stop a stopped vehicle.
 *
 * The request arrives as left/right wheel speeds. Limiting them individually
 * would destroy the steering the ESP32 computed, so the pair is decomposed into
 * a forward component and a turn component, each limited on its own terms, then
 * recomposed:
 *
 *     forward = (left + right) / 2      left  = forward + turn
 *     turn    = (left - right) / 2      right = forward - turn
 *
 * A positive turn drives the left wheels faster, which steers right. This
 * matches the ESP32 mixer, where positive steering is a right turn.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_types.h"

typedef struct {
  int16_t requested_left;
  int16_t requested_right;

  /* Filtered distance per UltrasonicIndex. Meaningless unless the valid bit is set. */
  uint16_t distance_mm[ULTRASONIC_COUNT];
  uint8_t sensor_valid_mask;
  uint8_t sensor_fault_mask;
} SafetyInput;

typedef struct {
  int16_t limited_left;
  int16_t limited_right;
  uint8_t obstacle_flags;
  uint8_t limit_reason;
} SafetyOutput;

typedef struct {
  /*
   * Latched blocks. Once a direction is blocked it stays blocked until the
   * obstacle is at least SAFETY_RELEASE_DISTANCE_MM away, so a distance
   * hovering on the threshold cannot make the vehicle stutter.
   */
  bool front_blocked;
  bool rear_blocked;
  bool left_limited;
  bool right_limited;
} SafetyController;

/* Clears every latch. */
void Safety_Init(SafetyController *safety);

/*
 * Applies obstacle limiting to one request. Pure with respect to everything
 * except the latch state in safety, which is why the tests can drive it with a
 * plain table of distances.
 */
void Safety_Apply(SafetyController *safety, const SafetyInput *input,
                  SafetyOutput *output);

/*
 * True when the module currently considers ultrasonic input usable at all.
 * With the feature disabled, or every sensor faulted, limiting is inert under
 * the FAIL_OPEN policy.
 */
bool Safety_IsLimiting(const SafetyController *safety);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_CONTROLLER_H */
