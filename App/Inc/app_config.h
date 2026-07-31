#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * Central tuning and feature configuration for the XboxCar STM32 control end.
 *
 * Every tunable constant used by the portable application layer lives here so
 * that bring-up calibration touches one file. Modules must not define their own
 * copies of these values.
 *
 * This header is deliberately free of STM32 HAL includes: the host test build
 * compiles it unchanged.
 */

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Configuration identity                                                     */
/* ------------------------------------------------------------------------- */

/* Bumped whenever the meaning of a value below changes incompatibly. */
#define CONFIG_VERSION 1U

/* ------------------------------------------------------------------------- */
/* Feature switches                                                           */
/* ------------------------------------------------------------------------- */

/*
 * Ultrasonic sensing stays OFF until every HC-SR04 ECHO line has a divider or
 * level shifter fitted. The PCB routes ECHO straight into the MCU, and HC-SR04
 * drives ECHO at 5 V. See config/PCB_HARDWARE_WARNINGS.md.
 */
#ifndef APP_FEATURE_ULTRASONIC
#define APP_FEATURE_ULTRASONIC 0
#endif

/*
 * The OLED shares the PCB's 4-pin I2C header (PB8/PB9 + its own 3V3/GND pins),
 * which does not collide with anything else. It is enabled, but the display is
 * never allowed to affect vehicle control: see APP_DISPLAY_* below.
 */
#ifndef APP_FEATURE_DISPLAY
#define APP_FEATURE_DISPLAY 1
#endif

/* Allows a build that runs the full control chain with the output stage muted. */
#ifndef APP_FEATURE_MOTOR_OUTPUT
#define APP_FEATURE_MOTOR_OUTPUT 1
#endif

/* ------------------------------------------------------------------------- */
/* Clock                                                                      */
/* ------------------------------------------------------------------------- */

/*
 * The established CubeMX baseline runs directly from HSI at 8 MHz, with no
 * PLL and no external crystal. Keep the application constants aligned with
 * the generated clock tree rather than silently changing the board clock.
 */
#define APP_SYSCLK_HZ 8000000UL
#define APP_APB1_TIMER_CLK_HZ 8000000UL

/* ------------------------------------------------------------------------- */
/* Control loop                                                               */
/* ------------------------------------------------------------------------- */

#define CONTROL_PERIOD_MS 10U

/* ------------------------------------------------------------------------- */
/* Motors                                                                     */
/* ------------------------------------------------------------------------- */

#define MOTOR_COUNT 4U

/* Speed domain shared with the ESP32 protocol: -1000..+1000. */
#define MOTOR_SPEED_MAX 1000

/* Ramp steps applied once per control period, in speed units. */
#define MOTOR_ACCEL_STEP 25
#define MOTOR_DECEL_STEP 40

/*
 * Control periods spent at exactly zero before a motor is allowed to drive the
 * opposite direction. Protects the TB6612 output stage and the gearbox from a
 * hard reversal.
 */
#define MOTOR_REVERSE_ZERO_HOLD 2U

/*
 * Duty below which a motor stalls instead of turning, expressed in PWM counts.
 * Zero disables the compensation. Measure per motor during bring-up and raise
 * this only after the four motors are known to spin.
 */
#define MOTOR_MIN_START_PWM 0U

/*
 * Per-motor direction inversion. Every one of these is UNVERIFIED until the
 * PCB is populated and each wheel is observed. See the bring-up guide.
 */
#define MOTOR_FRONT_LEFT_INVERTED 0
#define MOTOR_REAR_LEFT_INVERTED 0
#define MOTOR_FRONT_RIGHT_INVERTED 1
#define MOTOR_REAR_RIGHT_INVERTED 1

/* ------------------------------------------------------------------------- */
/* Software PWM                                                               */
/* ------------------------------------------------------------------------- */

/* Duty resolution: duty ranges 0..SOFT_PWM_RESOLUTION inclusive. */
#define SOFT_PWM_RESOLUTION 100U

/* Carrier frequency seen by the TB6612. */
#define SOFT_PWM_FREQUENCY_HZ 200U

/* Derived interrupt rate. One tick per duty step. */
#define SOFT_PWM_ISR_HZ (SOFT_PWM_RESOLUTION * SOFT_PWM_FREQUENCY_HZ)

/* ------------------------------------------------------------------------- */
/* Communication                                                              */
/* ------------------------------------------------------------------------- */

/*
 * No accepted control frame within this window drops the vehicle to a stop.
 * The ESP32 sends at 50 Hz (20 ms), so this tolerates 14 consecutive losses.
 */
#define COMM_TIMEOUT_MS 300U

/* Receive ring buffer, sized to hold several whole frames across a display flush. */
#define COMM_RX_RING_SIZE 256U

/* Consecutive accepted non-emergency frames required to leave a latched stop. */
#define COMM_RECOVERY_FRAME_COUNT 3U

/*
 * Forward sequence-number jump still treated as the same ESP32 stream. Anything
 * beyond this is a stream discontinuity and needs re-baselining.
 */
#define COMM_SEQ_FORWARD_WINDOW 100U

/* Sequence numbers wrap here, matching the ESP32 emitter. */
#define COMM_SEQ_MODULUS 10000U

/* ------------------------------------------------------------------------- */
/* Ultrasonic                                                                 */
/* ------------------------------------------------------------------------- */

#define ULTRASONIC_COUNT 4U

/* Trigger pulse width demanded by HC-SR04. */
#define ULTRASONIC_TRIGGER_US 12U

/* Give up waiting for the echo line to rise after the trigger. */
#define ULTRASONIC_RISE_TIMEOUT_US 6000UL

/* Give up waiting for the echo line to fall, ~4.3 m of flight time. */
#define ULTRASONIC_FALL_TIMEOUT_US 25000UL

/* Quiet time after each sensor so the previous burst cannot alias into the next. */
#define ULTRASONIC_COOLDOWN_US 12000UL

/* Echo pulses outside this band are rejected before any distance is derived. */
#define ULTRASONIC_MIN_PULSE_US 60UL
#define ULTRASONIC_MAX_PULSE_US 23000UL

/* Distance band accepted as a real measurement. */
#define ULTRASONIC_MIN_VALID_MM 20U
#define ULTRASONIC_MAX_VALID_MM 4000U

/* Median window. Must be odd and no larger than ULTRASONIC_MEDIAN_MAX. */
#define ULTRASONIC_MEDIAN_WINDOW 5U
#define ULTRASONIC_MEDIAN_MAX 5U

/* Consecutive failures before a sensor is declared faulty, and successes to clear. */
#define ULTRASONIC_FAULT_THRESHOLD 5U
#define ULTRASONIC_RECOVER_THRESHOLD 3U

/* Speed of sound used for the integer distance conversion, in m/s. */
#define ULTRASONIC_SOUND_SPEED_MS 343U

/* ------------------------------------------------------------------------- */
/* Obstacle avoidance                                                         */
/* ------------------------------------------------------------------------- */

/* Below this the direction is blocked outright. */
#define SAFETY_STOP_DISTANCE_MM 250U

/* Between stop and slow the allowed speed is scaled linearly. */
#define SAFETY_SLOW_DISTANCE_MM 700U

/* A blocked direction stays blocked until the obstacle is at least this far. */
#define SAFETY_RELEASE_DISTANCE_MM 350U

/* Side sensors start limiting turn-toward-obstacle below this. */
#define SAFETY_SIDE_LIMIT_DISTANCE_MM 300U

/* Side sensors stop limiting once the obstacle is beyond this. */
#define SAFETY_SIDE_RELEASE_DISTANCE_MM 420U

/* Turn authority still permitted toward a blocked side, in speed units. */
#define SAFETY_SIDE_MIN_TURN 150

/*
 * What to do with a direction whose sensor is faulty or has no valid reading.
 * FAIL_OPEN keeps the vehicle drivable, FAIL_SAFE blocks that direction.
 *
 * Default is FAIL_OPEN: with ultrasonic disabled by default, FAIL_SAFE would
 * make the vehicle immobile, and the operator plus the e-stop remain the
 * primary safety mechanism. Revisit once the sensors are trusted.
 */
#define SAFETY_FAIL_OPEN 0
#define SAFETY_FAIL_SAFE 1

#ifndef SAFETY_SENSOR_FAULT_POLICY
#define SAFETY_SENSOR_FAULT_POLICY SAFETY_FAIL_OPEN
#endif

/* ------------------------------------------------------------------------- */
/* Display                                                                    */
/* ------------------------------------------------------------------------- */

/* One page pushed per tick keeps the blocking I2C write bounded to ~3 ms. */
#define APP_DISPLAY_TICK_MS 50U

/* Automatic page cycling. */
#define APP_DISPLAY_PAGE_COUNT 5U
#define APP_DISPLAY_PAGE_DWELL_MS 2500U

/* Consecutive I2C failures after which the display is abandoned for good. */
#define APP_DISPLAY_FAIL_LIMIT 3U

/* Short enough that a missing panel cannot stall a control period for long. */
#define APP_DISPLAY_I2C_TIMEOUT_MS 20U

/* ------------------------------------------------------------------------- */
/* Compile-time checks                                                        */
/* ------------------------------------------------------------------------- */

_Static_assert(MOTOR_COUNT == 4U, "four fixed motors are assumed throughout");
_Static_assert(ULTRASONIC_COUNT == 4U, "four ultrasonic positions are assumed");

_Static_assert(SOFT_PWM_RESOLUTION > 0U, "PWM resolution must be non-zero");
_Static_assert(SOFT_PWM_RESOLUTION <= 1000U, "PWM resolution beyond the ISR budget");
_Static_assert(SOFT_PWM_FREQUENCY_HZ > 0U, "PWM frequency must be non-zero");

/* Leave at least 200 CPU cycles per software PWM interrupt. */
_Static_assert(APP_SYSCLK_HZ / SOFT_PWM_ISR_HZ >= 200UL,
               "software PWM interrupt rate leaves too few CPU cycles");

_Static_assert(MOTOR_SPEED_MAX > 0, "speed maximum must be positive");
_Static_assert(MOTOR_SPEED_MAX <= 32767, "speed maximum must fit in int16_t");
_Static_assert(MOTOR_ACCEL_STEP > 0, "acceleration step must be non-zero");
_Static_assert(MOTOR_DECEL_STEP > 0, "deceleration step must be non-zero");
_Static_assert(MOTOR_ACCEL_STEP <= MOTOR_SPEED_MAX, "acceleration step too large");
_Static_assert(MOTOR_DECEL_STEP <= MOTOR_SPEED_MAX, "deceleration step too large");
_Static_assert(MOTOR_MIN_START_PWM <= SOFT_PWM_RESOLUTION,
               "minimum start duty exceeds PWM resolution");

_Static_assert(CONTROL_PERIOD_MS > 0U, "control period must be non-zero");
_Static_assert(COMM_TIMEOUT_MS > CONTROL_PERIOD_MS,
               "communication timeout must outlast a control period");
_Static_assert(COMM_RECOVERY_FRAME_COUNT >= 1U,
               "at least one frame is needed to leave a latched stop");

/* The ring must survive a display flush without losing a frame. */
_Static_assert(COMM_RX_RING_SIZE >= 30U * 4U,
               "receive ring cannot hold several control frames");
_Static_assert((COMM_RX_RING_SIZE & (COMM_RX_RING_SIZE - 1U)) == 0U,
               "receive ring size must be a power of two");
_Static_assert(COMM_SEQ_FORWARD_WINDOW < COMM_SEQ_MODULUS / 2U,
               "sequence window must stay below half the modulus");

_Static_assert(SAFETY_STOP_DISTANCE_MM < SAFETY_SLOW_DISTANCE_MM,
               "stop distance must be nearer than the slow-down distance");
_Static_assert(SAFETY_RELEASE_DISTANCE_MM >= SAFETY_STOP_DISTANCE_MM,
               "release distance must not be nearer than the stop distance");
_Static_assert(SAFETY_RELEASE_DISTANCE_MM <= SAFETY_SLOW_DISTANCE_MM,
               "release distance beyond the slow-down band gives no hysteresis");
_Static_assert(SAFETY_SIDE_RELEASE_DISTANCE_MM > SAFETY_SIDE_LIMIT_DISTANCE_MM,
               "side release distance must exceed the side limit distance");
_Static_assert(SAFETY_SIDE_MIN_TURN >= 0 && SAFETY_SIDE_MIN_TURN <= MOTOR_SPEED_MAX,
               "residual turn authority out of range");

_Static_assert(ULTRASONIC_MEDIAN_WINDOW % 2U == 1U, "median window must be odd");
_Static_assert(ULTRASONIC_MEDIAN_WINDOW <= ULTRASONIC_MEDIAN_MAX,
               "median window exceeds the reserved sample storage");
_Static_assert(ULTRASONIC_MIN_PULSE_US < ULTRASONIC_MAX_PULSE_US,
               "ultrasonic pulse band is inverted");
_Static_assert(ULTRASONIC_MAX_PULSE_US < ULTRASONIC_FALL_TIMEOUT_US,
               "echo fall timeout must outlast the longest accepted pulse");
_Static_assert(ULTRASONIC_MIN_VALID_MM < ULTRASONIC_MAX_VALID_MM,
               "ultrasonic distance band is inverted");
_Static_assert(ULTRASONIC_FAULT_THRESHOLD > 0U && ULTRASONIC_RECOVER_THRESHOLD > 0U,
               "sensor fault thresholds must be non-zero");

_Static_assert(SAFETY_SENSOR_FAULT_POLICY == SAFETY_FAIL_OPEN ||
                   SAFETY_SENSOR_FAULT_POLICY == SAFETY_FAIL_SAFE,
               "unknown sensor fault policy");

_Static_assert(APP_DISPLAY_PAGE_COUNT > 0U, "display needs at least one page");
_Static_assert(APP_DISPLAY_TICK_MS > 0U, "display tick must be non-zero");

#endif /* APP_CONFIG_H */
