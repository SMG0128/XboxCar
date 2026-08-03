#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The single runtime state snapshot.
 *
 * Exactly one module writes it: the control system, once per control period,
 * from values it already owns. Everything else reads. The display, the tests
 * and any future debug channel all consume this structure rather than reaching
 * into the protocol, motor or sensor internals.
 *
 * Deliberately free of text formatting. Turning numbers into strings is the
 * display's job, and doing it here would drag stdio into the control path.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_types.h"

typedef struct {
  uint32_t uptime_ms;

  /* Communication counters, cumulative since reset. */
  uint32_t uart_rx_bytes;
  uint32_t valid_frames;
  uint32_t crc_errors;
  uint32_t format_errors;
  uint32_t range_errors;
  uint32_t sequence_errors;
  uint32_t duplicate_frames;
  uint32_t rx_overflows;
  uint32_t communication_timeouts;
  uint32_t v1_frames;
  uint32_t v2_frames;

  /* ----------------------------------------------------------------- */
  /* Controller presence and link health                                */
  /* ----------------------------------------------------------------- */

  /*
   * The single answer to "is there usable Xbox input right now". The display,
   * the motor arbitration and the log all read this one field, which is what
   * stops the three from disagreeing.
   */
  uint8_t xbox_connected;
  /* Set while an accepted frame is inside the timeout window. */
  uint8_t frame_valid;
  /* NoXboxReason. NONE only while xbox_connected is true. */
  uint8_t no_xbox_reason;
  /* Frame format of the last accepted frame: 1 or 2, 0 before the first. */
  uint8_t protocol_version;
  /* Reason code of the most recent protocol rejection (XboxRejectReason). */
  uint8_t last_reject_reason;

  /* Timestamp of the last accepted frame, and its age at snapshot time. */
  uint32_t last_rx_ms;
  uint32_t control_age_ms;

  /* ----------------------------------------------------------------- */
  /* Operator input, exactly as received                                */
  /* ----------------------------------------------------------------- */

  /* Raw stick counts from the controller, vehicle sign convention. */
  int16_t raw_up_down;
  int16_t raw_left_right;

  /*
   * The two control dimensions, -MOTOR_SPEED_MAX..+MOTOR_SPEED_MAX.
   * up_down_speed > 0 is forward, left_right_speed > 0 steers right.
   * Zeroed the instant the link is not usable, so the display can never show
   * a stale stick position.
   */
  int16_t up_down_speed;
  int16_t left_right_speed;

  /* The three stages of the speed pipeline, for comparison at a glance. */
  int16_t requested_left;  /* straight from the accepted frame */
  int16_t requested_right;
  int16_t limited_left; /* after obstacle limiting */
  int16_t limited_right;
  int16_t actual_left; /* after the ramp */
  int16_t actual_right;

  int16_t motor_target[MOTOR_COUNT]; /* signed, before the ramp */
  int16_t motor_speed[MOTOR_COUNT];  /* signed, after the ramp */
  uint16_t motor_pwm[MOTOR_COUNT];   /* duty actually programmed, 0..SOFT_PWM_RESOLUTION */
  uint8_t motor_dir[MOTOR_COUNT];    /* MotorDirection, after inversion */

  /* Set for the periods in which a safety rule forced every target to zero. */
  uint8_t safety_forced_zero;

  uint16_t ultrasonic_raw_mm[ULTRASONIC_COUNT];
  uint16_t ultrasonic_filtered_mm[ULTRASONIC_COUNT];
  uint8_t ultrasonic_valid_mask;
  uint8_t ultrasonic_fault_mask;
  uint32_t ultrasonic_timeouts[ULTRASONIC_COUNT];

  uint8_t control_state; /* ControlState */
  uint8_t communication_online;
  uint8_t emergency_locked;
  uint8_t obstacle_flags;
  uint8_t limit_reason;
  uint8_t motor_output_enabled;

  /* Frames still needed to leave a latched emergency stop. */
  uint8_t recovery_frames_remaining;

  /* Last accepted frame's action code and sequence number. */
  uint8_t last_command;
  uint16_t last_sequence;

  uint32_t motor_faults;
  uint32_t internal_faults;

  uint16_t last_fault_code; /* FaultCode */
  uint32_t last_fault_time_ms;
} AppDebugState;

/* Zeroes the snapshot. */
void Diagnostics_Init(AppDebugState *state);

/*
 * Records a fault. Keeps the most recent one for display and ORs the internal
 * flag set so a transient fault is still visible after it clears.
 */
void Diagnostics_RecordFault(AppDebugState *state, uint16_t fault_code,
                             uint32_t internal_flag, uint32_t now_ms);

/* Stable short name for a NoXboxReason, shared by the log and the tests. */
const char *Diagnostics_NoXboxReasonName(uint8_t reason);

/* Stable short name for a ControlState. */
const char *Diagnostics_ControlStateName(uint8_t state);

/* Stable short name for a MotorDirection. */
const char *Diagnostics_MotorDirectionName(uint8_t direction);

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSTICS_H */
