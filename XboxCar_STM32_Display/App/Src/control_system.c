#include "control_system.h"

#include <string.h>

void ControlSystem_Init(ControlSystem *system)
{
  if (system == NULL)
  {
    return;
  }

  memset(system, 0, sizeof(*system));

  XboxProtocol_Init(&system->protocol);
  Motor_Init(&system->motor);
  Safety_Init(&system->safety);
  Diagnostics_Init(&system->debug);

  system->state = CONTROL_STATE_STARTUP_SAFE;
  system->previous_state = CONTROL_STATE_STARTUP_SAFE;

  /*
   * Nothing drives until a frame has been accepted. This is the power-on
   * contract: an unattended vehicle must not move because a floating input
   * happened to look like data.
   */
  Motor_SetOutputEnabled(&system->motor, false);
}

void ControlSystem_PushRxByte(ControlSystem *system, uint8_t byte)
{
  if (system == NULL)
  {
    return;
  }
  XboxProtocol_PushByte(&system->protocol, byte);
}

void ControlSystem_ReportInternalFault(ControlSystem *system, uint16_t fault_code,
                                       uint32_t internal_flag, uint32_t now_ms)
{
  if (system == NULL)
  {
    return;
  }

  system->internal_faults |= internal_flag;
  Diagnostics_RecordFault(&system->debug, fault_code, internal_flag, now_ms);

  /* Cut the output immediately rather than waiting for the next period. */
  Motor_SetOutputEnabled(&system->motor, false);
}

/* Handles one accepted frame. */
static void AcceptFrame(ControlSystem *system, const XboxControlFrame *frame,
                        uint32_t now_ms)
{
  system->last_accepted_ms = now_ms;
  system->have_accepted_frame = true;
  system->rejects_since_accept = 0U;

  system->debug.last_command = frame->command;
  system->debug.last_sequence = frame->sequence;

  /*
   * Presence, the raw sticks and the protocol version describe the link rather
   * than the request, so they are recorded even for a frame that commands a
   * stop. Without that, an emergency or a NoInput frame would blank the display
   * instead of showing a connected controller at rest.
   */
  system->frame_connected = frame->connected;
  system->protocol_version = frame->version;
  system->raw_up_down = frame->raw_up_down;
  system->raw_left_right = frame->raw_left_right;

  if (frame->emergency)
  {
    /*
     * Latch, and discard any recovery progress. An emergency frame arriving
     * mid-recovery means the operator is still holding the button.
     */
    if (!system->emergency_locked)
    {
      Diagnostics_RecordFault(&system->debug, (uint16_t)FAULT_EMERGENCY_STOP, 0U,
                              now_ms);
    }
    system->emergency_locked = true;
    system->recovery_count = 0U;
    system->requested_left = 0;
    system->requested_right = 0;
    system->requested_up_down = 0;
    system->requested_left_right = 0;
    return;
  }

  if (system->emergency_locked)
  {
    /*
     * Recovery run. Only accepted, non-emergency frames count. Duplicates and
     * rejected frames are handled by the caller and never reach here, so the
     * run really is consecutive good frames.
     */
    if (system->recovery_count < 0xFFU)
    {
      ++system->recovery_count;
    }

    if (system->recovery_count >= COMM_RECOVERY_FRAME_COUNT)
    {
      system->emergency_locked = false;
      system->recovery_count = 0U;
    }

    /*
     * While still latched the request stays zero, so the vehicle resumes from
     * standstill rather than jumping to whatever the stick was doing.
     */
    system->requested_left = 0;
    system->requested_right = 0;
    system->requested_up_down = 0;
    system->requested_left_right = 0;
    return;
  }

  system->recovery_count = 0U;
  system->requested_left = frame->left;
  system->requested_right = frame->right;
  system->requested_up_down = frame->up_down;
  system->requested_left_right = frame->left_right;
}

/* Drains the receive ring, applying each outcome. */
static void ProcessIncoming(ControlSystem *system, uint32_t now_ms)
{
  XboxControlFrame frame;
  XboxPollResult result;
  uint8_t guard = 0U;

  /*
   * Bounded so a flood of bytes cannot monopolise a control period. The ring
   * holds a few frames and the emitter runs at 50 Hz, so this ceiling is never
   * reached in normal operation.
   */
  while (guard < 8U)
  {
    result = XboxProtocol_Poll(&system->protocol, &frame);
    if (result == XBOX_POLL_IDLE)
    {
      break;
    }
    ++guard;

    switch (result)
    {
      case XBOX_POLL_FRAME:
        AcceptFrame(system, &frame, now_ms);
        break;

      case XBOX_POLL_DUPLICATE:
        /*
         * A repeat carries no new information. It must not refresh the
         * watchdog and must not earn recovery credit, otherwise a stuck
         * emitter would look like a healthy link.
         */
        break;

      case XBOX_POLL_ERROR:
      default:
        /* Any rejected frame breaks a recovery run in progress. */
        system->recovery_count = 0U;
        if (system->rejects_since_accept < 0xFFFFU)
        {
          ++system->rejects_since_accept;
        }
        break;
    }
  }
}

/*
 * Why there is no usable controller input, or NONE.
 *
 * Evaluated in the same order as the control priority chain so the reason on
 * the display and in the log names the condition that actually stopped the
 * vehicle, not a lesser one that happens to also be true.
 */
static NoXboxReason DecideNoXboxReason(const ControlSystem *system,
                                       ControlState state)
{
  if (state == CONTROL_STATE_INTERNAL_FAULT)
  {
    return NO_XBOX_REASON_INTERNAL_FAULT;
  }

  if (state == CONTROL_STATE_EMERGENCY_LOCKED)
  {
    return NO_XBOX_REASON_EMERGENCY_LOCKED;
  }

  if (state == CONTROL_STATE_STARTUP_SAFE)
  {
    return NO_XBOX_REASON_NO_FRAME_YET;
  }

  if (state == CONTROL_STATE_COMM_TIMEOUT)
  {
    /*
     * A link delivering bytes that never validate looks identical to a dead
     * link from the watchdog's point of view, but they need different repairs:
     * one is a wiring or baud fault, the other is a cut wire or a stopped
     * emitter.
     */
    if (system->rejects_since_accept >= COMM_FRAME_ERROR_LIMIT)
    {
      return NO_XBOX_REASON_FRAME_ERRORS;
    }
    return NO_XBOX_REASON_CONTROL_TIMEOUT;
  }

  if (!system->frame_connected)
  {
    return NO_XBOX_REASON_ESP_REPORTED_DISCONNECTED;
  }

  return NO_XBOX_REASON_NONE;
}

/* Chooses the state for this period, highest priority first. */
static ControlState DecideState(const ControlSystem *system, uint32_t now_ms)
{
  if (system->internal_faults != 0U)
  {
    return CONTROL_STATE_INTERNAL_FAULT;
  }

  if (system->emergency_locked)
  {
    return CONTROL_STATE_EMERGENCY_LOCKED;
  }

  if (!system->have_accepted_frame)
  {
    return CONTROL_STATE_STARTUP_SAFE;
  }

  if ((uint32_t)(now_ms - system->last_accepted_ms) >= COMM_TIMEOUT_MS)
  {
    return CONTROL_STATE_COMM_TIMEOUT;
  }

  return CONTROL_STATE_ONLINE;
}

static void PublishSnapshot(ControlSystem *system, uint32_t now_ms,
                            const SensorSnapshot *sensors,
                            const SafetyOutput *limited)
{
  AppDebugState *debug = &system->debug;
  XboxProtocolStats stats;
  NoXboxReason reason;
  bool has_xbox;
  uint8_t index;

  debug->uptime_ms = now_ms;

  XboxProtocol_GetStats(&system->protocol, &stats);
  debug->uart_rx_bytes = stats.rx_bytes;
  debug->valid_frames = stats.valid_frames;
  debug->crc_errors = stats.crc_errors;
  debug->format_errors = stats.format_errors;
  debug->range_errors = stats.range_errors;
  debug->sequence_errors = stats.sequence_errors;
  debug->duplicate_frames = stats.duplicate_frames;
  debug->rx_overflows = stats.rx_overflows;
  debug->v1_frames = stats.v1_frames;
  debug->v2_frames = stats.v2_frames;
  debug->last_reject_reason = XboxProtocol_GetLastRejectReason(&system->protocol);

  reason = DecideNoXboxReason(system, system->state);
  has_xbox = (reason == NO_XBOX_REASON_NONE);

  debug->no_xbox_reason = (uint8_t)reason;
  debug->xbox_connected = has_xbox ? 1U : 0U;
  debug->frame_valid = (system->state == CONTROL_STATE_ONLINE) ? 1U : 0U;
  debug->protocol_version = system->protocol_version;
  debug->last_rx_ms = system->last_accepted_ms;
  debug->control_age_ms =
      system->have_accepted_frame
          ? (uint32_t)(now_ms - system->last_accepted_ms)
          : now_ms;

  /*
   * The operator axes are published only while the input is usable. Holding the
   * last stick position through a timeout would put a speed on the OLED that no
   * longer commands anything, which is exactly the failure the bring-up
   * requirement calls out.
   */
  if (has_xbox)
  {
    debug->up_down_speed = system->requested_up_down;
    debug->left_right_speed = system->requested_left_right;
    debug->raw_up_down = system->raw_up_down;
    debug->raw_left_right = system->raw_left_right;
  }
  else
  {
    debug->up_down_speed = 0;
    debug->left_right_speed = 0;
    debug->raw_up_down = 0;
    debug->raw_left_right = 0;
  }

  debug->requested_left = system->requested_left;
  debug->requested_right = system->requested_right;
  debug->limited_left = limited->limited_left;
  debug->limited_right = limited->limited_right;
  debug->actual_left = Motor_GetSpeed(&system->motor, MOTOR_FRONT_LEFT);
  debug->actual_right = Motor_GetSpeed(&system->motor, MOTOR_FRONT_RIGHT);

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    MotorOutput output;

    debug->motor_speed[index] = Motor_GetSpeed(&system->motor, index);
    debug->motor_target[index] = Motor_GetTarget(&system->motor, index);
    (void)Motor_GetOutput(&system->motor, index, &output);
    debug->motor_pwm[index] = output.duty;

    /*
     * Taken from the pin pair rather than from the sign of the speed, so the
     * log reports the direction the TB6612 is actually being told to drive,
     * inversion included. That is the value worth checking against a wheel.
     */
    if (output.in1 && !output.in2)
    {
      debug->motor_dir[index] = (uint8_t)MOTOR_DIR_FORWARD;
    }
    else if (!output.in1 && output.in2)
    {
      debug->motor_dir[index] = (uint8_t)MOTOR_DIR_REVERSE;
    }
    else
    {
      debug->motor_dir[index] = (uint8_t)MOTOR_DIR_STOP;
    }
  }

  /*
   * True when a rule above the operator zeroed the output: any non-online
   * state, or an obstacle limit that took a non-zero request down to nothing.
   */
  debug->safety_forced_zero =
      ((system->state != CONTROL_STATE_ONLINE) ||
       ((limited->limited_left == 0) && (limited->limited_right == 0) &&
        ((system->requested_left != 0) || (system->requested_right != 0))))
          ? 1U
          : 0U;

  if (sensors != NULL)
  {
    for (index = 0U; index < ULTRASONIC_COUNT; ++index)
    {
      debug->ultrasonic_raw_mm[index] = sensors->raw_mm[index];
      debug->ultrasonic_filtered_mm[index] = sensors->distance_mm[index];
      debug->ultrasonic_timeouts[index] = sensors->timeouts[index];
    }
    debug->ultrasonic_valid_mask = sensors->valid_mask;
    debug->ultrasonic_fault_mask = sensors->fault_mask;
  }
  else
  {
    memset(debug->ultrasonic_raw_mm, 0, sizeof(debug->ultrasonic_raw_mm));
    memset(debug->ultrasonic_filtered_mm, 0,
           sizeof(debug->ultrasonic_filtered_mm));
    memset(debug->ultrasonic_timeouts, 0, sizeof(debug->ultrasonic_timeouts));
    debug->ultrasonic_valid_mask = 0U;
    debug->ultrasonic_fault_mask = 0U;
  }

  debug->control_state = (uint8_t)system->state;
  debug->communication_online = (system->state == CONTROL_STATE_ONLINE) ? 1U : 0U;
  debug->emergency_locked = system->emergency_locked ? 1U : 0U;
  debug->obstacle_flags = limited->obstacle_flags;
  debug->limit_reason = limited->limit_reason;
  debug->motor_output_enabled = Motor_IsOutputEnabled(&system->motor) ? 1U : 0U;
  debug->internal_faults = system->internal_faults;

  debug->recovery_frames_remaining =
      system->emergency_locked
          ? (uint8_t)(COMM_RECOVERY_FRAME_COUNT - system->recovery_count)
          : 0U;
}

void ControlSystem_Update(ControlSystem *system, uint32_t now_ms,
                          const SensorSnapshot *sensors)
{
  SafetyOutput limited;
  ControlState state;
  bool drive;

  if (system == NULL)
  {
    return;
  }

  memset(&limited, 0, sizeof(limited));

  ProcessIncoming(system, now_ms);

  state = DecideState(system, now_ms);
  system->state = state;

  /* Count each entry into the timeout state, not each period spent there. */
  if (state == CONTROL_STATE_COMM_TIMEOUT &&
      system->previous_state != CONTROL_STATE_COMM_TIMEOUT)
  {
    ++system->debug.communication_timeouts;
    Diagnostics_RecordFault(&system->debug, (uint16_t)FAULT_COMM_TIMEOUT, 0U,
                            now_ms);

    /*
     * The link is gone. Drop the sequence baseline so that whatever comes back,
     * including an ESP32 that restarted at sequence 0000, is accepted at once
     * instead of being mistaken for stale traffic.
     */
    XboxProtocol_ResetSequence(&system->protocol);

    system->requested_left = 0;
    system->requested_right = 0;
    system->requested_up_down = 0;
    system->requested_left_right = 0;
  }

  /*
   * One predicate decides whether the vehicle may move, and the same predicate
   * decides what the display and the log say. An ESP32 that reports the
   * controller as absent stops the vehicle exactly as hard as a timeout does:
   * frames are still arriving, but none of them carry an operator.
   */
  drive = (state == CONTROL_STATE_ONLINE) && system->frame_connected;

  if (drive)
  {
#if APP_FEATURE_ULTRASONIC
    SafetyInput input;
    uint8_t index;

    input.requested_left = system->requested_left;
    input.requested_right = system->requested_right;
    if (sensors != NULL)
    {
      for (index = 0U; index < ULTRASONIC_COUNT; ++index)
      {
        input.distance_mm[index] = sensors->distance_mm[index];
      }
      input.sensor_valid_mask = sensors->valid_mask;
      input.sensor_fault_mask = sensors->fault_mask;
    }
    else
    {
      memset(input.distance_mm, 0, sizeof(input.distance_mm));
      input.sensor_valid_mask = 0U;
      input.sensor_fault_mask = 0U;
    }

    Safety_Apply(&system->safety, &input, &limited);
#else
    /* The 5 V ECHO inputs are not trusted until level shifting is fitted. */
    limited.limited_left = system->requested_left;
    limited.limited_right = system->requested_right;
#endif

    Motor_SetOutputEnabled(&system->motor, true);
    Motor_SetTargets(&system->motor, limited.limited_left, limited.limited_right);
  }
  else
  {
    /*
     * Every non-driving state stops the vehicle without a deceleration curve.
     * Disabling the output forces the speeds to zero in the same call, which is
     * what "immediately" has to mean for a timeout, a disconnect or an
     * emergency. The previous frame's speeds are discarded rather than held.
     */
    Motor_SetOutputEnabled(&system->motor, false);
    system->requested_up_down = 0;
    system->requested_left_right = 0;
    system->safety.front_blocked = false;
    system->safety.rear_blocked = false;
    system->safety.left_limited = false;
    system->safety.right_limited = false;
  }

  Motor_Update(&system->motor);

  /*
   * Integrity check after the ramp. A speed or hold outside its permitted range
   * means memory has been corrupted, and continuing to drive on it would be
   * worse than stopping.
   */
  if (!Motor_Validate(&system->motor))
  {
    ControlSystem_ReportInternalFault(system,
                                      (uint16_t)FAULT_INTERNAL_STATE_CORRUPT,
                                      INTERNAL_FAULT_STATE_CORRUPT, now_ms);
    system->state = CONTROL_STATE_INTERNAL_FAULT;
  }

  PublishSnapshot(system, now_ms, sensors, &limited);
  system->previous_state = system->state;
}

ControlState ControlSystem_GetState(const ControlSystem *system)
{
  if (system == NULL)
  {
    return CONTROL_STATE_INTERNAL_FAULT;
  }
  return system->state;
}

const AppDebugState *ControlSystem_GetDebugState(const ControlSystem *system)
{
  if (system == NULL)
  {
    return NULL;
  }
  return &system->debug;
}

bool ControlSystem_IsOnline(const ControlSystem *system)
{
  return (system != NULL) && (system->state == CONTROL_STATE_ONLINE);
}

bool ControlSystem_IsEmergencyLocked(const ControlSystem *system)
{
  return (system != NULL) && system->emergency_locked;
}
