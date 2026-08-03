#include "diagnostics.h"

#include <string.h>

void Diagnostics_Init(AppDebugState *state)
{
  if (state == NULL)
  {
    return;
  }
  memset(state, 0, sizeof(*state));
  state->control_state = (uint8_t)CONTROL_STATE_STARTUP_SAFE;
  state->last_fault_code = (uint16_t)FAULT_NONE;
  state->no_xbox_reason = (uint8_t)NO_XBOX_REASON_NO_FRAME_YET;
}

const char *Diagnostics_NoXboxReasonName(uint8_t reason)
{
  switch (reason)
  {
    case NO_XBOX_REASON_NONE:
      return "NONE";
    case NO_XBOX_REASON_INTERNAL_FAULT:
      return "INTERNAL_FAULT";
    case NO_XBOX_REASON_EMERGENCY_LOCKED:
      return "EMERGENCY_LOCKED";
    case NO_XBOX_REASON_NO_FRAME_YET:
      return "NO_FRAME_YET";
    case NO_XBOX_REASON_CONTROL_TIMEOUT:
      return "CONTROL_TIMEOUT";
    case NO_XBOX_REASON_ESP_REPORTED_DISCONNECTED:
      return "ESP_REPORTED_DISCONNECTED";
    case NO_XBOX_REASON_FRAME_ERRORS:
      return "FRAME_ERRORS";
    default:
      return "UNKNOWN";
  }
}

const char *Diagnostics_ControlStateName(uint8_t state)
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
      return "EMERGENCY_LOCKED";
    case CONTROL_STATE_INTERNAL_FAULT:
      return "INTERNAL_FAULT";
    default:
      return "UNKNOWN";
  }
}

const char *Diagnostics_MotorDirectionName(uint8_t direction)
{
  switch (direction)
  {
    case MOTOR_DIR_FORWARD:
      return "FWD";
    case MOTOR_DIR_REVERSE:
      return "REV";
    case MOTOR_DIR_STOP:
    default:
      return "OFF";
  }
}

void Diagnostics_RecordFault(AppDebugState *state, uint16_t fault_code,
                             uint32_t internal_flag, uint32_t now_ms)
{
  if (state == NULL)
  {
    return;
  }

  state->last_fault_code = fault_code;
  state->last_fault_time_ms = now_ms;

  /*
   * Internal faults accumulate rather than replace: a latched vehicle should
   * still show every reason it latched, not only the newest.
   */
  state->internal_faults |= internal_flag;
}
