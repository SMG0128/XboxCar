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
