#include "control_report.h"

#include <stdio.h>
#include <string.h>

#include "debug_log.h"
#include "xbox_protocol.h"

/*
 * Used until the board layer installs the real map, and by the host tests that
 * only care about the numeric fields. The pin numbers are deliberately absent
 * rather than guessed: a log that invents PA0 would be worse than one that
 * admits it does not know yet.
 */
static const PwmChannelInfo kUnknownPwmMap[MOTOR_COUNT] = {
    {"FL", "TIM4_SOFTPWM", "SOFT_CH1", '?', 0U, '?', 0U, '?', 0U},
    {"RL", "TIM4_SOFTPWM", "SOFT_CH2", '?', 0U, '?', 0U, '?', 0U},
    {"FR", "TIM4_SOFTPWM", "SOFT_CH3", '?', 0U, '?', 0U, '?', 0U},
    {"RR", "TIM4_SOFTPWM", "SOFT_CH4", '?', 0U, '?', 0U, '?', 0U},
};

static const PwmChannelInfo *g_pwm_map = kUnknownPwmMap;

uint16_t ControlReport_SpeedToPercent(int16_t speed)
{
  int32_t magnitude = (speed < 0) ? -(int32_t)speed : (int32_t)speed;

  if (magnitude > (int32_t)MOTOR_SPEED_MAX)
  {
    magnitude = (int32_t)MOTOR_SPEED_MAX;
  }

  return (uint16_t)((magnitude * (int32_t)SOFT_PWM_RESOLUTION) /
                    (int32_t)MOTOR_SPEED_MAX);
}

bool ControlReport_HasXbox(const AppDebugState *state)
{
  return (state != NULL) && (state->xbox_connected != 0U);
}

void ControlReport_GetActualAxes(const AppDebugState *state, int16_t *up_down,
                                 int16_t *left_right)
{
  int32_t left = 0;
  int32_t right = 0;

  if (state != NULL)
  {
    left = state->actual_left;
    right = state->actual_right;
  }
  if (up_down != NULL)
  {
    *up_down = (int16_t)((left + right) / 2);
  }
  if (left_right != NULL)
  {
    *left_right = (int16_t)((left - right) / 2);
  }
}

/* Shared by both axes: a five character label, a colon, three digits. */
static void FormatAxis(char *out, uint16_t size, const char *label,
                       int16_t value)
{
  if (out == NULL || size == 0U)
  {
    return;
  }

  if (size < (CONTROL_REPORT_LINE_CHARS + 1U))
  {
    out[0] = '\0';
    return;
  }

  (void)snprintf(out, (size_t)size, "%-5s:%03u", label,
                 (unsigned)ControlReport_SpeedToPercent(value));
}

void ControlReport_FormatUpDown(char *out, uint16_t size, int16_t up_down)
{
  const char *label;

  if (up_down > 0)
  {
    label = "UP";
  }
  else if (up_down < 0)
  {
    label = "DOWN";
  }
  else
  {
    /* Centre stick: name both directions rather than implying one. */
    label = "UP/DN";
  }

  FormatAxis(out, size, label, up_down);
}

void ControlReport_FormatLeftRight(char *out, uint16_t size, int16_t left_right)
{
  const char *label;

  /* Positive steering is a right turn, matching the ESP32 mixer. */
  if (left_right > 0)
  {
    label = "RIGHT";
  }
  else if (left_right < 0)
  {
    label = "LEFT";
  }
  else
  {
    label = "LT/RT";
  }

  FormatAxis(out, size, label, left_right);
}

bool ControlReport_BuildDisplayLines(const AppDebugState *state, char *line1,
                                     uint16_t line1_size, char *line2,
                                     uint16_t line2_size)
{
  int16_t actual_up_down;
  int16_t actual_left_right;

  if (line1 == NULL || line2 == NULL || line1_size == 0U || line2_size == 0U)
  {
    return false;
  }

  if (!ControlReport_HasXbox(state))
  {
    line1[0] = '\0';
    line2[0] = '\0';
    return false;
  }

  ControlReport_GetActualAxes(state, &actual_up_down, &actual_left_right);
  ControlReport_FormatUpDown(line1, line1_size, actual_up_down);
  ControlReport_FormatLeftRight(line2, line2_size, actual_left_right);
  return true;
}

void ControlReport_Init(ControlReporter *reporter)
{
  if (reporter == NULL)
  {
    return;
  }
  memset(reporter, 0, sizeof(*reporter));
}

void ControlReport_SetPwmMap(const PwmChannelInfo *map)
{
  g_pwm_map = (map != NULL) ? map : kUnknownPwmMap;
}

const PwmChannelInfo *ControlReport_GetPwmMap(void)
{
  return g_pwm_map;
}

void ControlReport_LogBoot(uint32_t sysclk_hz, uint32_t pwm_carrier_hz,
                           uint32_t pwm_isr_hz)
{
#if XBOXCAR_DEBUG_LOG
  uint8_t index;

  DebugLog_Printf("[BOOT] XboxCar STM32 debug started\r\n");
  DebugLog_Printf("[BOOT] MCU=STM32F103C8T6 SYSCLK=%luHz\r\n",
                  (unsigned long)sysclk_hz);
  DebugLog_Printf("[BOOT] ESP_UART=USART1 remap PB6=TX PB7=RX %u 8N1\r\n",
                  (unsigned)DEBUG_UART_BAUD);
  DebugLog_Printf(
      "[BOOT] DEBUG_UART=USART1 shared TX-only PB6 %u 8N1 (do not drive PB7)\r\n",
      (unsigned)DEBUG_UART_BAUD);
  DebugLog_Printf("[BOOT] OLED_I2C=I2C1 remap PB8=SCL PB9=SDA addr=0x3C\r\n");
  DebugLog_Printf(
      "[BOOT] PROTO=v2 $XD %uB + v1 $XC %uB speed=+/-%d timeout=%ums\r\n",
      (unsigned)XBOX_FRAME_V2_LENGTH, (unsigned)XBOX_FRAME_V1_LENGTH,
      (int)MOTOR_SPEED_MAX, (unsigned)COMM_TIMEOUT_MS);

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    const PwmChannelInfo *info = &g_pwm_map[index];

    DebugLog_Printf(
        "[BOOT] PWM%u motor=%s pin=P%c%u timer=%s ch=%s in1=P%c%u in2=P%c%u\r\n",
        (unsigned)(index + 1U), info->wheel, info->port, (unsigned)info->pin,
        info->timer, info->channel, info->in1_port, (unsigned)info->in1_pin,
        info->in2_port, (unsigned)info->in2_pin);
  }

  DebugLog_Printf(
      "[BOOT] PWM outputs initialized arr=%u carrier=%luHz isr=%luHz\r\n",
      (unsigned)SOFT_PWM_RESOLUTION, (unsigned long)pwm_carrier_hz,
      (unsigned long)pwm_isr_hz);
#else
  (void)sysclk_hz;
  (void)pwm_carrier_hz;
  (void)pwm_isr_hz;
#endif
}

#if XBOXCAR_DEBUG_LOG

/* Total rejections seen by the protocol layer, across every reason. */
static uint32_t ProtocolErrorTotal(const AppDebugState *state)
{
  return state->crc_errors + state->format_errors + state->range_errors +
         state->sequence_errors;
}

static void LogControlLines(const AppDebugState *state)
{
  DebugLog_Printf("[CTRL] xbox=%u seq=%u age=%lums valid=%u state=%s proto=v%u\r\n",
                  (unsigned)state->xbox_connected,
                  (unsigned)state->last_sequence,
                  (unsigned long)state->control_age_ms,
                  (unsigned)state->frame_valid,
                  Diagnostics_ControlStateName(state->control_state),
                  (unsigned)state->protocol_version);

  if (state->xbox_connected == 0U)
  {
    DebugLog_Printf("[CTRL] xbox=0 reason=%s age=%lums\r\n",
                    Diagnostics_NoXboxReasonName(state->no_xbox_reason),
                    (unsigned long)state->control_age_ms);
    return;
  }

  DebugLog_Printf("[AXIS] raw_ud=%d raw_lr=%d\r\n", (int)state->raw_up_down,
                  (int)state->raw_left_right);

  DebugLog_Printf("[CMD] up_down=%s speed=%u left_right=%s speed=%u\r\n",
                  (state->up_down_speed > 0)
                      ? "UP"
                      : ((state->up_down_speed < 0) ? "DOWN" : "NEUTRAL"),
                  (unsigned)ControlReport_SpeedToPercent(state->up_down_speed),
                  (state->left_right_speed > 0)
                      ? "RIGHT"
                      : ((state->left_right_speed < 0) ? "LEFT" : "NEUTRAL"),
                  (unsigned)ControlReport_SpeedToPercent(
                      state->left_right_speed));
}

static void LogMixLine(const AppDebugState *state)
{
  DebugLog_Printf(
      "[MIX] FL=%d FR=%d RL=%d RR=%d\r\n",
      (int)ControlReport_SpeedToPercent(state->motor_speed[MOTOR_FRONT_LEFT]),
      (int)ControlReport_SpeedToPercent(state->motor_speed[MOTOR_FRONT_RIGHT]),
      (int)ControlReport_SpeedToPercent(state->motor_speed[MOTOR_REAR_LEFT]),
      (int)ControlReport_SpeedToPercent(state->motor_speed[MOTOR_REAR_RIGHT]));
}

#if XBOXCAR_PWM_LOG
static void LogPwmChannels(const AppDebugState *state)
{
  uint8_t index;

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    const PwmChannelInfo *info = &g_pwm_map[index];

    DebugLog_Printf(
        "[PWM%u] motor=%s pin=P%c%u timer=%s ch=%s dir=%s target=%u actual=%u "
        "ccr=%u arr=%u forced=%u\r\n",
        (unsigned)(index + 1U), info->wheel, info->port, (unsigned)info->pin,
        info->timer, info->channel,
        Diagnostics_MotorDirectionName(state->motor_dir[index]),
        (unsigned)ControlReport_SpeedToPercent(state->motor_target[index]),
        (unsigned)ControlReport_SpeedToPercent(state->motor_speed[index]),
        (unsigned)state->motor_pwm[index], (unsigned)SOFT_PWM_RESOLUTION,
        (unsigned)state->safety_forced_zero);
  }
}
#endif /* XBOXCAR_PWM_LOG */

static void LogSafetyStop(const AppDebugState *state)
{
  DebugLog_Printf("[SAFE] all motor targets forced to zero reason=%s\r\n",
                  Diagnostics_NoXboxReasonName(state->no_xbox_reason));
  DebugLog_Printf("[SAFE] PWM1=%u PWM2=%u PWM3=%u PWM4=%u\r\n",
                  (unsigned)state->motor_pwm[0], (unsigned)state->motor_pwm[1],
                  (unsigned)state->motor_pwm[2], (unsigned)state->motor_pwm[3]);
}

/*
 * A duty step this size or larger is worth a line of its own. Smaller steps are
 * the ramp doing its job and would print on every control period.
 */
static bool PwmChanged(const ControlReporter *reporter,
                       const AppDebugState *state)
{
  uint8_t index;

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    int32_t previous = (int32_t)reporter->last_pwm[index];
    int32_t current = (int32_t)state->motor_pwm[index];
    int32_t delta = current - previous;

    if (delta < 0)
    {
      delta = -delta;
    }
    if (delta >= (int32_t)DEBUG_LOG_PWM_CHANGE_STEP)
    {
      return true;
    }
    if (reporter->last_dir[index] != state->motor_dir[index])
    {
      return true;
    }
  }

  return false;
}

static void RememberBaseline(ControlReporter *reporter,
                             const AppDebugState *state, uint32_t now_ms)
{
  uint8_t index;

  reporter->have_previous = true;
  reporter->last_control_state = state->control_state;
  reporter->last_xbox_connected = state->xbox_connected;
  reporter->last_no_xbox_reason = state->no_xbox_reason;
  reporter->last_snapshot_ms = now_ms;

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    reporter->last_pwm[index] = state->motor_pwm[index];
    reporter->last_dir[index] = state->motor_dir[index];
  }
}

/*
 * Protocol rejections are the one event a hostile or broken link can generate
 * without limit, so they get their own rate limiter and report how many lines
 * were suppressed rather than pretending the burst was a single error.
 */
static void ReportProtocolErrors(ControlReporter *reporter,
                                 const AppDebugState *state, uint32_t now_ms)
{
  const uint32_t total = ProtocolErrorTotal(state);
  uint32_t added;

  if (!reporter->have_proto_error_baseline)
  {
    reporter->have_proto_error_baseline = true;
    reporter->last_proto_error_total = total;
    reporter->last_proto_error_ms = now_ms;
    return;
  }

  if (total == reporter->last_proto_error_total)
  {
    return;
  }

  added = total - reporter->last_proto_error_total;
  reporter->last_proto_error_total = total;

  if ((uint32_t)(now_ms - reporter->last_proto_error_ms) <
      DEBUG_LOG_PROTO_ERROR_MS)
  {
    reporter->suppressed_proto_errors += added;
    return;
  }

  reporter->last_proto_error_ms = now_ms;
  DebugLog_Printf(
      "[PROTO] invalid frame reason=%s new=%lu suppressed=%lu "
      "crc=%lu fmt=%lu range=%lu seq=%lu\r\n",
      XboxProtocol_RejectReasonName(state->last_reject_reason),
      (unsigned long)added, (unsigned long)reporter->suppressed_proto_errors,
      (unsigned long)state->crc_errors, (unsigned long)state->format_errors,
      (unsigned long)state->range_errors, (unsigned long)state->sequence_errors);
  reporter->suppressed_proto_errors = 0U;
}

#endif /* XBOXCAR_DEBUG_LOG */

void ControlReport_Update(ControlReporter *reporter, const AppDebugState *state,
                          uint32_t now_ms)
{
#if XBOXCAR_DEBUG_LOG
  bool state_changed;
  bool presence_changed;
  bool periodic;
  bool emit;

  if (reporter == NULL || state == NULL)
  {
    return;
  }

  ReportProtocolErrors(reporter, state, now_ms);

  state_changed = !reporter->have_previous ||
                  reporter->last_control_state != state->control_state;
  presence_changed = !reporter->have_previous ||
                     reporter->last_xbox_connected != state->xbox_connected ||
                     reporter->last_no_xbox_reason != state->no_xbox_reason;
  periodic = !reporter->have_previous ||
             (uint32_t)(now_ms - reporter->last_snapshot_ms) >=
                 DEBUG_LOG_SNAPSHOT_MS;

  emit = state_changed || presence_changed || periodic ||
         PwmChanged(reporter, state);
  if (!emit)
  {
    return;
  }

  LogControlLines(state);

  if (state->xbox_connected != 0U)
  {
    LogMixLine(state);
  }
  else if (state_changed || presence_changed)
  {
    /*
     * Only on the transition. Repeating the safe-stop banner every 250 ms while
     * parked would bury the line that says why it happened.
     */
    LogSafetyStop(state);
  }

#if XBOXCAR_PWM_LOG
  LogPwmChannels(state);
#endif

  RememberBaseline(reporter, state, now_ms);
#else
  (void)reporter;
  (void)state;
  (void)now_ms;
#endif /* XBOXCAR_DEBUG_LOG */
}
