#ifndef CONTROL_REPORT_H
#define CONTROL_REPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Presentation for the single runtime snapshot.
 *
 * Both the OLED and the USB-TTL log are generated here, from the same
 * AppDebugState and through the same scaling helpers. That is the point of the
 * module: before it existed the display derived its numbers from the post-ramp
 * motor pair while the log would have derived its own, and the two could
 * disagree about what the operator was doing. Now a value on the panel and the
 * matching value in the log come from one field and one conversion.
 *
 * Portable: no STM32 HAL, no I2C, no UART. The host tests assert on the exact
 * strings this module produces, which is what makes the display format and the
 * log format testable without hardware.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_types.h"
#include "diagnostics.h"

/* "RIGHT:100" plus the terminator; every line is exactly 9 characters. */
#define CONTROL_REPORT_LINE_CHARS 9U
#define CONTROL_REPORT_LINE_MAX 12U

/* Text shown while there is no usable controller input. */
#define CONTROL_REPORT_NO_XBOX_TEXT "No Xbox"

/*
 * One software PWM channel, described in the terms the PCB uses.
 *
 * The board layer builds this from the same board_pins.h X-macro list that
 * configures the GPIOs, so the pin printed in the log cannot drift away from
 * the pin being driven. The host tests supply their own table.
 */
typedef struct {
  const char *wheel;   /* "FL", "RL", "FR", "RR" */
  const char *timer;   /* "TIM4_SOFTPWM" */
  const char *channel; /* "SOFT_CH1" */
  char port;           /* 'A' or 'B' */
  uint8_t pin;         /* 0..15 */
  char in1_port;
  uint8_t in1_pin;
  char in2_port;
  uint8_t in2_pin;
} PwmChannelInfo;

/*
 * Emission policy state. Separated from the snapshot so that the decision to
 * print is never confused with the data being printed.
 */
typedef struct {
  bool have_previous;

  uint8_t last_control_state;
  uint8_t last_xbox_connected;
  uint8_t last_no_xbox_reason;
  uint16_t last_pwm[MOTOR_COUNT];
  uint8_t last_dir[MOTOR_COUNT];

  uint32_t last_snapshot_ms;
  uint32_t last_proto_error_ms;
  uint32_t last_proto_error_total;
  uint32_t suppressed_proto_errors;
  bool have_proto_error_baseline;
} ControlReporter;

/* ------------------------------------------------------------------------- */
/* Scaling                                                                    */
/* ------------------------------------------------------------------------- */

/*
 * Converts a signed speed in the protocol domain to the 0..SOFT_PWM_RESOLUTION
 * display and duty domain. Deliberately the same integer expression the motor
 * module uses, so an OLED reading of 072 and a log line reading ccr=72 describe
 * the same number rather than two roundings of it.
 */
uint16_t ControlReport_SpeedToPercent(int16_t speed);

/* ------------------------------------------------------------------------- */
/* Display                                                                    */
/* ------------------------------------------------------------------------- */

/* True when the snapshot says there is usable controller input. */
bool ControlReport_HasXbox(const AppDebugState *state);

/*
 * Decomposes the post-limiter, post-ramp wheel speeds into the two axes shown
 * on the OLED. These are actual commanded motor speeds, not Xbox requests.
 */
void ControlReport_GetActualAxes(const AppDebugState *state, int16_t *up_down,
                                 int16_t *left_right);

/*
 * Fixed width "UP   :072" / "DOWN :048" / "UP/DN:000" for the forward axis, and
 * "LEFT :035" / "RIGHT:021" / "LT/RT:000" for the steering axis.
 *
 * Fixed width matters on this panel: the renderer draws glyphs without clearing
 * behind them, so a line that shrinks would leave the old trailing character on
 * screen. Every line is CONTROL_REPORT_LINE_CHARS characters.
 */
void ControlReport_FormatUpDown(char *out, uint16_t size, int16_t up_down);
void ControlReport_FormatLeftRight(char *out, uint16_t size, int16_t left_right);

/*
 * Both display lines for the current snapshot, derived from the actual motor
 * speeds after ultrasonic limiting and acceleration/deceleration ramping.
 * Returns false when there is no usable input, in which case the caller shows
 * CONTROL_REPORT_NO_XBOX_TEXT and the line buffers are set to empty strings.
 */
bool ControlReport_BuildDisplayLines(const AppDebugState *state, char *line1,
                                     uint16_t line1_size, char *line2,
                                     uint16_t line2_size);

/* ------------------------------------------------------------------------- */
/* Logging                                                                    */
/* ------------------------------------------------------------------------- */

/* Clears the change-detection baseline and the rate limiter. */
void ControlReport_Init(ControlReporter *reporter);

/*
 * Installs the PWM channel description table, MOTOR_COUNT entries indexed by
 * MotorIndex. Passing NULL restores a neutral built-in table, which keeps the
 * log readable if the board layer has not run yet.
 */
void ControlReport_SetPwmMap(const PwmChannelInfo *map);

/* The installed table, never NULL. */
const PwmChannelInfo *ControlReport_GetPwmMap(void);

/*
 * Emits the one-off banner describing the MCU, the two UART roles, the I2C
 * display and every PWM channel. Called once, after the peripherals are up.
 */
void ControlReport_LogBoot(uint32_t sysclk_hz, uint32_t pwm_carrier_hz,
                           uint32_t pwm_isr_hz);

/*
 * Decides what to print for this control period and prints it.
 *
 * Immediate on a state, presence or PWM change; otherwise a full four-channel
 * snapshot every DEBUG_LOG_SNAPSHOT_MS. Protocol rejections are reported at
 * most once per DEBUG_LOG_PROTO_ERROR_MS with a count of what was suppressed,
 * so a noisy line cannot flood the transport.
 */
void ControlReport_Update(ControlReporter *reporter, const AppDebugState *state,
                          uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_REPORT_H */
