#include <string.h>

#include "control_report.h"
#include "debug_log.h"
#include "test_framework.h"

static size_t DrainLog(char *out, size_t capacity)
{
  uint8_t byte;
  size_t length = 0U;

  while (length + 1U < capacity && DebugLog_PopByte(&byte))
  {
    out[length++] = (char)byte;
  }
  out[length] = '\0';
  return length;
}

static void TestDisplayLinesUseFixedWidthAxes(void)
{
  char line1[CONTROL_REPORT_LINE_MAX];
  char line2[CONTROL_REPORT_LINE_MAX];
  AppDebugState state = {0};

  TEST_CASE("OLED lines use the shared control snapshot format");

  ControlReport_FormatUpDown(line1, sizeof(line1), 720);
  ControlReport_FormatLeftRight(line2, sizeof(line2), -350);
  TEST_CHECK(strcmp(line1, "UP   :072") == 0);
  TEST_CHECK(strcmp(line2, "LEFT :035") == 0);

  ControlReport_FormatUpDown(line1, sizeof(line1), 0);
  ControlReport_FormatLeftRight(line2, sizeof(line2), 0);
  TEST_CHECK(strcmp(line1, "UP/DN:000") == 0);
  TEST_CHECK(strcmp(line2, "LT/RT:000") == 0);

  state.xbox_connected = 1U;
  state.actual_left = -270;
  state.actual_right = -690;
  state.up_down_speed = -480;
  state.left_right_speed = 210;
  TEST_CHECK(ControlReport_BuildDisplayLines(
      &state, line1, sizeof(line1), line2, sizeof(line2)));
  TEST_CHECK(strcmp(line1, "DOWN :048") == 0);
  TEST_CHECK(strcmp(line2, "RIGHT:021") == 0);

  /* OLED follows the post-ramp motor pair, not a changed Xbox request. */
  state.up_down_speed = 900;
  state.left_right_speed = -800;
  TEST_CHECK(ControlReport_BuildDisplayLines(
      &state, line1, sizeof(line1), line2, sizeof(line2)));
  TEST_CHECK(strcmp(line1, "DOWN :048") == 0);
  TEST_CHECK(strcmp(line2, "RIGHT:021") == 0);

  state.xbox_connected = 0U;
  TEST_CHECK(!ControlReport_BuildDisplayLines(
      &state, line1, sizeof(line1), line2, sizeof(line2)));
}

static void TestBootAndPwmLogsUseBoardMap(void)
{
  static const PwmChannelInfo map[MOTOR_COUNT] = {
      {"FL", "TIM4_SOFTPWM", "SOFT_CH1", 'A', 1U, 'A', 2U, 'A', 3U},
      {"RL", "TIM4_SOFTPWM", "SOFT_CH2", 'A', 6U, 'A', 4U, 'A', 5U},
      {"FR", "TIM4_SOFTPWM", "SOFT_CH3", 'B', 3U, 'A', 12U, 'A', 15U},
      {"RR", "TIM4_SOFTPWM", "SOFT_CH4", 'B', 15U, 'A', 8U, 'A', 9U},
  };
  ControlReporter reporter;
  AppDebugState state = {0};
  char log[DEBUG_LOG_RING_SIZE];
  uint8_t index;

  TEST_CASE("boot and PWM logs expose the real board map");

  DebugLog_Init();
  ControlReport_SetPwmMap(map);
  ControlReport_LogBoot(8000000UL, 200UL, 20000UL);
  DrainLog(log, sizeof(log));
  TEST_CHECK(strstr(log, "[BOOT] ESP_UART=USART1 remap PB6=TX PB7=RX") != NULL);
  TEST_CHECK(strstr(log, "[BOOT] OLED_I2C=I2C1 remap PB8=SCL PB9=SDA") != NULL);
  TEST_CHECK(strstr(log, "[BOOT] PWM1 motor=FL pin=PA1 timer=TIM4_SOFTPWM") != NULL);

  DebugLog_Init();
  ControlReport_Init(&reporter);
  state.xbox_connected = 1U;
  state.frame_valid = 1U;
  state.control_state = CONTROL_STATE_ONLINE;
  state.protocol_version = 2U;
  state.up_down_speed = 720;
  state.left_right_speed = -350;
  state.raw_up_down = 511;
  state.raw_left_right = -300;
  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    state.motor_target[index] = 720;
    state.motor_speed[index] = 680;
    state.motor_pwm[index] = 68U;
    state.motor_dir[index] = MOTOR_DIR_FORWARD;
  }
  ControlReport_Update(&reporter, &state, 100U);
  DrainLog(log, sizeof(log));
  TEST_CHECK(strstr(log, "[CTRL] xbox=1") != NULL);
  TEST_CHECK(strstr(log, "[AXIS] raw_ud=511 raw_lr=-300") != NULL);
  TEST_CHECK(strstr(log, "[MIX] FL=68 FR=68 RL=68 RR=68") != NULL);
  TEST_CHECK(strstr(log, "[PWM1] motor=FL pin=PA1 timer=TIM4_SOFTPWM") != NULL);
}

int run_report_tests(void)
{
  printf("-- report --\n");
  TestDisplayLinesUseFixedWidthAxes();
  TestBootAndPwmLogsUseBoardMap();
  return 0;
}
