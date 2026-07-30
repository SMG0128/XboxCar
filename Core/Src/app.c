#include "app.h"

#include "control_protocol.h"
#include "control_system.h"
#include "main.h"
#include "motor.h"
#include "ssd1306_simple.h"
#include "ultrasonic.h"

#include <stdbool.h>

static bool oled_ready;
static uint32_t last_1ms;
static uint32_t last_10ms;
static uint32_t last_display_ms;

static void UpdateDisplay(uint32_t now)
{
  const ControlStatus *status;

  if (!oled_ready || (uint32_t)(now - last_display_ms) < 100U)
  {
    return;
  }
  last_display_ms = now;
  status = ControlSystem_GetStatus();
  (void)SSD1306_ShowControl(status->enabled && !status->emergency_stop &&
                           !status->communication_timeout,
                           status->current_left, status->current_right);
}

void App_Init(UART_HandleTypeDef *control_uart, I2C_HandleTypeDef *display_i2c)
{
  Motor_Init();
  Ultrasonic_Init();
  ControlProtocol_Init(control_uart);
  ControlSystem_Init();

  oled_ready = false;
  if (display_i2c != NULL)
  {
    HAL_Delay(50U);
    oled_ready = SSD1306_Init(display_i2c);
    if (oled_ready)
    {
      (void)SSD1306_ShowControl(false, 0, 0);
    }
  }
  if (ControlProtocol_StartReceive() != HAL_OK)
  {
    Motor_EmergencyStop();
    Error_Handler();
  }

  last_1ms = HAL_GetTick();
  last_10ms = last_1ms;
  last_display_ms = last_1ms;
}

void App_Loop(void)
{
  uint32_t now;

  ControlSystem_ProcessProtocol();
  now = HAL_GetTick();
  if (now != last_1ms)
  {
    last_1ms = now;
    ControlSystem_Task1ms();
  }
  if ((uint32_t)(now - last_10ms) >= 10U)
  {
    last_10ms = now;
    Motor_Task10ms();
    ControlSystem_Task10ms();
  }
  Ultrasonic_Task();
  UpdateDisplay(now);
}
