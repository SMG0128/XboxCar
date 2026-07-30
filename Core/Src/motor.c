#include "motor.h"

#include "app_config.h"
#include "board_config.h"
#include "control_mixer.h"
#include "soft_pwm.h"

#include "stm32f1xx_hal.h"

typedef struct
{
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  SoftPwmChannel pwm_channel;
  bool inverted;
} MotorHardware;

static const MotorHardware hardware[MOTOR_COUNT] =
{
  {MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, MOTOR_LF_IN2_PORT, MOTOR_LF_IN2_PIN,
   SOFT_PWM_LEFT_FRONT, MOTOR_LEFT_FRONT_INVERTED != 0},
  {MOTOR_LR_IN1_PORT, MOTOR_LR_IN1_PIN, MOTOR_LR_IN2_PORT, MOTOR_LR_IN2_PIN,
   SOFT_PWM_LEFT_REAR, MOTOR_LEFT_REAR_INVERTED != 0},
  {MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, MOTOR_RF_IN2_PORT, MOTOR_RF_IN2_PIN,
   SOFT_PWM_RIGHT_FRONT, MOTOR_RIGHT_FRONT_INVERTED != 0},
  {MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN, MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN,
   SOFT_PWM_RIGHT_REAR, MOTOR_RIGHT_REAR_INVERTED != 0}
};

static int16_t target_speed[MOTOR_COUNT];
static int16_t current_speed[MOTOR_COUNT];
static bool emergency_stopped;

static int16_t ClampSpeed(int16_t speed)
{
  if (speed > MOTOR_COMMAND_MAX)
  {
    return MOTOR_COMMAND_MAX;
  }
  if (speed < -MOTOR_COMMAND_MAX)
  {
    return -MOTOR_COMMAND_MAX;
  }
  return speed;
}

static void WritePin(GPIO_TypeDef *port, uint16_t pin, bool high)
{
  port->BSRR = high ? pin : ((uint32_t)pin << 16U);
}

static void ApplyOutput(MotorId id, int16_t speed)
{
  const MotorHardware *motor = &hardware[id];
  int32_t magnitude = speed;
  bool forward;
  uint8_t duty;

  if (magnitude < 0)
  {
    magnitude = -magnitude;
  }
  duty = (uint8_t)(magnitude * SOFT_PWM_MAX / MOTOR_COMMAND_MAX);

  if (speed == 0)
  {
    SoftPwm_SetDuty(motor->pwm_channel, 0U);
    WritePin(motor->in1_port, motor->in1_pin, false);
    WritePin(motor->in2_port, motor->in2_pin, false);
    return;
  }

  forward = speed > 0;
  if (motor->inverted)
  {
    forward = !forward;
  }
  WritePin(motor->in1_port, motor->in1_pin, forward);
  WritePin(motor->in2_port, motor->in2_pin, !forward);
  SoftPwm_SetDuty(motor->pwm_channel, duty);
}

static void ConfigureDirectionPins(void)
{
  GPIO_InitTypeDef gpio = {0};
  const uint16_t pins_a = MOTOR_LF_IN1_PIN | MOTOR_LF_IN2_PIN |
                          MOTOR_LR_IN1_PIN | MOTOR_LR_IN2_PIN |
                          MOTOR_RF_IN1_PIN | MOTOR_RF_IN2_PIN |
                          MOTOR_RR_IN1_PIN | MOTOR_RR_IN2_PIN;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  GPIOA->BSRR = (uint32_t)pins_a << 16U;
  gpio.Pin = pins_a;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void Motor_Init(void)
{
  uint8_t index;

  ConfigureDirectionPins();
  for (index = 0U; index < (uint8_t)MOTOR_COUNT; ++index)
  {
    target_speed[index] = 0;
    current_speed[index] = 0;
  }
  emergency_stopped = false;
  SoftPwm_Init();
  Motor_StopAll();
}

void Motor_SetSingle(MotorId id, int16_t speed)
{
  if ((uint32_t)id >= (uint32_t)MOTOR_COUNT || emergency_stopped)
  {
    return;
  }
  target_speed[id] = ClampSpeed(speed);
}

void Motor_SetLeftRight(int16_t left, int16_t right)
{
  if (emergency_stopped)
  {
    return;
  }
  Motor_SetSingle(MOTOR_LEFT_FRONT, left);
  Motor_SetSingle(MOTOR_LEFT_REAR, left);
  Motor_SetSingle(MOTOR_RIGHT_FRONT, right);
  Motor_SetSingle(MOTOR_RIGHT_REAR, right);
}

void Motor_StopAll(void)
{
  uint8_t index;

  for (index = 0U; index < (uint8_t)MOTOR_COUNT; ++index)
  {
    target_speed[index] = 0;
    current_speed[index] = 0;
    ApplyOutput((MotorId)index, 0);
  }
  SoftPwm_StopAll();
}

void Motor_EmergencyStop(void)
{
  emergency_stopped = true;
  Motor_StopAll();
}

void Motor_ClearEmergencyStop(void)
{
  emergency_stopped = false;
}

bool Motor_IsEmergencyStopped(void)
{
  return emergency_stopped;
}

void Motor_Task10ms(void)
{
  uint8_t index;
  int16_t effective_target;

  if (emergency_stopped)
  {
    Motor_StopAll();
    return;
  }

  for (index = 0U; index < (uint8_t)MOTOR_COUNT; ++index)
  {
    effective_target = target_speed[index];
    if ((current_speed[index] > 0 && effective_target < 0) ||
        (current_speed[index] < 0 && effective_target > 0))
    {
      effective_target = 0;
    }
    current_speed[index] = ControlMixer_Approach(
        current_speed[index], effective_target, MOTOR_SLEW_STEP);
    ApplyOutput((MotorId)index, current_speed[index]);
  }
}

int16_t Motor_GetCurrentLeft(void)
{
  return (int16_t)(((int32_t)current_speed[MOTOR_LEFT_FRONT] +
                    current_speed[MOTOR_LEFT_REAR]) / 2);
}

int16_t Motor_GetCurrentRight(void)
{
  return (int16_t)(((int32_t)current_speed[MOTOR_RIGHT_FRONT] +
                    current_speed[MOTOR_RIGHT_REAR]) / 2);
}
