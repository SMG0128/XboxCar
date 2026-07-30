#include "soft_pwm.h"

#include "app_config.h"
#include "board_config.h"

#include "stm32f1xx_hal.h"

static volatile uint8_t pwm_duty[SOFT_PWM_CHANNEL_COUNT];
static volatile uint8_t pwm_phase;

static void ConfigurePwmPins(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  MOTOR_LF_PWM_PORT->BSRR = (uint32_t)MOTOR_LF_PWM_PIN << 16U;
  MOTOR_LR_PWM_PORT->BSRR = (uint32_t)MOTOR_LR_PWM_PIN << 16U;
  MOTOR_RF_PWM_PORT->BSRR = (uint32_t)MOTOR_RF_PWM_PIN << 16U;
  MOTOR_RR_PWM_PORT->BSRR = (uint32_t)MOTOR_RR_PWM_PIN << 16U;

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = MOTOR_LF_PWM_PIN | MOTOR_LR_PWM_PIN;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = MOTOR_RF_PWM_PIN | MOTOR_RR_PWM_PIN;
  HAL_GPIO_Init(GPIOB, &gpio);
}

void SoftPwm_Init(void)
{
  uint32_t timer_clock;
  uint32_t reload;

  ConfigurePwmPins();
  SoftPwm_StopAll();
  pwm_phase = 0U;

  __HAL_RCC_TIM4_CLK_ENABLE();
  timer_clock = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    timer_clock *= 2U;
  }
  reload = timer_clock / SOFT_PWM_ISR_HZ;
  if (reload == 0U)
  {
    reload = 1U;
  }

  TIM4->CR1 = 0U;
  TIM4->PSC = 0U;
  TIM4->ARR = reload - 1U;
  TIM4->CNT = 0U;
  TIM4->EGR = TIM_EGR_UG;
  TIM4->SR = 0U;
  TIM4->DIER = TIM_DIER_UIE;

  HAL_NVIC_SetPriority(TIM4_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  TIM4->CR1 = TIM_CR1_CEN;
}

void SoftPwm_SetDuty(SoftPwmChannel channel, uint8_t duty)
{
  if ((uint32_t)channel >= (uint32_t)SOFT_PWM_CHANNEL_COUNT)
  {
    return;
  }
  if (duty > SOFT_PWM_MAX)
  {
    duty = SOFT_PWM_MAX;
  }
  pwm_duty[channel] = duty;
}

void SoftPwm_StopAll(void)
{
  uint8_t index;

  for (index = 0U; index < (uint8_t)SOFT_PWM_CHANNEL_COUNT; ++index)
  {
    pwm_duty[index] = 0U;
  }
  GPIOA->BSRR = ((uint32_t)(MOTOR_LF_PWM_PIN | MOTOR_LR_PWM_PIN)) << 16U;
  GPIOB->BSRR = ((uint32_t)(MOTOR_RF_PWM_PIN | MOTOR_RR_PWM_PIN)) << 16U;
}

void SoftPwm_TimerIrqHandler(void)
{
  uint32_t set_a = 0U;
  uint32_t set_b = 0U;
  uint32_t reset_a = 0U;
  uint32_t reset_b = 0U;

  if ((TIM4->SR & TIM_SR_UIF) == 0U)
  {
    return;
  }
  TIM4->SR = (uint16_t)~TIM_SR_UIF;

  if (pwm_phase < pwm_duty[SOFT_PWM_LEFT_FRONT])
  {
    set_a |= MOTOR_LF_PWM_PIN;
  }
  else
  {
    reset_a |= MOTOR_LF_PWM_PIN;
  }
  if (pwm_phase < pwm_duty[SOFT_PWM_LEFT_REAR])
  {
    set_a |= MOTOR_LR_PWM_PIN;
  }
  else
  {
    reset_a |= MOTOR_LR_PWM_PIN;
  }
  if (pwm_phase < pwm_duty[SOFT_PWM_RIGHT_FRONT])
  {
    set_b |= MOTOR_RF_PWM_PIN;
  }
  else
  {
    reset_b |= MOTOR_RF_PWM_PIN;
  }
  if (pwm_phase < pwm_duty[SOFT_PWM_RIGHT_REAR])
  {
    set_b |= MOTOR_RR_PWM_PIN;
  }
  else
  {
    reset_b |= MOTOR_RR_PWM_PIN;
  }

  GPIOA->BSRR = set_a | (reset_a << 16U);
  GPIOB->BSRR = set_b | (reset_b << 16U);

  ++pwm_phase;
  if (pwm_phase >= SOFT_PWM_MAX)
  {
    pwm_phase = 0U;
  }
}
