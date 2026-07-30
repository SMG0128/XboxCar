#include "ultrasonic.h"

#include "app_config.h"
#include "board_config.h"

#include "stm32f1xx_hal.h"

typedef enum
{
  ULTRA_GUARD = 0,
  ULTRA_TRIGGER,
  ULTRA_WAIT_RISE,
  ULTRA_WAIT_FALL
} UltrasonicState;

static const UltrasonicPins pins[ULTRASONIC_SENSOR_COUNT] =
{
  {GPIOB, GPIO_PIN_11, GPIOB, GPIO_PIN_10},
  {GPIOB, GPIO_PIN_13, GPIOB, GPIO_PIN_12},
  {GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1},
  {GPIOA, GPIO_PIN_7, GPIOB, GPIO_PIN_14}
};

static uint16_t distance_mm[ULTRASONIC_SENSOR_COUNT];
static bool distance_valid[ULTRASONIC_SENSOR_COUNT];

#if ULTRASONIC_ENABLED
static UltrasonicState state;
static UltrasonicId active_sensor;
static uint16_t state_started_us;

static uint16_t Micros16(void)
{
  return (uint16_t)TIM3->CNT;
}

static bool ElapsedUs(uint16_t now, uint16_t start, uint16_t duration)
{
  return (uint16_t)(now - start) >= duration;
}

static void AdvanceSensor(uint16_t now)
{
  active_sensor = (UltrasonicId)(((uint32_t)active_sensor + 1U) %
                                 (uint32_t)ULTRASONIC_SENSOR_COUNT);
  state = ULTRA_GUARD;
  state_started_us = now;
}
#endif

void Ultrasonic_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t index;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  for (index = 0U; index < (uint8_t)ULTRASONIC_SENSOR_COUNT; ++index)
  {
    pins[index].trig_port->BSRR = (uint32_t)pins[index].trig_pin << 16U;
    gpio.Pin = pins[index].trig_pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pins[index].trig_port, &gpio);

    gpio.Pin = pins[index].echo_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(pins[index].echo_port, &gpio);
    distance_mm[index] = 0U;
    distance_valid[index] = false;
  }

#if ULTRASONIC_ENABLED
  {
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
      timer_clock *= 2U;
    }
    __HAL_RCC_TIM3_CLK_ENABLE();
    TIM3->CR1 = 0U;
    TIM3->PSC = timer_clock / 1000000U - 1U;
    TIM3->ARR = 0xffffU;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
  }
  active_sensor = ULTRASONIC_FRONT_LEFT;
  state = ULTRA_GUARD;
  state_started_us = Micros16();
#endif
}

void Ultrasonic_Task(void)
{
#if ULTRASONIC_ENABLED
  const UltrasonicPins *sensor = &pins[active_sensor];
  uint16_t now = Micros16();
  bool echo_high = (sensor->echo_port->IDR & sensor->echo_pin) != 0U;

  switch (state)
  {
    case ULTRA_GUARD:
      if (ElapsedUs(now, state_started_us, ULTRASONIC_GUARD_US))
      {
        sensor->trig_port->BSRR = sensor->trig_pin;
        state = ULTRA_TRIGGER;
        state_started_us = now;
      }
      break;

    case ULTRA_TRIGGER:
      if (ElapsedUs(now, state_started_us, ULTRASONIC_TRIGGER_US))
      {
        sensor->trig_port->BSRR = (uint32_t)sensor->trig_pin << 16U;
        state = ULTRA_WAIT_RISE;
        state_started_us = now;
      }
      break;

    case ULTRA_WAIT_RISE:
      if (echo_high)
      {
        state = ULTRA_WAIT_FALL;
        state_started_us = now;
      }
      else if (ElapsedUs(now, state_started_us, ULTRASONIC_ECHO_TIMEOUT_US))
      {
        distance_valid[active_sensor] = false;
        AdvanceSensor(now);
      }
      break;

    case ULTRA_WAIT_FALL:
      if (!echo_high)
      {
        uint16_t pulse_us = (uint16_t)(now - state_started_us);
        distance_mm[active_sensor] = (uint16_t)(((uint32_t)pulse_us * 10U) / 58U);
        distance_valid[active_sensor] = true;
        AdvanceSensor(now);
      }
      else if (ElapsedUs(now, state_started_us, ULTRASONIC_ECHO_TIMEOUT_US))
      {
        distance_valid[active_sensor] = false;
        AdvanceSensor(now);
      }
      break;

    default:
      state = ULTRA_GUARD;
      state_started_us = now;
      break;
  }
#endif
}

bool Ultrasonic_GetDistance(UltrasonicId id, uint16_t *result_mm)
{
  if ((uint32_t)id >= (uint32_t)ULTRASONIC_SENSOR_COUNT ||
      result_mm == NULL || !distance_valid[id])
  {
    return false;
  }
  *result_mm = distance_mm[id];
  return true;
}
