#include "board_runtime.h"

#include "app_config.h"
#include "control_report.h"
#include "control_system.h"
#include "debug_log.h"
#include "ssd1306_simple.h"
#include "ultrasonic.h"

#include <stddef.h>
#include <string.h>

typedef enum {
#define BOARD_PIN(name, port, number) BOARD_PIN_ID_##name,
#include "board_pins.h"
#undef BOARD_PIN
  BOARD_PIN_ID_COUNT
} BoardPinId;

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} BoardGpio;

static const BoardGpio kBoardPins[BOARD_PIN_ID_COUNT] = {
#define BOARD_PIN(name, port, number)                                         \
  [BOARD_PIN_ID_##name] = {GPIO##port, GPIO_PIN_##number},
#include "board_pins.h"
#undef BOARD_PIN
};

/*
 * The same pin list again, as printable port letter and pin number. Generating
 * both tables from one X-macro list is what guarantees the pin named in a log
 * line is the pin the ISR is actually toggling; a hand-maintained string table
 * would be free to drift the moment a pin moved.
 */
static const char kBoardPinPort[BOARD_PIN_ID_COUNT] = {
#define BOARD_PIN(name, port, number) [BOARD_PIN_ID_##name] = #port[0],
#include "board_pins.h"
#undef BOARD_PIN
};

static const uint8_t kBoardPinNumber[BOARD_PIN_ID_COUNT] = {
#define BOARD_PIN(name, port, number) [BOARD_PIN_ID_##name] = (uint8_t)(number),
#include "board_pins.h"
#undef BOARD_PIN
};

typedef struct {
  BoardPinId pwm;
  BoardPinId in1;
  BoardPinId in2;
} MotorPins;

/* PCB channel mapping: left B is front, left A is rear; right A/B are front/rear. */
static const MotorPins kMotorPins[MOTOR_COUNT] = {
    [MOTOR_FRONT_LEFT] = {BOARD_PIN_ID_MOTOR_LEFT_B_PWM,
                          BOARD_PIN_ID_MOTOR_LEFT_B_IN1,
                          BOARD_PIN_ID_MOTOR_LEFT_B_IN2},
    [MOTOR_REAR_LEFT] = {BOARD_PIN_ID_MOTOR_LEFT_A_PWM,
                         BOARD_PIN_ID_MOTOR_LEFT_A_IN1,
                         BOARD_PIN_ID_MOTOR_LEFT_A_IN2},
    [MOTOR_FRONT_RIGHT] = {BOARD_PIN_ID_MOTOR_RIGHT_A_PWM,
                           BOARD_PIN_ID_MOTOR_RIGHT_A_IN1,
                           BOARD_PIN_ID_MOTOR_RIGHT_A_IN2},
    [MOTOR_REAR_RIGHT] = {BOARD_PIN_ID_MOTOR_RIGHT_B_PWM,
                          BOARD_PIN_ID_MOTOR_RIGHT_B_IN1,
                          BOARD_PIN_ID_MOTOR_RIGHT_B_IN2},
};

#if APP_FEATURE_ULTRASONIC
static const BoardPinId kUltrasonicTrigger[ULTRASONIC_COUNT] = {
    [ULTRASONIC_FRONT] = BOARD_PIN_ID_ULTRASONIC_FRONT_TRIG,
    [ULTRASONIC_REAR] = BOARD_PIN_ID_ULTRASONIC_REAR_TRIG,
    [ULTRASONIC_LEFT] = BOARD_PIN_ID_ULTRASONIC_LEFT_TRIG,
    [ULTRASONIC_RIGHT] = BOARD_PIN_ID_ULTRASONIC_RIGHT_TRIG,
};

static const BoardPinId kUltrasonicEcho[ULTRASONIC_COUNT] = {
    [ULTRASONIC_FRONT] = BOARD_PIN_ID_ULTRASONIC_FRONT_ECHO,
    [ULTRASONIC_REAR] = BOARD_PIN_ID_ULTRASONIC_REAR_ECHO,
    [ULTRASONIC_LEFT] = BOARD_PIN_ID_ULTRASONIC_LEFT_ECHO,
    [ULTRASONIC_RIGHT] = BOARD_PIN_ID_ULTRASONIC_RIGHT_ECHO,
};
#endif

static ControlSystem g_control;
static Ultrasonic g_ultrasonic;
static UART_HandleTypeDef *g_control_uart;
static uint8_t g_rx_byte;
static volatile bool g_uart_fault_pending;
static bool g_initialized;

static volatile uint16_t g_pwm_duty[MOTOR_COUNT];
static volatile uint16_t g_pwm_phase;

static uint32_t g_last_control_ms;
static ControlReporter g_reporter;
static PwmChannelInfo g_pwm_report_map[MOTOR_COUNT];

static const char *const kMotorWheelNames[MOTOR_COUNT] = {
    "FL", "RL", "FR", "RR"};

#if APP_FEATURE_DISPLAY
static bool g_display_ready;
static bool g_display_pending;
static uint8_t g_display_page;
static uint8_t g_display_failures;
static uint32_t g_last_display_flush_ms;
static bool g_rendered_xbox;
static int16_t g_rendered_up_down;
static int16_t g_rendered_left_right;
#endif

static void InitializePwmReportMap(void)
{
  uint8_t index;

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    const MotorPins *motor = &kMotorPins[index];

    g_pwm_report_map[index] = (PwmChannelInfo){
        kMotorWheelNames[index],
        "TIM4_SOFTPWM",
        (index == 0U) ? "SOFT_CH1"
        : (index == 1U) ? "SOFT_CH2"
        : (index == 2U) ? "SOFT_CH3"
                        : "SOFT_CH4",
        kBoardPinPort[motor->pwm],
        kBoardPinNumber[motor->pwm],
        kBoardPinPort[motor->in1],
        kBoardPinNumber[motor->in1],
        kBoardPinPort[motor->in2],
        kBoardPinNumber[motor->in2]};
  }
}

static void DrainDebugLog(void)
{
  uint8_t byte;
  uint16_t count = 0U;

  if (g_control_uart == NULL || g_control_uart->Instance == NULL)
  {
    return;
  }

  while (count < DEBUG_LOG_BYTES_PER_PASS &&
         (g_control_uart->Instance->SR & USART_SR_TXE) != 0U)
  {
    if (!DebugLog_PopByte(&byte))
    {
      break;
    }
    g_control_uart->Instance->DR = byte;
    ++count;
  }
}

#if APP_FEATURE_ULTRASONIC
static uint32_t g_micros;
static uint32_t g_last_cycle;
static uint32_t g_cycle_remainder;
#endif

#define MOTOR_GPIOA_MASK                                                     \
  (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |          \
   GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12 | GPIO_PIN_15)
#define MOTOR_GPIOB_MASK (GPIO_PIN_3 | GPIO_PIN_15)

static void WritePin(BoardPinId id, bool high)
{
  const BoardGpio *pin = &kBoardPins[id];
  pin->port->BSRR = high ? pin->pin : ((uint32_t)pin->pin << 16U);
}

static void SetMotorPinsLow(void)
{
  uint8_t index;

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    g_pwm_duty[index] = 0U;
  }
  GPIOA->BSRR = (uint32_t)MOTOR_GPIOA_MASK << 16U;
  GPIOB->BSRR = (uint32_t)MOTOR_GPIOB_MASK << 16U;
}

void BoardRuntime_FaultStop(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  (void)RCC->APB2ENR;
  TIM4->DIER = 0U;
  TIM4->CR1 = 0U;
  SetMotorPinsLow();
}

static bool ConfigureMotorHardware(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint32_t timer_clock;
  uint32_t reload;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  SetMotorPinsLow();
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = MOTOR_GPIOA_MASK;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = MOTOR_GPIOB_MASK;
  HAL_GPIO_Init(GPIOB, &gpio);

  __HAL_RCC_TIM4_CLK_ENABLE();
  timer_clock = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    timer_clock *= 2U;
  }
  if (timer_clock != APP_APB1_TIMER_CLK_HZ || SOFT_PWM_ISR_HZ == 0U)
  {
    return false;
  }

  reload = timer_clock / SOFT_PWM_ISR_HZ;
  if (reload == 0U || (timer_clock % SOFT_PWM_ISR_HZ) != 0U)
  {
    return false;
  }

  g_pwm_phase = 0U;
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
  return true;
}

void BoardRuntime_TimerIrqHandler(void)
{
  uint32_t set_a = 0U;
  uint32_t reset_a = 0U;
  uint32_t set_b = 0U;
  uint32_t reset_b = 0U;
  const uint16_t phase = g_pwm_phase;

  if ((TIM4->SR & TIM_SR_UIF) == 0U)
  {
    return;
  }
  TIM4->SR = 0U;

  /*
   * Keep the 20 kHz ISR unrolled: at an 8 MHz system clock there are only
   * 400 cycles per tick. Parsing, state changes and table walking stay out of
   * this path; it performs four comparisons and two BSRR writes.
   */
  if (phase < g_pwm_duty[MOTOR_FRONT_LEFT])
  {
    set_a |= GPIO_PIN_1;
  }
  else
  {
    reset_a |= GPIO_PIN_1;
  }
  if (phase < g_pwm_duty[MOTOR_REAR_LEFT])
  {
    set_a |= GPIO_PIN_6;
  }
  else
  {
    reset_a |= GPIO_PIN_6;
  }
  if (phase < g_pwm_duty[MOTOR_FRONT_RIGHT])
  {
    set_b |= GPIO_PIN_3;
  }
  else
  {
    reset_b |= GPIO_PIN_3;
  }
  if (phase < g_pwm_duty[MOTOR_REAR_RIGHT])
  {
    set_b |= GPIO_PIN_15;
  }
  else
  {
    reset_b |= GPIO_PIN_15;
  }

  GPIOA->BSRR = set_a | (reset_a << 16U);
  GPIOB->BSRR = set_b | (reset_b << 16U);

  ++g_pwm_phase;
  if (g_pwm_phase >= SOFT_PWM_RESOLUTION)
  {
    g_pwm_phase = 0U;
  }
}

static bool ApplyMotorOutputs(uint32_t now_ms)
{
  MotorOutput outputs[MOTOR_COUNT];
  uint32_t primask;
  uint8_t index;

  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    if (!Motor_GetOutput(&g_control.motor, index, &outputs[index]))
    {
      ControlSystem_ReportInternalFault(
          &g_control, (uint16_t)FAULT_INTERNAL_MOTOR_INDEX,
          INTERNAL_FAULT_MOTOR_INDEX, now_ms);
      SetMotorPinsLow();
      return false;
    }
#if !APP_FEATURE_MOTOR_OUTPUT
    outputs[index].duty = 0U;
    outputs[index].in1 = false;
    outputs[index].in2 = false;
#endif
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (index = 0U; index < MOTOR_COUNT; ++index)
  {
    WritePin(kMotorPins[index].in1, outputs[index].in1);
    WritePin(kMotorPins[index].in2, outputs[index].in2);
    g_pwm_duty[index] = outputs[index].duty;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

#if APP_FEATURE_ULTRASONIC
static uint32_t GetMicros(void)
{
  const uint32_t cycle = DWT->CYCCNT;
  const uint32_t elapsed_cycles = cycle - g_last_cycle;
  const uint32_t accumulated = g_cycle_remainder + elapsed_cycles;

  g_last_cycle = cycle;
  g_micros += accumulated / (APP_SYSCLK_HZ / 1000000UL);
  g_cycle_remainder = accumulated % (APP_SYSCLK_HZ / 1000000UL);
  return g_micros;
}

static void SetTrigger(uint8_t sensor, bool level)
{
  if (sensor < ULTRASONIC_COUNT &&
      (ULTRASONIC_ENABLED_MASK & (1UL << sensor)) != 0U)
  {
    WritePin(kUltrasonicTrigger[sensor], level);
  }
}

static bool ReadEcho(uint8_t sensor)
{
  const BoardGpio *pin;

  if (sensor >= ULTRASONIC_COUNT ||
      (ULTRASONIC_ENABLED_MASK & (1UL << sensor)) == 0U)
  {
    return false;
  }
  pin = &kBoardPins[kUltrasonicEcho[sensor]];
  return (pin->port->IDR & pin->pin) != 0U;
}

static void ConfigureUltrasonic(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t index;
  UltrasonicHal hal;

  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    const BoardGpio *trigger = &kBoardPins[kUltrasonicTrigger[index]];
    if ((ULTRASONIC_ENABLED_MASK & (1UL << index)) == 0U)
    {
      continue;
    }
    WritePin(kUltrasonicTrigger[index], false);
    gpio.Pin = trigger->pin;
    HAL_GPIO_Init(trigger->port, &gpio);
  }

  gpio.Mode = GPIO_MODE_INPUT;
  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    const BoardGpio *echo = &kBoardPins[kUltrasonicEcho[index]];
    if ((ULTRASONIC_ENABLED_MASK & (1UL << index)) == 0U)
    {
      continue;
    }
    gpio.Pin = echo->pin;
    HAL_GPIO_Init(echo->port, &gpio);
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  g_micros = 0U;
  g_cycle_remainder = 0U;
  g_last_cycle = DWT->CYCCNT;

  hal.get_micros = GetMicros;
  hal.set_trigger = SetTrigger;
  hal.read_echo = ReadEcho;
  Ultrasonic_Init(&g_ultrasonic, &hal);
  Ultrasonic_SetEnabled(&g_ultrasonic, true);
}
#endif

static void BuildSensorSnapshot(SensorSnapshot *snapshot)
{
  memset(snapshot, 0, sizeof(*snapshot));
#if APP_FEATURE_ULTRASONIC
  uint8_t index;

  snapshot->valid_mask = Ultrasonic_GetValidMask(&g_ultrasonic);
  snapshot->fault_mask = Ultrasonic_GetFaultMask(&g_ultrasonic);
  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    snapshot->distance_mm[index] = Ultrasonic_GetDistance(&g_ultrasonic, index);
    snapshot->raw_mm[index] = Ultrasonic_GetRawDistance(&g_ultrasonic, index);
    snapshot->timeouts[index] =
        Ultrasonic_GetTimeoutCount(&g_ultrasonic, index);
  }
#endif
}

#if APP_FEATURE_DISPLAY
static void UpdateDisplay(uint32_t now_ms)
{
  const AppDebugState *debug = ControlSystem_GetDebugState(&g_control);
  const bool has_xbox = ControlReport_HasXbox(debug);
  int16_t up_down;
  int16_t left_right;

  ControlReport_GetActualAxes(debug, &up_down, &left_right);

  if (!g_display_ready)
  {
    return;
  }

  if (!g_display_pending &&
      (has_xbox != g_rendered_xbox || up_down != g_rendered_up_down ||
       left_right != g_rendered_left_right))
  {
    char line1[CONTROL_REPORT_LINE_MAX];
    char line2[CONTROL_REPORT_LINE_MAX];
    const bool rendered =
        ControlReport_BuildDisplayLines(debug, line1, sizeof(line1), line2,
                                        sizeof(line2))
            ? SSD1306_RenderLines(line1, line2)
            : SSD1306_RenderNoXbox();
    if (!rendered)
    {
      g_display_ready = false;
      return;
    }
    g_rendered_xbox = has_xbox;
    g_rendered_up_down = up_down;
    g_rendered_left_right = left_right;
    g_display_page = 0U;
    g_display_pending = true;
  }

  if (!g_display_pending ||
      (uint32_t)(now_ms - g_last_display_flush_ms) < APP_DISPLAY_TICK_MS)
  {
    return;
  }
  g_last_display_flush_ms = now_ms;

  if (!SSD1306_FlushPage(g_display_page))
  {
    if (++g_display_failures >= APP_DISPLAY_FAIL_LIMIT)
    {
      g_display_ready = false;
      g_display_pending = false;
    }
    return;
  }

  g_display_failures = 0U;
  ++g_display_page;
  if (g_display_page >= 8U)
  {
    g_display_pending = false;
  }
}
#endif

bool BoardRuntime_Init(UART_HandleTypeDef *control_uart,
                       I2C_HandleTypeDef *display_i2c)
{
  if (control_uart == NULL || control_uart->Instance != USART1)
  {
    return false;
  }

  g_initialized = false;
  g_control_uart = control_uart;
  g_uart_fault_pending = false;
  DebugLog_Init();
  ControlReport_Init(&g_reporter);
  InitializePwmReportMap();
  ControlReport_SetPwmMap(g_pwm_report_map);
  ControlSystem_Init(&g_control);

  if (!ConfigureMotorHardware())
  {
    ControlSystem_ReportInternalFault(
        &g_control, (uint16_t)FAULT_INTERNAL_PWM_TIMER,
        INTERNAL_FAULT_PWM_TIMER, HAL_GetTick());
    BoardRuntime_FaultStop();
    return false;
  }

  ControlReport_LogBoot(APP_SYSCLK_HZ, SOFT_PWM_FREQUENCY_HZ,
                        SOFT_PWM_ISR_HZ);

#if APP_FEATURE_ULTRASONIC
  ConfigureUltrasonic();
#else
  Ultrasonic_Init(&g_ultrasonic, NULL);
#endif

#if APP_FEATURE_DISPLAY
  g_display_ready = false;
  g_display_pending = false;
  if (display_i2c != NULL)
  {
    HAL_Delay(50U);
    g_display_ready = SSD1306_Init(display_i2c);
    if (g_display_ready && SSD1306_RenderNoXbox())
    {
      g_rendered_xbox = false;
      g_rendered_up_down = 0;
      g_rendered_left_right = 0;
      g_display_page = 0U;
      g_display_pending = true;
      g_last_display_flush_ms = HAL_GetTick();
    }
  }
#else
  (void)display_i2c;
#endif

  g_last_control_ms = HAL_GetTick();
  g_initialized = true;
  if (HAL_UART_Receive_IT(g_control_uart, &g_rx_byte, 1U) != HAL_OK)
  {
    g_initialized = false;
    ControlSystem_ReportInternalFault(
        &g_control, (uint16_t)FAULT_INTERNAL_UART_INIT,
        INTERNAL_FAULT_UART_INIT, HAL_GetTick());
    BoardRuntime_FaultStop();
    return false;
  }

  return true;
}

void BoardRuntime_Run(void)
{
  SensorSnapshot sensors;
  uint32_t now_ms;

  if (!g_initialized)
  {
    return;
  }

  DrainDebugLog();

#if APP_FEATURE_ULTRASONIC
  Ultrasonic_Update(&g_ultrasonic);
#endif

  now_ms = HAL_GetTick();
  if (g_uart_fault_pending)
  {
    g_uart_fault_pending = false;
    ControlSystem_ReportInternalFault(
        &g_control, (uint16_t)FAULT_INTERNAL_UART_INIT,
        INTERNAL_FAULT_UART_INIT, now_ms);
  }

  if ((uint32_t)(now_ms - g_last_control_ms) >= CONTROL_PERIOD_MS)
  {
    g_last_control_ms = now_ms;
    BuildSensorSnapshot(&sensors);
    ControlSystem_Update(&g_control, now_ms, &sensors);
    ControlReport_Update(&g_reporter, ControlSystem_GetDebugState(&g_control),
                         now_ms);
    (void)ApplyMotorOutputs(now_ms);
  }

#if APP_FEATURE_DISPLAY
  UpdateDisplay(now_ms);
#endif
  DrainDebugLog();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (g_initialized && uart == g_control_uart)
  {
    ControlSystem_PushRxByte(&g_control, g_rx_byte);
    if (HAL_UART_Receive_IT(g_control_uart, &g_rx_byte, 1U) != HAL_OK)
    {
      SetMotorPinsLow();
      g_uart_fault_pending = true;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (g_initialized && uart == g_control_uart &&
      HAL_UART_Receive_IT(g_control_uart, &g_rx_byte, 1U) != HAL_OK)
  {
    SetMotorPinsLow();
    g_uart_fault_pending = true;
  }
}
