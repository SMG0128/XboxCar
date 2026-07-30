#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f1xx_hal.h"

/* Left TB6612: B channel is front, A channel is rear. */
#define MOTOR_LF_PWM_PORT GPIOA
#define MOTOR_LF_PWM_PIN  GPIO_PIN_1
#define MOTOR_LF_IN1_PORT GPIOA
#define MOTOR_LF_IN1_PIN  GPIO_PIN_2
#define MOTOR_LF_IN2_PORT GPIOA
#define MOTOR_LF_IN2_PIN  GPIO_PIN_3

#define MOTOR_LR_PWM_PORT GPIOA
#define MOTOR_LR_PWM_PIN  GPIO_PIN_6
#define MOTOR_LR_IN1_PORT GPIOA
#define MOTOR_LR_IN1_PIN  GPIO_PIN_4
#define MOTOR_LR_IN2_PORT GPIOA
#define MOTOR_LR_IN2_PIN  GPIO_PIN_5

/* Right TB6612: A channel is front, B channel is rear. */
#define MOTOR_RF_PWM_PORT GPIOB
#define MOTOR_RF_PWM_PIN  GPIO_PIN_3
#define MOTOR_RF_IN1_PORT GPIOA
#define MOTOR_RF_IN1_PIN  GPIO_PIN_12
#define MOTOR_RF_IN2_PORT GPIOA
#define MOTOR_RF_IN2_PIN  GPIO_PIN_15

#define MOTOR_RR_PWM_PORT GPIOB
#define MOTOR_RR_PWM_PIN  GPIO_PIN_15
#define MOTOR_RR_IN1_PORT GPIOA
#define MOTOR_RR_IN1_PIN  GPIO_PIN_8
#define MOTOR_RR_IN2_PORT GPIOA
#define MOTOR_RR_IN2_PIN  GPIO_PIN_9

/* ESP32 control UART: USART1 full remap. */
#define CONTROL_UART_TX_PORT GPIOB
#define CONTROL_UART_TX_PIN  GPIO_PIN_6
#define CONTROL_UART_RX_PORT GPIOB
#define CONTROL_UART_RX_PIN  GPIO_PIN_7

#define OLED_SCL_PORT GPIOB
#define OLED_SCL_PIN  GPIO_PIN_8
#define OLED_SDA_PORT GPIOB
#define OLED_SDA_PIN  GPIO_PIN_9

typedef struct
{
  GPIO_TypeDef *trig_port;
  uint16_t trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t echo_pin;
} UltrasonicPins;

#endif /* BOARD_CONFIG_H */
