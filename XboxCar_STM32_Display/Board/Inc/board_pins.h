/*
 * XboxCar PCB pin map.
 *
 * This is an X-macro list, not a normal header: it has no include guard and is
 * meant to be included repeatedly with BOARD_PIN defined differently each time.
 *
 * Source of truth: config/XboxCar_PCB_接口反推报告.md, reverse engineered from
 * Gerber_XboxCar_PCB_DOWN_2026-07-30.zip. tools/validate_config.py parses THIS
 * file, so keep the BOARD_PIN(...) lines machine readable: one per line, in the
 * form BOARD_PIN(name, port, pin_number).
 *
 * Nothing here may be changed without re-checking the PCB. Several assignments
 * are load bearing in non-obvious ways:
 *   - PB3 and PA15 are JTAG pins by default and only work as GPIO after the
 *     SWJ remap to SW-DP. PA13/PA14 stay reserved for SWD.
 *   - PB6/PB7 are the ESP32 link (USART1 remapped), NOT general purpose IO.
 *   - PA1 and PB3 both alias TIM2_CH2 in different remap positions, which is
 *     why all four motor PWM outputs are software generated.
 */

/* --------------------------------------------------------------------- */
/* Left TB6612                                                            */
/* --------------------------------------------------------------------- */
/* Channel A drives one left wheel, channel B the other. */
BOARD_PIN(MOTOR_LEFT_A_PWM, A, 6)
BOARD_PIN(MOTOR_LEFT_A_IN1, A, 4)
BOARD_PIN(MOTOR_LEFT_A_IN2, A, 5)
BOARD_PIN(MOTOR_LEFT_B_PWM, A, 1)
BOARD_PIN(MOTOR_LEFT_B_IN1, A, 2)
BOARD_PIN(MOTOR_LEFT_B_IN2, A, 3)

/* --------------------------------------------------------------------- */
/* Right TB6612                                                           */
/* --------------------------------------------------------------------- */
BOARD_PIN(MOTOR_RIGHT_A_PWM, B, 3)  /* JTAG TDO by default */
BOARD_PIN(MOTOR_RIGHT_A_IN1, A, 12)
BOARD_PIN(MOTOR_RIGHT_A_IN2, A, 15) /* JTAG TDI by default */
BOARD_PIN(MOTOR_RIGHT_B_PWM, B, 15)
BOARD_PIN(MOTOR_RIGHT_B_IN1, A, 8)
BOARD_PIN(MOTOR_RIGHT_B_IN2, A, 9)

/* --------------------------------------------------------------------- */
/* ESP32 link, USART1 remapped to PB6/PB7                                 */
/* --------------------------------------------------------------------- */
BOARD_PIN(ESP_UART_TX, B, 6)
BOARD_PIN(ESP_UART_RX, B, 7)

/* --------------------------------------------------------------------- */
/* I2C1 remapped to PB8/PB9, 4-pin header with its own 3V3 and GND        */
/* --------------------------------------------------------------------- */
BOARD_PIN(OLED_SCL, B, 8)
BOARD_PIN(OLED_SDA, B, 9)

/* --------------------------------------------------------------------- */
/* Ultrasonic 2x6 header                                                  */
/* --------------------------------------------------------------------- */
/* Silkscreen FL/FR/BL/BR map to the front/rear/left/right sensing roles. */
BOARD_PIN(ULTRASONIC_FRONT_TRIG, B, 11) /* FL_T */
BOARD_PIN(ULTRASONIC_FRONT_ECHO, B, 10) /* FL_E */
BOARD_PIN(ULTRASONIC_RIGHT_TRIG, B, 13) /* FR_T */
BOARD_PIN(ULTRASONIC_RIGHT_ECHO, B, 12) /* FR_E */
BOARD_PIN(ULTRASONIC_LEFT_TRIG, B, 0)   /* BL_T */
BOARD_PIN(ULTRASONIC_LEFT_ECHO, B, 1)   /* BL_E */
BOARD_PIN(ULTRASONIC_REAR_TRIG, A, 7)   /* BR_T */
BOARD_PIN(ULTRASONIC_REAR_ECHO, B, 14)  /* BR_E */

/* --------------------------------------------------------------------- */
/* Debug, reserved                                                        */
/* --------------------------------------------------------------------- */
BOARD_PIN(SWDIO, A, 13)
BOARD_PIN(SWCLK, A, 14)
