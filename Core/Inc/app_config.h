#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define CONTROL_DEADZONE             80
#define CONTROL_TIMEOUT_MS           300U
#define CONTROL_RECOVERY_FRAMES      3U
#define CONTROL_TASK_PERIOD_MS       10U

#define MOTOR_SLEW_STEP              40
#define MOTOR_COMMAND_MAX            1000

#define SOFT_PWM_MAX                 100U
#define SOFT_PWM_ISR_HZ              20000U
#define SOFT_PWM_OUTPUT_HZ           (SOFT_PWM_ISR_HZ / SOFT_PWM_MAX)

/* Ultrasonic acquisition is opt-in until all four ECHO inputs are level-shifted. */
#define ULTRASONIC_ENABLED           0
#define ULTRASONIC_TRIGGER_US        10U
#define ULTRASONIC_ECHO_TIMEOUT_US   30000U
#define ULTRASONIC_GUARD_US          60000U

#define MOTOR_LEFT_FRONT_INVERTED    0
#define MOTOR_LEFT_REAR_INVERTED     0
#define MOTOR_RIGHT_FRONT_INVERTED   1
#define MOTOR_RIGHT_REAR_INVERTED    1

#endif /* APP_CONFIG_H */
