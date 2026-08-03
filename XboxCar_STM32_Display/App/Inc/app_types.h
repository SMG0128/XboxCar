#ifndef APP_TYPES_H
#define APP_TYPES_H

/*
 * Types shared across the portable application layer.
 *
 * Free of STM32 HAL includes so the host test build can use them unchanged.
 */

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Motors                                                                     */
/* ------------------------------------------------------------------------- */

/*
 * Index order is fixed and relied upon by the diagnostics snapshot, the display
 * pages and the tests. Left pair first, then right pair.
 */
typedef enum {
  MOTOR_FRONT_LEFT = 0,
  MOTOR_REAR_LEFT = 1,
  MOTOR_FRONT_RIGHT = 2,
  MOTOR_REAR_RIGHT = 3
} MotorIndex;

/* ------------------------------------------------------------------------- */
/* Ultrasonic positions                                                       */
/* ------------------------------------------------------------------------- */

typedef enum {
  ULTRASONIC_FRONT = 0,
  ULTRASONIC_REAR = 1,
  ULTRASONIC_LEFT = 2,
  ULTRASONIC_RIGHT = 3
} UltrasonicIndex;

/* ------------------------------------------------------------------------- */
/* Obstacle reporting                                                         */
/* ------------------------------------------------------------------------- */

/* Bit positions match UltrasonicIndex so a sensor index maps to 1u << index. */
#define OBSTACLE_FLAG_FRONT 0x01U
#define OBSTACLE_FLAG_REAR 0x02U
#define OBSTACLE_FLAG_LEFT 0x04U
#define OBSTACLE_FLAG_RIGHT 0x08U

/* Set when the corresponding direction is fully blocked rather than only slowed. */
#define OBSTACLE_FLAG_FRONT_BLOCKED 0x10U
#define OBSTACLE_FLAG_REAR_BLOCKED 0x20U

/* Why the safety stage altered the request. */
#define LIMIT_REASON_NONE 0x00U
#define LIMIT_REASON_FRONT_SLOW 0x01U
#define LIMIT_REASON_FRONT_STOP 0x02U
#define LIMIT_REASON_REAR_SLOW 0x04U
#define LIMIT_REASON_REAR_STOP 0x08U
#define LIMIT_REASON_SIDE_LEFT 0x10U
#define LIMIT_REASON_SIDE_RIGHT 0x20U
#define LIMIT_REASON_SENSOR_FAULT 0x40U

/* ------------------------------------------------------------------------- */
/* Fault codes                                                                */
/* ------------------------------------------------------------------------- */

/*
 * Recorded in AppDebugState::last_fault_code. Values are stable so that a
 * number read off the OLED can be looked up in the bring-up guide.
 */
typedef enum {
  FAULT_NONE = 0,

  /* Communication */
  FAULT_COMM_TIMEOUT = 0x10,
  FAULT_COMM_RX_OVERFLOW = 0x11,

  /* Operator initiated */
  FAULT_EMERGENCY_STOP = 0x20,

  /* Internal, all latch the output stage off */
  FAULT_INTERNAL_PWM_TIMER = 0x30,
  FAULT_INTERNAL_GPIO_INIT = 0x31,
  FAULT_INTERNAL_CONFIG_CHECK = 0x32,
  FAULT_INTERNAL_MOTOR_INDEX = 0x33,
  FAULT_INTERNAL_STATE_CORRUPT = 0x34,
  FAULT_INTERNAL_UART_INIT = 0x35,

  /* Peripheral, non-fatal */
  FAULT_DISPLAY_UNAVAILABLE = 0x40,
  FAULT_ULTRASONIC_SENSOR = 0x41
} FaultCode;

/* Bit flags accumulated in AppDebugState::internal_faults. */
#define INTERNAL_FAULT_PWM_TIMER 0x0001U
#define INTERNAL_FAULT_GPIO_INIT 0x0002U
#define INTERNAL_FAULT_CONFIG_CHECK 0x0004U
#define INTERNAL_FAULT_MOTOR_INDEX 0x0008U
#define INTERNAL_FAULT_STATE_CORRUPT 0x0010U
#define INTERNAL_FAULT_UART_INIT 0x0020U

/* ------------------------------------------------------------------------- */
/* Control state machine                                                      */
/* ------------------------------------------------------------------------- */

typedef enum {
  /* Powered up, nothing accepted yet. Output stage is off. */
  CONTROL_STATE_STARTUP_SAFE = 0,
  /* Accepted frames are arriving inside the timeout window. */
  CONTROL_STATE_ONLINE = 1,
  /* No accepted frame inside the timeout window. Output forced to zero. */
  CONTROL_STATE_COMM_TIMEOUT = 2,
  /* Latched. Only a run of accepted non-emergency frames clears it. */
  CONTROL_STATE_EMERGENCY_LOCKED = 3,
  /* Unrecoverable internal problem. Output stays off until reset. */
  CONTROL_STATE_INTERNAL_FAULT = 4
} ControlState;

/* ------------------------------------------------------------------------- */
/* Controller presence                                                        */
/* ------------------------------------------------------------------------- */

/*
 * Why the vehicle is not accepting stick input. Exactly one reason is published
 * per control period; the display turns it into "No Xbox" and the debug log
 * prints the name, so an operator watching either one sees the same cause.
 *
 * Ordered by the arbitration priority in ControlSystem_Update, most severe
 * first, so a numerically larger reason never masks a more serious one.
 */
typedef enum {
  NO_XBOX_REASON_NONE = 0,
  /* An internal fault latched the output stage off. */
  NO_XBOX_REASON_INTERNAL_FAULT = 1,
  /* Emergency stop latched, waiting for the recovery run. */
  NO_XBOX_REASON_EMERGENCY_LOCKED = 2,
  /* Powered up but no frame has ever passed validation. */
  NO_XBOX_REASON_NO_FRAME_YET = 3,
  /* No accepted frame inside COMM_TIMEOUT_MS. Covers a cut ESP32 link. */
  NO_XBOX_REASON_CONTROL_TIMEOUT = 4,
  /* Frames are arriving and the ESP32 says the controller is not paired. */
  NO_XBOX_REASON_ESP_REPORTED_DISCONNECTED = 5,
  /* Frames are arriving but consecutively failing validation. */
  NO_XBOX_REASON_FRAME_ERRORS = 6
} NoXboxReason;

/* ------------------------------------------------------------------------- */
/* Motor direction, as published to the diagnostics snapshot                  */
/* ------------------------------------------------------------------------- */

typedef enum {
  MOTOR_DIR_STOP = 0,
  MOTOR_DIR_FORWARD = 1,
  MOTOR_DIR_REVERSE = 2
} MotorDirection;

#endif /* APP_TYPES_H */
