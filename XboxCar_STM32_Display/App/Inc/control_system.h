#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The control state machine and the one place the control chain is sequenced.
 *
 *     UART byte -> ring -> streaming parse -> accepted frame
 *       -> communication and sequence management
 *       -> emergency / fault arbitration
 *       -> obstacle limiting
 *       -> speed ramp
 *       -> four motor outputs
 *
 * Priority is strict and is the reason this module exists rather than being
 * spread across the callers:
 *
 *     internal fault > emergency stop > communication timeout > obstacle limit
 *
 * A sensor can only make the vehicle slower. Nothing a sensor reports can
 * release a latched stop, and no ordinary control frame can either.
 *
 * Holds no HAL dependency. Ultrasonic readings arrive as a plain snapshot, so
 * the host tests drive the whole chain with a table of numbers.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_types.h"
#include "diagnostics.h"
#include "motor.h"
#include "safety_controller.h"
#include "xbox_protocol.h"

/* Ultrasonic readings as handed to the control system. */
typedef struct {
  uint16_t distance_mm[ULTRASONIC_COUNT];
  uint16_t raw_mm[ULTRASONIC_COUNT];
  uint32_t timeouts[ULTRASONIC_COUNT];
  uint8_t valid_mask;
  uint8_t fault_mask;
} SensorSnapshot;

typedef struct {
  XboxProtocol protocol;
  MotorController motor;
  SafetyController safety;
  AppDebugState debug;

  ControlState state;
  ControlState previous_state;

  /* Timestamp of the last frame that passed every check. */
  uint32_t last_accepted_ms;
  bool have_accepted_frame;

  /* Emergency latch and its recovery run. */
  bool emergency_locked;
  uint8_t recovery_count;

  /* Accumulated INTERNAL_FAULT_* bits. Any bit latches the output stage off. */
  uint32_t internal_faults;

  /* Most recent accepted request, before limiting. */
  int16_t requested_left;
  int16_t requested_right;
} ControlSystem;

/* Brings up every submodule with the output stage disabled. */
void ControlSystem_Init(ControlSystem *system);

/*
 * Hands one received byte to the protocol ring. Safe to call from the UART
 * interrupt; performs no parsing.
 */
void ControlSystem_PushRxByte(ControlSystem *system, uint8_t byte);

/*
 * Runs one control period. Must be called every CONTROL_PERIOD_MS.
 *
 * sensors may be NULL, which is equivalent to a snapshot with no valid sensors.
 */
void ControlSystem_Update(ControlSystem *system, uint32_t now_ms,
                          const SensorSnapshot *sensors);

/*
 * Latches an internal fault. The output stage is cut on the next update and
 * stays off until the MCU is reset, which is the only honest response to a
 * peripheral or state integrity failure.
 */
void ControlSystem_ReportInternalFault(ControlSystem *system, uint16_t fault_code,
                                       uint32_t internal_flag, uint32_t now_ms);

/* Current state. */
ControlState ControlSystem_GetState(const ControlSystem *system);

/* Read-only view of the runtime snapshot. */
const AppDebugState *ControlSystem_GetDebugState(const ControlSystem *system);

/* True when accepted frames are arriving inside the timeout window. */
bool ControlSystem_IsOnline(const ControlSystem *system);

/* True while the emergency latch is engaged. */
bool ControlSystem_IsEmergencyLocked(const ControlSystem *system);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SYSTEM_H */
