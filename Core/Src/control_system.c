#include "control_system.h"

#include "app_config.h"
#include "control_protocol.h"
#include "motor.h"

#include "stm32f1xx_hal.h"

#include <string.h>

#define COMMAND_EMERGENCY_STOP 0x07U
#define COMMAND_NO_INPUT       0x08U
#define COMMAND_DISCONNECTED   0x09U
#define COMMAND_ERROR          0x0fU

static ControlStatus status;
static uint32_t last_valid_frame_ms;
static bool has_valid_frame;
static uint8_t recovery_frames;

static void RefreshStatistics(void)
{
  const ControlProtocolStats *protocol = ControlProtocol_GetStats();

  status.valid_frames = protocol->valid_frames;
  status.crc_errors = protocol->crc_errors;
  status.invalid_frames = protocol->invalid_frames;
  status.uart_overflow_count = protocol->uart_overflow_count;
  status.current_left = Motor_GetCurrentLeft();
  status.current_right = Motor_GetCurrentRight();
  status.emergency_stop = Motor_IsEmergencyStopped();
}

static void EnterEmergencyStop(void)
{
  status.enabled = false;
  status.emergency_stop = true;
  status.target_left = 0;
  status.target_right = 0;
  recovery_frames = 0U;
  Motor_EmergencyStop();
}

static void HandleCommand(const ControlCommand *command)
{
  bool drive_command;

  drive_command = command->command <= 0x06U;
  if (command->command == COMMAND_EMERGENCY_STOP ||
      command->command == COMMAND_ERROR)
  {
    EnterEmergencyStop();
    return;
  }
  if (command->command == COMMAND_NO_INPUT ||
      command->command == COMMAND_DISCONNECTED)
  {
    recovery_frames = 0U;
    status.enabled = false;
    status.target_left = 0;
    status.target_right = 0;
    Motor_StopAll();
    return;
  }
  if (!drive_command)
  {
    EnterEmergencyStop();
    return;
  }

  if (Motor_IsEmergencyStopped())
  {
    ++recovery_frames;
    if (recovery_frames < CONTROL_RECOVERY_FRAMES)
    {
      return;
    }
    Motor_ClearEmergencyStop();
    recovery_frames = 0U;
  }

  status.enabled = true;
  status.target_left = command->left;
  status.target_right = command->right;
  /* Existing ESP32 protocol carries already mixed LEFT/RIGHT values. */
  status.throttle = (int16_t)(((int32_t)command->left + command->right) / 2);
  status.steering = (int16_t)(((int32_t)command->left - command->right) / 2);
  Motor_SetLeftRight(command->left, command->right);
}

void ControlSystem_Init(void)
{
  memset(&status, 0, sizeof(status));
  status.communication_timeout = true;
  last_valid_frame_ms = HAL_GetTick();
  has_valid_frame = false;
  recovery_frames = 0U;
  Motor_StopAll();
}

void ControlSystem_ProcessProtocol(void)
{
  ControlCommand command;
  ControlProtocolResult result;
  uint8_t processed = 0U;

  do
  {
    result = ControlProtocol_Process(&command);
    if (result == CONTROL_PROTOCOL_SEVERE_ERROR)
    {
      EnterEmergencyStop();
    }
    else if (result == CONTROL_PROTOCOL_VALID)
    {
      has_valid_frame = true;
      last_valid_frame_ms = HAL_GetTick();
      status.communication_timeout = false;
      HandleCommand(&command);
      ++processed;
    }
  } while (result != CONTROL_PROTOCOL_NONE && processed < 8U);
  RefreshStatistics();
}

void ControlSystem_Task1ms(void)
{
  uint32_t now = HAL_GetTick();

  if (has_valid_frame &&
      !status.communication_timeout &&
      (uint32_t)(now - last_valid_frame_ms) > CONTROL_TIMEOUT_MS)
  {
    status.communication_timeout = true;
    status.enabled = false;
    status.target_left = 0;
    status.target_right = 0;
    recovery_frames = 0U;
    ++status.timeout_count;
    Motor_StopAll();
  }
  RefreshStatistics();
}

void ControlSystem_Task10ms(void)
{
  RefreshStatistics();
}

void ControlSystem_ReportWatchdogFailure(void)
{
  EnterEmergencyStop();
}

const ControlStatus *ControlSystem_GetStatus(void)
{
  return &status;
}
