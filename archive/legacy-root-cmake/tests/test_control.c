#include "app_config.h"
#include "control_mixer.h"
#include "control_protocol.h"
#include "control_system.h"
#include "motor.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t fake_tick;
static bool fake_emergency;
static int16_t fake_left;
static int16_t fake_right;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,
                                     uint8_t *data, uint16_t size)
{
  (void)uart;
  (void)data;
  (void)size;
  return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
  return fake_tick;
}

void Motor_Init(void) {}
void Motor_SetSingle(MotorId id, int16_t speed)
{
  (void)id;
  (void)speed;
}
void Motor_SetLeftRight(int16_t left, int16_t right)
{
  if (!fake_emergency)
  {
    fake_left = left;
    fake_right = right;
  }
}
void Motor_StopAll(void)
{
  fake_left = 0;
  fake_right = 0;
}
void Motor_EmergencyStop(void)
{
  fake_emergency = true;
  Motor_StopAll();
}
void Motor_ClearEmergencyStop(void)
{
  fake_emergency = false;
}
bool Motor_IsEmergencyStopped(void)
{
  return fake_emergency;
}
void Motor_Task10ms(void) {}
int16_t Motor_GetCurrentLeft(void)
{
  return fake_left;
}
int16_t Motor_GetCurrentRight(void)
{
  return fake_right;
}

static void BuildFrame(char frame[31], const char command[5],
                       int left, int right, unsigned sequence)
{
  uint8_t crc = 0U;
  unsigned index;
  int written;

  written = snprintf(frame, 31, "$XC,%s,%+05d,%+05d,%04u,00\r\n",
                     command, left, right, sequence);
  assert(written == 30);
  for (index = 1U; index <= 24U; ++index)
  {
    crc ^= (uint8_t)frame[index];
  }
  (void)snprintf(&frame[26], 5, "%02X\r\n", crc);
}

static void Feed(const char *data, size_t length)
{
  size_t index;
  for (index = 0U; index < length; ++index)
  {
    ControlProtocol_FeedByteForTest((uint8_t)data[index]);
  }
}

static ControlProtocolResult FeedFrame(const char frame[31],
                                       ControlCommand *command)
{
  Feed(frame, CONTROL_PROTOCOL_FRAME_LENGTH);
  return ControlProtocol_Process(command);
}

static void TestDeadzoneAndMixer(void)
{
  int16_t left;
  int16_t right;

  assert(ControlMixer_ApplyDeadzone(0) == 0);
  assert(ControlMixer_ApplyDeadzone(79) == 0);
  assert(ControlMixer_ApplyDeadzone(80) == 0);
  assert(ControlMixer_ApplyDeadzone(1000) == 1000);
  assert(ControlMixer_ApplyDeadzone(-1000) == -1000);

  ControlMixer_Mix(1000, 0, &left, &right);
  assert(left == 1000 && right == 1000);
  ControlMixer_Mix(-1000, 0, &left, &right);
  assert(left == -1000 && right == -1000);
  ControlMixer_Mix(0, 1000, &left, &right);
  assert(left == 1000 && right == -1000);
  ControlMixer_Mix(0, -1000, &left, &right);
  assert(left == -1000 && right == 1000);
  ControlMixer_Mix(500, 500, &left, &right);
  assert(left == 1000 && right == 0);
  ControlMixer_Mix(1000, 500, &left, &right);
  assert(left <= 1000 && right > 0 && left > right);

  assert(ControlMixer_Approach(0, 1000, 40) == 40);
  assert(ControlMixer_Approach(980, 1000, 40) == 1000);
  assert(ControlMixer_Approach(0, -1000, 40) == -40);
}

static void TestProtocol(void)
{
  UART_HandleTypeDef uart = {0};
  ControlCommand command;
  ControlProtocolResult result;
  const ControlProtocolStats *stats;
  char frame[31];
  char second[31];

  ControlProtocol_Init(&uart);
  BuildFrame(frame, "0001", 800, 800, 25);
  assert(FeedFrame(frame, &command) == CONTROL_PROTOCOL_VALID);
  assert(command.left == 800 && command.right == 800 &&
         command.sequence == 25U && command.command == 1U);

  frame[26] = frame[26] == '0' ? '1' : '0';
  assert(FeedFrame(frame, &command) == CONTROL_PROTOCOL_NONE);
  stats = ControlProtocol_GetStats();
  assert(stats->crc_errors == 1U);

  BuildFrame(frame, "0101", -650, 650, 9999);
  Feed("noise$broken", 12U);
  Feed(frame, 11U);
  assert(ControlProtocol_Process(&command) == CONTROL_PROTOCOL_NONE);
  Feed(&frame[11], CONTROL_PROTOCOL_FRAME_LENGTH - 11U);
  assert(ControlProtocol_Process(&command) == CONTROL_PROTOCOL_VALID);
  assert(command.sequence == 9999U);

  BuildFrame(frame, "0000", 0, 0, 0);
  BuildFrame(second, "0110", 650, -650, 1);
  Feed(frame, CONTROL_PROTOCOL_FRAME_LENGTH);
  Feed(second, CONTROL_PROTOCOL_FRAME_LENGTH);
  result = ControlProtocol_Process(&command);
  assert(result == CONTROL_PROTOCOL_VALID && command.sequence == 0U);
  result = ControlProtocol_Process(&command);
  assert(result == CONTROL_PROTOCOL_VALID && command.sequence == 1U);

  BuildFrame(frame, "0001", 1001, 0, 2);
  assert(FeedFrame(frame, &command) == CONTROL_PROTOCOL_SEVERE_ERROR);
  stats = ControlProtocol_GetStats();
  assert(stats->invalid_frames >= 1U);

  ControlProtocol_Init(&uart);
  for (unsigned index = 0U; index < 300U; ++index)
  {
    ControlProtocol_FeedByteForTest((uint8_t)'x');
  }
  assert(ControlProtocol_Process(&command) == CONTROL_PROTOCOL_SEVERE_ERROR);
  assert(ControlProtocol_GetStats()->uart_overflow_count > 0U);
}

static void TestTimeoutAndEmergencyRecovery(void)
{
  UART_HandleTypeDef uart = {0};
  const ControlStatus *status;
  char frame[31];

  fake_tick = 100U;
  fake_emergency = false;
  fake_left = 0;
  fake_right = 0;
  ControlProtocol_Init(&uart);
  ControlSystem_Init();

  BuildFrame(frame, "0001", 400, 400, 10);
  Feed(frame, CONTROL_PROTOCOL_FRAME_LENGTH);
  ControlSystem_ProcessProtocol();
  status = ControlSystem_GetStatus();
  assert(status->enabled && fake_left == 400 && fake_right == 400);

  fake_tick += CONTROL_TIMEOUT_MS + 1U;
  ControlSystem_Task1ms();
  status = ControlSystem_GetStatus();
  assert(status->communication_timeout && !status->enabled);
  assert(fake_left == 0 && fake_right == 0 && status->timeout_count == 1U);

  BuildFrame(frame, "0111", 0, 0, 11);
  Feed(frame, CONTROL_PROTOCOL_FRAME_LENGTH);
  ControlSystem_ProcessProtocol();
  assert(ControlSystem_GetStatus()->emergency_stop);

  BuildFrame(frame, "0001", 300, 300, 12);
  Feed(frame, CONTROL_PROTOCOL_FRAME_LENGTH);
  ControlSystem_ProcessProtocol();
  Feed(frame, CONTROL_PROTOCOL_FRAME_LENGTH);
  ControlSystem_ProcessProtocol();
  assert(fake_emergency && fake_left == 0);
  Feed(frame, CONTROL_PROTOCOL_FRAME_LENGTH);
  ControlSystem_ProcessProtocol();
  assert(!fake_emergency && fake_left == 300 && fake_right == 300);
}

int main(void)
{
  TestDeadzoneAndMixer();
  TestProtocol();
  TestTimeoutAndEmergencyRecovery();
  puts("All STM32 control tests passed.");
  return 0;
}
