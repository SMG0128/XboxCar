#include "test_helpers.h"

#include <stdio.h>
#include <string.h>

size_t TestHelper_BuildFrame(char *out, uint8_t command, int16_t left,
                             int16_t right, uint16_t sequence)
{
  char payload[32];
  char command_text[5];
  int index;
  int payload_length;
  uint8_t crc;

  for (index = 3; index >= 0; --index)
  {
    command_text[3 - index] = ((command & (1U << index)) != 0U) ? '1' : '0';
  }
  command_text[4] = '\0';

  payload_length = snprintf(payload, sizeof(payload), "XC,%s,%+05d,%+05d,%04u",
                            command_text, (int)left, (int)right,
                            (unsigned)(sequence % 10000U));

  crc = XboxProtocol_Crc((const uint8_t *)payload, (uint16_t)payload_length);

  return (size_t)snprintf(out, XBOX_FRAME_LENGTH + 1U, "$%s,%02X\r\n", payload,
                          crc);
}

void TestHelper_PushBytes(ControlSystem *system, const char *data, size_t length)
{
  size_t index;

  for (index = 0U; index < length; ++index)
  {
    ControlSystem_PushRxByte(system, (uint8_t)data[index]);
  }
}

void TestHelper_PushString(ControlSystem *system, const char *text)
{
  TestHelper_PushBytes(system, text, strlen(text));
}

uint16_t TestHelper_PushFrame(ControlSystem *system, uint8_t command,
                              int16_t left, int16_t right, uint16_t sequence)
{
  char frame[XBOX_FRAME_LENGTH + 1U];
  size_t length = TestHelper_BuildFrame(frame, command, left, right, sequence);

  TestHelper_PushBytes(system, frame, length);
  return sequence;
}

uint32_t TestHelper_RunFor(ControlSystem *system, uint32_t now_ms,
                           uint32_t duration_ms, const SensorSnapshot *sensors)
{
  uint32_t elapsed = 0U;

  while (elapsed < duration_ms)
  {
    now_ms += CONTROL_PERIOD_MS;
    elapsed += CONTROL_PERIOD_MS;
    ControlSystem_Update(system, now_ms, sensors);
  }

  return now_ms;
}

void TestHelper_UniformSensors(SensorSnapshot *sensors, uint16_t distance_mm)
{
  uint8_t index;

  memset(sensors, 0, sizeof(*sensors));
  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    sensors->distance_mm[index] = distance_mm;
    sensors->raw_mm[index] = distance_mm;
  }
  sensors->valid_mask = 0x0FU;
  sensors->fault_mask = 0x00U;
}

void TestHelper_NoSensors(SensorSnapshot *sensors)
{
  memset(sensors, 0, sizeof(*sensors));
}
