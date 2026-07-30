#include "control_protocol.h"

#include <stddef.h>
#include <string.h>

#define RX_RING_SIZE 256U
#define RX_RING_MASK (RX_RING_SIZE - 1U)

typedef enum
{
  FRAME_INVALID = 0,
  FRAME_VALID,
  FRAME_CRC_ERROR,
  FRAME_RANGE_ERROR
} FrameResult;

static UART_HandleTypeDef *protocol_uart;
static uint8_t receive_byte;
static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t irq_overflow_count;
static uint32_t handled_fault_count;

static uint8_t parser_frame[CONTROL_PROTOCOL_FRAME_LENGTH];
static uint8_t parser_index;
static ControlProtocolStats statistics;

static bool ParseUnsigned4(const uint8_t *text, uint16_t *value)
{
  uint16_t result = 0U;
  uint8_t index;

  if (text == NULL || value == NULL)
  {
    return false;
  }
  for (index = 0U; index < 4U; ++index)
  {
    if (text[index] < (uint8_t)'0' || text[index] > (uint8_t)'9')
    {
      return false;
    }
    result = (uint16_t)(result * 10U + text[index] - (uint8_t)'0');
  }
  *value = result;
  return true;
}

static bool ParseSigned4(const uint8_t *text, int16_t *value, bool *out_of_range)
{
  uint16_t magnitude;

  if (text == NULL || value == NULL || out_of_range == NULL ||
      (text[0] != (uint8_t)'+' && text[0] != (uint8_t)'-') ||
      !ParseUnsigned4(&text[1], &magnitude))
  {
    return false;
  }
  if (magnitude > 1000U)
  {
    *out_of_range = true;
    return false;
  }
  *value = (text[0] == (uint8_t)'-') ? -(int16_t)magnitude
                                      : (int16_t)magnitude;
  return true;
}

static bool ParseHexByte(const uint8_t *text, uint8_t *value)
{
  uint8_t result = 0U;
  uint8_t index;
  uint8_t nibble;

  if (text == NULL || value == NULL)
  {
    return false;
  }
  for (index = 0U; index < 2U; ++index)
  {
    if (text[index] >= (uint8_t)'0' && text[index] <= (uint8_t)'9')
    {
      nibble = (uint8_t)(text[index] - (uint8_t)'0');
    }
    else if (text[index] >= (uint8_t)'A' && text[index] <= (uint8_t)'F')
    {
      nibble = (uint8_t)(text[index] - (uint8_t)'A' + 10U);
    }
    else
    {
      return false;
    }
    result = (uint8_t)((result << 4U) | nibble);
  }
  *value = result;
  return true;
}

static bool ParseCommand(const uint8_t *text, uint8_t *command)
{
  uint8_t result = 0U;
  uint8_t index;

  if (text == NULL || command == NULL)
  {
    return false;
  }
  for (index = 0U; index < 4U; ++index)
  {
    if (text[index] != (uint8_t)'0' && text[index] != (uint8_t)'1')
    {
      return false;
    }
    result = (uint8_t)((result << 1U) |
                       (text[index] - (uint8_t)'0'));
  }
  *command = result;
  return true;
}

static FrameResult ParseFrame(const uint8_t *frame, ControlCommand *command)
{
  static const uint8_t comma_positions[] = {3U, 8U, 14U, 20U, 25U};
  uint8_t calculated_crc = 0U;
  uint8_t received_crc;
  uint8_t index;
  bool out_of_range = false;

  if (frame == NULL || command == NULL ||
      frame[0] != (uint8_t)'$' || frame[1] != (uint8_t)'X' ||
      frame[2] != (uint8_t)'C' || frame[28] != (uint8_t)'\r' ||
      frame[29] != (uint8_t)'\n')
  {
    return FRAME_INVALID;
  }
  for (index = 0U; index < (uint8_t)sizeof(comma_positions); ++index)
  {
    if (frame[comma_positions[index]] != (uint8_t)',')
    {
      return FRAME_INVALID;
    }
  }
  for (index = 1U; index <= 24U; ++index)
  {
    calculated_crc ^= frame[index];
  }
  if (!ParseHexByte(&frame[26], &received_crc))
  {
    return FRAME_INVALID;
  }
  if (received_crc != calculated_crc)
  {
    return FRAME_CRC_ERROR;
  }
  if (!ParseCommand(&frame[4], &command->command) ||
      !ParseUnsigned4(&frame[21], &command->sequence))
  {
    return FRAME_INVALID;
  }
  if (!ParseSigned4(&frame[9], &command->left, &out_of_range) ||
      !ParseSigned4(&frame[15], &command->right, &out_of_range))
  {
    return out_of_range ? FRAME_RANGE_ERROR : FRAME_INVALID;
  }
  if (command->command > 0x09U && command->command != 0x0fU)
  {
    return FRAME_INVALID;
  }
  return FRAME_VALID;
}

static bool PopByte(uint8_t *value)
{
  uint16_t tail;

  if (value == NULL || rx_tail == rx_head)
  {
    return false;
  }
  tail = rx_tail;
  *value = rx_ring[tail];
  rx_tail = (uint16_t)((tail + 1U) & RX_RING_MASK);
  return true;
}

void ControlProtocol_Init(UART_HandleTypeDef *uart)
{
  protocol_uart = uart;
  rx_head = 0U;
  rx_tail = 0U;
  irq_overflow_count = 0U;
  handled_fault_count = 0U;
  parser_index = 0U;
  memset(&statistics, 0, sizeof(statistics));
}

HAL_StatusTypeDef ControlProtocol_StartReceive(void)
{
  if (protocol_uart == NULL)
  {
    return HAL_ERROR;
  }
  return HAL_UART_Receive_IT(protocol_uart, &receive_byte, 1U);
}

ControlProtocolResult ControlProtocol_Process(ControlCommand *command)
{
  uint8_t byte;
  FrameResult frame_result;

  if (handled_fault_count != irq_overflow_count)
  {
    handled_fault_count = irq_overflow_count;
    parser_index = 0U;
    return CONTROL_PROTOCOL_SEVERE_ERROR;
  }

  while (PopByte(&byte))
  {
    if (byte == (uint8_t)'$')
    {
      parser_index = 0U;
      parser_frame[parser_index++] = byte;
      continue;
    }
    if (parser_index == 0U)
    {
      continue;
    }

    parser_frame[parser_index++] = byte;
    if (parser_index < CONTROL_PROTOCOL_FRAME_LENGTH)
    {
      continue;
    }
    parser_index = 0U;
    frame_result = ParseFrame(parser_frame, command);
    if (frame_result == FRAME_VALID)
    {
      ++statistics.valid_frames;
      return CONTROL_PROTOCOL_VALID;
    }
    if (frame_result == FRAME_CRC_ERROR)
    {
      ++statistics.crc_errors;
    }
    else
    {
      ++statistics.invalid_frames;
      if (frame_result == FRAME_RANGE_ERROR)
      {
        return CONTROL_PROTOCOL_SEVERE_ERROR;
      }
    }
  }
  return CONTROL_PROTOCOL_NONE;
}

const ControlProtocolStats *ControlProtocol_GetStats(void)
{
  statistics.uart_overflow_count = irq_overflow_count;
  return &statistics;
}

#ifdef UNIT_TEST
void ControlProtocol_FeedByteForTest(uint8_t byte)
{
  uint16_t head = rx_head;
  uint16_t next = (uint16_t)((head + 1U) & RX_RING_MASK);

  if (next == rx_tail)
  {
    ++irq_overflow_count;
    return;
  }
  rx_ring[head] = byte;
  rx_head = next;
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  uint16_t head;
  uint16_t next;

  if (protocol_uart == NULL || uart != protocol_uart)
  {
    return;
  }
  head = rx_head;
  next = (uint16_t)((head + 1U) & RX_RING_MASK);
  if (next == rx_tail)
  {
    ++irq_overflow_count;
  }
  else
  {
    rx_ring[head] = receive_byte;
    rx_head = next;
  }
  (void)HAL_UART_Receive_IT(protocol_uart, &receive_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (protocol_uart == NULL || uart != protocol_uart)
  {
    return;
  }
  ++irq_overflow_count;
  (void)HAL_UART_Receive_IT(protocol_uart, &receive_byte, 1U);
}
