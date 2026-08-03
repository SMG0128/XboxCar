#include "xbox_protocol.h"

#include <stddef.h>
#include <string.h>

static UART_HandleTypeDef *protocol_uart;
static uint8_t receive_byte;
static uint8_t receive_frame[XBOX_PROTOCOL_FRAME_LENGTH];
static uint8_t pending_frame[XBOX_PROTOCOL_FRAME_LENGTH];
static volatile uint8_t receive_index;
static volatile bool pending_ready;

static XboxControlState current_state;
static bool has_valid_frame;
static uint32_t last_valid_frame_ms;

static bool ParseDigit(char value, uint8_t *digit)
{
  if (value < '0' || value > '9' || digit == NULL)
  {
    return false;
  }
  *digit = (uint8_t)(value - '0');
  return true;
}

static bool ParseUnsigned4(const uint8_t *text, uint16_t *value)
{
  uint16_t result = 0U;
  uint8_t digit;
  uint8_t index;

  if (text == NULL || value == NULL)
  {
    return false;
  }

  for (index = 0U; index < 4U; ++index)
  {
    if (!ParseDigit((char)text[index], &digit))
    {
      return false;
    }
    result = (uint16_t)(result * 10U + digit);
  }

  *value = result;
  return true;
}

static bool ParseSigned4(const uint8_t *text, int16_t *value)
{
  uint16_t magnitude;

  if (text == NULL || value == NULL ||
      (text[0] != (uint8_t)'+' && text[0] != (uint8_t)'-') ||
      !ParseUnsigned4(&text[1], &magnitude) || magnitude > 1000U)
  {
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
    result = (uint8_t)((result << 1U) | (text[index] - (uint8_t)'0'));
  }

  *command = result;
  return true;
}

static bool ParseFrame(const uint8_t *frame, XboxControlState *state)
{
  static const uint8_t comma_positions[] = {3U, 8U, 14U, 20U, 25U};
  uint8_t calculated_crc = 0U;
  uint8_t received_crc;
  uint8_t index;

  if (frame == NULL || state == NULL ||
      frame[0] != (uint8_t)'$' || frame[1] != (uint8_t)'X' ||
      frame[2] != (uint8_t)'C' ||
      frame[28] != (uint8_t)'\r' || frame[29] != (uint8_t)'\n')
  {
    return false;
  }

  for (index = 0U; index < sizeof(comma_positions); ++index)
  {
    if (frame[comma_positions[index]] != (uint8_t)',')
    {
      return false;
    }
  }

  for (index = 1U; index <= 24U; ++index)
  {
    calculated_crc ^= frame[index];
  }

  if (!ParseHexByte(&frame[26], &received_crc) ||
      received_crc != calculated_crc ||
      !ParseCommand(&frame[4], &state->command) ||
      !ParseSigned4(&frame[9], &state->left) ||
      !ParseSigned4(&frame[15], &state->right) ||
      !ParseUnsigned4(&frame[21], &state->sequence))
  {
    return false;
  }

  if (state->command > 0x09U && state->command != 0x0fU)
  {
    return false;
  }

  state->connected = state->command != 0x09U;

  if (state->command == 0x07U || state->command == 0x08U ||
      state->command == 0x09U || state->command == 0x0fU)
  {
    state->left = 0;
    state->right = 0;
  }

  return true;
}

void XboxProtocol_Init(UART_HandleTypeDef *uart)
{
  protocol_uart = uart;
  receive_index = 0U;
  pending_ready = false;
  has_valid_frame = false;
  last_valid_frame_ms = 0U;
  memset(&current_state, 0, sizeof(current_state));
}

HAL_StatusTypeDef XboxProtocol_StartReceive(void)
{
  if (protocol_uart == NULL)
  {
    return HAL_ERROR;
  }
  return HAL_UART_Receive_IT(protocol_uart, &receive_byte, 1U);
}

bool XboxProtocol_Process(void)
{
  uint8_t local_frame[XBOX_PROTOCOL_FRAME_LENGTH];
  XboxControlState parsed_state;
  uint32_t interrupt_state;

  if (!pending_ready)
  {
    return false;
  }

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  memcpy(local_frame, pending_frame, sizeof(local_frame));
  pending_ready = false;
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }

  if (!ParseFrame(local_frame, &parsed_state))
  {
    return false;
  }

  current_state = parsed_state;
  last_valid_frame_ms = HAL_GetTick();
  has_valid_frame = true;
  return true;
}

bool XboxProtocol_GetState(uint32_t now_ms, XboxControlState *state)
{
  if (state == NULL)
  {
    return false;
  }

  if (!has_valid_frame ||
      (uint32_t)(now_ms - last_valid_frame_ms) > XBOX_PROTOCOL_TIMEOUT_MS ||
      !current_state.connected)
  {
    memset(state, 0, sizeof(*state));
    state->connected = false;
    return false;
  }

  *state = current_state;
  return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (protocol_uart == NULL || uart != protocol_uart)
  {
    return;
  }

  if (receive_byte == (uint8_t)'$')
  {
    receive_index = 0U;
    receive_frame[receive_index++] = receive_byte;
  }
  else if (receive_index > 0U)
  {
    receive_frame[receive_index++] = receive_byte;
    if (receive_index == XBOX_PROTOCOL_FRAME_LENGTH)
    {
      if (!pending_ready)
      {
        memcpy(pending_frame, receive_frame, sizeof(pending_frame));
        pending_ready = true;
      }
      receive_index = 0U;
    }
  }

  (void)HAL_UART_Receive_IT(protocol_uart, &receive_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (protocol_uart == NULL || uart != protocol_uart)
  {
    return;
  }

  receive_index = 0U;
  (void)HAL_UART_Receive_IT(protocol_uart, &receive_byte, 1U);
}
