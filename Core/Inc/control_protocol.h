#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define CONTROL_PROTOCOL_FRAME_LENGTH 30U

typedef struct
{
  int16_t left;
  int16_t right;
  uint16_t sequence;
  uint8_t command;
} ControlCommand;

typedef struct
{
  uint32_t valid_frames;
  uint32_t crc_errors;
  uint32_t invalid_frames;
  uint32_t uart_overflow_count;
} ControlProtocolStats;

typedef enum
{
  CONTROL_PROTOCOL_NONE = 0,
  CONTROL_PROTOCOL_VALID,
  CONTROL_PROTOCOL_SEVERE_ERROR
} ControlProtocolResult;

void ControlProtocol_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef ControlProtocol_StartReceive(void);
ControlProtocolResult ControlProtocol_Process(ControlCommand *command);
const ControlProtocolStats *ControlProtocol_GetStats(void);

#ifdef UNIT_TEST
void ControlProtocol_FeedByteForTest(uint8_t byte);
#endif

#endif /* CONTROL_PROTOCOL_H */
