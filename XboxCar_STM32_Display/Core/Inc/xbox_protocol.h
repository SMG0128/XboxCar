#ifndef XBOX_PROTOCOL_H
#define XBOX_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define XBOX_PROTOCOL_FRAME_LENGTH 30U
#define XBOX_PROTOCOL_TIMEOUT_MS 200U

typedef struct
{
  int16_t left;
  int16_t right;
  uint16_t sequence;
  uint8_t command;
  bool connected;
} XboxControlState;

void XboxProtocol_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef XboxProtocol_StartReceive(void);
bool XboxProtocol_Process(void);
bool XboxProtocol_GetState(uint32_t now_ms, XboxControlState *state);

#ifdef __cplusplus
}
#endif

#endif /* XBOX_PROTOCOL_H */
