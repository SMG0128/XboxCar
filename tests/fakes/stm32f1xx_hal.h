#ifndef TEST_STM32F1XX_HAL_H
#define TEST_STM32F1XX_HAL_H

#include <stdint.h>

typedef struct
{
  void *Instance;
} UART_HandleTypeDef;

typedef enum
{
  HAL_OK = 0,
  HAL_ERROR = 1
} HAL_StatusTypeDef;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,
                                     uint8_t *data, uint16_t size);
uint32_t HAL_GetTick(void);

#endif /* TEST_STM32F1XX_HAL_H */
