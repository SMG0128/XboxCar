#ifndef SSD1306_SIMPLE_H
#define SSD1306_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

bool SSD1306_Init(I2C_HandleTypeDef *i2c);
bool SSD1306_RenderControl(bool connected, int16_t left, int16_t right);
bool SSD1306_FlushPage(uint8_t page);
bool SSD1306_ShowControl(bool connected, int16_t left, int16_t right);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_SIMPLE_H */
