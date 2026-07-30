#ifndef APP_H
#define APP_H

#include "stm32f1xx_hal.h"

void App_Init(UART_HandleTypeDef *control_uart, I2C_HandleTypeDef *display_i2c);
void App_Loop(void);

#endif /* APP_H */
