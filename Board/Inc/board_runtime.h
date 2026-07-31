#ifndef BOARD_RUNTIME_H
#define BOARD_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#include <stdbool.h>

/*
 * STM32 adapter for the portable App layer. The adapter owns GPIO, TIM4,
 * USART1 receive re-arming, the control scheduler and the optional display.
 */
bool BoardRuntime_Init(UART_HandleTypeDef *control_uart,
                       I2C_HandleTypeDef *display_i2c);
void BoardRuntime_Run(void);

/* Called only by the TIM4 update ISR. */
void BoardRuntime_TimerIrqHandler(void);

/* Safe even during an exception or before BoardRuntime_Init has completed. */
void BoardRuntime_FaultStop(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_RUNTIME_H */
