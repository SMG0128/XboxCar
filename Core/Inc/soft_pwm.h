#ifndef SOFT_PWM_H
#define SOFT_PWM_H

#include <stdint.h>

typedef enum
{
  SOFT_PWM_LEFT_FRONT = 0,
  SOFT_PWM_LEFT_REAR,
  SOFT_PWM_RIGHT_FRONT,
  SOFT_PWM_RIGHT_REAR,
  SOFT_PWM_CHANNEL_COUNT
} SoftPwmChannel;

void SoftPwm_Init(void);
void SoftPwm_SetDuty(SoftPwmChannel channel, uint8_t duty);
void SoftPwm_StopAll(void);
void SoftPwm_TimerIrqHandler(void);

#endif /* SOFT_PWM_H */
