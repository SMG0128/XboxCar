#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  MOTOR_LEFT_FRONT = 0,
  MOTOR_LEFT_REAR,
  MOTOR_RIGHT_FRONT,
  MOTOR_RIGHT_REAR,
  MOTOR_COUNT
} MotorId;

void Motor_Init(void);
void Motor_SetSingle(MotorId id, int16_t speed);
void Motor_SetLeftRight(int16_t left, int16_t right);
void Motor_StopAll(void);
void Motor_EmergencyStop(void);
void Motor_ClearEmergencyStop(void);
bool Motor_IsEmergencyStopped(void);
void Motor_Task10ms(void);
int16_t Motor_GetCurrentLeft(void);
int16_t Motor_GetCurrentRight(void);

#endif /* MOTOR_H */
