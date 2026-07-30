#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint32_t valid_frames;
  uint32_t crc_errors;
  uint32_t invalid_frames;
  uint32_t timeout_count;
  uint32_t uart_overflow_count;
  int16_t throttle;
  int16_t steering;
  int16_t target_left;
  int16_t target_right;
  int16_t current_left;
  int16_t current_right;
  bool enabled;
  bool emergency_stop;
  bool communication_timeout;
} ControlStatus;

void ControlSystem_Init(void);
void ControlSystem_ProcessProtocol(void);
void ControlSystem_Task1ms(void);
void ControlSystem_Task10ms(void);
void ControlSystem_ReportWatchdogFailure(void);
const ControlStatus *ControlSystem_GetStatus(void);

#endif /* CONTROL_SYSTEM_H */
