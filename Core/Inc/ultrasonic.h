#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  ULTRASONIC_FRONT_LEFT = 0,
  ULTRASONIC_FRONT_RIGHT,
  ULTRASONIC_BACK_LEFT,
  ULTRASONIC_BACK_RIGHT,
  ULTRASONIC_SENSOR_COUNT
} UltrasonicId;

void Ultrasonic_Init(void);
void Ultrasonic_Task(void);
bool Ultrasonic_GetDistance(UltrasonicId id, uint16_t *distance_mm);

#endif /* ULTRASONIC_H */
