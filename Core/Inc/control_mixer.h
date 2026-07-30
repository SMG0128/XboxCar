#ifndef CONTROL_MIXER_H
#define CONTROL_MIXER_H

#include <stdint.h>

int16_t ControlMixer_ApplyDeadzone(int16_t value);
void ControlMixer_Mix(int16_t throttle, int16_t steering,
                      int16_t *left, int16_t *right);
int16_t ControlMixer_Approach(int16_t current, int16_t target, int16_t step);

#endif /* CONTROL_MIXER_H */
