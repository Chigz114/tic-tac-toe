#ifndef HOMING_H
#define HOMING_H

#include <stdint.h>

void Homing_Run(void);
uint8_t Homing_IsInProgress(void);
void Homing_SetCompleted(void);
void Homing_LimitSwitchCallback(uint16_t GPIO_Pin);

#endif // HOMING_H 
