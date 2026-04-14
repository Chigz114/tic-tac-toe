#ifndef AXIS_CONTROL_H
#define AXIS_CONTROL_H

#include "main.h"

// It's good practice to include necessary headers directly.
#include <stdint.h>

/* Type Definitions */
typedef enum {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
} Axis_t;

typedef enum {
    DIR_NEGATIVE = 0,
    DIR_POSITIVE = 1
} Direction_t;

// Expose timer handles used by axis control functions
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;


/* Function Prototypes */

void AxisControl_Init(void);
void Axis_UpdateFrequency(Axis_t axis, uint32_t freq_hz);
void Axis_SetDirection(Axis_t axis, Direction_t dir);
void Axis_StartPWM(Axis_t axis, uint32_t duration_ms);
void Axis_Stop(Axis_t axis);
uint8_t Axis_IsAnyMoving(void);
void Axis_PulseHandler(void);
void Axis_LimitSwitchCallback(uint16_t GPIO_Pin);
void Axis_ResetAllTimers(void);

// Getters for axis state
uint8_t Axis_IsHoming(void);

#endif // AXIS_CONTROL_H 
