#include "motion_planner.h"
#include "axis_control.h"
#include "homing.h"
#include <stdlib.h> // for abs()
#include "main.h" // For HAL_Delay

#define MIN_FREQUENCY_HZ 100

/* Coordinate System */
static volatile int32_t current_pos_x = 0;
static volatile int32_t current_pos_y = 0;
static volatile int32_t current_pos_z = 0;
static volatile int32_t target_pos_x  = 0;
static volatile int32_t target_pos_y  = 0;
static volatile int32_t target_pos_z  = 0;


uint8_t Motion_MoveTo(int32_t tx, int32_t ty, int32_t tz, uint32_t vmax)
{
    Axis_ResetAllTimers();
    
    if (Axis_IsAnyMoving()) return 1;

    int32_t dx = tx - current_pos_x,  sx = abs(dx);
    int32_t dy = ty - current_pos_y,  sy = abs(dy);
    int32_t dz = tz - current_pos_z,  sz = abs(dz);
    if (!sx && !sy && !sz) return 2;

    uint32_t freq = vmax ? vmax : 5000;
    if (freq < MIN_FREQUENCY_HZ) freq = MIN_FREQUENCY_HZ;
    if (freq > 50000) freq = 50000;
    
    if (sx) Axis_UpdateFrequency(AXIS_X, freq);
    if (sy) Axis_UpdateFrequency(AXIS_Y, freq);
    if (sz) Axis_UpdateFrequency(AXIS_Z, freq);

    if (sx > 0) {
        Axis_SetDirection(AXIS_X, (dx > 0) ? DIR_POSITIVE : DIR_NEGATIVE);
    }
    if (sy > 0) {
        Axis_SetDirection(AXIS_Y, (dy > 0) ? DIR_POSITIVE : DIR_NEGATIVE);
    }
    if (sz > 0) {
        Axis_SetDirection(AXIS_Z, (dz > 0) ? DIR_POSITIVE : DIR_NEGATIVE);
    }

    HAL_Delay(2);

    if (sy) {
        Axis_StartPWM(AXIS_Y, (sy*1000)/freq);
    }
    if (sz) {
        Axis_StartPWM(AXIS_Z, (sz*1000)/freq);
    }
    if (sx) {
        Axis_StartPWM(AXIS_X, (sx*1000)/freq);
    }

    if (sx) target_pos_x = tx;
    if (sy) target_pos_y = ty;
    if (sz) target_pos_z = tz;

    return 0;
}

void Motion_WaitAll(void)
{
    while (Axis_IsAnyMoving()) {
        Axis_PulseHandler();
        HAL_Delay(1);
    }
    // After movement, update current position to target position
    current_pos_x = target_pos_x;
    current_pos_y = target_pos_y;
    current_pos_z = target_pos_z;
}

void Motion_GetPosition(int32_t *x, int32_t *y, int32_t *z)
{
    if (x) *x = current_pos_x;
    if (y) *y = current_pos_y;
    if (z) *z = current_pos_z;
}

void Motion_LimitSwitchCallback(uint16_t GPIO_Pin)
{
    if (Homing_IsInProgress())
    {
        switch (GPIO_Pin)
        {
        case GPIO_PIN_0:
            current_pos_x = 0;
            target_pos_x = 0;
            break;
        case GPIO_PIN_1:
            current_pos_y = 0;
            target_pos_y = 0;
            break;
        case GPIO_PIN_2:
            current_pos_z = 0;
            target_pos_z = 0;
            break;
        }
    }
}
void Motion_ResetPosition(void){
    current_pos_x=0;
    current_pos_y=0;
    current_pos_z=0;
    target_pos_x=0;
    target_pos_y=0;
    target_pos_z=0;
} 
