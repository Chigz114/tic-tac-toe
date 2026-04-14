#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <stdint.h>
#include "chess_logic.h"

uint8_t Motion_MoveTo(int32_t tx, int32_t ty, int32_t tz, uint32_t vmax);
void Motion_WaitAll(void);
void Motion_GetPosition(int32_t *x, int32_t *y, int32_t *z);
void Motion_UpdatePosition(int32_t dx, int32_t dy, int32_t dz);
void Motion_ResetPosition(void);
void Motion_SetTargetPosition(int32_t tx, int32_t ty, int32_t tz);
void Motion_SetCurrentPosition(int32_t x, int32_t y, int32_t z);
void Motion_LimitSwitchCallback(uint16_t GPIO_Pin);
void Motion_PickAndPlace(const BoardPositionCoord_t *src, const BoardPositionCoord_t *dst);

#endif // MOTION_PLANNER_H 
