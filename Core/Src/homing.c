#include "homing.h"
#include "axis_control.h"
#include "motion_planner.h"
#include "main.h"

#define HOMING_FREQUENCY   4000
#define HOMING_DURATION_MS 200000

static volatile uint8_t homingInProgress = 0;
static volatile uint8_t systemHomed = 0;

void Homing_Run(void)
{
    homingInProgress = 1;

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

    Axis_SetDirection(AXIS_Y, DIR_NEGATIVE);
    Axis_SetDirection(AXIS_Z, DIR_NEGATIVE);
    Axis_SetDirection(AXIS_X, DIR_NEGATIVE);

    HAL_Delay(10);

    Axis_UpdateFrequency(AXIS_X, HOMING_FREQUENCY);
    Axis_UpdateFrequency(AXIS_Y, HOMING_FREQUENCY);
    Axis_UpdateFrequency(AXIS_Z, HOMING_FREQUENCY);
    
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) {
        Axis_StartPWM(AXIS_X, HOMING_DURATION_MS);
    } else {
        Motion_ResetPosition(); // Reset X part of position
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) {
        Axis_StartPWM(AXIS_Y, HOMING_DURATION_MS);
    } else {
        Motion_ResetPosition(); // Reset Y part of position
    }

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET) {
        Axis_StartPWM(AXIS_Z, HOMING_DURATION_MS);
    } else {
        Motion_ResetPosition(); // Reset Z part of position
    }

    while (Axis_IsAnyMoving()) {
        // This wait could be handled better, but for now, just wait.
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

    homingInProgress = 0;
    systemHomed = 1;

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
}

uint8_t Homing_IsInProgress(void)
{
    return homingInProgress;
}

void Homing_LimitSwitchCallback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case GPIO_PIN_0:
        Axis_Stop(AXIS_X);
        break;
    case GPIO_PIN_1:
        Axis_Stop(AXIS_Y);
        break;
    case GPIO_PIN_2:
        Axis_Stop(AXIS_Z);
        break;
    }
    Motion_LimitSwitchCallback(GPIO_Pin);
} 
