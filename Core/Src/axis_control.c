#include "axis_control.h"
#include "homing.h"
#include "motion_planner.h"
#include <stdlib.h>

/* === AXIS <-> TIMER / CHANNEL 统一定义 ======================== */
#define Y_HTIM        htim1
#define Y_CHANNEL     TIM_CHANNEL_1

#define Z_HTIM        htim2
#define Z_CHANNEL     TIM_CHANNEL_1

#define X_HTIM        htim8
#define X_CHANNEL_A   TIM_CHANNEL_1   // 双电机 A
#define X_CHANNEL_B   TIM_CHANNEL_2   // 双电机 B
/* ============================================================= */

#define MIN_FREQUENCY_HZ   100

/* Structure to manage state of each PWM channel for pulse generation */
typedef struct {
    volatile uint8_t  active;              /* Is this channel generating pulses?   */
    volatile uint32_t stop_time;           /* Tick time to stop the generation    */
    volatile uint8_t  limit_triggered;     /* Flag to indicate limit switch hit   */
    volatile uint32_t total_pulses;        /* Total pulses (for future use)       */
} PWM_Pulse_State;

/* Array to hold state for the 4 channels (Y, Z, X-A, X-B) */
volatile PWM_Pulse_State pwm_pulse_states[4];

// Flag to indicate homing is in progress
volatile uint8_t homingInProgress = 0;

void AxisControl_Init(void)
{
    for (int i = 0; i < 4; i++) {
        pwm_pulse_states[i].active           = 0;
        pwm_pulse_states[i].stop_time        = 0;
        pwm_pulse_states[i].limit_triggered  = 0;
        pwm_pulse_states[i].total_pulses     = 0;
    }
}

/**
  * @brief  设置指定定时器的 PWM 频率
  * @note   16-bit 自动寻合适 PSC & ARR，shadow→UG 立即生效
  */
static void UpdateAxisFrequency(TIM_HandleTypeDef *htim, uint32_t freq_hz)
{
    if (freq_hz < MIN_FREQUENCY_HZ)   freq_hz = MIN_FREQUENCY_HZ;
    if (freq_hz > 50000)              freq_hz = 50000;

    uint32_t tim_clk;
    
    // Explicitly determine clock source for each timer
    if (htim->Instance == TIM2 || htim->Instance == TIM3 || htim->Instance == TIM4 || 
        htim->Instance == TIM5 || htim->Instance == TIM6 || htim->Instance == TIM7) {
        tim_clk = 84000000; // APB1 timers
    } else { // TIM1, TIM8, TIM9-TIM14 are APB2 timers
        tim_clk = 168000000; // APB2 timers
    }

    uint64_t target  = (uint64_t)tim_clk / freq_hz;

    uint16_t psc = 0, arr = 0;
    for (uint16_t tpsc = 0; tpsc < 0xFFFF; ++tpsc) {
        uint32_t tarr = target / (tpsc + 1);
        if (tarr > 0 && tarr <= 0xFFFF) { psc = tpsc; arr = tarr - 1; break; }
    }

    // Save current CCER state
    uint32_t ccer_backup = htim->Instance->CCER;
    
    HAL_TIM_Base_Stop(htim);
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    
    // Do not clear CCER, but temporarily disable then restore
    htim->Instance->CCER = 0;          // Temporarily disable all channels
    __HAL_TIM_SET_COUNTER(htim, 0);
    htim->Instance->EGR = TIM_EGR_UG;  // Force load new parameters
    htim->Instance->CCER = ccer_backup; // Restore previous channel enable state
    
    HAL_TIM_Base_Start(htim);
}

void Axis_UpdateFrequency(Axis_t axis, uint32_t freq_hz)
{
    switch(axis)
    {
        case AXIS_X:
            UpdateAxisFrequency(&X_HTIM, freq_hz);
            break;
        case AXIS_Y:
            UpdateAxisFrequency(&Y_HTIM, freq_hz);
            break;
        case AXIS_Z:
            UpdateAxisFrequency(&Z_HTIM, freq_hz);
            break;
    }
}

void Axis_SetDirection(Axis_t axis, Direction_t dir)
{
    switch(axis)
    {
        case AXIS_X:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, (dir == DIR_POSITIVE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, (dir == DIR_POSITIVE) ? GPIO_PIN_RESET : GPIO_PIN_SET);
            break;
        case AXIS_Y:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (dir == DIR_POSITIVE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
        case AXIS_Z:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (dir == DIR_NEGATIVE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            break;
    }
}

void Axis_Stop(Axis_t axis)
{
    switch (axis) {
    case AXIS_Y:
        HAL_TIM_PWM_Stop(&Y_HTIM, Y_CHANNEL);
        pwm_pulse_states[0].active = pwm_pulse_states[0].limit_triggered = 0;
        break;
    case AXIS_Z:
        HAL_TIM_PWM_Stop(&Z_HTIM, Z_CHANNEL);
        pwm_pulse_states[1].active = pwm_pulse_states[1].limit_triggered = 0;
        break;
    case AXIS_X:
        HAL_TIM_PWM_Stop(&X_HTIM, X_CHANNEL_A);
        HAL_TIM_PWM_Stop(&X_HTIM, X_CHANNEL_B);
        pwm_pulse_states[2].active = pwm_pulse_states[3].active = 0;
        pwm_pulse_states[2].limit_triggered = pwm_pulse_states[3].limit_triggered = 0;
        break;
    }
}

void Axis_StartPWM(Axis_t axis, uint32_t duration_ms)
{
    uint32_t t0 = HAL_GetTick();
    switch(axis)
    {
        case AXIS_Y:
            pwm_pulse_states[0].active = 1;
            pwm_pulse_states[0].limit_triggered = 0;
            pwm_pulse_states[0].stop_time = t0 + duration_ms;
            HAL_TIM_PWM_Start(&Y_HTIM, Y_CHANNEL);
            __HAL_TIM_SET_COMPARE(&Y_HTIM, Y_CHANNEL, (__HAL_TIM_GET_AUTORELOAD(&Y_HTIM)+1)/2);
            break;
        case AXIS_Z:
            pwm_pulse_states[1].active = 1;
            pwm_pulse_states[1].limit_triggered = 0;
            pwm_pulse_states[1].stop_time = t0 + duration_ms;
            HAL_TIM_PWM_Start(&Z_HTIM, Z_CHANNEL);
            __HAL_TIM_SET_COMPARE(&Z_HTIM, Z_CHANNEL, (__HAL_TIM_GET_AUTORELOAD(&Z_HTIM)+1)/2);
            break;
        case AXIS_X:
            pwm_pulse_states[2].active = pwm_pulse_states[3].active = 1;
            pwm_pulse_states[2].limit_triggered = pwm_pulse_states[3].limit_triggered = 0;
            pwm_pulse_states[2].stop_time = pwm_pulse_states[3].stop_time = t0 + duration_ms;
            HAL_TIM_PWM_Start(&X_HTIM, X_CHANNEL_A);
            HAL_TIM_PWM_Start(&X_HTIM, X_CHANNEL_B);
            uint16_t duty = (__HAL_TIM_GET_AUTORELOAD(&X_HTIM)+1)/2;
            __HAL_TIM_SET_COMPARE(&X_HTIM, X_CHANNEL_A, duty);
            __HAL_TIM_SET_COMPARE(&X_HTIM, X_CHANNEL_B, duty);
            break;
    }
}

uint8_t Axis_IsAnyMoving(void)
{
    return (pwm_pulse_states[0].active ||  // Y轴
            pwm_pulse_states[1].active ||  // Z轴
            pwm_pulse_states[2].active ||  // X轴 motor A
            pwm_pulse_states[3].active);   // X轴 motor B
}

void Axis_PulseHandler(void)
{
    uint32_t current_tick = HAL_GetTick();
    int32_t current_pos_x, current_pos_y, current_pos_z;
    Motion_GetPosition(&current_pos_x, &current_pos_y, &current_pos_z);


    // Y-axis stop check
    if (pwm_pulse_states[0].active && !pwm_pulse_states[0].limit_triggered) {
        if (current_tick >= pwm_pulse_states[0].stop_time) {
            Axis_Stop(AXIS_Y);
            // This direct manipulation of target position is not ideal.
            // Motion_SetCurrentPosition should be used.
        }
    }

    // Z-axis stop check
    if (pwm_pulse_states[1].active && !pwm_pulse_states[1].limit_triggered) {
        if (current_tick >= pwm_pulse_states[1].stop_time) {
            Axis_Stop(AXIS_Z);
        }
    }

    // X-axis stop check
    if ((pwm_pulse_states[2].active || pwm_pulse_states[3].active) &&
        !pwm_pulse_states[2].limit_triggered && !pwm_pulse_states[3].limit_triggered) {

        uint8_t should_stop_x = 0;
        if ((pwm_pulse_states[2].active && current_tick >= pwm_pulse_states[2].stop_time) ||
            (pwm_pulse_states[3].active && current_tick >= pwm_pulse_states[3].stop_time)) {
            should_stop_x = 1;
        }

        if (should_stop_x) {
            Axis_Stop(AXIS_X);
        }
    }
}

static void ResetAxisTimer(TIM_HandleTypeDef *htim, uint32_t channel)
{
    HAL_TIM_PWM_Stop(htim, channel);
    htim->Instance->CCER = 0;
    
    // Handle BDTR only for advanced timers (TIM1, TIM8)
    if (htim->Instance == TIM1 || htim->Instance == TIM8) {
        htim->Instance->BDTR &= ~TIM_BDTR_MOE;
    }
    
    __HAL_TIM_SET_COUNTER(htim, 0);
}

void Axis_ResetAllTimers(void)
{
    ResetAxisTimer(&Y_HTIM, Y_CHANNEL);
    ResetAxisTimer(&Z_HTIM, Z_CHANNEL);
    ResetAxisTimer(&X_HTIM, X_CHANNEL_A);
    ResetAxisTimer(&X_HTIM, X_CHANNEL_B);
}

void PC0_Interrupt_Handler(void)
{
    // X 轴限位触发
    pwm_pulse_states[2].limit_triggered = 1;
    pwm_pulse_states[3].limit_triggered = 1;
    Axis_Stop(AXIS_X);
    Motion_LimitSwitchCallback(GPIO_PIN_0);
}

void PC1_Interrupt_Handler(void)
{
    // Y 轴限位触发
    pwm_pulse_states[0].limit_triggered = 1;
    Axis_Stop(AXIS_Y);
    Motion_LimitSwitchCallback(GPIO_PIN_1);
}

void PC2_Interrupt_Handler(void)
{
    // Z 轴限位触发
    pwm_pulse_states[1].limit_triggered = 1;
    Axis_Stop(AXIS_Z);
    Motion_LimitSwitchCallback(GPIO_PIN_2);
}

void Axis_LimitSwitchCallback(uint16_t GPIO_Pin)
{
    if(Homing_IsInProgress())
    {
        Homing_LimitSwitchCallback(GPIO_Pin);
    }
    else
    {
        // Handle limit switches during normal operation if needed
        switch (GPIO_Pin) {
            case GPIO_PIN_0: // X-axis
                pwm_pulse_states[2].limit_triggered = 1;
                pwm_pulse_states[3].limit_triggered = 1;
                Axis_Stop(AXIS_X);
                break;
            case GPIO_PIN_1: // Y-axis
                pwm_pulse_states[0].limit_triggered = 1;
                Axis_Stop(AXIS_Y);
                break;
            case GPIO_PIN_2: // Z-axis
                pwm_pulse_states[1].limit_triggered = 1;
                Axis_Stop(AXIS_Z);
                break;
        }
    }
}
uint8_t Axis_IsHoming(void){
    return homingInProgress;
} 
