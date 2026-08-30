/**
  Author: sascha_lammers@gmx.de
*/

#if HAVE_MOTOR_VIBES

#include "main.h"
#include "motor_vibes.h"

// === MotorVibes implementation ===

void MotorVibes::init()
{
    // store values and deinit timer
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
    prescaler = htim1.Init.Prescaler;
    period = htim1.Init.Period;
    arr = PID_MOTOR_PWM_TIMER->ARR;
    HAL_TIM_PWM_DeInit(&htim1);

    // store current limits and remove them
    motorCurrentLimit = DAC_GET_MOTOR_CURRENT();
    inputCurrentLimit = DAC_GET_INPUT_CURRENT();
    DAC_SET_MOTOR_CURRENT(~0U);
    DAC_SET_INPUT_CURRENT(~0U);

    // change PWM frequency
    htim1.Init.Prescaler = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    htim1.Init.Period = kTonePeriod;
    HAL_TIM_PWM_Init(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void MotorVibes::deinit()
{
    // deinit and restore previous values
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
    HAL_TIM_PWM_DeInit(&htim1);

    // restore current limits
    DAC_SET_MOTOR_CURRENT(motorCurrentLimit);
    DAC_SET_INPUT_CURRENT(inputCurrentLimit);

    // re-initialize timer with previous values
    htim1.Init.Prescaler = prescaler;
    htim1.Init.Period = period;
    HAL_TIM_PWM_Init(&htim1);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void MotorVibes::playTone(uint32_t frequency)
{
    if (frequency == 0) {
        stopTone();
        return;
    }
    // change pwm frequency
    const uint32_t arr = F_CPU / ((kTonePeriod + 1) * frequency) - 1;
    PID_MOTOR_PWM_TIMER->ARR = arr;
    // play tone with reduced duty cycle to prevent the motor from spinning
    PID_MOTOR_PWM_TIMER->CCR1 = arr / kPWMDivider;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
}

void MotorVibes::stopTone()
{
    // 100% brake in case it spinned up
    PID_MOTOR_PWM_TIMER->CCR1 = PID_MOTOR_PWM_TIMER->ARR;
    PID_MOTOR_PWM_TIMER->CCR2 = PID_MOTOR_PWM_TIMER->ARR;
}

#endif
