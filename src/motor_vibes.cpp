/**
  Author: sascha_lammers@gmx.de
*/

#if HAVE_MOTOR_VIBES

#include "main.h"
#include "motor_vibes.h"

// === MotorVibes implementation ===

extern TIM_HandleTypeDef tim1;

void MotorVibes::init()
{
    // store values and deinit timer
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
    prescaler = tim1.Init.Prescaler;
    period = tim1.Init.Period;
    arr = PID_MOTOR_PWM_TIMER->ARR;
    HAL_TIM_PWM_DeInit(&tim1);

    // store current limits and remove them
    motorCurrentLimit = DAC_GET_MOTOR_CURRENT();
    inputCurrentLimit = DAC_GET_INPUT_CURRENT();
    DAC_SET_MOTOR_CURRENT(0xffff);
    DAC_SET_INPUT_CURRENT(0xffff);

    // change PWM frequency
    tim1.Init.Prescaler = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    tim1.Init.Period = kTonePeriod;
    HAL_TIM_PWM_Init(&tim1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
}

void MotorVibes::deinit()
{
    // deinit and restore previous values
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
    HAL_TIM_PWM_DeInit(&tim1);

    // restore current limits
    DAC_SET_MOTOR_CURRENT(motorCurrentLimit);
    DAC_SET_INPUT_CURRENT(inputCurrentLimit);

    // re-initialize timer with previous values
    tim1.Init.Prescaler = prescaler;
    tim1.Init.Period = period;
    HAL_TIM_PWM_Init(&tim1);
    __HAL_TIM_SET_AUTORELOAD(&tim1, arr);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);
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
