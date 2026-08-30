/**
  Author: sascha_lammers@gmx.de
*/

#if HAVE_MOTOR_VIBES

#include "main.h"
#include "motor_vibes.h"

// === MotorVibes implementation ===

void MotorVibes::init()
{
    // stop PWM outputs
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;
    // store current prescaler and auto-reload value
    prescaler = PID_MOTOR_PWM_TIMER->PSC;
    arr = PID_MOTOR_PWM_TIMER->ARR;

    // store current limits and remove them
    motorCurrentLimit = DAC_GET_MOTOR_CURRENT();
    inputCurrentLimit = DAC_GET_INPUT_CURRENT();
    DAC_SET_MOTOR_CURRENT(~0U);
    DAC_SET_INPUT_CURRENT(~0U);

    // change prescaler and period on the fly without reinitializing the timer
    PID_MOTOR_PWM_TIMER->PSC = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    PID_MOTOR_PWM_TIMER->ARR = kTonePeriod;
    PID_MOTOR_PWM_TIMER->EGR = TIM_EGR_UG; // force update event to load PSC/ARR and reset the counter
}

void MotorVibes::deinit()
{
    // stop PWM outputs
    PID_MOTOR_PWM_TIMER->CCR1 = 0;
    PID_MOTOR_PWM_TIMER->CCR2 = 0;

    // restore current limits
    DAC_SET_MOTOR_CURRENT(motorCurrentLimit);
    DAC_SET_INPUT_CURRENT(inputCurrentLimit);

    // restore prescaler and period on the fly
    PID_MOTOR_PWM_TIMER->PSC = prescaler;
    PID_MOTOR_PWM_TIMER->ARR = arr;
    PID_MOTOR_PWM_TIMER->EGR = TIM_EGR_UG; // force update event to load PSC/ARR and reset the counter
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
