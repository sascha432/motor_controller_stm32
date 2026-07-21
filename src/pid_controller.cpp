/**
  Author: sascha_lammers@gmx.de
*/

#include "pid_controller.h"
#include "mt6701_encoder.h"

PidController pid;
MotorEncoder motorEncoder;

void PidController::motorOn() 
{
    __disable_irq();
    if (!running) {
        running = true;
        __enable_irq();
        reset();
        setRPM(eeprom.getSpeed());
        Serial.println("START");
        DEBUG_PRINT(DEBUG_DEBUG, "START: rpm=%d", getRPM());
    }
    else {
        __enable_irq();
        Serial.println("ERR=RUNNING");
        DEBUG_PRINT(DEBUG_ERROR, "MOTOR RUNNING");
    }
}

void PidController::motorOff() 
{
    PID_WRITE_MOTOR_PWM_OFF();
    __disable_irq();
    if (running) {
        running = false;
        __enable_irq();
        Serial.println("STOP");
        DEBUG_PRINT(DEBUG_DEBUG, "STOP");
    }
    else {
        __enable_irq();
        Serial.println("ERR=NOT_RUNNING");
        DEBUG_PRINT(DEBUG_ERROR, "MOTOR NOT RUNNING");
    }
}

bool PidController::motorToggle() 
{
    if (running) {
        motorOff();
        return false;
    }
    else {
        motorOn();
        return true;
    }
    return false;
}
