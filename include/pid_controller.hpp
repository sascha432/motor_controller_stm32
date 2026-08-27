/**
  Author: sascha_lammers@gmx.de
*/

inline void PidController::ovp_check(uint16_t vSense)
{
    if (vSense > faults.vsenseMax) {
        if (errorCode != ErrorCodeType::OVP) {
            // hard fault if we have an OVP condition, mostly likely due to reverse currents while braking
            setErrorCode(ErrorCodeType::OVP);
        }
    }
}

inline void PidController::ocp_start()
{
    ocp.state = OcpStateType::TRIGGERED;

    // ramp motor current limit down to stay within input current limit
    uint32_t motorCurrent = DAC_GET_MOTOR_CURRENT();
    motorCurrent -= motorCurrent / kOcpCurrentRampDown;
    if (motorCurrent < kOcpCurrentRampUp) {
        motorCurrent = kOcpCurrentRampUp; // keep at kOcpCurrentRampUp to get a non zero increment for the ramp up
    }
    DAC_SET_MOTOR_CURRENT(motorCurrent);

    // counter handles the warning LED and keeps it on the longer the OCP condition lasts
    if (ocp.counter < adc.isenseSmoothing) { // adc.isenseSmoothing = PWM frequency / 10 = limits the max. time to about 100ms before the LED turns off after the condition has been cleared
        if (ocp.counter++ == 0) {
            LEDs::onLEDWarning();
        }
    }
}

inline void PidController::ocp_stop()
{
    if (ocp.state != OcpStateType::NONE) {
        if (ocp.counter) {
            ocp.counter--;
        }
        // ramp motor current limit up
        uint32_t motorCurrent = DAC_GET_MOTOR_CURRENT();
        if (motorCurrent < ocp.dacMotorCurrent) {
            motorCurrent += motorCurrent / kOcpCurrentRampUp;
            if (motorCurrent >= ocp.dacMotorCurrent) {
                // previous level reached, disabled OCP condition
                motorCurrent = ocp.dacMotorCurrent;
                ocp.state = OcpStateType::NONE;
                ocp.counter = 0;
            }
            DAC_SET_MOTOR_CURRENT(motorCurrent);
        }
    }
}
