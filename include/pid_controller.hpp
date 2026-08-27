/**
  Author: sascha_lammers@gmx.de
*/

inline uint32_t PidController::ocp_get_ramp() const
{
    switch(eeprom.getCurrentLimitLevel()) {
        case 0:
            return 128;
        case 1:
            return 64;
        case 2:
            return 16;
        case 3:
            return 1;
    }
    return 64;
}

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
    motorCurrent -= motorCurrent / ocp_get_ramp();
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
            motorCurrent += std::max<uint32_t>(1, motorCurrent / ocp_get_ramp());
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
