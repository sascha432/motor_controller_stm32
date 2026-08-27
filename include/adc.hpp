/**
  Author: sascha_lammers@gmx.de
*/

inline void ADC::isr()
{
    // check ovp condition
    pid.ovp_check(getVSenseValue());

    // update filtered temperature values
    motorTemperatureFiltered = filterValue<uint16_t, 16>(motorTemperatureFiltered, getMotorNTCValue());
    mosfetTemperatureFiltered = filterValue<uint16_t, 16>(mosfetTemperatureFiltered, getMosfetNTCValue());

    dmaTransferComplete = true;
}

inline void ADC::isrInjected()
{
    // rank 1: PA2/IN2 current, rank 2: PA3/IN3 voltage (JDR1 is the first injected conversion)
    const uint16_t iSense = ADC1->JDR1;
    const uint16_t vSense = ADC1->JDR2;

    // Rev1.0 lacks the proper RC filter for the current, so we do it in software (4 and 8 seems to work well, 2 is not enough)
    isenseFiltered = filterValue<uint16_t, 4>(isenseFiltered, iSense);

    // max. value
    if (isenseFiltered > isenseMaxFiltered) {
        isenseMaxFiltered = isenseFiltered;
    }

    // average current for display purposes
    isenseSum += isenseFiltered;
    if (++isenseCount >= isenseSmoothing) {
        isenseSum -= isenseSum / kISenseCountDecayDivider;
        isenseCount -= isenseCount / kISenseCountDecayDivider;
    }

    // check ovp condition
    pid.ovp_check(vSense);

    // update min/max. voltage
    if (vSense > vsenseMax) {
        vsenseMax = vSense;
    }
    if (vSense < vsenseMin) {
        vsenseMin = vSense;
    }

    // handle OCP detection
    if (isenseFiltered > pid.faults.isenseMax) {
        pid.ocp_start();
    }
    else {
        pid.ocp_stop();
    }
}

inline void ADC::updateInjectedTriggerPoint(uint16_t pwmLevel)
{
    // the trigger point can be adjusted to get the best measurement
    // this was tested on PCB rev 1.0 with a different RC filter for the INA381 and might require adjustments for 1.2
    #if 0
        // measure in the middle of the DC
        PID_MOTOR_PWM_TIMER->CCR4 = pwmLevel / 2;
    #elif 0
        // DC falling edge - delay
        uint32_t ccr = pwmLevel;
        if (ccr > kInjectionStartDelayTicks) {
            ccr -= kInjectionStartDelayTicks;
        }
    #elif 0
        // DC falling edge + delay
        uint32_t ccr = pwmLevel + kInjectionStartDelayTicks;
        if (ccr > pid.getPWMLevelARR()) {
            ccr -= pid.getPWMLevelARR(); // must be non zero
        }
        PID_MOTOR_PWM_TIMER->CCR4 = ccr;
    #else
        (void)pwmLevel;
        // use fixed trigger point, DC raising edge + delay
        PID_MOTOR_PWM_TIMER->CCR4 = kInjectionStartDelayTicks;
    #endif
}
