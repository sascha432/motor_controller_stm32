/**
  Author: sascha_lammers@gmx.de
*/

#include "pid_controller.h"
#include "mt6701_encoder.h"

#define PID_ISR_DEBUG_PRINT 0

PidController pid;
MotorEncoder motorEncoder;
TIM_HandleTypeDef tim1;

void PidController::init()
{
    // === PWM on TIM1 CH1 (PA8, PA9) ===

    // Enable clocks
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    // PA8 / PA9 AF push-pull
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // TIM1 PWM setup
    tim1.Instance = TIM1;
    tim1.Init.Prescaler = 0;
    tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim1.Init.Period = pwmLevel.getMax() - 1;
    tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&tim1);

    // PWM mode 1 CH1 + CH2
    TIM_OC_InitTypeDef sConfigOC = {};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = 0;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&tim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&tim1, &sConfigOC, TIM_CHANNEL_2);

    // TIM1 BDTR MOE
    __HAL_TIM_MOE_ENABLE(&tim1);

    // Start PWM
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_2);

    // TIM1 CH4 OC4REF used as injected ADC trigger (PA2 current + PA3 voltage sample)
    // PWM mode 2: OC4REF goes high when CNT reaches CCR4, so the rising edge (which the
    // ADC triggers on) fires at exactly the CCR4 point in the cycle set by
    // ADC::updateInjectedTriggerPoint(). Output is internal only, NOT routed to PA11.
    // starts disabled (Pulse = 0 keeps OC4REF flat high -> no rising edge -> no trigger)
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 0;
    HAL_TIM_PWM_ConfigChannel(&tim1, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_4);

    // Fault interrupt pins DRV_FAULT, OCP_INT, DRV_SNSOUT
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct = {};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin = DRV_FAULT_Pin | OCP_INT_Pin;
    HAL_GPIO_Init(DRV_FAULT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DRV_SNSOUT_Pin;
    HAL_GPIO_Init(DRV_SNSOUT_GPIO_Port, &GPIO_InitStruct);
}

void PidController::reset()
{
    __disable_irq();
    PID_WRITE_MOTOR_PWM_OFF();
    LEDs::off();
    running = false;
    setPWMFrequency(eeprom.getPWMFrequency());
    lastEncoderCounter = PID_READ_ENCODER_COUNTER();
    lastError = 0;
    lastDerivative = 0;
    integral = 0;
    stats.reset();
    ::stats.reset();
    errorCode = ErrorCodeType::NONE;
    releaseBrakeCounter = 0;
    faults.reset(ADCConverter::Voltage::reverse(eeprom.getOvpProtection()));
    lastRpmCounter = PID_READ_RPM_COUNTER();
    lastRpmCounterUpdated = HAL_GetTick();
    ocp.reset();
    applyPIDParams();


    #if PID_ISR_DEBUG_PRINT
        DEBUG_PRINT(DebugType::PID, "reset() Kp=%s Ki=%s Kd=%s RPM=%u windup=%s OCP=%u/%u/%u OVP=%u",
            debugFloatToString(Kp, 6, true),
            debugFloatToString(Ki, 6, true),
            debugFloatToString(Kd, 6, true),
            rpm,
            debugFloatToString(antiWindup / static_cast<float>(UIConstants::kAntiWindupFactor), 2, true),
            eeprom.getInputCurrentLimit(),
            eeprom.getMotorCurrentLimit(),
            eeprom.getCurrentLimitStrength(),
            eeprom.getOvpProtection()
        );
    #endif
}

void PidController::motorOn()
{
    __disable_irq();
    if (!running) {
        reset();
        running = true;
        __enable_irq();
    }
    else {
        __enable_irq();
        DEBUG_PRINT(DebugType::ERROR, "motorOn() MOTOR RUNNING");
    }
}

void PidController::motorOff()
{
    __disable_irq();
    PID_WRITE_MOTOR_PWM_OFF();
    if (running) {
        running = false;
        releaseBrakeCounter = (kReleaseBrakeTimeMillis / kPIDInterval) + 1;
        if (releaseBrakeCounter == 0) {
            // stop if not braking, we need to injection to monitor OVP
            adc.stopInjectedTrigger();
        }
        else {
            // enable brake
            const uint32_t level = clampPWMLevel(eeprom.getMotorBrake() * pwmLevel.getMax() / 100);
            PID_WRITE_MOTOR_PWM_BRAKE(level);
        }
        __enable_irq();
    }
    else {
        __enable_irq();
        DEBUG_PRINT(DebugType::ERROR, "motorOff() MOTOR NOT RUNNING");
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

void PidController::isr()
{
    // most timers are 16bit counters only
    int32_t delta = getCountsDelta(PID_READ_ENCODER_COUNTER());
    // determine direction from fixed sensor, motor and pid motor direction
    if (eeprom.compareWithSensorDirection(motorDirection) == eeprom.isReverseMotorDirection()) {
        delta = -delta;
    }
    stats.counter.pulse += delta;

    // calculate PWM level based on PID or fixed PWM value
    int32_t newPwmLevel, clampedPwmLevel;

    if (eeprom.isPIDMode()) {
        // calculate error and derivative
        #if PID_USE_FLOATING_POINT_MATH
            PidValueType error = (getCountsPerInterval() - delta) / kMaxError;
        #else
            PidValueType error = getCountsPerInterval() - delta;
        #endif
        PidValueType derivative = (error - getLastError());
        setLastError(error);

        // apply filter
        derivative = (derivative + getLastDerivative()) / 2;
        setLastDerivative(derivative);

        // update integral
        updateIntegral(error);

        // get pwm level and set output
        newPwmLevel = calcPWMLevel(error, getIntegral(), derivative);
        // clamp pwm level to max. allowed value
        clampedPwmLevel = clampPWMLevel(newPwmLevel);

        // apply anti-windup
        if (antiWindup) {
            if ((newPwmLevel < pwmLevel.getLower()) || (newPwmLevel > pwmLevel.getUpper())) {
                #if PID_USE_FLOATING_POINT_MATH
                    setIntegral(getIntegral() * antiWindup * (0.01f / UIConstants::kAntiWindupFactor));
                #else
                    setIntegral((getIntegral() * antiWindup) / static_cast<PidValueType>(UIConstants::kAntiWindupFactor * 100));
                #endif
            }
        }
    }
    else {
        // fixed pwm from settings
        newPwmLevel = (eeprom.getMotorPWM() * pwmLevel.getARR()) / 100;
        // clamp pwm level to max. allowed value
        clampedPwmLevel = clampPWMLevel(newPwmLevel);
    }

    // apply new PWM level if motor is running
    if (running) {
        PID_WRITE_MOTOR_PWM_ON(clampedPwmLevel, motorDirection);
        adc.updateInjectedTriggerPoint(clampedPwmLevel);
    }
    else if (releaseBrakeCounter) {
        // countdown once set
        if (--releaseBrakeCounter == 0) {
            PID_WRITE_MOTOR_PWM_OFF();
            adc.stopInjectedTrigger(); // stop after braking to keep ovp alive
            #if PID_ISR_DEBUG_PRINT
                DEBUG_PRINT(DebugType::PID, "Brake released");
            #endif
        }
    }

    // start new ADC DMA transfer
    adc.startDMAIfReady();

    // update pwm stats
    stats.pwm.update(clampedPwmLevel);

    // update rpm stats
    const int32_t deltaRPM = kIntCountsToRPM(delta);
    stats.rpm.update(deltaRPM);
    ::stats.sampleRPM(deltaRPM);

    if (running) {
        // initial stall and sensor check
        if (stats.counter.loop == static_cast<uint32_t>(kInitialSensorCheckTimeMillis / kPIDInterval)) {
            if (stats.counter.pulse < -10) { // sensor counts backwards, wrong direction set
                setErrorCode(ErrorCodeType::SENSOR_REVERSE);
            }
            else if (PID_READ_RPM_COUNTER() >= 1 && stats.counter.pulse < 10) { // we have 1 rotation but less than 10 pulses, something is wrong with the sensor
                setErrorCode(ErrorCodeType::SENSOR);
            }
            else if (stats.counter.pulse < (kCPR / 8)) { // 1/8th of a rotation or less, motor has stalled
                setErrorCode(ErrorCodeType::STALL);
            }
        }

        // check for motor stall
        const uint32_t now = HAL_GetTick();
        const uint32_t newRpmCounter = PID_READ_RPM_COUNTER();
        if (newRpmCounter >= lastRpmCounter + 2) { // require more than one rotation before updating the value
            lastRpmCounter = newRpmCounter;
            lastRpmCounterUpdated = now;
        }
        else if (now - lastRpmCounterUpdated > eeprom.getMotorStallTimeout()) {
            setErrorCode(ErrorCodeType::STALL);
        }

    }

    stats.counter.loop++;

    // send PID tuning data if tuning is enabled
    if (SWO::data.enabled != SWO::EnableState::DISABLED) {
        PidLoopType item;
        item.rpm = static_cast<uint16_t>(deltaRPM);
        item.voltage = adc.getVSenseValue();
        item.currentOcp = adc.getAndClearISenseMaxValue();
        item.currentAverage = adc.getISenseAverageValue();
        item.motorTemperature = adc.getMotorTemperatureFiltered();
        item.mosfetTemperature = adc.getMosfetTemperatureFiltered();
        item.dacMotorCurrent = DAC_GET_MOTOR_CURRENT();
        item.dacInputCurrent =  DAC_GET_INPUT_CURRENT();
        #if PID_USE_FLOATING_POINT_MATH
            item.error = static_cast<uint16_t>(getLastError() * kFloatToUint16Multiplier);
            item.integral = static_cast<uint16_t>(getIntegral() * kFloatToUint16Multiplier);
            item.derivative = static_cast<uint16_t>(getLastDerivative() * kFloatToUint16Multiplier);
        #else
            constexpr float tmp = (1.0f / PidController::kFPFactor) * kFloatToUint16Multiplier;
            item.error = static_cast<uint16_t>(getLastError() * tmp);
            item.integral = static_cast<uint16_t>(getIntegral() * tmp);
            item.derivative = static_cast<uint16_t>(getLastDerivative() * tmp);
        #endif
        item.pwmLevel = static_cast<uint8_t>((clampedPwmLevel * 100) / pwmLevel.getARR());
        item.running = running ? 1U : 0U;
        item.drv8701Fault = faults.drv8701Fault ? 1U : 0U;
        item.ocpFault = ocp.isTriggered() ? 1U : 0U;
        item.snsoutFault = faults.snsoutFault ? 1U : 0U;
        pidLoopBuffer.push(item);

        if (SWO::data.changed) {
            // apply data to EEPROM and PID controller
            eeprom.setKd(SWO::data.Kd);
            eeprom.setKp(SWO::data.Kp);
            eeprom.setKi(SWO::data.Ki);
            eeprom.setMotorRPM(SWO::data.rpm);
            eeprom.setAntiWindup(SWO::data.antiWindup);
            eeprom.setInputCurrentLimit(SWO::data.inputCurrentLimit);
            eeprom.setCurrentLimitStrength(static_cast<EEPROM::CurrentLimitStrength>(SWO::data.currentLimitStrength));
            SWO::data.changed = false;

            // apply to PID controller
            pid.setKp(eeprom.getKp());
            pid.setKd(eeprom.getKd());
            pid.setKi(eeprom.getKi());
            pid.setRPM(eeprom.getMotorRPM());
            pid.setAntiWindup(eeprom.getAntiWindup());
            pid.setInputCurrentLimit(eeprom.getInputCurrentLimit());

            #if PID_ISR_DEBUG_PRINT
                DEBUG_PRINT(DebugType::PID, "SWO PID tuning: Kp=%s Ki=%s Kd=%s RPM=%u windup=%s OCP=%u/%u",
                    debugFloatToString(SWO::data.Kp, 6, true),
                    debugFloatToString(SWO::data.Ki, 6, true),
                    debugFloatToString(SWO::data.Kd, 6, true),
                    SWO::data.rpm,
                    debugFloatToString(SWO::data.antiWindup / static_cast<float>(UIConstants::kAntiWindupFactor), 2, true),
                    SWO::data.inputCurrentLimit,
                    SWO::data.currentLimitStrength
                );
            #endif
        }
    }
}

size_t PidController::errorPrintf(char *buf, size_t bufSize) const
{
    switch(errorCode) {
        case ErrorCodeType::STALL:
            return snprintf(buf, bufSize, "MOTOR STALL");
        case ErrorCodeType::SENSOR:
            return snprintf(buf, bufSize, "SENSOR ERROR");
        case ErrorCodeType::SENSOR_REVERSE:
            return snprintf(buf, bufSize, "SENSOR REVERSE");
        case ErrorCodeType::MOTOR_OVER_TEMPERATURE:
            return snprintf(buf, bufSize, "MOTOR %d" DEGREE_UTF8 "C", ::stats.motorTemp);
        case ErrorCodeType::MOSFET_OVER_TEMPERATURE:
            return snprintf(buf, bufSize, "MOSFET %d" DEGREE_UTF8 "C", ::stats.mosfetTemp);
        case ErrorCodeType::OVP:
            return snprintf(buf, bufSize, "OVP %u.%uV", CONVERT_TO_FP1(eeprom.getOvpProtection()));
        case ErrorCodeType::OCP:
            return snprintf(buf, bufSize, "OCP");
        case ErrorCodeType::FAULT:
            return snprintf(buf, bufSize, "DRV8701 FAULT");
        case ErrorCodeType::SNSOUT:
            return snprintf(buf, bufSize, "DRV8701 SNSOUT");
        case ErrorCodeType::NONE:
            return snprintf(buf, bufSize, "NONE");
    }
    #if DEBUG
        return snprintf(buf, bufSize, "ERROR #%d", static_cast<int>(errorCode));
    #else
        *buf = 0;
        return 0;
    #endif
}

void PidController::setPWMFrequency(uint32_t frequency)
{
    // clamp frequency to min/max values
    constexpr uint32_t kMinPWMFrequency = kARRToPWMFrequency(0xffff);       // 16bit timer limit
    constexpr uint32_t kMaxPWMFrequency = kARRToPWMFrequency(1 << 10);      // at least 10bit pwm resolution
    frequency = std::clamp<uint32_t>(frequency, kMinPWMFrequency, kMaxPWMFrequency);

    // set pwm level
    pwmLevel.setMax(kPWMFrequencyToARR(frequency));
    // get the max. pwm level for the auto-reload register
    const uint32_t arr = pwmLevel.getMax() - 1;

    // stop timer and update pwm frequency
    __HAL_TIM_DISABLE(&tim1);
    __HAL_TIM_SET_AUTORELOAD(&tim1, arr);

    // update pre-calculated PID parameters
    setKp(Kp);
    setKi(Ki);
    setKd(Kd);

    // update frequency for injection group
    adc.initInjection(frequency);

    // start timer
    __HAL_TIM_SET_COUNTER(&tim1, 0);
    __HAL_TIM_ENABLE(&tim1);
}
