/**
  Author: sascha_lammers@gmx.de
*/

#include "pid_controller.h"
#include "mt6701_encoder.h"
#include "leds.h"

PidController pid;
MotorEncoder motorEncoder;

void PidController::reset()
{
    lastEncoderCounter = readEncoderCounter();
    lastError = 0;
    lastDerivative = 0;
    integral = 0;
    stats.reset(readRpmCounter());
    errorCode = ErrorCodeType::NONE;
    faults.reset();
    faults.isenseMax = ADC::_currentLimitValueToDAC(eeprom.getInputCurrentLimit());
    faults.vsenseMax = ADCConverter::Voltage::reverse(eeprom.getOvpProtection());
    ocp.reset();
    resetFaults();
    applyPIDParams();

    #if DEBUG
    char pBuf[32];
    char iBuf[32];
    char dBuf[32];
    FloatToString::convertTrimmed(pBuf, sizeof(pBuf), Kp, 6);
    FloatToString::convertTrimmed(iBuf, sizeof(iBuf), Ki, 6);
    FloatToString::convertTrimmed(dBuf, sizeof(dBuf), Kd, 6);
    DEBUG_PRINT(DEBUG_DEBUG, "Kp=%s Ki=%s Kd=%s RPM=%u antiWindupReduction=%u", pBuf, iBuf, dBuf, rpm, antiWindupReduction);
    #endif
}

void PidController::init()
{
    // // === PWM on TIM1 CH1 (PA8, PA9) ===
    // Enable clocks
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    // PA8 / PA9 AF push-pull
    GPIO_InitTypeDef GPIO_InitStructPP = {};
    GPIO_InitStructPP.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStructPP.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructPP.Pull = GPIO_NOPULL;
    GPIO_InitStructPP.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructPP);

    // TIM1 PWM setup
    TIM_HandleTypeDef tim1 = {};
    tim1.Instance = TIM1;
    tim1.Init.Prescaler = 0;
    tim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim1.Init.Period = kMaxPWMLevel - 1;
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

    // TIM4 setup MT6701 encoder on PB6 PB7
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    TIM_HandleTypeDef tim4 = {};
    tim4.Instance = TIM4;
    tim4.Init.Prescaler = 0;
    tim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim4.Init.Period = 0xFFFF;
    tim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    TIM_Encoder_InitTypeDef sEncoderConfig = {};
    // 4x mode, count on both edges of both channels
    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    // channel 1
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 0;
    // channel 2
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 0;

    HAL_TIM_Encoder_Init(&tim4, &sEncoderConfig);
    __HAL_TIM_SET_COUNTER(&tim4, 0);
    HAL_TIM_Encoder_Start(&tim4, TIM_CHANNEL_ALL);

    // TIM5 setup for RPM counter
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    // PA1 (TIM5_CH2) input floating
    GPIO_InitTypeDef GPIO_InitStructNpPullup = {};
    GPIO_InitStructNpPullup.Pin = GPIO_PIN_1;
    GPIO_InitStructNpPullup.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructNpPullup.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructNpPullup);

    // Reset TIM5
    TIM5->CR1 = 0;
    TIM5->CR2 = 0;
    TIM5->SMCR = 0;
    TIM5->DIER = 0;
    TIM5->CCMR1 = 0;
    TIM5->CCER = 0;
    // CH2 as input, mapped to TI2
    TIM5->CCMR1 |= TIM_CCMR1_CC2S_0;
    // Falling edge detection
    TIM5->CCER |= TIM_CCER_CC2P;
    // Select TI2FP2 as trigger input
    // TS = 110
    TIM5->SMCR |= (6 << TIM_SMCR_TS_Pos);
    // External clock mode 1
    // SMS = 111
    TIM5->SMCR |= (7 << TIM_SMCR_SMS_Pos);
    // 32-bit counter
    TIM5->ARR = 0xFFFFFFFF;
    // Start
    TIM5->CNT = 0;
    TIM5->CR1 |= TIM_CR1_CEN;

    // Fault interrupt pins DRV8701_FAULT_PIN, OCP_INT_PIN, DRV_SNSOUT_PIN
    __HAL_RCC_GPIOx_CLK_ENABLE<DRV8701_FAULT_PIN>();
    __HAL_RCC_GPIOx_CLK_ENABLE<OCP_INT_PIN>();
    __HAL_RCC_GPIOx_CLK_ENABLE<DRV_SNSOUT_PIN>();

    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = digitalPinToHAL<DRV8701_FAULT_PIN>();
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(digitalPinToGPIO<DRV8701_FAULT_PIN>(), &GPIO_InitStruct);

    GPIO_InitStruct.Pin = digitalPinToHAL<OCP_INT_PIN>();
    HAL_GPIO_Init(digitalPinToGPIO<OCP_INT_PIN>(), &GPIO_InitStruct);

    GPIO_InitStruct.Pin = digitalPinToHAL<DRV_SNSOUT_PIN>();
    HAL_GPIO_Init(digitalPinToGPIO<DRV_SNSOUT_PIN>(), &GPIO_InitStruct);
}

void PidController::motorOn()
{
    __disable_irq();
    if (!running) {
        PID_WRITE_MOTOR_PWM_OFF();
        running = true;
        __enable_irq();
        reset();
    }
    else {
        __enable_irq();
        DEBUG_PRINT(DEBUG_ERROR, "MOTOR RUNNING");
    }
}

void PidController::motorOff()
{
    __disable_irq();
    PID_WRITE_MOTOR_PWM_OFF();
    if (running) {
        running = false;
        uint32_t level = clampPWMLevel(eeprom.getMotorBrake() * kMaxPWMLevel / 100);
        PID_WRITE_MOTOR_PWM_BREAK(level);
        __enable_irq();
    }
    else {
        __enable_irq();
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

void PidController::isr()
{
    // most timers are 16bit counters only
    int32_t delta = getDelta(readEncoderCounter());
    // apply fixed sensor, motor and selected motor direction to delta
    if (
        eeprom.isForwardMotorDirection() ?
            (eeprom.getSensorDirection() != motorDirection) :
            (eeprom.getSensorDirection() == motorDirection)
    ) {
        delta = -delta;
    }
    stats.counter.pulse += delta;

    int32_t pwmLevel;

    if (eeprom.isPIDMode()) {
        // calculate error and derivative
        int32_t error = (getCountsPerInterval() - delta);
        int32_t derivative = (error - getLastError());
        setLastError(error);

        // apply filter
        derivative = (derivative + getLastDerivative()) / 2;
        setLastDerivative(derivative);

        // update integral
        updateIntegral(error);

        // get pwm level and set output
        pwmLevel = calcPWMLevel(error, getIntegral(), derivative);
    }
    else {
        pwmLevel = eeprom.getMotorPWM() * kMaxPWMLevel / 100;
    }

    // clamp pwm level to max. allowed value
    int32_t clampedPwmLevel = clampPWMLevel(pwmLevel);

    if (eeprom.isPIDMode()) {
        if (ocp.state != OcpStateType::NONE) {
            setIntegral((getIntegral() * 850) / 1024); // strong anti windup reduction during OCP condition
        }
        else if (antiWindupReduction) {
            if (pwmLevel < -kMaxPWMLevel || pwmLevel > (kMaxPWMLevel * 2)) {
                setIntegral((getIntegral() * antiWindupReduction) / kAntiWindupFactor);
            }
        }
    }

    // apply new PWM level if motor is running
    if (running) {
        // do not update PWM during OCP condition
        if (ocp.state != OcpStateType::TRIGGERED) {
            PID_WRITE_MOTOR_PWM_ON(clampedPwmLevel, motorDirection);
        }
    }

    // update pwm stats
    stats.pwm.update(clampedPwmLevel);

    // update rpm stats
    int32_t deltaRPM = kIntCountsToRPM(delta);
    stats.rpm.update(deltaRPM);

    if (running) {
        // initial stall and sensor check after 500ms
        if (stats.counter.loop == (500 / kPIDInterval)) {
            if (stats.counter.pulse < -10) { // sensor counts backwards, wrong direction set
                setErrorCode(ErrorCodeType::SENSOR_REVERSE);
            }
            else if (readRpmCounter() >= 1 && stats.counter.pulse < 10) { // we have 1 rotation but less than 10 pulses, something is wrong with the sensor
                setErrorCode(ErrorCodeType::SENSOR);
            }
            else if (stats.counter.pulse < (kCPR / 4)) { // quarter of a rotation or less, motor has stalled
                setErrorCode(ErrorCodeType::STALL);
            }
        }
    }

    stats.counter.loop++;

    // send PID tuning data if enabled
    if (eeprom.getPidTuning() != EEPROM::kPidTuningDisabled) {
        PidLoopType item;
        item.sequence = stats.counter.loop;
        item.dataAddress = reinterpret_cast<uint32_t>(&SWO::data);
        item.rpm = static_cast<uint16_t>(deltaRPM);
        item.pwmLevel = static_cast<uint16_t>(clampedPwmLevel);
        item.voltage = adc.getVSenseValue();
        item.currentOcp = adc.getISenseOcpFilteredValue();
        item.currentAverage = adc.getISenseAverageValue();
        item.motorTemperature = adc.getMotorNTCValue();
        item.mosfetTemperature = adc.getMosfetNTCValue();
        item.integral = getIntegral();
        item.running = running ? 1U : 0U;
        item.drv8701Fault = faults.drv8701Fault ? 1U : 0U;
        item.ocpFault = (ocp.state != OcpStateType::NONE) ? 1U : 0U;
        item.snsoutFault = faults.snsoutFault ? 1U : 0U;
        pidLoopBuffer.push(item);

        if (SWO::data.changed) {
            // apply data to EEPROM and PID controller
            eeprom.setKd(SWO::data.Kd);
            eeprom.setKp(SWO::data.Kp);
            eeprom.setKi(SWO::data.Ki);
            eeprom.setMotorRPM(SWO::data.rpm);
            eeprom.setAntiWindupReduction(SWO::data.antiWindupReduction * UIConstants::kAntiWindupFactor);
            SWO::data.changed = false;

            // apply to PID controller
            pid.setKp(eeprom.getKp());
            pid.setKd(eeprom.getKd());
            pid.setKi(eeprom.getKi());
            pid.setRPM(eeprom.getMotorRPM());
            pid.setAntiWindupReduction(eeprom.getAntiWindupReduction());

            #if DEBUG
                char bufKp[16], bufKi[16], bufKd[16];
                FloatToString::convertTrimmed(bufKp, sizeof(bufKp), SWO::data.Kp, 6);
                FloatToString::convertTrimmed(bufKi, sizeof(bufKi), SWO::data.Ki, 6);
                FloatToString::convertTrimmed(bufKd, sizeof(bufKd), SWO::data.Kd, 6);
                DEBUG_PRINT(DEBUG_DEBUG, "PID tuning via SWO: Kp=%s Ki=%s Kd=%s RPM=%u", bufKp, bufKi, bufKd, SWO::data.rpm);
            #endif
        }
    }
}

void PidController::ocp_isr()
{
    if (ocp.state == OcpStateType::TRIGGERED) {
        ocp.counter++;
        if (ocp.counter >= kOcpForceRecoveryTicks) {
            trigger_ocp_recovery(); // force recovery
        }
        else if (ocp.counter >= kOcpRecoveryMinTicks) {
            if (adc.getISenseOcpFilteredValue() < faults.isenseMax) {
                trigger_ocp_recovery(); // recovery condition met
            }
        }
    }
}

void PidController::trigger_ocp()
{
    if (ocp.state == OcpStateType::RECOVERING) {
        if (++ocp.counter <= kOcpRetriggerMinTicks) {
            // do not allow to retrigger for n ticks
            return;
        }
        ocp.state = OcpStateType::NONE;
    }
    if (ocp.state == OcpStateType::NONE) {
        if (adc.getISenseOcpFilteredValue() > faults.isenseMax) {
            // set triggered state and turn motor off
            ocp.state = OcpStateType::TRIGGERED;
            ocp.counter = 0;
            // restore a fraction of the PWM to avoid retriggering OCP immediately after recovery, the PID loop will restore the correct PWM level
            ocp.pwmLevel1 = PID_READ_MOTOR_PWM_DRV_IN1();
            ocp.pwmLevel2 = PID_READ_MOTOR_PWM_DRV_IN2();
            PID_WRITE_MOTOR_PWM_OFF();
            LEDs::onLED2();
        }
    }
}

void PidController::trigger_ocp_recovery()
{
    // set recovering state and restore PWM levels
    ocp.state = OcpStateType::RECOVERING;
    ocp.counter = 0;
    if (running) {
        PID_WRITE_MOTOR_PWM_DRV_IN1(ocp.pwmLevel1);
        PID_WRITE_MOTOR_PWM_DRV_IN2(ocp.pwmLevel2);
    }
    LEDs::offLED1and2();
}
