/**
  Author: sascha_lammers@gmx.de
*/

#include "pid_controller.h"
#include "mt6701_encoder.h"

PidController pid;
MotorEncoder motorEncoder;

void PidController::reset()
{
    lastCounter = readEncoderCounter();
    lastError = 0;
    lastDerivative = 0;
    integral = 0;
    stats.reset(readRpmCounter());
    errorCode = ErrorCodeType::NONE;
    faults.reset();
    readFaults();
}

void PidController::disable()
{
    TIM4->CR1 &= ~TIM_CR1_CEN;
    timer.pause();
    detachInterrupt(digitalPinToInterrupt(DRV8701_FAULT_PIN));
    detachInterrupt(digitalPinToInterrupt(OCP_INT_PIN));
    detachInterrupt(digitalPinToInterrupt(DRV_SNSOUT_PIN));
}

void PidController::enable(InterruptCallbackType callback)
{
    TIM4->CR1 |= TIM_CR1_CEN;
    // all faults are active low
    attachInterrupt(digitalPinToInterrupt(DRV8701_FAULT_PIN), callback, FALLING);
    attachInterrupt(digitalPinToInterrupt(OCP_INT_PIN), callback, FALLING);
    attachInterrupt(digitalPinToInterrupt(DRV_SNSOUT_PIN), callback, FALLING);
    timer.resume();
    // update faults
    readFaults();
}

void PidController::init(InterruptCallbackType callback, InterruptCallbackType faultCallback)
{
    running = false;

    // === PWM on TIM1 CH1 (PA8, PA9) ===
    // Enable AFIO and GPIOA for PA8/PA9
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
    // Enable TIM1 (APB2)
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PA8 AF push-pull (CRH: pin 8)
    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    GPIOA->CRH |= (GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1); // 50MHz, AF push-pull

    // PA9 AF push-pull (CRH: pin 9)
    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |= (GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1); // 50MHz, AF push-pull

    // PWM setup on TIM1 CH1 and CH2
    TIM1->PSC = 0;
    TIM1->ARR = kMaxPWMLevel - 1;
    PID_WRITE_MOTOR_PWM_OFF();
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | (6 << TIM_CCMR1_OC2M_Pos); // PWM mode 1 on CH1 and CH2
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR |= TIM_BDTR_MOE; // main output enable for TIM1
    TIM1->CR1 |= TIM_CR1_CEN;

    // TIM4 setup
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // alternative function clock enable
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable TIM4 clock

    // TIM4 Encoder Mode for MT6701
    TIM4->CR1 = 0;
    TIM4->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // Encoder mode 3
    TIM4->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM4->CCER = 0;
    TIM4->ARR = 0xFFFF;
    TIM4->CNT = 0;

    // TIM5 setup for RPM counter
    // Enable GPIOA and TIM5 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    // PA1 input floating
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF1_0;       // input floating

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
    RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(DRV8701_FAULT_PIN) | RCC_APB2ENR_IOPxEN(OCP_INT_PIN) | RCC_APB2ENR_IOPxEN(DRV_SNSOUT_PIN);

    // clear conf
    GPIO_CRx_REG<DRV8701_FAULT_PIN>() &= ~(0xf << digitalPinShift(DRV8701_FAULT_PIN));
    GPIO_CRx_REG<OCP_INT_PIN>() &= ~(0xf << digitalPinShift(OCP_INT_PIN));
    GPIO_CRx_REG<DRV_SNSOUT_PIN>() &= ~(0xf << digitalPinShift(DRV_SNSOUT_PIN));

    // CNF=10, MODE=00
    GPIO_CRx_REG<DRV8701_FAULT_PIN>() |= (0x8 << digitalPinShift(DRV8701_FAULT_PIN));
    GPIO_CRx_REG<OCP_INT_PIN>() |= (0x8 << digitalPinShift(OCP_INT_PIN));
    GPIO_CRx_REG<DRV_SNSOUT_PIN>() |= (0x8 << digitalPinShift(DRV_SNSOUT_PIN));

    // Select pull-up (ODR bit = 1)
    digitalPinToGPIO<DRV8701_FAULT_PIN>()->ODR |= (1 << digitalPinToBit(DRV8701_FAULT_PIN));
    digitalPinToGPIO<OCP_INT_PIN>()->ODR |= (1 << digitalPinToBit(OCP_INT_PIN));
    digitalPinToGPIO<DRV_SNSOUT_PIN>()->ODR |= (1 << digitalPinToBit(DRV_SNSOUT_PIN));

    // PID loop, call ISR every 5 ms
    timer.setOverflow(kPIDInterval * 1000, MICROSEC_FORMAT);
    timer.attachInterrupt(callback);
    enable(faultCallback);
}

void PidController::motorOn()
{
    __disable_irq();
    if (!running) {
        running = true;
        __enable_irq();
        reset();
        setRPM(eeprom.getMotorRPM());
        DEBUG_PRINT(DEBUG_DEBUG, "START: rpm=%d", getRPM());
    }
    else {
        __enable_irq();
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
        DEBUG_PRINT(DEBUG_DEBUG, "STOP");
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

    if (eeprom.isPIDMode())
    {
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
    else
    {
        pwmLevel = eeprom.getMotorPWM() * kMaxPWMLevel / 100;
    }

    // update pwm stats
    int32_t clampedPwmLevel = clampPWMLevel(pwmLevel);
    stats.pwm.update(clampedPwmLevel);

    // apply new PWM level if motor is running
    if (running)
    {
        PID_WRITE_MOTOR_PWM_ON(clampedPwmLevel, motorDirection);

        if (eeprom.isPIDMode()) {
            if (antiWindupReduction) {
                if (pwmLevel < -kMaxPWMLevel || pwmLevel > (kMaxPWMLevel * 2)) {
                    setIntegral(getIntegral() * antiWindupReduction / 100);
                }
            }
        }
    }

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

    if ((stats.counter.loop%200)==0) {
        __enable_irq();
        DEBUG_PRINT(DEBUG_DEBUG, "rc=%u ra=%d p=%d d=%d", readRpmCounter(), 0,/*stats.rpmAnalog.get(),*/ stats.counter.pulse, delta);
    }

    // // use analog sensor signal to calculate RPM
    // // this is independent of the direction
    // if ((stats.counter.loop & 0x1f) == 0x1f) {
    //     static constexpr uint32_t kRPMInterval = kPIDInterval * (0x1f + 1);
    //     uint32_t rpmCounter = readRpmCounter();
    //     uint32_t diff = rpmCounter - stats.lastValue.rpmCounter;
    //     stats.lastValue.rpmCounter = rpmCounter;
    //     stats.lastValue.analogRpmMeasured = diff * (60000 / kRPMInterval);
    //     stats.rpmAnalog.update(stats.lastValue.analogRpmMeasured);
    // }

    stats.counter.loop++;
}

void PidController::fault_isr()
{
    if ((digitalPinToGPIO<DRV8701_FAULT_PIN>()->IDR & (1 << digitalPinToBit(DRV8701_FAULT_PIN))) == 0)
    {
        faults.drv8701Fault = true;
#if DEBUG
        faults.count++;
#endif
    }
    if ((digitalPinToGPIO<OCP_INT_PIN>()->IDR & (1 << digitalPinToBit(OCP_INT_PIN))) == 0)
    {
        faults.ocpFault = true;
#if DEBUG
        faults.count++;
#endif
    }
    if ((digitalPinToGPIO<DRV_SNSOUT_PIN>()->IDR & (1 << digitalPinToBit(DRV_SNSOUT_PIN))) == 0)
    {
        faults.snsoutFault = true;
#if DEBUG
        faults.count++;
#endif
    }
    // TODO add emergency stop and fault handling
}
