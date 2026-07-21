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
    stats.rpm.reset();
    stats.pwm.reset();
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

    // Fault interrupt pins DRV8701_FAULT_PIN, OCP_INT_PIN, DRV_SNSOUT_PIN
    RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(DRV8701_FAULT_PIN) | RCC_APB2ENR_IOPxEN(OCP_INT_PIN) | RCC_APB2ENR_IOPxEN(DRV_SNSOUT_PIN);

    // clear conf
    GPIO_CRx_REG<DRV8701_FAULT_PIN>()   &= ~(0xf << digitalPinShift(DRV8701_FAULT_PIN));
    GPIO_CRx_REG<OCP_INT_PIN>()         &= ~(0xf << digitalPinShift(OCP_INT_PIN));
    GPIO_CRx_REG<DRV_SNSOUT_PIN>()      &= ~(0xf << digitalPinShift(DRV_SNSOUT_PIN));

    // CNF=10, MODE=00
    GPIO_CRx_REG<DRV8701_FAULT_PIN>()   |= (0x8 << digitalPinShift(DRV8701_FAULT_PIN));
    GPIO_CRx_REG<OCP_INT_PIN>()         |= (0x8 << digitalPinShift(OCP_INT_PIN));
    GPIO_CRx_REG<DRV_SNSOUT_PIN>()      |= (0x8 << digitalPinShift(DRV_SNSOUT_PIN));

    // Select pull-up (ODR bit = 1)
    digitalPinToGPIO<DRV8701_FAULT_PIN>()->ODR  |= (1 << digitalPinToBit(DRV8701_FAULT_PIN));
    digitalPinToGPIO<OCP_INT_PIN>()->ODR        |= (1 << digitalPinToBit(OCP_INT_PIN));
    digitalPinToGPIO<DRV_SNSOUT_PIN>()->ODR     |= (1 << digitalPinToBit(DRV_SNSOUT_PIN));

    // PID loop, call ISR every 5 ms
    timer.setOverflow(kPIDInterval * 1000, MICROSEC_FORMAT);
    timer.attachInterrupt(callback);
    enable(faultCallback);
}

void PidController::printDebug(Stream &stream) const
{
    stream.println("=== PID DEBUG START ===");
    stream.print("RPM=");
    stream.print(rpm);
    stream.print(" Kp=");
    stream.print(Kp, 2);
    stream.print(" Ki=");
    stream.print(Ki, 2);
    stream.print(" Kd=");
    stream.println(Kd, 2);
    stream.print("KpPreCalc=");
    stream.print(KpPreCalc);
    stream.print(" KiPreCalc=");
    stream.print(KiPreCalc);
    stream.print(" KdPreCalc=");
    stream.println(KdPreCalc);
    stream.print("cpi=");
    stream.print(cpi);
    stream.print(" cpiIntegralLimit=");
    stream.print(cpiIntegralLimit);
    stream.print(" antiWindupReduction=");
    stream.print(antiWindupReduction);
    stream.print(" integralTimeLimit=");
    stream.println(integralTimeLimit);
    stream.print("integral=");
    stream.print(integral);
    stream.print(" lastError=");
    stream.print(lastError);
    stream.print(" lastDerivative=");
    stream.println(lastDerivative);
    stream.print("encoderCounter=");
    stream.println(readEncoderCounter());
    stream.println("=== PID DEBUG END ===");
}

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

void PidController::isr()
{
    // most timers are 16bit counters only
    int32_t delta = getDelta(readEncoderCounter());
    // invert if sensor and motor direction are different
    if (eeprom.getMotorDirection() != eeprom.getSensorDirection()) {
        delta = -delta;
    }

    int32_t pwmLevel;

    if (eeprom.isPIDMode()) {
        // calculate error and derivative
        int32_t error = (getCountsPerInterval() - delta) ;
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

    auto clampedPwmLevel = clampPWMLevel(pwmLevel);

    // apply new PWM level if motor is running
    if (running) {
        PID_WRITE_MOTOR_PWM_ON(clampedPwmLevel, eeprom.isReverseDirection());
    }

    if (eeprom.isPIDMode()) {
        if (antiWindupReduction) {
            if (pwmLevel < -kMaxPWMLevel || pwmLevel > (kMaxPWMLevel * 2)) {
                setIntegral(getIntegral() * antiWindupReduction / 100);
            }
        }
    }

    // uint32_t rpm = delta > 0 ? kIntCountsToRPM(delta) : 0;
    uint32_t deltaRPM = kIntCountsToRPM(delta & ~(delta >> 31));

    stats.rpm.add(deltaRPM);
    stats.pwm.add(clampedPwmLevel);

    #if HAVE_DEBUG_PID_CONTROLLER
        lastPwmLevel = pwmLevel;
        lastRpmMeasured = deltaRPM;
        lastDebugNewData = true;
    #endif
}

void PidController::fault_isr()
{
    if ((digitalPinToGPIO<DRV8701_FAULT_PIN>()->IDR & (1 << digitalPinToBit(DRV8701_FAULT_PIN))) == 0) {
        faults.drv8701Fault = true;
        #if DEBUG
        faults.count++;
        #endif
    }
    if ((digitalPinToGPIO<OCP_INT_PIN>()->IDR & (1 << digitalPinToBit(OCP_INT_PIN))) == 0) {
        faults.ocpFault = true;
        #if DEBUG
        faults.count++;
        #endif
    }
    if ((digitalPinToGPIO<DRV_SNSOUT_PIN>()->IDR & (1 << digitalPinToBit(DRV_SNSOUT_PIN))) == 0) {
        faults.snsoutFault = true;
        #if DEBUG
        faults.count++;
        #endif
    }
    // TODO add emergency stop and fault handling
}
