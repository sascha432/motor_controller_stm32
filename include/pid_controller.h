/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <HardwareTimer.h>
#include "helpers.h"
#include "eeprom.h"

struct PidController 
{
    static constexpr float kKpDefault = 0.12f;
    static constexpr float kKiDefault = 0.8f;
    static constexpr float kKdDefault = 0.0f;
    static constexpr uint16_t kMaxPWMLevel = kPWMFrequencyToARR<20000>();   // Motor PWM 20Khz
    static constexpr uint16_t kPPR = 1024;                                  // MT6701 PPR
    static constexpr uint16_t kCPR = kPPR * 4;                              // 4x Mode PPR to CPR
    static constexpr uint32_t kPIDInterval = 5;                             // PID update rate in millis
    static constexpr uint32_t kAntiWindupReduction = 97;                    // reduce integral if error is out of range
    static constexpr uint32_t kIntegralTimeLimit = 2000;                    // limit integral to 2000ms worth of max. error
    static constexpr float kPWMScaleMultiplier = 1.0 / (100.0 / kMaxPWMLevel);
    static constexpr int32_t kScaleFactor =                                 // scale factor for PID calculations
        static_cast<int32_t>(
            16384.0f *
            (kCPR / 4096.0f) *
            (5.0f / kPIDInterval) *
            (1800.0f / kMaxPWMLevel) *
            (18.0f / kPWMScaleMultiplier)
        );

    /*
        the kScaleFactor calculation gives the following range for tuning PID values (Kp, Ki, Kd)

        | Gain   | Minimum Kx (`PreCalc=10`) | Maximum Kx (`PreCalc=max int32`) |
        | ------ | ------------------------: | -------------------------------: |
        | **Kp** |                  0.000026 |                             5655 |
        | **Ki** |                    0.0053 |                          1131000 |
        | **Kd** |                0.00000013 |                               28 |
    */

    // convert counts/RPM for 200Hz/5ms interval
    template<int32_t VALUE>
    static constexpr uint32_t kRPMToIntCountsT() {
        return (VALUE * kCPR) / (60000 / kPIDInterval);
    }

    static constexpr int32_t kIntCountsToRPM(int32_t value) {
        return (value * (60000 / kPIDInterval)) / kCPR;
    }

    PidController() :
        timer(TIM2),
        rpm(0),
        integralTimeLimit(kIntegralTimeLimit),
        antiWindupReduction(kAntiWindupReduction),
        avgRPM(0),
        running(false),
        reverseDirection(false)
    {
        setKp(kKpDefault);
        setKi(kKiDefault);
        setKd(kKdDefault);
    }

    PidController(float Kp, float Ki, float Kd) {
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
    }

    /**
     * @brief initialize QDEC pins for MT6701 encoder
     * 
     */
    inline void initQDECpins() {
        // ENC1_A and ENC1_B on PB6 and PB7 / GPIOB
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

        // Input with pull-up/pull-down (CNF=10, MODE=00)
        GPIOB->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4))); // Clear pin configs
        GPIOB->CRL |=  ((0x8 << (6 * 4)) | (0x8 << (7 * 4))); // CNF=10, MODE=00

        // Select pull-up (ODR bit = 1)
        GPIOB->ODR |= (1 << 6) | (1 << 7);
    }

    /**
     * @brief disable PID controller
     * 
     */
    inline void disable() {
        TIM4->CR1 &= ~TIM_CR1_CEN;
        timer.pause();   
    }

    /**
     * @brief enable PID controller
     * 
     */
    inline void enable() {
        TIM4->CR1 |= TIM_CR1_CEN;
        timer.resume();
    }

    /**
     * @brief initialize PID controller
     * 
     * @param callback PID loop callback
     */
    template<typename ISRCallback>
    void init(ISRCallback callback) {
        running = false;
        setRPM(EEPROM::getInstance().getMotorRPM());

        // === PWM on TIM1 CH1 (PA8, PA9) ===
        // Enable AFIO and GPIOA for PA8/PA9
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN;
        // Enable TIM1 (APB2)
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

        // PA8 AF push-pull (CRH: pin 8)
        GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
        GPIOA->CRH |= (GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1); // 50MHz, AF push-pull

        #if 0 // disabled for serial debug
        // PA9 AF push-pull (CRH: pin 9)
        GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
        GPIOA->CRH |= (GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1); // 50MHz, AF push-pull    
        #endif

        // PWM setup on TIM1 CH1
        TIM1->PSC = 0;
        TIM1->ARR = kMaxPWMLevel - 1;
        PID_WRITE_MOTOR_PWM(0);
        TIM1->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
        TIM1->CCER |= TIM_CCER_CC1E;
        TIM1->BDTR |= TIM_BDTR_MOE; // main output enable for TIM1
        TIM1->CR1 |= TIM_CR1_CEN;

        initQDECpins();

        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // alternative function clock enable
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable TIM4 clock

        // TIM4 Encoder Mode for MT6701
        TIM4->CR1 = 0;
        TIM4->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // Encoder mode 3
        TIM4->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
        TIM4->CCER = 0;
        TIM4->ARR = 0xFFFF;
        TIM4->CNT = 0;

        // Call PID ISR every 5 ms
        timer.setOverflow(kPIDInterval * 1000, MICROSEC_FORMAT);
        timer.attachInterrupt(callback);
        enable();
    }

    inline void setKp(float value) {
        Kp = value;
        // pre calculate Kp to reduce calculations to 3 multiplications in the PID loop
        // scale value and reduce CPR
        // KpPreCalc = value * kFPScale / (kRPMToIntCounts(kCPR) / (float)kMaxPWMLevel);
        // output to pwm scaler
        // KiPreCalc = KiPreCalc * kPWMScaleMultiplier;

        // optimized version using double precision inside the constexpr and a single float multiplication operation
        KpPreCalc = value * static_cast<float>(kScaleFactor / ((kRPMToIntCountsT<kCPR>() / static_cast<double>(kMaxPWMLevel)) / static_cast<double>(kPWMScaleMultiplier)));
    }

    inline void setKi(float value) {
        Ki = value;
        // scale value and reduce CPR
        // KiPreCalc = value * kFPScale / (kRPMToIntCounts(kCPR) / (float)kMaxPWMLevel);
        // add dt into precalc
        // KiPreCalc = KiPreCalc * kPIDInterval / 1000; // KiPreCalc = KiPreCalc * dt
        // output to pwm scaler
        // KiPreCalc = KiPreCalc * kPWMScaleMultiplier;
        KiPreCalc = value * static_cast<float>(kScaleFactor * kPIDInterval / ((kRPMToIntCountsT<kCPR>() * 1000 / static_cast<double>(kMaxPWMLevel)) / static_cast<double>(kPWMScaleMultiplier)));
    }

    inline void setKd(float value) {
        Kd = value;
        // scale value and reduce CPR
        // KdPreCalc = value * kFPScale / (kRPMToIntCounts(kCPR) / (float)kMaxPWMLevel);
        // add dt into precalc
        // KdPreCalc = KdPreCalc * 1000 / kPIDInterval; // KdInt = KdInt / dt
        // output to pwm scaler
        // KdPreCalc = KdPreCalc * kPWMScaleMultiplier;
        KdPreCalc = value * static_cast<float>(kScaleFactor * 1000 / ((kRPMToIntCountsT<kCPR>() * kPIDInterval / static_cast<double>(kMaxPWMLevel)) / static_cast<double>(kPWMScaleMultiplier)));
    }

    inline int32_t calcPWMLevel(int32_t error, int32_t integral, int32_t derivative) const {
        return (
            (error * (int64_t)KpPreCalc) + 
            (integral * (int64_t)KiPreCalc) + 
            (derivative * (int64_t)KdPreCalc)
        ) / kScaleFactor;
    }

    inline int32_t clampPWMLevel(int32_t value) const {
        return std::clamp<int32_t>(value, 0, kMaxPWMLevel);
    }

    inline void setRPM(uint32_t value) {
        // rev. per minute
        rpm = value;
        // to counts per interval
        cpi = (rpm * kCPR) / (60000 / kPIDInterval);
        // anti windup limit for integral term
        cpiIntegralLimit = cpi * integralTimeLimit / kPIDInterval;
    }

    inline uint32_t getRPM() const {
        return rpm;
    }

    inline uint32_t clampRPM(int32_t value) const {
        return std::clamp<int32_t>(value, 0, 55000);
    }

    // get delta since last call, counter is 16bit only
    inline int32_t getDelta(uint32_t counter) {
        int16_t delta = (int16_t)counter - (int16_t)lastCounter;
        lastCounter = counter;
        return delta;
    }

    inline uint16_t readEncoderCounter() const {
        return TIM4->CNT;
    }

    void reset() {
        lastCounter = readEncoderCounter();
        lastError = 0;
        lastDerivative = 0;
        integral = 0;
        avgRPM = 0;
    }

    void setIntegral(int32_t value) {
        // cap the integral
        if (value > cpiIntegralLimit) {
            integral = cpiIntegralLimit;
        }
        else if (value < -cpiIntegralLimit) {
            integral = -cpiIntegralLimit;
        }
        else {
            integral = value;
        }
    }

    inline int32_t getIntegral() const {
        return integral;
    }

    inline void updateIntegral(int32_t error) {
        setIntegral(integral + error);
    }

    inline void setLastError(int32_t value) {
        lastError = value;
    }

    inline int32_t getLastError() const {
        return lastError;
    }

    inline void setLastDerivative(int32_t value) {
        lastDerivative = value;
    }

    inline int32_t getLastDerivative() const {
        return lastDerivative;
    }

    inline int32_t getCountsPerInterval() const {
        return cpi;
    }

    void printDebug(Stream &stream) const {
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

    void isr() {
        // most timers are 16bit counters only
        int32_t delta = getDelta(readEncoderCounter());
        // reverse direction if wired the opposite way
        if (reverseDirection) {  
            delta = -delta;
        }

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
        int32_t pwmLevel = calcPWMLevel(error, getIntegral(), derivative);

        // apply new PWM level if motor is running
        if (running) {
            PID_WRITE_MOTOR_PWM(clampPWMLevel(pwmLevel));
        }

        #if 1
        if (antiWindupReduction) {
            if (pwmLevel < -kMaxPWMLevel || pwmLevel > (kMaxPWMLevel * 2)) {
                setIntegral(getIntegral() * antiWindupReduction / 100);
            }
        }
        #endif

        // uint32_t rpm = delta > 0 ? kIntCountsToRPM(delta) : 0;
        uint32_t rpm = kIntCountsToRPM(delta & ~(delta >> 31));

        constexpr uint16_t kAvgInterval = 100 / kPIDInterval; // 100Hz avg
        avgRPM = ((avgRPM * (kAvgInterval - 1)) + rpm) / kAvgInterval;

        #if HAVE_DEBUG_PID_CONTROLLER
            lastPwmLevel = pwmLevel;
            lastRpmMeasured = rpm;
            lastDebugNewData = true;
        #endif
    }

    HardwareTimer timer;

    float Kp;
    float Ki;
    float Kd;
    uint32_t rpm;
    uint16_t integralTimeLimit;
    uint16_t antiWindupReduction;

    uint32_t lastCounter;
    int32_t integral;
    int32_t lastError;
    int32_t lastDerivative;

    int32_t KpPreCalc;
    int32_t KiPreCalc;
    int32_t KdPreCalc;
    int32_t cpi;
    int32_t cpiIntegralLimit;
    uint32_t avgRPM;

    bool running;
    bool reverseDirection;

    #if HAVE_DEBUG_PID_CONTROLLER
        volatile uint32_t lastRpmMeasured;
        volatile uint32_t lastPwmLevel;
        volatile bool lastDebugNewData = false;
    #endif
};
