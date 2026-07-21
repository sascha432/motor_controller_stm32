/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <HardwareTimer.h>
#include "helpers.h"
#include "pins.h"
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

    enum class ErrorCodeType : int32_t {
        NONE = 0,
        STALL,
        SENSOR,
        MOTOR_OVER_TEMPERATURE,
        MOSFET_OVER_TEMPERATURE,
        FAULT,
        OCP,
        SNSOUT,
    };

    PidController() :
        timer(TIM2),
        rpm(0),
        integralTimeLimit(kIntegralTimeLimit),
        antiWindupReduction(kAntiWindupReduction),
        running(false)
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
     * @brief disable PID controller
     *
     */
    void disable();

    /**
     * @brief enable PID controller
     *
     */
    void enable(InterruptCallbackType callback);

    /**
     * @brief initialize PID controller
     *
     * @param callback PID loop callback
     */
    void init(InterruptCallbackType callback, InterruptCallbackType faultCallback);

    inline void setKp(float value) 
    {
        Kp = value;
        // pre calculate Kp to reduce calculations to 3 multiplications in the PID loop
        // scale value and reduce CPR
        // KpPreCalc = value * kFPScale / (kRPMToIntCounts(kCPR) / (float)kMaxPWMLevel);
        // output to pwm scaler
        // KiPreCalc = KiPreCalc * kPWMScaleMultiplier;

        // optimized version using double precision inside the constexpr and a single float multiplication operation
        KpPreCalc = value * static_cast<float>(kScaleFactor / ((kRPMToIntCountsT<kCPR>() / static_cast<double>(kMaxPWMLevel)) / static_cast<double>(kPWMScaleMultiplier)));
    }

    inline void setKi(float value) 
    {
        Ki = value;
        // scale value and reduce CPR
        // KiPreCalc = value * kFPScale / (kRPMToIntCounts(kCPR) / (float)kMaxPWMLevel);
        // add dt into precalc
        // KiPreCalc = KiPreCalc * kPIDInterval / 1000; // KiPreCalc = KiPreCalc * dt
        // output to pwm scaler
        // KiPreCalc = KiPreCalc * kPWMScaleMultiplier;
        KiPreCalc = value * static_cast<float>(kScaleFactor * kPIDInterval / ((kRPMToIntCountsT<kCPR>() * 1000 / static_cast<double>(kMaxPWMLevel)) / static_cast<double>(kPWMScaleMultiplier)));
    }

    inline void setKd(float value) 
    {
        Kd = value;
        // scale value and reduce CPR
        // KdPreCalc = value * kFPScale / (kRPMToIntCounts(kCPR) / (float)kMaxPWMLevel);
        // add dt into precalc
        // KdPreCalc = KdPreCalc * 1000 / kPIDInterval; // KdInt = KdInt / dt
        // output to pwm scaler
        // KdPreCalc = KdPreCalc * kPWMScaleMultiplier;
        KdPreCalc = value * static_cast<float>(kScaleFactor * 1000 / ((kRPMToIntCountsT<kCPR>() * kPIDInterval / static_cast<double>(kMaxPWMLevel)) / static_cast<double>(kPWMScaleMultiplier)));
    }

    inline int32_t calcPWMLevel(int32_t error, int32_t integral, int32_t derivative) const 
    {
        return (
            (error * (int64_t)KpPreCalc) +
            (integral * (int64_t)KiPreCalc) +
            (derivative * (int64_t)KdPreCalc)
        ) / kScaleFactor;
    }

    inline int32_t clampPWMLevel(int32_t value) const 
    {
        return std::clamp<int32_t>(value, 0, kMaxPWMLevel);
    }

    inline void setRPM(uint32_t value) 
    {
        // rev. per minute
        rpm = value;
        // to counts per interval
        cpi = (rpm * kCPR) / (60000 / kPIDInterval);
        // anti windup limit for integral term
        cpiIntegralLimit = cpi * integralTimeLimit / kPIDInterval;
    }

    inline uint32_t getRPM() const 
    {
        return rpm;
    }

    inline uint32_t clampRPM(int32_t value) const 
    {
        return std::clamp<int32_t>(value, 0, 55000);
    }

    // get delta since last call, counter is 16bit only
    inline int32_t getDelta(uint32_t counter) 
    {
        int16_t delta = (int16_t)counter - (int16_t)lastCounter;
        lastCounter = counter;
        return delta;
    }

    inline uint16_t readEncoderCounter() const 
    {
        return TIM4->CNT;
    }

    inline uint16_t readRpmCounter() const 
    {
        return TIM5->CNT;
    }

    void reset();

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

    void printDebug(Stream &stream) const;

    /**
     * @brief Interrupt service routine for the PID controller.
     * 
     * This function should be called at a fixed interval defined by kPIDInterval.
     * It reads the encoder counter, calculates the error, derivative, and integral,
     * updates the PWM output, and handles anti-windup.
     */
    void isr();

    /**
     * @brief Interrupt service routine for handling motor controller faults
     */
    void fault_isr();

    /**
     * @brief Update internal fault states
     * 
     */
    void readFaults() 
    {
        faults.drv8701Fault = (digitalPinToGPIO<DRV8701_FAULT_PIN>()->IDR & (1 << digitalPinToBit(DRV8701_FAULT_PIN))) == 0;
        faults.ocpFault = (digitalPinToGPIO<OCP_INT_PIN>()->IDR & (1 << digitalPinToBit(OCP_INT_PIN))) == 0;
        faults.snsoutFault = (digitalPinToGPIO<DRV_SNSOUT_PIN>()->IDR & (1 << digitalPinToBit(DRV_SNSOUT_PIN))) == 0;
        faults.count = 0;
    }

    void motorOn();
    void motorOff();
    bool motorToggle();

    void setErrorCode(ErrorCodeType code) 
    {
        PID_WRITE_MOTOR_PWM_OFF();
        running = false;
        errorCode = code;
    }

    ErrorCodeType getErrorCode() const 
    {
        return errorCode;
    }

    void debugPrintFaults() const 
    {
        DEBUG_PRINT(DEBUG_DEBUG, "FAULT=%d OCP=%d SNSOUT=%d COUNT=%d", faults.drv8701Fault, faults.ocpFault, faults.snsoutFault, faults.count);
    }

public:
    struct FaultStates {
        FaultStates() : count(0), drv8701Fault(false), ocpFault(false), snsoutFault(false) {}
        volatile uint32_t count;
        volatile bool drv8701Fault : 1;
        volatile bool ocpFault : 1;
        volatile bool snsoutFault : 1;

        void reset() {
            __disable_irq();
            count = 0;
            drv8701Fault = false;
            ocpFault = false;
            snsoutFault = false;
            __enable_irq();
        }
    };

public:
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

    struct Average {
        uint32_t sum;
        uint32_t count;

        Average() : sum(0), count(0) {}

        void reset() {
            *this = Average();
        }

        void add(uint32_t value) {
            sum += value;
            if (++count > 1000 / kPIDInterval) { // rolling average over 1 second
                sum -= sum / 16;
                count -= count / 16;
            }
        }

        uint32_t avg() const {
            return count ? sum / count : 0;
        }
    };

    struct {
        Average rpm;
        Average pwm;
    } stats;

    bool running;
    FaultStates faults;
    uint32_t loopCounter;
    uint32_t pulseCounter;
    ErrorCodeType errorCode;

    #if HAVE_DEBUG_PID_CONTROLLER
        volatile uint32_t lastRpmMeasured;
        volatile uint32_t lastPwmLevel;
        volatile bool lastDebugNewData = false;
    #endif
};

extern PidController pid;
