/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"
#include "pins.h"
#include "eeprom.h"
#include "stats.h"
#include "leds.h"

struct PidController
{
    static constexpr uint16_t kMaxPWMLevel = kPWMFrequencyToARR<20000>();           // Motor PWM 20Khz
    static constexpr uint16_t kPPR = 1024;                                          // MT6701 PPR
    static constexpr uint16_t kCPR = kPPR * 4;                                      // 4x Mode PPR to CPR
    static constexpr float kPIDIntervalFloat = 5.12f;                               // PID update rate in millis used for precise RPM calculation
    static constexpr uint32_t kPIDInterval = 5;                                     // PID update rate in millis
    static constexpr uint32_t kAntiWindupFactor = 100;                              // anti-windup factor
    static constexpr uint32_t kAntiWindupReduction = 0.97f * kAntiWindupFactor;     // reduce integral if error is out of range (97%)
    static constexpr uint32_t kIntegralTimeLimit = 2500;                            // anti windup integral time limit in milliseconds
    static constexpr bool kProgramPPR = false;                                      // set to true to program the MT6701 encoder during boot over i2c

    // PID scaling factors
    static constexpr float kPWMScaleMultiplier = 1.0 / (100.0 / kMaxPWMLevel);      // convert pwm level to percentage
    static constexpr int32_t kScaleFactor = 4096;                                   // scale factor for fixed point PID calculations
    static constexpr float kPulsesPerRPMPerInterval = (kCPR) / (60000 / kPIDIntervalFloat);
    static constexpr float kPIDScaleFactor = (kScaleFactor / kPulsesPerRPMPerInterval) * kPWMScaleMultiplier;
    static constexpr float kKpScaleFactor = kPIDScaleFactor;                                        // scale_factor
    static constexpr float kKiScaleFactor = kPIDScaleFactor * (kPIDInterval / 1000.0);              // scale_factor * dt
    static constexpr float kKdScaleFactor = kPIDScaleFactor * (1.0 / (kPIDInterval / 1000.0));      // scale_factor / dt

    // useable min/max values for fixed point calculation
    static constexpr float kMinKpValue = 10.0 / kKpScaleFactor;
    static constexpr float kMaxKpValue = UINT32_MAX / kKpScaleFactor;
    static constexpr float kMinKiValue = 10.0 / kKiScaleFactor;
    static constexpr float kMaxKiValue = UINT32_MAX / kKiScaleFactor;
    static constexpr float kMinKdValue = 10.0 / kKdScaleFactor;
    static constexpr float kMaxKdValue = UINT32_MAX / kKdScaleFactor;

    // assert possible ranges
    // TODO limit values in the UI
    static_assert(kMinKpValue < 0.0001, "kMinKpValue");
    static_assert(kMaxKpValue > 1000, "kMaxKpValue");
    static_assert(kMinKiValue < 0.005, "kMinKiValue");
    static_assert(kMaxKiValue > 10000, "kMaxKiValue");
    static_assert(kMinKdValue < 0.000001, "kMinKdValue");
    static_assert(kMaxKdValue > 50, "kMaxKdValue");

    // convert counts/RPM for 200Hz/5ms interval
    template<int32_t VALUE>
    static constexpr uint32_t kRPMToIntCountsT() {
        return (VALUE * kCPR) / (60000 / kPIDIntervalFloat);
    }

    static constexpr int32_t kIntCountsToRPM(int32_t value) {
        return (value * (uint32_t)(60000 / kPIDIntervalFloat)) / kCPR;
    }

    enum class ErrorCodeType : int32_t {
        NONE = 0,
        STALL,
        SENSOR,
        SENSOR_REVERSE,
        MOTOR_OVER_TEMPERATURE,
        MOSFET_OVER_TEMPERATURE,
        FAULT,
        OCP,
        SNSOUT,
        OVP
    };

    /**
     * @brief Construct a new Pid Controller object
     *
     */
    PidController() :
        rpm(0),
        motorDirection(EEPROM::kMotorDirectionForward),
        antiWindupReduction(kAntiWindupReduction),
        errorCode(ErrorCodeType::NONE),
        running(false)
    {
        setKp(1.0f);
        setKi(0.5f);
        setKd(0.0f);
    }

    /**
     * @brief Construct a new Pid Controller object
     *
     * @param Kp
     * @param Ki
     * @param Kd
     */
    PidController(float Kp, float Ki, float Kd)
    {
        setKp(Kp);
        setKi(Ki);
        setKd(Kd);
    }

    /**
     * @brief initialize PID controller
     *
     * @param callback PID loop callback
     */
    void init();

    /**
     * @brief Set the Kp value and pre-calculate KpPreCalc for PID loop
     *
     * @param value
     */
    inline void setKp(float value)
    {
        Kp = value;
        KpPreCalc = value * kKpScaleFactor;
        SWO::data.Kp = value;
    }

    /**
     * @brief Set the Ki value and pre-calculate KiPreCalc for PID loop
     *
     * @param value
     */
    inline void setKi(float value)
    {
        Ki = value;
        KiPreCalc = value * kKiScaleFactor;
        SWO::data.Ki = value;
    }

    /**
     * @brief Set the Kd value and pre-calculate KdPreCalc for PID loop
     *
     * @param value
     */
    inline void setKd(float value)
    {
        Kd = value;
        KdPreCalc = value * kKdScaleFactor;
        SWO::data.Kd = value;
    }

    inline int32_t calcPWMLevel(int32_t error, int32_t integral, int32_t derivative) const
    {
        #if 0
            // floating point version for testing
            const float Kp = this->Kp * (kKpScaleFactor / kScaleFactor);
            const float Ki = this->Ki * (kKiScaleFactor / kScaleFactor);
            const float Kd = this->Kd * (kKdScaleFactor / kScaleFactor);
            return static_cast<int32_t>((error * Kp) + (integral * Ki) + (derivative * Kd));
        #else
            // FP version kScaleFactor
            return (
                (error * (int64_t)KpPreCalc) +
                (integral * (int64_t)KiPreCalc) +
                (derivative * (int64_t)KdPreCalc)
            ) / kScaleFactor;
        #endif
    }

    inline int32_t clampPWMLevel(int32_t value) const
    {
        return std::clamp<int32_t>(value, 0, kMaxPWMLevel);
    }

    /**
     * @brief Set target RPM and update limits
     *
     * @param value
     */
    inline void setRPM(uint32_t value)
    {
        // PID tuning via SWO
        SWO::data.rpm = value;
        // rev. per minute
        rpm = value;
        // to counts per interval
        cpi = (rpm * kCPR) / (int32_t)(60000 / kPIDIntervalFloat);
        // anti windup limit for integral term
        cpiIntegralLimit = cpi * (int32_t)(kIntegralTimeLimit * kPIDIntervalFloat / 1000.0f);
    }

    /**
     * @brief Set the Anti Windup Reduction
     *
     * @param value
     */
    inline void setAntiWindupReduction(uint16_t value)
    {
        antiWindupReduction = value;
        SWO::data.antiWindupReduction = value / (float)UIConstants::kAntiWindupFactor;
    }

    /**
     * @brief Get target RPM
     *
     * @return uint32_t
     */
    inline uint32_t getRPM() const
    {
        return rpm;
    }

    /**
     * @brief Set the motor direction
     *
     * @param forward
     */
    void setMotorDirection(bool forward)
    {
        motorDirection = forward ? EEPROM::kMotorDirectionForward : EEPROM::kMotorDirectionReverse;
    }

    /**
     * @brief Return true if the motor direction is forward, false otherwise
     *
     * @return true
     * @return false
     */
    bool isForwardMotorDirection() const
    {
        return motorDirection == EEPROM::kMotorDirectionForward;
    }

    /**
     * @brief Toggle the motor direction
     */
    void toggleMotorDirection()
    {
        motorDirection = (motorDirection == EEPROM::kMotorDirectionForward) ? EEPROM::kMotorDirectionReverse : EEPROM::kMotorDirectionForward;
    }

    /**
     * @brief Clamp RPM
     *
     * @param value
     * @return uint32_t
     */
    inline uint32_t clampRPM(int32_t value) const
    {
        return std::clamp<int32_t>(value, 0, 55000);
    }

    /**
     * @brief Get delta since last call, counter is 16bit only
     *
     * @param counter The current encoder counter value
     * @return int32_t The delta since the last call
     */
    inline int32_t getDelta(uint32_t counter)
    {
        int16_t delta = (int16_t)counter - (int16_t)lastEncoderCounter;
        lastEncoderCounter = counter;
        return delta;
    }

    /**
     * @brief Reset controller
     *
     */
    void reset();

    void setIntegral(int32_t value)
    {
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

    inline int32_t getIntegral() const
    {
        return integral;
    }

    inline void updateIntegral(int32_t error)
    {
        setIntegral(integral + error);
    }

    inline void setLastError(int32_t value)
    {
        lastError = value;
    }

    inline int32_t getLastError() const
    {
        return lastError;
    }

    inline void setLastDerivative(int32_t value)
    {
        lastDerivative = value;
    }

    inline int32_t getLastDerivative() const
    {
        return lastDerivative;
    }

    inline int32_t getCountsPerInterval() const
    {
        return cpi;
    }

    /**
     * @brief Interrupt service routine for the PID controller.
     *
     * This function should be called at a fixed interval defined by kPIDInterval.
     * It reads the encoder counter, calculates the error, derivative, and integral,
     * updates the PWM output, and handles anti-windup.
     */
    void isr();

    /**
     * @brief OCP interrupt service routine for the PID controller
     *
     */
    void ocp_isr();

    /**
     * @brief Trigger OCP event
     */
    void trigger_ocp();

    /**
     * @brief Update internal fault states
     *
     */
    void resetFaults()
    {
        faults.drv8701Fault = (digitalPinToGPIO<DRV8701_FAULT_PIN>()->IDR & (1 << digitalPinToBit(DRV8701_FAULT_PIN))) == 0;
        faults.ocpFault = (digitalPinToGPIO<OCP_INT_PIN>()->IDR & (1 << digitalPinToBit(OCP_INT_PIN))) == 0;
        faults.snsoutFault = (digitalPinToGPIO<DRV_SNSOUT_PIN>()->IDR & (1 << digitalPinToBit(DRV_SNSOUT_PIN))) == 0;
    }

    /**
     * @brief Apply PID parameters from EEPROM to the controller
     *
     */
    void applyPIDParams()
    {
        setKp(eeprom.getKp());
        setKi(eeprom.getKi());
        setKd(eeprom.getKd());
        setAntiWindupReduction(eeprom.getAntiWindupReduction());
        setRPM(eeprom.getMotorRPM());
    }

    /**
     * @brief Turn motor on in the specified direction
     *
     * @param direction
     */
    void motorOn();

    /**
     * @brief Turn motor off
     *
     */
    void motorOff();

    /**
     * @brief Toggle motor state. If the motor is running, it will be turned off. If the motor is off, it will be turned on in the specified direction
     *
     * @param direction
     * @return true the motor is running after the call
     * @return false the motor is not running after the call
     */
    bool motorToggle();

    /**
     * @brief Set the Error Code and stop PID controller
     *
     * @param code
     */
    void setErrorCode(ErrorCodeType code)
    {
        PID_WRITE_MOTOR_PWM_OFF();
        running = false;
        errorCode = code;
        if (code != ErrorCodeType::NONE) {
            LEDs::onLEDError();
        }
    }

    /**
     * @brief Get the Error Code
     *
     * @return ErrorCodeType
     */
    ErrorCodeType getErrorCode() const
    {
        return errorCode;
    }

    /**
     * @brief Return true if an error code is set, false otherwise
     *
     * @return true
     * @return false
     */
    bool hasErrorCode() const
    {
        return errorCode != ErrorCodeType::NONE;
    }

    /**
     * @brief Print the current error code as a string into the provided buffer.
     *
     * @param buf Buffer to store the error string.
     * @param bufSize Size of the buffer.
     * @return size_t Number of characters written.
     */
    size_t errorPrintf(char *buf, size_t bufSize) const
    {
        switch(errorCode) {
            case ErrorCodeType::NONE:
                return snprintf(buf, bufSize, "NONE");
            case ErrorCodeType::STALL:
                return snprintf(buf, bufSize, "STALL");
            case ErrorCodeType::SENSOR:
                return snprintf(buf, bufSize, "SENSOR");
            case ErrorCodeType::SENSOR_REVERSE:
                return snprintf(buf, bufSize, "SENSOR REVERSE");
            case ErrorCodeType::MOTOR_OVER_TEMPERATURE:
                return snprintf(buf, bufSize, "MOTOR %d" DEGREE_UTF8 "C", ::stats.motorTemp);
            case ErrorCodeType::MOSFET_OVER_TEMPERATURE:
                return snprintf(buf, bufSize, "MOSFET %d" DEGREE_UTF8 "C", ::stats.mosfetTemp);
            case ErrorCodeType::OVP:
                return snprintf(buf, bufSize, "OVP");
            case ErrorCodeType::OCP:
                return snprintf(buf, bufSize, "OCP");
            case ErrorCodeType::FAULT:
                return snprintf(buf, bufSize, "DRV8701 FAULT");
            default:
                break;
            // case ErrorCodeType::OCP:
            //     return snprintf(buf, bufSize, "OCP");
            // case ErrorCodeType::SNSOUT:
            //     return snprintf(buf, bufSize, "DRV8701 SNSOUT");
        }
        return snprintf(buf, bufSize, "ERROR #%d", static_cast<int>(errorCode));
    }

public:
    // === Fault states data structure ===

    struct FaultStates
    {
        uint32_t isenseMax;             // maximum current as ADC value
        uint32_t vsenseMax;             // maximum voltage as ADC value
        bool drv8701Fault : 1;          // DRV8701 fault pin state
        bool ocpFault : 1;              // OCP(INA381) fault pin state
        bool snsoutFault : 1;           // SNSOUT(motor current limit) fault pin state

        FaultStates() :
            isenseMax(INT32_MAX),
            vsenseMax(INT32_MAX),
            drv8701Fault(false),
            ocpFault(false),
            snsoutFault(false)
        {
        }

        void reset()
        {
            drv8701Fault = false;
            ocpFault = false;
            snsoutFault = false;
        }
    };

    // === Statistics data structure ===

    struct StatsType
    {
        Helpers::LowPass<32> rpm;           // filtered RPM for displaying
        Helpers::LowPass<8> pwm;            // filtered PWM for displaying
        struct {
            uint32_t loop;                  // number of times the PID loop has been called
            int32_t pulse;                  // number of pulses received from the A/B motor encoder
        } counter;

        void reset(uint32_t rpmCounter)
        {
            rpm.reset();
            pwm.reset();
            counter.loop = 0;
            counter.pulse = 0;
        }
    };

    // === PID tuning data structure ===

    struct PidLoopType
    {
        uint32_t sequence;
        uint32_t dataAddress;
        uint16_t rpm;
        uint16_t pwmLevel;
        uint16_t voltage;
        uint16_t currentOcp;
        uint16_t currentAverage;
        uint16_t motorTemperature;
        uint16_t mosfetTemperature;
        uint16_t dacMotorCurrent;
        uint16_t dacInputCurrent;
        uint32_t integral;
        uint32_t running: 1;
        uint32_t drv8701Fault : 1;
        uint32_t ocpFault : 1;
        uint32_t snsoutFault : 1;

        PidLoopType() :
            dataAddress(reinterpret_cast<uint32_t>(&SWO::data))
        {
        }
    };
    static constexpr size_t kPidLoopTypeSize = sizeof(PidLoopType);

    // === OCP state machine ===

    enum class OcpStateType : uint32_t {
        NONE = 0,           // no OCP condition
        TRIGGERED = 1,      // OCP detected, decreasing motor current limit
        RECOVERY = 2        // OCP recovery, increasing motor current limit
    };

    struct OcpState
    {
        OcpStateType state;                 // state of the over current protection
        uint32_t counter;                   // 5us tick counter
        uint32_t lastCounter;
        uint16_t dacMotorCurrent;
        uint16_t dacInputCurrent;

        OcpState() :
            state(OcpStateType::NONE),
            counter(0),
            lastCounter(0),
            dacMotorCurrent(0),
            dacInputCurrent(0)
        {
        }

        void reset()
        {
            state = OcpStateType::NONE;
            counter = 0;
            lastCounter = 0;
            dacMotorCurrent = DAC_GET_MOTOR_CURRENT();
            dacInputCurrent = DAC_GET_INPUT_CURRENT();
        }
    };

public:
    float Kp;                           // PID K-values
    float Ki;
    float Kd;
    uint32_t rpm;                       // target RPM
    uint32_t motorDirection;            // motor direction
    uint16_t antiWindupReduction;       // anti-windup reduction factor

    uint32_t lastEncoderCounter;        // last encoder counter value
    int32_t integral;                   // PID variables
    int32_t lastError;
    int32_t lastDerivative;

    int32_t KpPreCalc;                  // fixed point pre-calculated K-values for PID loop
    int32_t KiPreCalc;
    int32_t KdPreCalc;
    int32_t cpi;                        // counts per interval (RPM)
    int32_t cpiIntegralLimit;           // cpi integral limit to avoid windup

    uint32_t lastRpmCounter;
    uint32_t lastRpmCounterUpdated;

    StatsType stats;                    // statistics
    FaultStates faults;                 // DRV8701 and ocp faults
    OcpState ocp;                       // OCP state machine

    ErrorCodeType errorCode;            // last error
    RingBuffer<PidLoopType, 8> pidLoopBuffer; // buffer for PID loop data for PID tuning

    volatile bool running;              // true if the PID controller is running
};

extern PidController pid;
