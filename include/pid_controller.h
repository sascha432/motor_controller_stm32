/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"
#include "pins.h"
#include "eeprom.h"
#include "stats.h"
#include "leds.h"

// 32bit floating point ~1800 clock cycles
// int64_t fixed point ~580 clock cycles
// the code has changed since i measured this, but the difference is still significant
#ifndef PID_USE_FLOATING_POINT_MATH
#define PID_USE_FLOATING_POINT_MATH 0
#endif

struct PidController
{
    #if PID_USE_FLOATING_POINT_MATH
    using PidValueType = float;
    #else
    using PidValueType = int32_t;
    #endif

    static constexpr uint32_t kReleaseBreakTimeMillis = 5000;                           // time to release the brake after motor is turned off
    static constexpr uint32_t kInitialSensorCheckTimeMillis = 750;                      // time to initially check for motor stall and sensor errors

    static constexpr uint16_t kPPR = 1024;                                              // MT6701 PPR
    static constexpr uint16_t kCPR = kPPR * 4;                                          // 4x Mode PPR to CPR
    // tested with 1.28, 2.56, 5.12 and 10.24...
    // lower values make the PID controller more responsive, but the graph update rate is reduced to the higher CPU load
    static constexpr float kPIDInterval = 2.56f;                                        // PID update rate in millis used for precise RPM calculation
    static constexpr uint16_t kAntiWindup = 97 * UIConstants::kAntiWindupFactor;        // reduce integral if error is out of range (97%)
    static constexpr bool kProgramPPR = false;                                          // set to true to program the MT6701 encoder during boot over i2c
    static constexpr uint32_t kMaxRPM = 55000;                                          // max. supported RPM by the encoder
    static constexpr float kMaxError = kCPR / (60000 / kPIDInterval) * kMaxRPM;         // factor to reduce error to a value between -1.0 and 1.0 for the PID loop
    #if PID_USE_FLOATING_POINT_MATH
        static constexpr int32_t kFPFactor = 1;
    #else
        static constexpr int32_t kFPFactor = kMaxError;
    #endif

    // convert RPM to counts per PID interval
    static constexpr uint32_t kRPMToIntCounts(uint32_t value)
    {
        return static_cast<uint32_t>((value * kCPR) / static_cast<int32_t>(60000 / kPIDInterval));
    }

    template<int32_t VALUE>
    static constexpr uint32_t kRPMToIntCountsT()
    {
        return kRPMToIntCounts(VALUE);
    }

    // convert counts per PID interval to RPM
    static constexpr int32_t kIntCountsToRPM(int32_t value)
    {
        return static_cast<int32_t>((value * static_cast<int32_t>(60000 / kPIDInterval)) / kCPR);
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
        motorDirection(EEPROM::MotorDirection::Forward),
        antiWindup(kAntiWindup),
        errorCode(ErrorCodeType::NONE),
        running(false),
        releaseBreakCounter(0)
    {
        setKp(UIConstants::kDefaultKp);
        setKi(UIConstants::kDefaultKi);
        setKd(UIConstants::kDefaultKd);
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
     * @brief Set the Kp value and pre-calculate KpPreCalc for PID loop
     *
     * @param value The new Kp value
     */
    inline void setKp(float value)
    {
        Kp = value;
        KpPreCalc = value * pwmLevel.getMax();
        SWO::data.Kp = value;
    }

    /**
     * @brief Set the Ki value and pre-calculate KiPreCalc for PID loop
     *
     * @param value The new Ki value
     */
    inline void setKi(float value)
    {
        Ki = value;
        KiPreCalc = value * pwmLevel.getMax();
        SWO::data.Ki = value;
    }

    /**
     * @brief Set the Kd value and pre-calculate KdPreCalc for PID loop
     *
     * @param value The new Kd value
     */
    inline void setKd(float value)
    {
        Kd = value;
        KdPreCalc = value * pwmLevel.getMax();
        SWO::data.Kd = value;
    }

    /**
     * @brief Calculate the PWM level based on the PID error, integral, and derivative values.
     *
     * @param error The current error value
     * @param integral The integral of the error
     * @param derivative The derivative of the error
     * @return int32_t The calculated PWM level
     */
    inline int32_t calcPWMLevel(PidValueType error, PidValueType integral, PidValueType derivative) const
    {
        #if PID_USE_FLOATING_POINT_MATH
            return (error * KpPreCalc + integral * KiPreCalc + derivative * KdPreCalc);
        #else
            return (error * (int64_t)KpPreCalc + integral * (int64_t)KiPreCalc + derivative * (int64_t)KdPreCalc) / kFPFactor; // use int64_t to avoid overflow
        #endif
    }

    /**
     * @brief Clamp the PWM level to the valid range
     *
     * @param value The PWM level to clamp
     * @return int32_t The clamped PWM level
     */
    inline int32_t clampPWMLevel(int32_t value) const
    {
        return pwmLevel.clamp(value);
    }

    /**
     * @brief Clamp RPM
     *
     * @param value The RPM value to clamp
     * @return uint32_t The clamped RPM value
     */
    inline uint32_t clampRPM(int32_t value) const
    {
        return std::clamp<int32_t>(value, 0, kMaxRPM);
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
        // RPM to counts per PID interval
        countsPerInterval = kRPMToIntCounts(rpm);
    }

    /**
     * @brief Set the Anti Windup
     *
     * @param value Anti-windup factor as EEPROM value not percent
     */
    inline void setAntiWindup(uint16_t value)
    {
        antiWindup = value ? std::clamp<PidValueType>(value, UIConstants::kLowestAntiWindup, UIConstants::kMaxAntiWindup) : 0;
        SWO::data.antiWindup = value;
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
        motorDirection = forward ? EEPROM::MotorDirection::Forward : EEPROM::MotorDirection::Reverse;
    }

    /**
     * @brief Return true if the motor direction is forward, false otherwise
     *
     * @return true
     * @return false
     */
    bool isForwardMotorDirection() const
    {
        return motorDirection == EEPROM::MotorDirection::Forward;
    }

    /**
     * @brief Toggle the motor direction
     */
    void toggleMotorDirection()
    {
        motorDirection = (motorDirection == EEPROM::MotorDirection::Forward) ? EEPROM::MotorDirection::Reverse : EEPROM::MotorDirection::Forward;
    }

    /**
     * @brief Get delta since last call, counter is 16bit only
     *
     * @param counter The current encoder counter value
     * @return int32_t The delta since the last call
     */
    inline int32_t getCountsDelta(uint32_t counter)
    {
        int16_t delta = (int16_t)counter - (int16_t)lastEncoderCounter;
        lastEncoderCounter = counter;
        return delta;
    }

    inline void setIntegral(PidValueType value)
    {
        integral = std::clamp<PidValueType>(value, -kFPFactor, kFPFactor);
    }

    inline PidValueType getIntegral() const
    {
        return integral;
    }

    inline void updateIntegral(PidValueType error)
    {
        setIntegral(integral + error);
    }

    inline void setLastError(PidValueType value)
    {
        lastError = value;
    }

    inline PidValueType getLastError() const
    {
        return lastError;
    }

    inline void setLastDerivative(PidValueType value)
    {
        lastDerivative = value;
    }

    inline PidValueType getLastDerivative() const
    {
        return lastDerivative;
    }

    inline int32_t getCountsPerInterval() const
    {
        return countsPerInterval;
    }

    /**
     * @brief Update internal fault states
     *
     */
    void resetFaults()
    {
        faults.drv8701Fault = !digitalRead<DRV8701_FAULT_PIN>();
        faults.ocpFault = !digitalRead<OCP_INT_PIN>();
        faults.snsoutFault = !digitalRead<DRV_SNSOUT_PIN>();
    }

    /**
     * @brief Apply PID parameters from EEPROM to the controller
     *
     */
    void applyPIDParams()
    {
        setMotorCurrentLimit(eeprom.getMotorCurrentLimit());
        setInputCurrentLimit(eeprom.getInputCurrentLimit());
        setKp(eeprom.getKp());
        setKi(eeprom.getKi());
        setKd(eeprom.getKd());
        setAntiWindup(eeprom.getAntiWindup());
        setRPM(eeprom.getMotorRPM());
    }

    /**
     * @brief Set the Input Current Limit
     *
     * @param value
     */
    void setInputCurrentLimit(uint16_t value)
    {
        faults.isenseMax = ADCConverter::Current::reverse(value);
        adc.setInputCurrentLimit(value);
    }

    /**
     * @brief Set the Motor Current Limit
     *
     * @param value
     */
    void setMotorCurrentLimit(uint16_t value)
    {
        adc.setMotorCurrentLimit(value);
    }

    /**
     * @brief Set the Error Code and stop PID controller
     *
     * @param code
     */
    void setErrorCode(ErrorCodeType code)
    {
        // stop updating the motor pwm in the PID loop
        running = false;
        // turn motor off
        PID_WRITE_MOTOR_PWM_OFF();
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
    inline ErrorCodeType getErrorCode() const
    {
        return errorCode;
    }

    /**
     * @brief Return true if an error code is set, false otherwise
     *
     * @return true
     * @return false
     */
    inline bool hasErrorCode() const
    {
        return errorCode != ErrorCodeType::NONE;
    }

    /**
     * @brief Get the Max PWM Level
     *
     * @return uint16_t
     */
    inline uint16_t getPWMLevelMax() const
    {
        return pwmLevel.getMax();
    }

    /**
     * @brief Get the ARR (Auto-Reload Register) value for the PWM level
     *
     * @return uint16_t
     */
    inline uint16_t getPWMLevelARR() const
    {
        return pwmLevel.getARR();
    }

    /**
     * @brief initialize PID controller
     */
    void init();

    /**
     * @brief Reset controller
     *
     */
    void reset();

    /**
     * @brief Interrupt service routine for the PID controller.
     *
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
     * @brief Print the current error code as a string into the provided buffer.
     *
     * @param buf Buffer to store the error string.
     * @param bufSize Size of the buffer.
     * @return size_t Number of characters written.
     */
    size_t errorPrintf(char *buf, size_t bufSize) const;

    /**
     * @brief Set PWM frequency
     *
     * @param value PWM frequency in Hz
     */
    void setPWMFrequency(uint32_t value);

public:
    // === Fault states data structure ===
    struct FaultStates
    {
        uint32_t isenseMax;             // maximum current as ADC value
        uint32_t vsenseMax;             // maximum voltage as ADC value
        volatile bool drv8701Fault;     // DRV8701 fault pin state
        volatile bool ocpFault;         // OCP(INA381) fault pin state
        volatile bool snsoutFault;      // SNSOUT(motor current limit) fault pin state

        FaultStates() :
            isenseMax(INT32_MAX),
            vsenseMax(INT32_MAX),
            drv8701Fault(false),
            ocpFault(false),
            snsoutFault(false)
        {
        }

        inline void reset()
        {
            drv8701Fault = false;
            ocpFault = false;
            snsoutFault = false;
        }
    };

    // === Statistics data structure ===
    struct StatsType
    {
        Helpers::FixedLowPass<(uint32_t)kPIDInterval, (uint32_t)(kPIDInterval * 8), 512, volatile int32_t> rpm;     // filtered RPM for displaying
        Helpers::FixedLowPass<(uint32_t)kPIDInterval, (uint32_t)(kPIDInterval * 2), 256, volatile int32_t> pwm;     // filtered PWM for displaying

        struct {
            volatile uint32_t loop;                  // number of times the PID loop has been called
            volatile int32_t pulse;                  // number of pulses received from the A/B motor encoder
        } counter;

        inline void reset()
        {
            rpm.reset();
            pwm.reset();
            counter = {};
        }
    };

    // === PID tuning data structure ===
    struct __attribute__((packed)) PidLoopType
    {
        uint16_t rpm;
        uint16_t voltage;
        uint16_t currentOcp;
        uint16_t currentAverage;
        uint16_t motorTemperature;
        uint16_t mosfetTemperature;
        uint16_t dacMotorCurrent;
        uint16_t dacInputCurrent;
        uint16_t error;
        uint16_t integral;
        uint16_t derivative;
        uint8_t pwmLevel;
        uint8_t running: 1;
        uint8_t drv8701Fault : 1;
        uint8_t ocpFault : 1;
        uint8_t snsoutFault : 1;
        uint8_t reserved : 4;
    };
    static constexpr size_t kPidLoopTypeSize = sizeof(PidLoopType);
    static_assert(sizeof(PidLoopType) % 4 == 0, "PidLoopType must be 4-byte aligned");

    // === OCP state machine and constants ===
    static constexpr float kMinADCTimeMicros = 1000000 / ADC::kTotalSamplesPerSecond;   // sample time for current measurement in microseconds, limits retrigger timeout and recovery interval

    static constexpr uint32_t kOcpTickInterval = 20;                                    // 20us tick interval
    static constexpr uint32_t kOcpISenseThreshold = 80;                                 // lower threshold in percent before the OCP condition is cleared
    static constexpr uint32_t kOcpRecoveryInterval = 20 / kOcpTickInterval;             // 20us interval
    static constexpr uint32_t kOcpRetriggerTimeout = 20 / kOcpTickInterval;             // 20us timeout
    static constexpr uint32_t kOcpCurrentRampUp = 16;                                   // increase current by 1/16 every tick
    static constexpr uint32_t kOcpCurrentRampDown = 16;                                 // reduce current by 1/16 every tick
    static constexpr uint32_t kOcpInputToMotorCurrentRatio = 8;                         // if the motor current limit is higher than x the input current limit, it will be reduced to x the input current limit, before ramping it down further
    static constexpr float kOcpAntiWindUpFloat = 0.8f;                                  // strong anti windup during OCP condition

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

        inline void reset()
        {
            state = OcpStateType::NONE;
            counter = 0;
            lastCounter = 0;
            dacMotorCurrent = DAC_GET_MOTOR_CURRENT();
            dacInputCurrent = DAC_GET_INPUT_CURRENT();
        }
    };

    // === PWM level data structure ===
    struct PWMLevel
    {
        PWMLevel() : level(kPWMFrequencyToARR(UIConstants::kDefaultPWMFrequency)) {
        }

        inline int32_t clamp(int32_t value) const
        {
            return std::clamp<int32_t>(value, 0, getARR());
        }

        inline uint16_t getMax() const
        {
            return level;
        }

        inline uint16_t getARR() const
        {
            return PID_MOTOR_PWM_TIMER->ARR; // this is (level - 1)
        }

        inline void setMax(uint16_t value)
        {
            level = value;
            upper = (value * 140U) / 128U;
            lower = -upper;
        }

        inline int32_t getLower() const
        {
            return lower;
        }

        inline int32_t getUpper() const
        {
            return upper;
        }

    protected:
        int32_t lower;
        int32_t upper;
        uint16_t level;
    };

public:
    float Kp;                                       // PID K-values
    float Ki;
    float Kd;
    PWMLevel pwmLevel;                              // stores max. pwm level and upper/lower bounds
    volatile uint32_t rpm;                          // target RPM
    EEPROM::MotorDirection motorDirection;          // motor direction
    PidValueType antiWindup;                        // anti-windup factor

    volatile uint32_t lastEncoderCounter;           // last encoder counter value
    PidValueType integral;                          // PID variables
    PidValueType lastError;
    PidValueType lastDerivative;

    PidValueType KpPreCalc;                         // pre-calculated K-values for PID loop
    PidValueType KiPreCalc;
    PidValueType KdPreCalc;
    int32_t countsPerInterval;                      // counts per interval (RPM)

    uint32_t lastRpmCounter;                        // counts rotations to detect a motor stall
    uint32_t lastRpmCounterUpdated;                 // last time the RPM counter was updated

    StatsType stats;                                // statistics
    FaultStates faults;                             // DRV8701 and ocp faults
    OcpState ocp;                                   // OCP state machine

    volatile ErrorCodeType errorCode;               // last error

    // buffer for PID loop data for SWO PID tuning
    // adjust size to the interval
    static constexpr float kPidLoopBufferSeconds = 0.128f; // how much data to store in seconds
    RingBuffer<PidLoopType, std::clamp<size_t>(((1000 * kPidLoopBufferSeconds) / kPIDInterval), 32, 80)> pidLoopBuffer;
    static constexpr size_t kPidLoopBufferCount = sizeof(pidLoopBuffer) / sizeof(PidLoopType);
    static constexpr size_t kPidLoopBufferSize = sizeof(pidLoopBuffer);

    volatile bool running;                          // true if the PID controller is running
    volatile uint32_t releaseBreakCounter;          // counter for releasing the brake after motor off
};

extern PidController pid;
