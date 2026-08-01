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
    static constexpr uint16_t kMaxPWMLevel = kPWMFrequencyToARR<20000>();               // Motor PWM 20Khz
    static constexpr uint16_t kPPR = 1024;                                              // MT6701 PPR
    static constexpr uint16_t kCPR = kPPR * 4;                                          // 4x Mode PPR to CPR
    static constexpr float kPIDIntervalFloat = 5.12f;                                   // PID update rate in millis used for precise RPM calculation
    static constexpr uint32_t kPIDInterval = 5;                                         // PID update rate in millis
    static constexpr float kAntiWindup = 0.97f;                                         // reduce integral if error is out of range (97%)
    static constexpr bool kProgramPPR = false;                                          // set to true to program the MT6701 encoder during boot over i2c
    static constexpr uint32_t kMaxRPM = 55000;                                          // max. supported RPM by the encoder
    static constexpr float kMaxError = kCPR / (60000 / kPIDIntervalFloat) * kMaxRPM;    // factor to reduce error to a value between -1.0 and 1.0 for the PID loop
    static constexpr uint32_t kReleaseBreakTime = 5000;                                 // time in ms to release the break after motor is turned off

    // convert RPM to counts per PID interval
    static constexpr uint32_t kRPMToIntCounts(uint32_t value)
    {
        return static_cast<uint32_t>((value * kCPR) / static_cast<uint32_t>(60000 / kPIDIntervalFloat));
    }

    template<int32_t VALUE>
    static constexpr uint32_t kRPMToIntCountsT()
    {
        return kRPMToIntCounts(VALUE);
    }

    // convert counts per PID interval to RPM
    static constexpr int32_t kIntCountsToRPM(int32_t value)
    {
        return static_cast<int32_t>((value * static_cast<uint32_t>(60000 / kPIDIntervalFloat)) / kCPR);
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
        antiWindup(kAntiWindup),
        errorCode(ErrorCodeType::NONE),
        running(false),
        releaseBreakCounter(0)
    {
        setKp(1.0f);
        setKi(1.0f);
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
     * @brief Set the Kp value and pre-calculate KpPreCalc for PID loop
     *
     * @param value The new Kp value
     */
    inline void setKp(float value)
    {
        Kp = value;
        KpPreCalc = value * kMaxPWMLevel;
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
        KiPreCalc = value * kMaxPWMLevel;
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
        KdPreCalc = value * kMaxPWMLevel;
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
    inline int32_t calcPWMLevel(float error, float integral, float derivative) const
    {
        return (error * KpPreCalc + integral * KiPreCalc + derivative * KdPreCalc);
    }

    /**
     * @brief Clamp the PWM level to the valid range
     *
     * @param value The PWM level to clamp
     * @return int32_t The clamped PWM level
     */
    inline int32_t clampPWMLevel(int32_t value) const
    {
        return std::clamp<int32_t>(value, 0, kMaxPWMLevel - 1);
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
        antiWindup = std::clamp<float>(value / (UIConstants::kAntiWindupFactor * 100.0f), 0.5f, 1.0f);
        SWO::data.antiWindup = antiWindup * 100.0f;
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

    inline void setIntegral(float value)
    {
        integral = std::clamp<float>(value, -1.0f, 1.0f);
    }

    inline float getIntegral() const
    {
        return integral;
    }

    inline void updateIntegral(float error)
    {
        setIntegral(integral + error);
    }

    inline void setLastError(float value)
    {
        lastError = value;
    }

    inline float getLastError() const
    {
        return lastError;
    }

    inline void setLastDerivative(float value)
    {
        lastDerivative = value;
    }

    inline int32_t getLastDerivative() const
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
        DEBUG_PRINT(DebugType::PID, "drv8701=%d ocp=%d snsout=%d", faults.drv8701Fault, faults.ocpFault, faults.snsoutFault);
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
        setAntiWindup(eeprom.getAntiWindup());
        setRPM(eeprom.getMotorRPM());
    }

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
        uint16_t rpm;
        uint16_t pwmLevel;
        uint16_t voltage;
        uint16_t currentOcp;
        uint16_t currentAverage;
        uint16_t motorTemperature;
        uint16_t mosfetTemperature;
        uint16_t dacMotorCurrent;
        uint16_t dacInputCurrent;
        float error;
        float integral;
        float derivative;
        uint32_t running: 1;
        uint32_t drv8701Fault : 1;
        uint32_t ocpFault : 1;
        uint32_t snsoutFault : 1;
        uint32_t ___reserved : 28;
    };
    static constexpr size_t kPidLoopTypeSize = sizeof(PidLoopType);

    // === OCP state machine and constants ===

    static constexpr uint32_t kOcpTickInterval = 5;                                     // 5us tick interval
    static constexpr uint32_t kOcpISenseThreshold = 90;                                 // lower threshold in percent before the OCP condition is cleared
    static constexpr uint32_t kOcpRecoveryInterval = 20 / kOcpTickInterval;             // 20us interval
    static constexpr uint32_t kOcpRetriggerTimeout = 10 / kOcpTickInterval;             // 10us timeout
    static constexpr uint32_t kOcpCurrentRampUp = 16;                                   // increase current by 1/16 every tick
    static constexpr uint32_t kOcpCurrentRampDown = 16;                                 // reduce current by 1/16 every tick
    static constexpr uint32_t kOcpInputToMotorCurrentRatio = 8;                         // if the motor current limit is higher than x the input current limit, it will be reduced to x the input current limit, before ramping it down further
    static constexpr float kOcpAntiWindUp = 0.8f;                                       // strong anti windup during OCP condition

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
    float Kp;                                       // PID K-values
    float Ki;
    float Kd;
    uint32_t rpm;                                   // target RPM
    uint32_t motorDirection;                        // motor direction
    float antiWindup;                               // anti-windup factor

    uint32_t lastEncoderCounter;                    // last encoder counter value
    float integral;                                 // PID variables
    float lastError;
    float lastDerivative;

    float KpPreCalc;                                // pre-calculated K-values for PID loop
    float KiPreCalc;
    float KdPreCalc;
    int32_t countsPerInterval;                      // counts per interval (RPM)

    uint32_t lastRpmCounter;
    uint32_t lastRpmCounterUpdated;

    StatsType stats;                                // statistics
    FaultStates faults;                             // DRV8701 and ocp faults
    OcpState ocp;                                   // OCP state machine

    ErrorCodeType errorCode;                        // last error
    RingBuffer<PidLoopType, 8> pidLoopBuffer;       // buffer for PID loop data for PID tuning

    volatile bool running;                          // true if the PID controller is running
    uint32_t releaseBreakCounter;                   // counter for releasing the brake after motor off
};

extern PidController pid;
