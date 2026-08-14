/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include "ui_constants.h"
#include "adc.h"
#include "i2c.h"
#include "adc_converters.h"

extern CRC_HandleTypeDef hcrc;

struct EEPROM
{
    static constexpr uint8_t kAddress = 0x50;                           // 7-bit device address
    static constexpr size_t kPageSize = 8;                              // bytes per page
    static constexpr size_t kSize = 256;                                // total bytes
    static constexpr uint32_t kWriteCycleWaitTimeoutMs = 7;             // wait for write cycle to complete (~5ms)
    static constexpr size_t kDefaultOffset = 0;                         // default offset for EEPROM data
    static constexpr size_t kBackupOffset = kSize / 2;                  // offset for backup EEPROM data (0 = DISABLE)
    static constexpr bool kValidateWrite = false;                       // validate write by reading back data and comparing with original

    static constexpr uint32_t kMagic = 0xDEADBEEF;
    static constexpr uint32_t kInvalidCRC = 0xffffffff;
    static constexpr uint32_t kInvalidData = 0xcccccccc;
    static constexpr uint32_t kVersion = 5;

    static constexpr uint32_t kPIDParamToUint32(float value) {
        return static_cast<uint32_t>(value * UIConstants::kPIDParamFactor);
    }

    static constexpr float kUint32ToPIDParam(uint32_t value) {
        return value / UIConstants::kPIDParamFactor;
    }

    enum class ControlMode : uint8_t {
        PWM = 0,
        PID = 1
    };

    enum class MotorDirection : uint8_t {
        Forward = 0,
        Reverse = 1
    };

    enum class SensorDirection : uint8_t {
        Forward = static_cast<uint8_t>(MotorDirection::Forward),
        Reverse = static_cast<uint8_t>(MotorDirection::Reverse)
    };

    /**
     * @brief EEPROM data structure
     *
     */
    struct Data
    {
        uint32_t magic;
        uint32_t version;
        uint32_t sequence;
        uint32_t crc;
        uint8_t tft_brightness;
        uint8_t led_brightness;
        uint16_t input_current_limit;
        uint16_t motor_current_limit;
        uint16_t min_rpm;
        uint16_t max_rpm;
        uint16_t motor_stall_timeout;
        MotorDirection motor_direction;
        SensorDirection sensor_direction;
        uint8_t motor_brake;
        ControlMode control_mode;
        uint8_t mosfet_temperature_limit;
        uint8_t motor_temperature_limit;
        uint8_t max_pwm;
        uint8_t motor_pwm;
        uint16_t motor_rpm;
        float Kp;
        float Ki;
        float Kd;
        uint16_t anti_windup;
        uint16_t ovp_protection;
        uint16_t pwm_frequency;
        bool motor_chime;

        /**
         * @brief Construct a new Data object with default settings
         *
         */
        Data() :
            magic(kMagic),
            version(kVersion),
            sequence(1),
            crc(kInvalidCRC),
            tft_brightness(UIConstants::kDefaultTFTBrightness),
            led_brightness(UIConstants::kDefaultLEDBrightness),
            input_current_limit(UIConstants::kDefaultInputCurrent),
            motor_current_limit(UIConstants::kDefaultMotorCurrent),
            min_rpm(UIConstants::kDefaultMinRPM),
            max_rpm(UIConstants::kDefaultMaxRPM),
            motor_stall_timeout(UIConstants::kDefaultMotorStallTimeout),
            motor_direction(MotorDirection::Forward),
            sensor_direction(SensorDirection::Forward),
            motor_brake(UIConstants::kDefaultMotorBrake),
            control_mode(ControlMode::PID),
            mosfet_temperature_limit(UIConstants::kDefaultMosfetTemperatureLimit),
            motor_temperature_limit(UIConstants::kDefaultMotorTemperatureLimit),
            max_pwm(UIConstants::kDefaultMaxPWM),
            motor_pwm(UIConstants::kDefaultMotorPWM),
            motor_rpm(UIConstants::kDefaultMotorRPM),
            Kp(UIConstants::kDefaultKp),
            Ki(UIConstants::kDefaultKi),
            Kd(UIConstants::kDefaultKd),
            anti_windup(UIConstants::kDefaultAntiWindup),
            ovp_protection(UIConstants::kDefaultOvpProtection),
            pwm_frequency(UIConstants::kDefaultPWMFrequency),
            motor_chime(UIConstants::kDefaultMotorChime)
        {}

        /**
         * @brief Get the data pointer without header
         *
         * @return uint32_t*
         */
        inline uint32_t *getDataPtr() const
        {
            return reinterpret_cast<uint32_t *>(const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(this)) + offsetof(Data, tft_brightness));
        }

        /**
         * @brief Get the data size without header
         *
         * @return constexpr size_t
         */
        constexpr size_t getDataSize() const
        {
            return sizeof(Data) - offsetof(Data, tft_brightness);
        }

        /**
         * @brief Compare if EEPROM data is equal without comparing the magic, version and sequence number
         *
         * @param other EEPROM data to compare with
         * @return true
         * @return false
         */
        bool operator==(const Data &other) const
        {
            if (crc == kInvalidCRC || other.crc == kInvalidCRC) {
                // invalid data
                return false;
            }
            // compare data directly, CRC is updated during read and write only
            return memcmp(this->getDataPtr(), other.getDataPtr(), other.getDataSize()) == 0;
        }

        /**
         * @brief Calculate CRC for EEPROM data without header
         *
         * @return uint32_t CRC value
         */
        inline uint32_t calculateCRC() const
        {
            return HAL_CRC_Calculate(&hcrc, getDataPtr(), getDataSize() / sizeof(uint32_t));
        }

        /**
         * @brief Validate CRC and mark data as invalid if it doesn't match
         *
         * @return uint32_t CRC value, or kInvalidCRC if invalid
         */
        uint32_t validateCRC()
        {
            uint32_t newCrc = calculateCRC();
            if (crc != newCrc) {
                DEBUG_PRINT(DebugType::ERROR, "EEPROM CRC mismatch: expected=%08x calculated=%08x", crc, newCrc);
                // mark as invalid
                crc = kInvalidCRC;
            }
            return crc;
        }

        /**
         * @brief Invalidate data
         *
         */
        inline void invalidate()
        {
            magic = kInvalidData;
            version = kInvalidData;
            sequence = 0;
            crc = kInvalidCRC;
        }
    };

    /**
     * @brief Initialize GPIO and I2C for EEPROM access
     *
     */
    void init();

    /**
     * @brief Read EEPROM configuration or restore defaults settings on failure
     *
     */
    void read();

    /**
     * @brief Write configuration data to EEPROM, only if it has changed since the last read/write operation.
     *
     * @return true data written
     * @return false data the same or failure
     */
    bool write();

    /**
     * @brief Restore default settings
     *
     */
    void resetDefaults();

    /**
     * @brief Get the EEPROM data object
     *
     * @return Data&
     */
    Data &getData()
    {
        return data;
    }

    uint8_t getTFTBrightness() const
    {
        return data.tft_brightness;
    }

    void setTFTBrightness(uint8_t value)
    {
        data.tft_brightness = value;
    }

    uint8_t getLEDBrightness() const
    {
        return data.led_brightness;
    }

    void setLEDBrightness(uint8_t value)
    {
        data.led_brightness = value;
    }

    uint16_t getInputCurrentLimit() const
    {
        return data.input_current_limit;
    }

    void setInputCurrentLimit(uint16_t value)
    {
        data.input_current_limit = value;
    }

    uint16_t getMotorCurrentLimit() const
    {
        return data.motor_current_limit;
    }

    void setMotorCurrentLimit(uint16_t value)
    {
        data.motor_current_limit = value;
    }

    uint16_t getMinRPM() const
    {
        return data.min_rpm;
    }

    void setMinRPM(uint16_t value)
    {
        data.min_rpm = value;
    }

    uint16_t getMaxRPM() const
    {
        return data.max_rpm;
    }

    void setMaxRPM(uint16_t value)
    {
        data.max_rpm = value;
    }

    uint16_t getMotorStallTimeout() const
    {
        return data.motor_stall_timeout;
    }

    void setMotorStallTimeout(uint16_t value)
    {
        data.motor_stall_timeout = value;
    }

    MotorDirection getMotorDirection() const
    {
        return data.motor_direction;
    }

    void setSensorDirection(SensorDirection value)
    {
        data.sensor_direction = value;
    }

    SensorDirection getSensorDirection() const
    {
        return data.sensor_direction;
    }

    void setMotorDirection(MotorDirection value)
    {
        data.motor_direction = value;
    }

    bool isForwardMotorDirection() const
    {
        return data.motor_direction == MotorDirection::Forward;
    }

    bool isReverseMotorDirection() const
    {
        return data.motor_direction == MotorDirection::Reverse;
    }

    bool isReverseSensorDirection() const
    {
        return data.sensor_direction == SensorDirection::Reverse;
    }

    bool compareWithSensorDirection(MotorDirection direction) const
    {
        return static_cast<uint8_t>(data.sensor_direction) == static_cast<uint8_t>(direction);
    }

    uint8_t getMotorBrake() const
    {
        return data.motor_brake;
    }

    void setMotorBrake(uint8_t value)
    {
        data.motor_brake = value;
    }

    ControlMode getControlMode() const
    {
        return data.control_mode;
    }

    bool isPIDMode() const
    {
        return data.control_mode == ControlMode::PID;
    }

    bool isPWMMode() const
    {
        return data.control_mode == ControlMode::PWM;
    }

    void setControlMode(ControlMode value)
    {
        data.control_mode = value;
    }

    uint8_t getMosfetTemperatureLimit() const
    {
        return data.mosfet_temperature_limit;
    }

    void setMosfetTemperatureLimit(uint8_t value);

    uint16_t getMosfetTemperatureLimitADC() const
    {
        return mosfet_temperature_limit_adc;
    }

    uint8_t getMotorTemperatureLimit() const
    {
        return data.motor_temperature_limit;
    }

    void setMotorTemperatureLimit(uint8_t value);

    uint16_t getMotorTemperatureLimitADC() const
    {
        return motor_temperature_limit_adc;
    }

    uint8_t getMaxPWM() const
    {
        return data.max_pwm;
    }

    void setMaxPWM(uint8_t value)
    {
        data.max_pwm = value;
    }

    uint8_t getMotorPWM() const
    {
        return data.motor_pwm;
    }

    void setMotorPWM(uint8_t value)
    {
        data.motor_pwm = value;
    }

    uint16_t getMotorRPM() const
    {
        return data.motor_rpm;
    }

    void setMotorRPM(uint16_t value)
    {
        data.motor_rpm = value;
    }

    void setSpeed(uint32_t value)
    {
        switch(data.control_mode) {
            case ControlMode::PWM:
                setMotorPWM(static_cast<uint8_t>(value));
                break;
            case ControlMode::PID:
                setMotorRPM(static_cast<uint16_t>(value));
                break;
        }
    }

    uint32_t getSpeed() const
    {
        return data.control_mode == ControlMode::PID ? getMotorRPM() : getMotorPWM();
    }

    void setKp(float value)
    {
        data.Kp = value;
    }

    float getKp() const
    {
        return data.Kp;
    }

    void setKi(float value)
    {
        data.Ki = value;
    }
    float getKi() const
    {
        return data.Ki;
    }
    void setKd(float value)
    {
        data.Kd = value;
    }

    float getKd() const
    {
        return data.Kd;
    }

    void setAntiWindup(uint16_t value)
    {
        data.anti_windup = value;
    }

    uint16_t getAntiWindup() const
    {
        return data.anti_windup;
    }

    void setOvpProtection(uint16_t value)
    {
        data.ovp_protection = value;
    }

    uint16_t getOvpProtection() const
    {
        return data.ovp_protection;
    }

    void setPWMFrequency(uint16_t value)
    {
        data.pwm_frequency = value;
    }

    uint16_t getPWMFrequency() const
    {
        return data.pwm_frequency;
    }

    bool getMotorChime() const
    {
        return data.motor_chime;
    }

    void setMotorChime(bool value)
    {
        data.motor_chime = value;
    }

protected:
    void updateTemperatureLimits();

protected:
    Data data;

    // sanity checks
    static constexpr uint16_t kDataSize = sizeof(Data);
    static_assert(sizeof(Data) % sizeof(uint32_t) == 0, "EEPROM data size must be a multiple of 4 bytes");
    static_assert(sizeof(Data) + kDefaultOffset <= kSize, "EEPROM data does not fit");
    static_assert(!kBackupOffset || sizeof(Data) + kBackupOffset <= kSize, "EEPROM backup data does not fit");
    static_assert(!kBackupOffset || sizeof(Data) + kDefaultOffset <= kBackupOffset, "EEPROM backup overlaps with data");

    // precalculated ADC values
    uint16_t mosfet_temperature_limit_adc;
    uint16_t motor_temperature_limit_adc;
};

extern I2CHelper i2c;
extern EEPROM eeprom;
