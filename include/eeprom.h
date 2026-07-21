/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include "ui_constants.h"
#include "adc.h"
#include "i2c.h"

struct EEPROM 
{
    static constexpr uint32_t kMagic = 0xDEADBEEF;
    static constexpr uint32_t kVersion = 1;

    // ~130A max.
    static constexpr uint16_t kCurrentToUint16(float current) {
        return static_cast<uint16_t>(current * 500.0f);
    }

    static constexpr float kUint16ToCurrent(uint16_t value) {
        return static_cast<float>(value) / 500.0f;
    }

    static constexpr uint8_t kControlModePWM = 0;
    static constexpr uint8_t kControlModePID = 1;

    static constexpr uint8_t kMotorDirectionForward = 0;
    static constexpr uint8_t kMotorDirectionReverse = 1;

    struct Data {
        uint32_t magic;
        uint32_t version;
        uint32_t sequence;
        uint8_t tft_brightness;
        uint8_t led_brightness;
        uint16_t input_current_limit;
        uint16_t motor_current_limit;
        uint16_t min_rpm;
        uint16_t max_rpm;
        uint16_t motor_stall_timeout;
        uint8_t motor_direction;
        uint8_t motor_brake;
        uint8_t control_mode;
        uint8_t mosfet_temperature_limit;
        uint8_t motor_temperature_limit;
        uint8_t max_pwm;
        uint8_t motor_pwm;
        uint16_t motor_rpm;

        Data() : 
            magic(kMagic), 
            version(kVersion), 
            sequence(1),
            tft_brightness(UIConstants::kDefaultTFTBrightness), 
            led_brightness(UIConstants::kDefaultLEDBrightness), 
            input_current_limit(kCurrentToUint16(UIConstants::kDefaultInputCurrent)), 
            motor_current_limit(kCurrentToUint16(UIConstants::kDefaultMotorCurrent)), 
            min_rpm(UIConstants::kDefaultMinRPM),
            max_rpm(UIConstants::kDefaultMaxRPM),
            motor_stall_timeout(UIConstants::kDefaultMotorStallTimeout),
            motor_direction(kMotorDirectionForward),
            motor_brake(UIConstants::kDefaultMotorBrake),
            control_mode(kControlModePID),
            mosfet_temperature_limit(UIConstants::kDefaultMosfetTemperatureLimit),
            motor_temperature_limit(UIConstants::kDefaultMotorTemperatureLimit),
            max_pwm(UIConstants::kDefaultMaxPWM),
            motor_pwm(UIConstants::kDefaultMotorPWM),
            motor_rpm(UIConstants::kDefaultMotorRPM)
        {}

        bool operator==(const Data &other) const 
        {
            return memcmp(
                &reinterpret_cast<const uint8_t *>(this)[offsetof(Data, tft_brightness)], 
                &reinterpret_cast<const uint8_t *>(&other)[offsetof(Data, tft_brightness)],
                sizeof(Data) - offsetof(Data, tft_brightness)
            ) == 0;
        }

        void invalidate() 
        {
            magic = 0xcccccccc;
            version = 0xcccccccc;
            sequence = 0xcccccccc;
        }
    };

    void init();
    void read();
    bool write();

    Data &getData()
    {
        return data;
    }

    void resetDefaults()
    {
        data = Data();
        updateTemperatureLimits();
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

    uint8_t getMotorDirection() const
    {
        return data.motor_direction;
    }

    void setMotorDirection(uint8_t value) 
    {
        data.motor_direction = value;
    }

    bool isReverseDirection() const
    {
        return data.motor_direction == kMotorDirectionReverse;
    }

    uint8_t getMotorBrake() const
    {
        return data.motor_brake;
    }

    void setMotorBrake(uint8_t value) 
    {
        data.motor_brake = value;
    }

    uint8_t getControlMode() const
    {
        return data.control_mode;
    }

    bool isPIDMode() const
    {
        return data.control_mode == kControlModePID;
    }

    bool isPWMMode() const
    {
        return data.control_mode == kControlModePWM;
    }

    void setControlMode(uint8_t value) 
    {
        data.control_mode = value;
    }

    uint8_t getMosfetTemperatureLimit() const
    {
        return data.mosfet_temperature_limit;
    }

    void setMosfetTemperatureLimit(uint8_t value) 
    {
        data.mosfet_temperature_limit = value;
        mosfet_temperature_limit_adc = ADCConverter::NTC::reverse(value);
    }

    uint16_t getMosfetTemperatureLimitADC() const
    {
        return mosfet_temperature_limit_adc;
    }

    uint8_t getMotorTemperatureLimit() const
    {
        return data.motor_temperature_limit;
    }

    void setMotorTemperatureLimit(uint8_t value) 
    {
        data.motor_temperature_limit = value;
        motor_temperature_limit_adc = ADCConverter::NTC::reverse(value);
    }

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
            case kControlModePWM:
                setMotorPWM(static_cast<uint8_t>(value));
                break;
            case kControlModePID:
                setMotorRPM(static_cast<uint16_t>(value));
                break;
        }
    }

    uint32_t getSpeed() const
    {
        return data.control_mode ? getMotorRPM() : getMotorPWM();
    }

protected:
    void updateTemperatureLimits();

protected:
    Data data;
    uint16_t mosfet_temperature_limit_adc;
    uint16_t motor_temperature_limit_adc;
};

extern I2CHelper i2c;
extern EEPROM eeprom;
