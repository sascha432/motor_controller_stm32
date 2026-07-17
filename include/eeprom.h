/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include "ui.h"

struct EEPROM 
{
    static constexpr uint32_t kMagic = 0xDEADBEEF;
    static constexpr uint32_t kVersion = 1;

    // 127A max.
    static constexpr uint16_t kCurrentToUint16(float current) {
        return static_cast<uint16_t>(current * 512.0f);
    }

    static constexpr float kUint16ToCurrent(uint16_t value) {
        return static_cast<float>(value) / 512.0f;
    }

    static constexpr uint8_t kControlModePWM = 0;
    static constexpr uint8_t kControlModePID = 1;

    static constexpr uint8_t kMotorDirectionForward = 0;
    static constexpr uint8_t kMotorDirectionReverse = 1;

    struct Data {
        uint32_t magic;
        uint32_t version;
        uint8_t tft_brightness;
        uint8_t led_brightness;
        uint16_t input_current_limit;
        uint16_t motor_current_limit;
        uint16_t min_rpm;
        uint16_t max_rpm;
        uint8_t motor_direction;
        uint8_t control_mode;

        Data() : 
            magic(kMagic), 
            version(kVersion), 
            tft_brightness(80), 
            led_brightness(50), 
            input_current_limit(kCurrentToUint16(10.0f)), 
            motor_current_limit(kCurrentToUint16(40.0f)), 
            min_rpm(1000),
            max_rpm(5000),
            motor_direction(kMotorDirectionForward), 
            control_mode(kControlModePID)
        {}
    };

    static void init();
    static void read(Data &data);
    static void write(const Data &data);

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

    uint8_t getMotorDirection() const
    {
        return data.motor_direction;
    }

    void setMotorDirection(uint8_t value) 
    {
        data.motor_direction = value;
    }

    uint8_t getControlMode() const
    {
        return data.control_mode;
    }

    void setControlMode(uint8_t value) 
    {
        data.control_mode = value;
    }

protected:
    Data data;
};
