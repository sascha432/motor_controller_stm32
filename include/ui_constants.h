/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>

#define VERSION_MAJOR           1
#define VERSION_MINOR           0
#define VERSION_PATCH           0

#define PCB_REV_MAJOR           1
#define PCB_REV_MINOR           1

// === UI constants ===
struct UIConstants 
{
    // ui slider min/max values
    static constexpr float kMinInputCurrent = 0.1f;                     // min. input current in A
    static constexpr float kMaxInputCurrent = 30.0f;                    // max. input current in A
    static constexpr float kInputCurrentStep = 0.1f;                    // input current step in A (make sure min/max is divisible by this value)
    static constexpr float kMinMotorCurrent = 0.5f;                     // min. peak motor current in A
    static constexpr float kMaxMotorCurrent = 40.0f;                    // max. peak motor current in A
    static constexpr float kMotorCurrentStep = 0.5f;                    // motor current step in A (make sure min/max is divisible by this value)
    static constexpr uint16_t kMinRPM = 10;                             // Min. Motor RPM
    static constexpr uint16_t kMaxRPM = 15000;                          // Max. Motor RPM
    static constexpr uint8_t kMinTFTBrightness = 5;                     // Min. TFT Brightness
    static constexpr uint8_t kMaxTFTBrightness = 100;                   // Max. TFT Brightness
    static constexpr uint8_t kMinLEDBrightness = 0;                     // Min. LED Brightness
    static constexpr uint8_t kMaxLEDBrightness = 100;                   // Max. LED Brightness
    static constexpr uint32_t kMinMotorStallTimeout = 100;              // Motor stall time in milliseconds
    static constexpr uint32_t kMaxMotorStallTimeout = 10000;            // Motor stall time in milliseconds
    static constexpr int32_t kMotorStallTimeoutStep = 100;              // Motor stall time step in milliseconds
    static constexpr uint8_t kMinMosfetTemperature = 50;                // Min. MOSFET temperature in °C
    static constexpr uint8_t kMaxMosfetTemperature = 175;               // Max. MOSFET temperature in °C
    static constexpr uint8_t kMinMotorTemperature = 30;                 // Min. Motor temperature in °C
    static constexpr uint8_t kMaxMotorTemperature = 105;                // Max. Motor temperature in °C

    // eeprom default values
    static constexpr uint8_t kDefaultTFTBrightness = 90;                // Default TFT Brightness
    static constexpr uint8_t kDefaultLEDBrightness = 25;                // Default LED Brightness
    static constexpr float kDefaultInputCurrent = 10.0f;                // Default input current in A
    static constexpr float kDefaultMotorCurrent = 40.0f;                // Default peak motor current in A
    static constexpr uint8_t kDefaultMotorBrake = 50;                   // Default motor brake in percentage (0-100)
    static constexpr uint16_t kDefaultMotorStallTimeout = 1500;         // Default motor stall time in milliseconds
    static constexpr uint16_t kDefaultMinRPM = 10;                      // Default min. motor RPM
    static constexpr uint16_t kDefaultMaxRPM = 15000;                   // Default max. motor RPM
    static constexpr uint8_t kDefaultMosfetTemperatureLimit = 85;       // Default MOSFET temperature limit in °C
    static constexpr uint8_t kDefaultMotorTemperatureLimit = 50;        // Default motor temperature limit in °C
    static constexpr uint8_t kDefaultMaxPWM = 100;                      // Max. PWM value in percentage (0-100)
    static constexpr uint8_t kDefaultMotorPWM = 20;                     // Default motor PWM value in percentage (0-100)
    static constexpr uint16_t kDefaultMotorRPM = 250;                   // Default motor RPM value in RPM

    // ui menu timeouts
    static constexpr uint32_t kWelcomeScreenTimeout = 3000;             // WelcomeScreen timeout in milliseconds
    static constexpr uint32_t kInfoScreenTimeout = 5000;                // Default InfoScreen timeout in milliseconds

    static constexpr bool kEnableIlluminationLEDFading = true;          // Enable fading of the illumination LED on the welcome screen
};
