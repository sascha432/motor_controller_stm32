/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>

#define VERSION_MAJOR           1
#define VERSION_MINOR           0
#define VERSION_PATCH           0

#define PCB_REV_MAJOR           1
#define PCB_REV_MINOR           0

// === UI constants ===
struct UIConstants 
{
    // ui slider min/max values
    static constexpr float kStepInputCurrent = 0.5f;                    // input current step in A
    static constexpr float kMinInputCurrent = kStepInputCurrent * 2;    // min. input current in A
    static constexpr float kMaxInputCurrent = 40.0f;                    // max. input current in A
    static constexpr float kStepMotorCurrent = 0.5f;                    // motor current step in A
    static constexpr float kMinMotorCurrent = kStepMotorCurrent * 2;    // min. peak motor current in A
    static constexpr float kMaxMotorCurrent = 40.0f;                    // max. peak motor current in A
    static constexpr uint16_t kStepMotorRPM = 10;                       // Motor RPM step
    static constexpr uint16_t kMinRPM = kStepMotorRPM * 2;              // Min. Motor RPM
    static constexpr uint16_t kMaxRPM = 55000;                          // Max. Motor RPM
    static constexpr uint8_t kMinTFTBrightness = 5;                     // Min. TFT Brightness
    static constexpr uint8_t kMaxTFTBrightness = 100;                   // Max. TFT Brightness
    static constexpr uint8_t kMinLEDBrightness = 0;                     // Min. LED Brightness
    static constexpr uint8_t kMaxLEDBrightness = 100;                   // Max. LED Brightness
    static constexpr uint32_t kMinMotorStallTimeout = 100;              // Motor stall time in milliseconds
    static constexpr uint32_t kMaxMotorStallTimeout = 10000;            // Motor stall time in milliseconds
    static constexpr int32_t kStepMotorStallTimeout = 100;              // Motor stall time step in milliseconds
    static constexpr uint8_t kMinMosfetTemperature = 25;                // Min. MOSFET temperature in °C
    static constexpr uint8_t kMaxMosfetTemperature = 125;               // Max. MOSFET temperature in °C
    static constexpr uint8_t kMinMotorTemperature = 25;                 // Min. Motor temperature in °C
    static constexpr uint8_t kMaxMotorTemperature = 85;                 // Max. Motor temperature in °C

    // eeprom default values
    static constexpr uint8_t kDefaultTFTBrightness = 90;                // Default TFT Brightness
    static constexpr uint8_t kDefaultLEDBrightness = 25;                // Default LED Brightness
    static constexpr float kDefaultInputCurrent = 5.0f;                 // Default input current in A
    static constexpr float kDefaultMotorCurrent = 20.0f;                // Default peak motor current in A
    static constexpr uint8_t kDefaultMotorBrake = 100;                  // Default motor brake in percentage (0-100)
    static constexpr uint16_t kDefaultMotorStallTimeout = 1500;         // Default motor stall time in milliseconds
    static constexpr uint16_t kDefaultMinRPM = 10;                      // Default min. motor RPM
    static constexpr uint16_t kDefaultMaxRPM = 15000;                   // Default max. motor RPM
    static constexpr uint8_t kDefaultMosfetTemperatureLimit = 95;       // Default MOSFET temperature limit in °C
    static constexpr uint8_t kDefaultMotorTemperatureLimit = 55;        // Default motor temperature limit in °C
    static constexpr uint8_t kDefaultMaxPWM = 100;                      // Max. PWM value in percentage (0-100)
    static constexpr uint8_t kDefaultMotorPWM = 20;                     // Default motor PWM value in percentage (0-100)
    static constexpr uint16_t kDefaultMotorRPM = 250;                   // Default motor RPM value in RPM

    // ui menu timeouts
    static constexpr uint32_t kWelcomeScreenTimeout = 3000;             // WelcomeScreen timeout in milliseconds
    static constexpr uint32_t kInfoScreenTimeout = 2500;                // Default InfoScreen timeout in milliseconds

    static constexpr bool kEnableIlluminationLEDFading = true;          // Enable fading of the illumination LED on the welcome screen
};
