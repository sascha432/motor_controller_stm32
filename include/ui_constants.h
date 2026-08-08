/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>

#define STR_HELPER(x)           #x
#define STR(x)                  STR_HELPER(x)

#define VERSION_MAJOR           1
#define VERSION_MINOR           0
#define VERSION_PATCH           0

#define PCB_REV_MAJOR           1
#define PCB_REV_MINOR           0

// === UI constants ===
struct UIConstants
{
    // ui slider min/max values
    static constexpr uint32_t kStepInputCurrent = 100;                      // input current step in mA
    static constexpr uint32_t kMinInputCurrent = kStepInputCurrent * 5;     // min. input current in mA
    static constexpr uint32_t kMaxInputCurrent = 40000;                     // max. input current in mA
    static constexpr uint32_t kStepMotorCurrent = 100;                      // motor current step in mA
    static constexpr uint32_t kMinMotorCurrent = kStepMotorCurrent * 5;     // min. peak motor current in mA
    static constexpr uint32_t kMaxMotorCurrent = 40000;                     // max. peak motor current in mA
    static constexpr uint16_t kMinRPM = 10;                                 // Min. Motor RPM
    static constexpr uint16_t kMaxRPM = 55000;                              // Max. Motor RPM
    static constexpr uint8_t kMinTFTBrightness = 5;                         // Min. TFT Brightness
    static constexpr uint8_t kMaxTFTBrightness = 100;                       // Max. TFT Brightness
    static constexpr uint8_t kMinLEDBrightness = 0;                         // Min. LED Brightness
    static constexpr uint8_t kMaxLEDBrightness = 100;                       // Max. LED Brightness
    static constexpr uint32_t kMinMotorStallTimeout = 250;                  // Motor stall time in milliseconds
    static constexpr uint32_t kMaxMotorStallTimeout = 10000;                // Motor stall time in milliseconds
    static constexpr int32_t kStepMotorStallTimeout = 100;                  // Motor stall time step in milliseconds
    static constexpr uint8_t kMinMosfetTemperature = 25;                    // Min. MOSFET temperature in °C
    static constexpr uint8_t kMaxMosfetTemperature = 125;                   // Max. MOSFET temperature in °C
    static constexpr uint8_t kMinMotorTemperature = 25;                     // Min. Motor temperature in °C
    static constexpr uint8_t kMaxMotorTemperature = 85;                     // Max. Motor temperature in °C
    static constexpr float kPIDParamFactor = 10000.0f;                      // PID parameter factor
    static constexpr uint32_t kMinKp = 0;                                   // Min. PID Kp value
    static constexpr uint32_t kMaxKp = 1000 * kPIDParamFactor;              // Max. PID Kp value
    static constexpr uint32_t kMinKi = kMinKp;                              // Min. PID Ki value
    static constexpr uint32_t kMaxKi = kMaxKp;                              // Max. PID Ki value
    static constexpr uint32_t kMinKd = kMinKp;                              // Min. PID Kd value
    static constexpr uint32_t kMaxKd = kMaxKp;                              // Max. PID Kd value
    static constexpr uint32_t kMaxPIDParamAcceleration = 100000;            // Max. PID parameter acceleration in uint32_t units per second
    static constexpr uint32_t kSelectKpSteps = 10;                          // Steps for Kp selection
    static constexpr uint32_t kSelectKiSteps = 10;                          // Steps for Ki selection
    static constexpr uint32_t kSelectKdSteps = 1;                           // Steps for Kd selection
    static constexpr uint32_t kAntiWindupFactor = 512;                      // anti-windup factor
    static constexpr uint32_t kMinAntiWindup = 0;                           // Min. PID anti-windup factor in percentage (0-100) * 100
    static constexpr uint32_t kMaxAntiWindup = 100 * kAntiWindupFactor;     // Max. PID anti-windup factor in percentage (0-100) * kAntiWindupFactor
    static constexpr int32_t kLowestAntiWindup = 50 * kAntiWindupFactor;    // Below 50% disable anti-windup
    static constexpr uint32_t kStepsAntiWindup = kAntiWindupFactor / 50;    // Steps for anti-windup selection
    static constexpr uint32_t kMinOvpProtection = 8000;                     // Min. OVP protection in millivolts
    static constexpr uint32_t kMaxOvpProtection = 40000;                    // Max. OVP protection in millivolts
    static constexpr uint32_t kStepOvpProtection = 100;                     // OVP protection step in millivolts
    static constexpr uint32_t kStepsRPM = 50;                               // RPM steps
    static constexpr uint32_t kStepsPWM = 1;                                // PWM steps
    static constexpr uint16_t kMinPWMFrequency = 5000;                      // Min. PWM frequency in Hz
    static constexpr uint16_t kMaxPWMFrequency = 40000;                     // Max. PWM frequency in Hz
    static constexpr uint16_t kStepPWMFrequency = 500;                      // PWM frequency step in Hz

    // eeprom default values
    static constexpr uint8_t kDefaultTFTBrightness = 90;                    // Default TFT Brightness
    static constexpr uint8_t kDefaultLEDBrightness = 25;                    // Default LED Brightness
    static constexpr uint32_t kDefaultInputCurrent = 5000;                  // Default input current in mA
    static constexpr uint32_t kDefaultMotorCurrent = 15000;                 // Default peak motor current in mA
    static constexpr uint8_t kDefaultMotorBrake = 100;                      // Default motor brake in percentage (0-100)
    static constexpr uint16_t kDefaultMotorStallTimeout = 1500;             // Default motor stall time in milliseconds
    static constexpr uint16_t kDefaultMinRPM = 100;                         // Default min. motor RPM
    static constexpr uint16_t kDefaultMaxRPM = 15000;                       // Default max. motor RPM
    static constexpr uint8_t kDefaultMosfetTemperatureLimit = 95;           // Default MOSFET temperature limit in °C
    static constexpr uint8_t kDefaultMotorTemperatureLimit = 55;            // Default motor temperature limit in °C
    static constexpr uint8_t kDefaultMaxPWM = 100;                          // Max. PWM value in percentage (0-100)
    static constexpr uint8_t kDefaultMotorPWM = 20;                         // Default motor PWM value in percentage (0-100)
    static constexpr uint16_t kDefaultMotorRPM = 250;                       // Default motor RPM value in RPM
    static constexpr float kDefaultKp = 1.0f;                               // Default PID Kp value
    static constexpr float kDefaultKi = 1.0f;                               // Default PID Ki value
    static constexpr float kDefaultKd = 0.0f;                               // Default PID Kd value
    static constexpr uint16_t kDefaultAntiWindup = 90 * kAntiWindupFactor;  // Default PID anti-windup in percentage (0-100) * kAntiWindupFactor
    static constexpr uint16_t kDefaultOvpProtection = 36500;                // Default OVP protection in millivolts
    static constexpr uint16_t kDefaultPWMFrequency = 16500;                 // Default PWM frequency in Hz

    // ui menu timeouts
    static constexpr uint32_t kWelcomeScreenTimeout = 2000;                 // WelcomeScreen timeout in milliseconds
    static constexpr uint32_t kInfoScreenTimeout = 1500;                    // Default InfoScreen timeout in milliseconds
    static constexpr uint32_t kLongPressDuration = 750;                     // Long press duration in milliseconds

    static constexpr bool kEnableIlluminationLEDFading = true;              // Enable fading of the illumination LED while showing the welcome screen
};
