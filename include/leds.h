/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <algorithm>
#include "helpers.h"

/**
 * @brief Charlieplexed LEDs
 *
 */
struct LEDs {

    static constexpr uint32_t kIlluminationResolution = 1024;

    static void init()
    {
        // PWM for illuminationLedSetPWM() is initalized in tft_driver_gpio_tim_init() / shared with TFT backlight LED

        // Enable GPIO port clocks
        __HAL_RCC_GPIOB_CLK_ENABLE();

        // LED 1 & 2
        off();
    }

    static inline void setGPIOConf(uint8_t conf)
    {
        constexpr uint8_t shiftedPin = (__builtin_ctz(MOTOR_LEDS_Pin) & 0x07) * 4;
        if constexpr (MOTOR_LEDS_Pin > 7) {
            MOTOR_LEDS_GPIO_Port->CRH &= ~(0xf << shiftedPin);
            MOTOR_LEDS_GPIO_Port->CRH |= (conf << shiftedPin);
        }
        else {
            MOTOR_LEDS_GPIO_Port->CRL &= ~(0xf << shiftedPin);
            MOTOR_LEDS_GPIO_Port->CRL |= (conf << shiftedPin);
        }
    }

    static inline uint8_t getGPIOConf()
    {
        constexpr uint8_t shiftedPin = (__builtin_ctz(MOTOR_LEDS_Pin) & 0x07) * 4;
        if constexpr (MOTOR_LEDS_Pin > 7) {
            return (MOTOR_LEDS_GPIO_Port->CRH >> shiftedPin) & 0x0f;
        }
        else {
            return (MOTOR_LEDS_GPIO_Port->CRL >> shiftedPin) & 0x0f;
        }
    }

    static void off()
    {
        // MODE=00, CNF=01 (floating input) to turn both LEDs off
        setGPIOConf(0x04);
    }

    static bool isAnyLEDOn()
    {
        return (getGPIOConf() == 0x02);
    }

    static bool isErrorLEDOn()
    {
        return MOTOR_LEDS_GPIO_Port->ODR & MOTOR_LEDS_Pin;
    }

    static bool isWarningLEDOn()
    {
        return (MOTOR_LEDS_GPIO_Port->ODR & MOTOR_LEDS_Pin) == 0;
    }

    static void onLEDError()
    {
        MOTOR_LEDS_GPIO_Port->BSRR = MOTOR_LEDS_Pin;
        // MODE=10 (2MHz), CNF=00 (push-pull)
        setGPIOConf(0x02);
    }

    static void onLEDWarning()
    {
        MOTOR_LEDS_GPIO_Port->BRR = MOTOR_LEDS_Pin;
        // MODE=10 (2MHz), CNF=00 (push-pull)
        setGPIOConf(0x02);
    }

    static void illuminationLedSetPWM(uint32_t value)
    {
        if (value == 0) {
            UI_ILLUMINATION_LED_SET_PWM(0);
            return;
        }
        // get more linear brightness by using a gamma curve and zero offset
        constexpr uint32_t kOffset = 15;
        const uint32_t clampedBrightness = std::clamp<uint32_t>(
            value + (kOffset * kIlluminationResolution),
            (kOffset * kIlluminationResolution),
            ((100 + kOffset) * kIlluminationResolution)
        );
        const float brightness = clampedBrightness * (1.0f / ((kOffset + 100.0f) * kIlluminationResolution));
        UI_ILLUMINATION_LED_SET_PWM(powf(brightness, 2.2f) * 1000.0f);
    }
};
