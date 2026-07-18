/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"

// MT6701 encoder pins
#if defined(STM32F107xC)
#define MT6701_I2C_ENABLE_PIN       PD7             // MT6701 encoder I2C pin
#else
#define MT6701_I2C_PIN              PB7             // MT6701 encoder I2C pin
#endif

// Rotary encoder pins
#define ROTARY_ENCODER_PIN_A        PA6
#define ROTARY_ENCODER_PIN_B        PA7

// Button pins
#if defined(STM32F107xC)
#define KNOB_BUTTON_PIN             PD8             // knob button pin
#define BACK_BUTTON_PIN             PD9             // back button pin
#define START_BUTTON_PIN            PD10            // start button pin
#else
#define KNOB_BUTTON_PIN             PD8             // knob button pin
#define BACK_BUTTON_PIN             PD9             // back button pin
#define START_BUTTON_PIN            PD10            // start button pin
#endif

// Charlieplexed LEDs
#if defined(STM32F107xC)
#define MOTOR_LEDS_PIN              PD12            // motor LEDs pin
#else
#define MOTOR_LEDS_PIN              PB12            // motor LEDs pin
#endif

// TFT pins
#define TFT_PIN_CLK                 PB13
#define TFT_PIN_MOSI                PB15
#define TFT_PIN_RS                  PC7
#define TFT_PIN_RST                 PC6
#define TFT_PIN_CS                  PD15
#define TFT_PIN_LED                 PB11

// PIN macros
#if TFT_PIN_RS != PC7 || TFT_PIN_RST != PC6 || TFT_PIN_CS != PD15
#error PINS are hardcoded
#else
#define TFT_PIN_RS_HIGH()           GPIOC->BSRR = (1 << 7)
#define TFT_PIN_RS_LOW()            GPIOC->BSRR = (1 << (7 + 16))
#define TFT_PIN_CS_HIGH()           GPIOD->BSRR = (1 << 15)
#define TFT_PIN_CS_LOW()            GPIOD->BSRR = (1 << (15 + 16))
#define TFT_PIN_RST_HIGH()          GPIOC->BSRR = (1 << 6)
#define TFT_PIN_RST_LOW()           GPIOC->BSRR = (1 << (6 +  16))
#endif

