/**
  Author: sascha_lammers@gmx.de

  Pin assignments see stm32_pin_assignment.md for details
*/

#pragma once

#include "helpers.h"

/*

Timer configurations:

    TIM1 motor pwm output
    TIM2 tft backlight and led brightness pwm output
    TIM3 rotary encoder input
    TIM4 MT6701 encoder input
    TIM5 RPM counter input
    TIM6 PID loop - 5us
    TIM7 SysTick_Handler() - 1us

*/

// MT6701 encoder pins
#define MT6701_I2C_ENABLE_PIN       PD7             // MT6701 encoder I2C pin
#define MT6701_I2C_SCL_PIN          PB6             // MT6701 encoder I2C SCL pin / A
#define MT6701_I2C_SDA_PIN          PB7             // MT6701 encoder I2C SDA pin / B

// Rotary encoder pins
#define ROTARY_ENCODER_PIN_A        PA6             // A
#define ROTARY_ENCODER_PIN_B        PA7             // B

// Button pins
#define KNOB_BUTTON_PIN             PD8             // BTN_1 knob button pin
#define BACK_BUTTON_PIN             PD9             // BTN_2 back button pin
#define START_BUTTON_PIN            PD10            // BTN_3 start button pin

#define DRV8701_FAULT_PIN           PB14            // DRV8701 nFAULT pin
#define OCP_INT_PIN                 PB12            // INA381 ALERT#
#define DRV_SNSOUT_PIN              PD11            // DRV8701 SNSOUT pin

#define EXT_NTC_PIN                 PC4             // ADC12_IN14
#define DRV_TEMP_PIN                PC5             // ADC12_IN15
#define ISENSE_PIN                  PA2             // ADC12_IN2
#define VSENSE_PIN                  PA3             // ADC12_IN3
#define ENC1_ANALOG_PIN             PA1             // ADC12_IN1

#define DRVOCP_VREF_DAC_PIN         PA4             // DRV8701 VREF overcurrent protection
#define OCP_VREF_DAC_PIN            PA5             // INA381 CMPREF overcurrent protection

// Charlieplexed LEDs
#define MOTOR_LEDS_PIN              PD12            // motor LEDs pin

#define ILLUMINATION_LED_PIN        PB10            // illumination LED pin

// USB
#define USB_DN_PIN                  PA11            // USB D- pin
#define USB_DP_PIN                  PA12            // USB D+ pin

// SWD
#define SWD_SWCLK_PIN               PA14            // SWCLK pin
#define SWD_SWDIO_PIN               PA13            // SWDIO pin
#define SWD_SWO_PIN                 PB3             // SWO pin

// debug UART pins requires PCB Rev 1.1
#define PAD_TX4                     PC10             // UART4 TX pin
#define PAD_RX4                     PC11             // UART4 RX pin

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
#define TFT_PIN_RS_HIGH()           digitalWriteHigh<TFT_PIN_RS>()
#define TFT_PIN_RS_LOW()            digitalWriteLow<TFT_PIN_RS>()
#define TFT_PIN_CS_HIGH()           digitalWriteHigh<TFT_PIN_CS>()
#define TFT_PIN_CS_LOW()            digitalWriteLow<TFT_PIN_CS>()
#define TFT_PIN_RST_HIGH()          digitalWriteHigh<TFT_PIN_RST>()
#define TFT_PIN_RST_LOW()           digitalWriteLow<TFT_PIN_RST>()
#endif

// motor driver pins are connected to channels 1 and 2 of TIM1 (PA8 and PA9)
#define PID_WRITE_MOTOR_PWM_DRV_IN1(level)          (TIM1->CCR1 = (level))
#define PID_WRITE_MOTOR_PWM_DRV_IN2(level)          (TIM1->CCR2 = (level))
#define PID_READ_MOTOR_PWM_DRV_IN1()                ((uint16_t)TIM1->CCR1)
#define PID_READ_MOTOR_PWM_DRV_IN2()                ((uint16_t)TIM1->CCR2)
#define PID_WRITE_MOTOR_PWM_FORWARD(level)          (TIM1->CCR1 = (level), TIM1->CCR2 = 0)
#define PID_WRITE_MOTOR_PWM_REVERSE(level)          (TIM1->CCR1 = 0, TIM1->CCR2 = (level))
#define PID_WRITE_MOTOR_PWM_ON(level, reverse)      (reverse) ? PID_WRITE_MOTOR_PWM_REVERSE(level) : PID_WRITE_MOTOR_PWM_FORWARD(level)
#define PID_WRITE_MOTOR_PWM_OFF()                   (TIM1->CCR1 = 0, TIM1->CCR2 = 0)
#define PID_WRITE_MOTOR_PWM_BREAK(level)            (TIM1->CCR1 = (level), TIM1->CCR2 = (level))

// DAC macros for DRV8701 and INA381 overcurrent protection
#define DAC_SET_MOTOR_CURRENT(value)                (DAC->DHR12R1 = (value) & 0xfff)
#define DAC_SET_INPUT_CURRENT(value)                (DAC->DHR12R2 = (value) & 0xfff)
#define DAC_GET_MOTOR_CURRENT()                     ((uint16_t)DAC->DHR12R1) // DRVOCP_VREF_DAC_PIN/PA4
#define DAC_GET_INPUT_CURRENT()                     ((uint16_t)DAC->DHR12R2) // OCP_VREF_DAC_PIN/PA5

// macros to read encoder pulse counter and rpm counter
#define PID_READ_ENCODER_COUNTER()                  ((uint16_t)TIM4->CNT) // QDEC on MT6701_I2C_SCL_PIN/MT6701_I2C_SDA_PIN
#define PID_READ_RPM_COUNTER()                      ((uint16_t)TIM5->CNT) // falling edge counter on ENC1_ANALOG_PIN

// rotary encoder macros
#define UI_READ_ROTARY_KNOB_COUNTER()               ((uint16_t)TIM3->CNT) // rotary encoder on ROTARY_ENCODER_PIN_A/ROTARY_ENCODER_PIN_B
#define UI_WRITE_ROTARY_KNOB_COUNTER(value)         (TIM3->CNT = (value))

// PWM for the LED CC driver
#define UI_ILLUMINATION_LED_SET_PWM(value)          (TIM2->CCR3 = (value)) // ILLUMINATION_LED_PIN

// PWM for the TFT backlight
#define UI_TFT_BACKLIGHT_SET_PWM(value)             (TIM2->CCR4 = (value)) // TFT_PIN_LED
