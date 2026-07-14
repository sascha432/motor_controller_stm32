/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>

#define ENC1_A PB6                  // MT6701 A pin
#define ENC1_B PB7                  // MT6701 B pin
#define ENC2_A PA6                  // rotary encoder A pin
#define ENC2_B PA7                  // rotary encoder B pin     
#define MT6701_I2C_PIN PB0          // MT6701 I2C enable pin
#define TOGGLE_PIN PB10             // start button pin
#define BACK_PIN PA11               // back button pin
#define DEFAULT_RPM 250             // default RPM

#define DEBUG_HUMAN 0
#define HAVE_DEBUG_PID_CONTROLLER 1

#define PID_WRITE_MOTOR_PWM(level)      (TIM1->CCR1 = (level))

/**
 * @brief translate arduino digital pin number to GPIO pin number
 * 
 * @tparam PIN_TYPE 
 * @param pin Arduino digital pin number
 * @return constexpr uint8_t GPIO pin number 0-15
 */
template<typename PIN_TYPE>
constexpr uint8_t digitalPinToBit(PIN_TYPE pin) {
    return __builtin_ctz(STM_GPIO_PIN(digitalPinToPinName(pin)));
}

/**
 * @brief translate arduino digital pin number to GPIO configuration register shift value
 * 
 * @param pin Arduino digital pin number
 * @param bits Defaults to 4, number of bits per pin in the configuration register
 * @return constexpr uint32_t 0-7 * bits
 */
constexpr uint32_t digitalPinShift(uint32_t pin, uint32_t bits = 4)
{
    return (digitalPinToBit(pin) & 7u) * bits;
}

/**
 * @brief translate GPIO port address to RCC_APB2ENR_IOPxEN bits
 * 
 * @param GPIO_PORT_ADDR GPIOx_BASE address
 * @return bit mask for APB2ENR
 */
constexpr uint32_t RCC_APB2ENR_IOPxEN(uint32_t GPIO_PORT_ADDR) {
    return (1U << ((GPIO_PORT_ADDR >> 10U) & 0x0f));
}

/**
 * @brief get configuration register for GPIO pin
 * 
 * @param gpio_addr GPIOx_BASE address
 * @param pin Arduino PIN number
 * @return volatile& 
 */
inline volatile uint32_t &GPIO_CRx_REG(uint32_t gpio_addr, uint32_t pin)
{
    return (digitalPinToBit(pin) < 8) ? ((GPIO_TypeDef *)gpio_addr)->CRL : ((GPIO_TypeDef *)gpio_addr)->CRH;
}

/**
 * @brief calculate the auto-reload register value for a given PWM frequency with no prescaler (PSC=0)
 * 
 * @tparam FREQUENCY PWM frequency in Hz
 * @return constexpr uint16_t
 */
template<uint32_t FREQUENCY>
static constexpr uint16_t kPWMFrequencyToARR() {
    constexpr uint32_t tmp = 72000000 / FREQUENCY;
    static_assert(tmp <= 0xFFFF, "PWM frequency too low for 16bit timer");
    return tmp;
}
