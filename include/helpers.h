/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <Arduino.h>

/**
 * @brief Float printf converters
 * 
 * printf("32.1=" SPRINTF_FP1_FMT "\n", CONVERT_TO_FP2(32.1))
 * printf("123.45=" SPRINTF_FP2_FMT "\n", CONVERT_TO_FP2(123.56))
 * 
 */
#define CONVERT_TO_FP1(value)               (int32_t)(value / 1000), ((uint32_t)(value / 100) % 10)
#define CONVERT_TO_FP2(value)               (int32_t)(value / 1000), ((uint32_t)(value / 10) % 100)

#define SPRINTF_FP1_FMT                     "%d.%u"
#define SPRINTF_FP2_FMT                     "%d.%02u"
#define DEBUG_HUMAN 0
#define HAVE_DEBUG_PID_CONTROLLER 1

#define sizeof_array(arr)                   (sizeof(arr) / sizeof(arr[0]))


using InterruptCallbackType = void (*)();

/**
 * @brief translate arduino digital pin number to GPIO pin number
 * 
 * @tparam PIN_TYPE 
 * @param pin Arduino digital pin number
 * @return constexpr uint8_t GPIO pin number 0-15
 */
template<typename PIN_TYPE>
constexpr uint8_t digitalPinToBit(PIN_TYPE pin) 
{
    switch(pin) {
        case PA0: case PB0: case PC0: case PD0: case PE0:
            return 0;
        case PA1: case PB1: case PC1: case PD1: case PE1:
            return 1;
        case PA2: case PB2: case PC2: case PD2: case PE2:
            return 2;
        case PA3: case PB3: case PC3: case PD3: case PE3:
            return 3;
        case PA4: case PB4: case PC4: case PD4: case PE4:
            return 4;
        case PA5: case PB5: case PC5: case PD5: case PE5:
            return 5;
        case PA6: case PB6: case PC6: case PD6: case PE6:
            return 6;
        case PA7: case PB7: case PC7: case PD7: case PE7:
            return 7;
        case PA8: case PB8: case PC8: case PD8: case PE8:
            return 8;
        case PA9: case PB9: case PC9: case PD9: case PE9:
            return 9;
        case PA10: case PB10: case PC10: case PD10: case PE10:
            return 10;
        case PA11: case PB11: case PC11: case PD11: case PE11:
            return 11;
        case PA12: case PB12: case PC12: case PD12: case PE12:
            return 12;
        case PA13: case PB13: case PC13: case PD13: case PE13:
            return 13;
        case PA14: case PB14: case PC14: case PD14: case PE14:
            return 14;
        case PA15: case PB15: case PC15: case PD15: case PE15:
            return 15;
    }
    return 0;
}

/**
 * @brief translate arduino digital pin number to GPIO pin number
 * 
 * @tparam PIN_TYPE 
 * @param pin Arduino digital pin number
 * @return constexpr uint8_t GPIO pin number 0-15
 */
template<uint32_t PIN>
constexpr uint8_t digitalPinToBit() 
{
    switch(PIN) {
        case PA0: case PB0: case PC0: case PD0: case PE0:
            return 0;
        case PA1: case PB1: case PC1: case PD1: case PE1:
            return 1;
        case PA2: case PB2: case PC2: case PD2: case PE2:
            return 2;
        case PA3: case PB3: case PC3: case PD3: case PE3:
            return 3;
        case PA4: case PB4: case PC4: case PD4: case PE4:
            return 4;
        case PA5: case PB5: case PC5: case PD5: case PE5:
            return 5;
        case PA6: case PB6: case PC6: case PD6: case PE6:
            return 6;
        case PA7: case PB7: case PC7: case PD7: case PE7:
            return 7;
        case PA8: case PB8: case PC8: case PD8: case PE8:
            return 8;
        case PA9: case PB9: case PC9: case PD9: case PE9:
            return 9;
        case PA10: case PB10: case PC10: case PD10: case PE10:
            return 10;
        case PA11: case PB11: case PC11: case PD11: case PE11:
            return 11;
        case PA12: case PB12: case PC12: case PD12: case PE12:
            return 12;
        case PA13: case PB13: case PC13: case PD13: case PE13:
            return 13;
        case PA14: case PB14: case PC14: case PD14: case PE14:
            return 14;
        case PA15: case PB15: case PC15: case PD15: case PE15:
            return 15;
    }
    return 0;
}

/**
 * @brief translate arduino pin macro to GPIOx_BASE address
 *
 * @tparam PIN Arduino pin macro (e.g. PA0, PB10, PD7)
 * @return constexpr uint32_t GPIOx_BASE address, or 0 for invalid port
 */
template<uint32_t PIN>
constexpr uint32_t digitalPinToGPIOBase() 
{
    switch(PIN) {
        case PA0: case PA1: case PA2: case PA3: case PA4: case PA5: case PA6: case PA7: case PA8: case PA9: case PA10: case PA11: case PA12: case PA13: case PA14: case PA15:
            return GPIOA_BASE;
        case PB0: case PB1: case PB2: case PB3: case PB4: case PB5: case PB6: case PB7: case PB8: case PB9: case PB10: case PB11: case PB12: case PB13: case PB14: case PB15:
            return GPIOB_BASE;
        case PC0: case PC1: case PC2: case PC3: case PC4: case PC5: case PC6: case PC7: case PC8: case PC9: case PC10: case PC11: case PC12: case PC13: case PC14: case PC15:
            return GPIOC_BASE;
        case PD0: case PD1: case PD2: case PD3: case PD4: case PD5: case PD6: case PD7: case PD8: case PD9: case PD10: case PD11: case PD12: case PD13: case PD14: case PD15:
            return GPIOD_BASE;  
        case PE0: case PE1: case PE2: case PE3: case PE4: case PE5: case PE6: case PE7: case PE8: case PE9: case PE10: case PE11: case PE12: case PE13: case PE14: case PE15:
            return GPIOE_BASE;
    }
    return 0;
}

/**
 * @brief translate arduino pin macro to GPIO_TypeDef pointer
 * 
 * @tparam PIN Arduino pin macro (e.g. PA0, PB10, PD7)
 * @return constexpr GPIO_TypeDef* GPIO_TypeDef pointer, or nullptr for invalid port 
 */
template<uint32_t PIN>
constexpr GPIO_TypeDef *digitalPinToGPIO() 
{
    return reinterpret_cast<GPIO_TypeDef *>(digitalPinToGPIOBase<PIN>());
}

/**
 * @brief runtime overload for Arduino digital pin numbers
 *
 * @tparam PIN_TYPE
 * @param pin Arduino digital pin number
 * @return uint32_t GPIOx_BASE address, or 0 for invalid port
 */
template<typename PIN_TYPE>
inline uint32_t digitalPinToGPIOBase(PIN_TYPE pin) 
{
    switch(pin) {
        case PA0: case PA1: case PA2: case PA3: case PA4: case PA5: case PA6: case PA7: case PA8: case PA9: case PA10: case PA11: case PA12: case PA13: case PA14: case PA15:
            return GPIOA_BASE;
        case PB0: case PB1: case PB2: case PB3: case PB4: case PB5: case PB6: case PB7: case PB8: case PB9: case PB10: case PB11: case PB12: case PB13: case PB14: case PB15:
            return GPIOB_BASE;
        case PC0: case PC1: case PC2: case PC3: case PC4: case PC5: case PC6: case PC7: case PC8: case PC9: case PC10: case PC11: case PC12: case PC13: case PC14: case PC15:
            return GPIOC_BASE;
        case PD0: case PD1: case PD2: case PD3: case PD4: case PD5: case PD6: case PD7: case PD8: case PD9: case PD10: case PD11: case PD12: case PD13: case PD14: case PD15:
            return GPIOD_BASE;  
        case PE0: case PE1: case PE2: case PE3: case PE4: case PE5: case PE6: case PE7: case PE8: case PE9: case PE10: case PE11: case PE12: case PE13: case PE14: case PE15:
            return GPIOE_BASE;
    }
    return 0;
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
constexpr uint32_t RCC_APB2ENR_IOPxEN(uint32_t GPIO_PORT_ADDR) 
{
    return (1U << ((GPIO_PORT_ADDR >> 10U) & 0x0f));
}

/**
 * @brief translate GPIO port address to RCC_APB2ENR_IOPxEN bits
 * 
 * @param gpio_port GPIO_TypeDef pointer
 * @return bit mask for APB2ENR
 */
constexpr uint32_t RCC_APB2ENR_IOPxEN(GPIO_TypeDef *gpio_port) 
{
    return (1U << (((uint32_t)gpio_port >> 10U) & 0x0f));
}

/**
 * @brief get configuration register for GPIO pin
 * 
 * @param gpio_addr GPIOx_BASE address
 * @param pin Arduino PIN number
 * @return volatile uint32_t&
 */
inline volatile uint32_t &GPIO_CRx_REG(GPIO_TypeDef *gpio_port, uint32_t pin)
{
    return (digitalPinToBit(pin) < 8) ? (gpio_port)->CRL : (gpio_port)->CRH;
}

/**
 * @brief get configuration register for GPIO pin
 * 
 * @param gpio_addr GPIOx_BASE address
 * @param pin Arduino PIN number
 * @return volatile uint32_t&
 */
inline volatile uint32_t &GPIO_CRx_REG(uint32_t gpio_addr, uint32_t pin)
{
    return GPIO_CRx_REG(reinterpret_cast<GPIO_TypeDef *>(gpio_addr), pin);
}

/**
 * @brief get configuration register for GPIO pin
 * 
 * @tparam Arduino PIN number (PA0, PB10, PD7, etc.)
 * @return volatile uint32_t&
 */
template<uint32_t PIN>
inline volatile uint32_t &GPIO_CRx_REG()
{
    return GPIO_CRx_REG(digitalPinToGPIO<PIN>(), PIN);
}

/**
 * @brief calculate the auto-reload register value for a given PWM frequency with no prescaler (PSC=0)
 * 
 * @tparam FREQUENCY PWM frequency in Hz
 * @return constexpr uint16_t
 */
template<uint32_t FREQUENCY>
static constexpr uint16_t kPWMFrequencyToARR() 
{
    constexpr uint32_t tmp = 72000000 / FREQUENCY;
    static_assert(tmp <= 0xFFFF, "PWM frequency too low for 16bit timer");
    return tmp;
}

