/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"

/**
 * @brief Charlieplexed LEDs
 * 
 */
template <uint32_t GPIO_PIN, uint32_t GPIO_PORT_ADDRESS = digitalPinToGPIOBase<GPIO_PIN>()>
struct LEDs {

    void init() {
        // Enable GPIO port clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(GPIO_PORT_ADDRESS);
        off();
    }

    void off() {
        // MODE=00, CNF=01 (floating input)
        ((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->CRH = 
            (((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->CRH & ~(0xF << digitalPinShift(PB12))) | 
            (0x4 << digitalPinShift(PB12)); 
    }

    void onLED1() {
        ((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->BSRR = GPIO_BSRR_BS12; // set pin high
        // MODE=10 (2MHz), CNF=00 (push-pull)
        ((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->CRH = 
            (((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->CRH & ~(0xF << digitalPinShift(PB12))) | 
            (0x2 << digitalPinShift(PB12)); 
    }

    void onLED2() {
        ((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->BSRR = GPIO_BSRR_BR12; // set pin low
        // MODE=10 (2MHz), CNF=00 (push-pull)
        ((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->CRH = 
            (((GPIO_TypeDef *)GPIO_PORT_ADDRESS)->CRH & ~(0xF << digitalPinShift(PB12))) | 
            (0x2 << digitalPinShift(PB12)); 
    }
};
