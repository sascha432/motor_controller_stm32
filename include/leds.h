/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"

/**
 * @brief charlieplexed LEDs on PB12
 * 
 */
struct LEDs {

    void init() {
        // Enable GPIOB clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
        off();
    }

    void off() {
        GPIOB->CRH = (GPIOB->CRH & ~(0xF << digitalPinShift(PB12))) | (0x4 << digitalPinShift(PB12)); // MODE=00, CNF=01 (floating input)
    }

    void onLED1() {
        GPIOB->BSRR = GPIO_BSRR_BS12; // set pin high
        GPIOB->CRH = (GPIOB->CRH & ~(0xF << digitalPinShift(PB12))) | (0x2 << digitalPinShift(PB12)); // MODE=10 (2MHz), CNF=00 (push-pull)
    }

    void onLED2() {
        GPIOB->BSRR = GPIO_BSRR_BR12; // set pin low
        GPIOB->CRH = (GPIOB->CRH & ~(0xF << digitalPinShift(PB12))) | (0x2 << digitalPinShift(PB12)); // MODE=10 (2MHz), CNF=00 (push-pull)
    }
};
