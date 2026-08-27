/**
  Author: sascha_lammers@gmx.de
*/

#include <memory.h>
#include "main.h"

// === CRC helper function ===

uint32_t stm32_CRC(const uint32_t *data, size_t size)
{
    #if USE_HAL_CRC_FUNCTIONS
        // HAL version with a little extra overhead and data must be 32bit aligned on certain MCUs
        const uint32_t bytes = size % sizeof(uint32_t);
        const uint32_t words = size / sizeof(uint32_t);
        if (bytes == 0) {
            return HAL_CRC_Calculate(&hcrc, data, words);
        }
        HAL_CRC_Calculate(&hcrc, data, words);
        uint32_t lastWord = 0;
        memcpy(&lastWord, &data[words], bytes);
        return HAL_CRC_Accumulate(&hcrc, &lastWord, 1);
    #else
        // reset CRC calculation unit
        CRC->CR |= CRC_CR_RESET;

        // copy words first
        while(size >= sizeof(uint32_t)) {
            CRC->DR = *data++;
            size -= sizeof(uint32_t);
        }

        // any bytes left?
        if (size) {
            CRC->DR = *data & ((1U << (size * 8U)) - 1);
        }
        return CRC->DR;
    #endif
}
