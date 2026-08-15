/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include <memory.h>
#include <stm32f1xx.h>

// === CRC helper function ===

extern CRC_HandleTypeDef hcrc;

/**
 * @brief Wrapper for HAL_CRC_ that supports non-32bit aligned data buffers by adding 0 byte padding
 *
 * @param data Pointer to the data buffer
 * @param size Size of the data buffer in bytes
 * @return uint32_t Calculated CRC32 value
 */
inline uint32_t stm32_CRC(const void *data, size_t size)
{
    if (!size) {
        return 0xffffffff;
    }
    uint32_t *dataPtr = reinterpret_cast<uint32_t *>(const_cast<void *>(data));
    const uint32_t bytes = size % sizeof(uint32_t);
    const uint32_t words = size / sizeof(uint32_t);
    if (bytes == 0) {
        return HAL_CRC_Calculate(&hcrc, dataPtr, words);
    }
    HAL_CRC_Calculate(&hcrc, dataPtr, words);
    uint32_t lastWord = 0;
    memcpy(&lastWord, &dataPtr[words], bytes);
    return HAL_CRC_Accumulate(&hcrc, &lastWord, 1);
}
