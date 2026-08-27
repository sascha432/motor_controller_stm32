/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>
#include <memory.h>

#define USE_HAL_CRC_FUNCTIONS 0

/**
 * @brief STM32 CRC calculation that supports sizes that are not a multiple of 4 bytes
 *
 * @param data to the data buffer
 * @param size Size of the data buffer in bytes
 * @return uint32_t Calculated CRC32 value
 */
uint32_t stm32_CRC(const uint32_t *data, size_t size);
