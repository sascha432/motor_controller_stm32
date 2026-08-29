#include "debug.h"
/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"

/**
 * @brief Helper macro with timeout
 *
 */
#define I2C_WAIT_TIMEOUT(condition, timeout, af_check) \
    do { \
        const uint16_t start = TIM7->CNT; \
        while((condition)) { \
            if constexpr (af_check) { \
                if (I2C1->SR1 & I2C_SR1_AF) { \
                    return I2CError(); \
                } \
            } \
            if (static_cast<uint16_t>(TIM7->CNT - start) >= timeout) { \
                return I2CError(); \
            } \
        } \
    } while(0);

/**
 * @brief initialization of the I2C bus and simple blocking functions to communicate
 *
 */
struct I2CHelper
{
    // wait timeout
    static const uint16_t kTimeoutMicros = 500;

    /**
     * @brief initialize I2C1 on PB8/PB9 (remapped)
     *
     */
    void initI2C1Remapped(void)
    {
        // Enable clocks
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

        // Remap I2C1 to PB8/PB9
        AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;

        // Clear the MODE and CNF fields for PB8 and PB9 before configuring them.
        GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 |
                        GPIO_CRH_MODE9 | GPIO_CRH_CNF9);

        // Configure PB8 and PB9 as 50 MHz alternate-function open-drain pins.
        GPIOB->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0
                   | GPIO_CRH_CNF8_1  | GPIO_CRH_CNF8_0
                   | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0
                   | GPIO_CRH_CNF9_1  | GPIO_CRH_CNF9_0;

        initI2C1Common();
    }

    /**
     * @brief initialize I2C1 on PB6/PB7 (default)
     *
     */
    void initI2C1()
    {
        // Enable clocks
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

        // Ensure I2C1 is NOT remapped (PB6/PB7)
        AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;

        // Clear the MODE and CNF fields for PB6 and PB7 before configuring them.
        GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
                        GPIO_CRL_MODE7 | GPIO_CRL_CNF7);

        // Configure PB6 and PB7 as 50 MHz alternate-function open-drain pins.
        GPIOB->CRL  |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1
                    | GPIO_CRL_CNF6_0  | GPIO_CRL_CNF6_1
                    | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1
                    | GPIO_CRL_CNF7_0  | GPIO_CRL_CNF7_1;

        initI2C1Common();
    }

    /**
     * @brief disable I2C1 and reset PB6, PB7, PB8, PB9 to floating input
     *
     */
    void deinitI2C1()
    {
        // Disable I2C peripheral
        I2C1->CR1 &= ~I2C_CR1_PE;

        // Optional: reset I2C registers
        RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

        // Remove remap
        AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;

        // Clear the MODE and CNF fields for PB6 and PB7.
        GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
        // Set CNF=01 and MODE=00: floating input, the GPIO reset state.
        GPIOB->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0;

        // Clear the MODE and CNF fields for PB8 and PB9.
        GPIOB->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 | GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
        // Set CNF=01 and MODE=00: floating input, the GPIO reset state.
        GPIOB->CRH |= GPIO_CRH_CNF8_0 | GPIO_CRH_CNF9_0;

        // disable peripheral clock
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;

        delay_us<10>();
    }

    /**
     * @brief send data to I2C bus
     *
     * @param address I2C address of the device
     * @param data Pointer to the data buffer
     * @param length Number of bytes to send
     * @param stop Whether to send a STOP condition after transmission
     * @return true if the transmission was successful
     * @return false if an error occurred
     */
    bool sendBytes(uint8_t address, const uint8_t *data, uint16_t length, bool stop = true)
    {
        // Start
        I2C1->CR1 |= I2C_CR1_START;
        I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_SB), kTimeoutMicros, false);

        // Send address
        I2C1->DR = address << 1;
        I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_ADDR), kTimeoutMicros, true);

        // Clear ADDR
        (void)I2C1->SR1;
        (void)I2C1->SR2;

        if (length) {
            while (length--) {
                I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_TXE), kTimeoutMicros, true);
                I2C1->DR = *data++;
            }
            I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_BTF), kTimeoutMicros, false);
        }

        if (stop) {
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        return true;
    }

    /**
     * @brief read data from I2C bus
     *
     * @param address I2C address of the device
     * @param data Pointer to the data buffer
     * @param length Number of bytes to read
     * @return true if the read was successful
     * @return false if an error occurred
     */

    bool readBytes(uint8_t address, uint8_t *data, uint16_t length)
    {
        if (length == 0) {
            return false;
        }

        // Keep receiver state in a known default configuration.
        I2C1->CR1 |= I2C_CR1_ACK;
        I2C1->CR1 &= ~I2C_CR1_POS;

        // Start
        I2C1->CR1 |= I2C_CR1_START;
        I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_SB), kTimeoutMicros, false);

        // Send address + read
        I2C1->DR = (address << 1) | 1;
        I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_ADDR), kTimeoutMicros, true);

        if (length == 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            // Clear ADDR before STOP for 1-byte read on STM32F1.
            (void)I2C1->SR1;
            (void)I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;

            I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_RXNE), kTimeoutMicros, false);
            *data = I2C1->DR;
        } else {
            // Clear ADDR, then read all bytes while scheduling NACK+STOP
            // before receiving the final byte.
            (void)I2C1->SR1;
            (void)I2C1->SR2;

            while (length > 0) {
                I2C_WAIT_TIMEOUT(!(I2C1->SR1 & I2C_SR1_RXNE), kTimeoutMicros, false);
                if (length == 2) {
                    I2C1->CR1 &= ~I2C_CR1_ACK;
                    I2C1->CR1 |= I2C_CR1_STOP;
                }
                *data++ = I2C1->DR;
                --length;
            }
        }

        I2C1->CR1 |= I2C_CR1_ACK;
        return true;
    }

    /**
     * @brief send a single byte to i2c bus
     *
     * @param address I2C address of the device
     * @param data Byte to send
     * @param stop Whether to send a STOP condition after transmission
     * @return true if the transmission was successful
     * @return false if an error occurred
     */
    inline bool sendByte(uint8_t address, uint8_t data, bool stop = true)
    {
        return sendBytes(address, &data, sizeof(data), stop);
    }

    /**
     * @brief read a single byte from i2c bus
     *
     * @param address I2C address of the device
     * @return int16_t The read byte, or -1 if an error occurred
     */
    int16_t readByte(uint8_t address)
    {
        uint8_t data;
        if (readBytes(address, &data, sizeof(data))) {
            return data;
        }
        return -1;
    }

private:
    void initI2C1Common()
    {
        // Reset I2C
        I2C1->CR1 = I2C_CR1_SWRST;
        I2C1->CR1 = 0;

        // APB1 = 36 MHz
        I2C1->CR2 = 36;

        // 100 kHz Standard Mode
        I2C1->CCR = 180;

        // Maximum rise time
        I2C1->TRISE = 37;

        // Enable I2C
        I2C1->CR1 = I2C_CR1_PE;

        delay_us<10>();
    }

    inline bool I2CError()
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        I2C1->SR1 &= ~I2C_SR1_AF;
        return false;
    }
};
