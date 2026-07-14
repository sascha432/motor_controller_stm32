/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "helpers.h"

/**
 * @brief initialization of the I2C bus and simple blocking functions to communicate
 * 
 */
struct I2CHelper {

    static const uint32_t kTimeoutMicros = 10000U;

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
        // AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;
        AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;

        // PB8, PB9 = Alternate Function Open Drain, 50MHz
        GPIOB->CRH &= ~((0xF << 0) | (0xF << 4));
        GPIOB->CRH |=  ((0xF << 0) | (0xF << 4));

        initCommon();
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

        // PB6, PB7 = Alternate Function Open-Drain, 50 MHz
        GPIOB->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
        GPIOB->CRL |=  ((0xF << (6 * 4)) | (0xF << (7 * 4)));

        initCommon();
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

        // PB6, PB7 = Floating input (reset state)
        GPIOB->CRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
        GPIOB->CRL |=  ((0x4 << (6 * 4)) | (0x4 << (7 * 4)));

        // PB8, PB9 = Floating input (reset state)
        GPIOB->CRH &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
        GPIOB->CRH |=  ((0x4 << (0 * 4)) | (0x4 << (1 * 4)));

        // Optional: disable peripheral clock if no longer needed
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;

        delay(10);
    }

    /**
     * @brief send data to i2c bus
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
        // START
        I2C1->CR1 |= I2C_CR1_START;

        uint32_t timeout = kTimeoutMicros;
        while (!(I2C1->SR1 & I2C_SR1_SB)) {
            if (isTimeout(timeout)) {
                return I2C_Error();
            }
        }

        // Address
        I2C1->DR = address << 1;

        timeout = kTimeoutMicros;
        while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
            if (I2C1->SR1 & I2C_SR1_AF) {
                return I2C_Error();
            }

            if (isTimeout(timeout)) {
                return I2C_Error();
            }
        }

        // Clear ADDR
        (void)I2C1->SR1;
        (void)I2C1->SR2;

        if (length) {

            while (length--) {
                timeout = kTimeoutMicros;

                while (!(I2C1->SR1 & I2C_SR1_TXE)) {
                    if (I2C1->SR1 & I2C_SR1_AF) {
                        return I2C_Error();
                    }

                    if (isTimeout(timeout)) {
                        return I2C_Error();
                    }
                }
                I2C1->DR = *data++;
            }

            timeout = kTimeoutMicros;
            while (!(I2C1->SR1 & I2C_SR1_BTF)) {
                if (isTimeout(timeout)) {
                    return I2C_Error();
                }
            }
        }

        if (stop) {
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        return true;
    }

    /**
     * @brief read data from i2c bus
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

        // START
        I2C1->CR1 |= I2C_CR1_START;

        uint32_t timeout = kTimeoutMicros;
        while (!(I2C1->SR1 & I2C_SR1_SB)) {
            if (isTimeout(timeout)) {
                return I2C_Error();
            }
        }

        // Address + read
        I2C1->DR = (address << 1) | 1;

        timeout = kTimeoutMicros;
        while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
            if (I2C1->SR1 & I2C_SR1_AF) {
                return I2C_Error();
            }

            if (isTimeout(timeout)) {
                return I2C_Error();
            }
        }

        // Single byte
        if (length == 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;

            (void)I2C1->SR1;
            (void)I2C1->SR2;

            I2C1->CR1 |= I2C_CR1_STOP;

            timeout = kTimeoutMicros;
            while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
                if (isTimeout(timeout)) {
                    return false;
                }
            }

            *data = I2C1->DR;
            I2C1->CR1 |= I2C_CR1_ACK;
            return true;
        }

        // Multiple bytes
        (void)I2C1->SR1;
        (void)I2C1->SR2;

        while (length--) {
            timeout = kTimeoutMicros;

            while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
                if (isTimeout(timeout)) {
                    return I2C_Error();
                }
            }

            if (length == 0) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                I2C1->CR1 |= I2C_CR1_STOP;
            }

            *data++ = I2C1->DR;
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
    void initCommon()
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

        delay(10);
    }

    inline bool I2C_Error() 
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        I2C1->SR1 &= ~I2C_SR1_AF;
        return false;
    }

    inline bool isTimeout(uint32_t &counter) 
    {
        delayMicroseconds(1);
        return --counter == 0;
    }
};
