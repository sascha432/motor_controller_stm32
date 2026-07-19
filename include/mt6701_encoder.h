/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "i2c.h"
#include "debug.h"

/**
 * @brief MT6701 magnetic encoder I2C configuration
 * 
 * Arduino Wire library is used for I2C communication
 * 
 */
struct MT6701Config {

    static constexpr uint8_t REG_ABZ_RES_HIGH = 0x30;
    static constexpr uint8_t REG_ABZ_RES_LOW = 0x31;
    static constexpr uint8_t REG_PROG_KEY = 0x09;
    static constexpr uint8_t REG_PROG_CMD = 0x0A;

    static constexpr uint8_t CMD_UNLOCK = 0xB3;
    static constexpr uint8_t CMD_COMMIT = 0x05;

    static constexpr uint8_t MT6701_ADDR = 0x06;

    MT6701Config(I2CHelper &i2c) : i2c(i2c) 
    {
    }

    /**
     * @brief check if the encoder is detected on the I2C bus
     * 
     * @return true if the encoder is detected
     * @return false if the encoder is not detected
     */
    bool init() 
    {
        return i2c.sendBytes(MT6701_ADDR, nullptr, 0);
    }

    /**
     * @brief read PPR from encoder
     * 
     * @return uint16_t pulses per revolution
     */
    uint16_t getPPR() const 
    {
        uint8_t high = readRegister(REG_ABZ_RES_HIGH);
        uint8_t low  = readRegister(REG_ABZ_RES_LOW);
        return (((uint16_t)high << 8) | (low)) + 1;
    }    

    /**
     * @brief set PPR of the encoder
     * 
     * @param ppr set pulses per revolution
     * @return true if the PPR was set successfully
     * @return false if an error occurred
     */
    bool setPPR(uint16_t ppr) const 
    {
        uint16_t res_value = ppr - 1;
        uint8_t high = (res_value >> 8);
        uint8_t low  = (res_value) & 0xff;

        if (!writeRegister(REG_ABZ_RES_HIGH, high)) {
            return false;
        }
        delayMicroseconds(10000);

        if (!writeRegister(REG_ABZ_RES_LOW, low)) {
            return false;
        }
        delayMicroseconds(10000);

        return (getPPR() == ppr);
    }

    /**
     * @brief store settings in EEPROM, requires 4.5V supply voltage for programming
     * 
     * @return true if the settings were stored successfully
     * @return false if an error occurred
     */
    bool writeEEPROM() 
    {
        // Programming unlock + commit
        if (!writeRegister(REG_PROG_KEY, CMD_UNLOCK)) {
            return false;
        }
        delayMicroseconds(10000);

        if (!writeRegister(REG_PROG_CMD, CMD_COMMIT)) {
            return false;
        }
        delayMicroseconds(800000);  // EEPROM write time
        return true;
    }

private:
    uint8_t readRegister(uint8_t reg) const 
    {
        if (!i2c.sendByte(MT6701_ADDR, reg, false)) {
            return 0xff;
        }
        return i2c.readByte(MT6701_ADDR);
    }

    bool writeRegister(uint8_t reg, uint8_t value) const 
    {
        uint8_t buf[2] = { reg, value };
        return i2c.sendBytes(MT6701_ADDR, buf, sizeof(buf), true);
    }    

    I2CHelper &i2c;
};

/**
 * @brief MT6701 magnetic encoder driver
 * 
 * @tparam GPIO_PIN GPIO pin number for I2C enable
 * @tparam GPIO_PORT_ADDR GPIO port address for I2C enable
 * @tparam ACTIVE_LOW_I2C true if I2C enable is active low, false if active high
 */
template<uint8_t GPIO_PIN, bool ACTIVE_LOW_I2C, uint32_t GPIO_PORT_ADDR = digitalPinToGPIOBase<GPIO_PIN>()>
struct MT6701Encoder {

    static constexpr GPIO_TypeDef *GPIO_PORT = reinterpret_cast<GPIO_TypeDef *>(GPIO_PORT_ADDR);

    /**
     * @brief select encoder mode
     * 
     * @param state true to enable I2C mode, false to enable A/B mode
     */
    void setI2CEnablePin(bool state) 
    {
        (ACTIVE_LOW_I2C ? !state : state) ?
            (GPIO_PORT->BSRR = (1 << digitalPinToBit(GPIO_PIN))) : 
            (GPIO_PORT->BRR  = (1 << digitalPinToBit(GPIO_PIN)))
        ;
        // wait for the encoder to change state
        delayMicroseconds(10);
    }

    /**
     * @brief initialize the encoder in A/B mode
     */
    void init() 
    {
        // Enable GPIOx clock
        RCC->APB2ENR |= RCC_APB2ENR_IOPxEN(GPIO_PORT_ADDR); 

        // PB0 = General-purpose push-pull output, 50 MHz
        GPIO_CRx_REG(GPIO_PORT_ADDR, GPIO_PIN) &= ~(0xF << digitalPinShift(GPIO_PIN));  // Clear MODE0/CNF0
        GPIO_CRx_REG(GPIO_PORT_ADDR, GPIO_PIN) |=  (0x3 << digitalPinShift(GPIO_PIN));  // MODE=11 (50MHz), CNF=00 (push-pull)

        // turn off
        setI2CEnablePin(ACTIVE_LOW_I2C);
    }

    /**
     * @brief set PPR over I2C
     * 
     * To store the settings in the EEPROM, the MT6701 requires a 4.5V supply voltage
     * 
     * @param i2c I2C helper instance
     * @param ppr set pulses per revolution
     * @param writeEEPROM whether to write the settings to EEPROM
     */
    void programPPR(I2CHelper &i2c, uint16_t ppr, bool writeEEPROM = false) 
    {
        setI2CEnablePin(!ACTIVE_LOW_I2C);

        // switch I2C1 pins for MT6701
        i2c.deinitI2C1();
        i2c.initI2C1();

        MT6701Config config(i2c);
        if (config.init()) {
            auto result = config.setPPR(ppr);
            if (result && writeEEPROM) {
                result = config.writeEEPROM();
                DEBUG_PRINT(DEBUG_DEBUG, "write EEPROM=%u", result);
            }
            DEBUG_PRINT(DEBUG_DEBUG, "PPR=%u", config.getPPR());
        }
        else {
            DEBUG_PRINT(DEBUG_DEBUG, "MT6701 not detected at address 0x%02x", MT6701Config::MT6701_ADDR);
        }

        setI2CEnablePin(ACTIVE_LOW_I2C);
        // remap I2C1 for EEPROM
        i2c.deinitI2C1();
        i2c.initI2C1Remapped();
    }
};
