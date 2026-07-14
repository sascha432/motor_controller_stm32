/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "i2c.h"

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

    bool init() {
        return i2c.sendBytes(MT6701_ADDR, nullptr, 0);
        // Wire.beginTransmission(MT6701_ADDR);
        // return (Wire.endTransmission() == 0);
    }

    uint8_t readRegister(uint8_t reg) const {
        i2c.sendByte(MT6701_ADDR, reg, false);
        return i2c.readByte(MT6701_ADDR);
        // Wire.beginTransmission(MT6701_ADDR);
        // Wire.write(reg);
        // Wire.endTransmission(false);
        // Wire.requestFrom(MT6701_ADDR, (uint8_t)1);
        // return Wire.available() ? Wire.read() : 0xFF;
    }

    bool writeRegister(uint8_t reg, uint8_t value) const {
        uint8_t buf[2] = { reg, value };
        return i2c.sendBytes(MT6701_ADDR, buf, sizeof(buf), true);
        // Wire.beginTransmission(MT6701_ADDR);
        // Wire.write(reg);
        // Wire.write(value);
        // return (Wire.endTransmission() == 0);
    }    

    uint16_t getPPR() const {
        uint8_t high = readRegister(REG_ABZ_RES_HIGH);
        uint8_t low  = readRegister(REG_ABZ_RES_LOW);
        return (((uint16_t)high << 8) | (low)) + 1;
    }    

    bool setPPR(uint16_t ppr) const {
        uint16_t res_value = ppr - 1;
        uint8_t high = (res_value >> 8);
        uint8_t low  = (res_value) & 0xff;

        if (!writeRegister(REG_ABZ_RES_HIGH, high)) {
            return false;
        }
        delay(10);
        if (!writeRegister(REG_ABZ_RES_LOW, low)) {
            return false;
        }
        delay(10);

        return (getPPR() == ppr);
    }

    // requires 4.5V supply voltage for programming
    bool writeEEPROM() {
        // Programming unlock + commit
        if (!writeRegister(REG_PROG_KEY, CMD_UNLOCK)) {
            return false;
        }
        delay(10);
        if (!writeRegister(REG_PROG_CMD, CMD_COMMIT)) {
            return false;
        }
        delay(1000);  // EEPROM write time
        return true;
    }

};

/**
 * @brief MT6701 magnetic encoder driver
 * 
 * @tparam GPIO_PIN GPIO pin number for I2C enable
 * @tparam GPIO_PORT_ADDR GPIO port address for I2C enable
 * @tparam ACTIVE_LOW_I2C true if I2C enable is active low, false if active high
 */
template<uint8_t GPIO_PIN, uint32_t GPIO_PORT_ADDR, bool ACTIVE_LOW_I2C>
struct MT6701Encoder {

    void setI2CEnablePin(bool state) {
        (ACTIVE_LOW_I2C ? !state : state) ?
            (((GPIO_TypeDef *)GPIO_PORT_ADDR)->BSRR = (1 << digitalPinToBit(GPIO_PIN))) : 
            (((GPIO_TypeDef *)GPIO_PORT_ADDR)->BRR  = (1 << digitalPinToBit(GPIO_PIN)))
        ;
        // wait for the encoder to change state
        delayMicroseconds(10);
    }

    void init() {
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
     */
    void programPPR(uint16_t ppr) 
    {
        setI2CEnablePin(!ACTIVE_LOW_I2C);

        // switch I2C1 pins for MT6701
        i2c.deinitI2C1();
        i2c.initI2C1();

        MT6701Config config;
        Serial.print("MT6701 ");
        if (config.init()) {
            config.setPPR(ppr);
            // config.writeEEPROM(); // writes disabled
            Serial.print("PPR: ");
            Serial.println(config.getPPR());
        }
        else {
            Serial.println("ERROR not detected");
        }

        setI2CEnablePin(ACTIVE_LOW_I2C);
        // remap I2C1 for EEPROM
        i2c.deinitI2C1();
        i2c.initI2C1Remapped();
    }
};
