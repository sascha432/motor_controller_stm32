/**
  Author: sascha_lammers@gmx.de
*/

#include "eeprom.h"
#include "adc.h"
#include "i2c.h"

// === AT24C02CM5/TR prototypes etc... ===

#define EEPROM_ADDRESS          0x50u       // 7-bit device address (A2=A1=A0=0)
#define EEPROM_PAGE_SIZE        8u          // bytes per page
#define EEPROM_SIZE             256u        // total bytes
#define EEPROM_WRITE_MS         5u          // typical write cycle time
#define EEPROM_WAIT_RETRIES     1000u       // number of retries for waiting for write cycle to complete
#define EEPROM_VALIDATE_WRITE   0

I2CHelper i2c;
EEPROM eeprom;

bool eepromWriteByte(uint8_t memAddress, uint8_t data);
int16_t eepromReadByte(uint8_t memAddress);
bool eepromWriteBytes(uint16_t memAddress, const uint8_t *data, uint16_t length);
bool eepromReadBytes(uint16_t memAddress, uint8_t *data, uint16_t length);
bool eepromWaitReady(void);

// === EEPROM implementation ===

void EEPROM::init()
{
    i2c.initI2C1Remapped();
    bool res = i2c.sendBytes(EEPROM_ADDRESS, nullptr, 0);
    DEBUG_PRINT(DEBUG_DEBUG, "EEPROM detected=%u", res);
}

void EEPROM::read()
{
    Data tmp;
    tmp.invalidate();
    bool result = eepromReadBytes(0, reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp));
    DEBUG_PRINT(DEBUG_WARNING, "read=%u magic=%08x version=%d sequence=%d", result, tmp.magic, tmp.version, tmp.sequence);

    if (!result || tmp.magic != kMagic || tmp.version != kVersion) {
        resetDefaults();
        return;
    }
    data = tmp;
    updateTemperatureLimits();
}

bool EEPROM::write()
{
    // read EEPROM and compare with current data to avoid unnecessary writes
    Data tmp;
    tmp.invalidate();
    bool result = eepromReadBytes(0, reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp));
    if (result) {
        if (tmp == data) {
            DEBUG_PRINT(DEBUG_DEBUG, "EEPROM write skipped, no changes");
            return false;
        }
    }
    else if (!result) {
        DEBUG_PRINT(DEBUG_WARNING, "EEPROM read failed");
    }

    // write data to EEPROM
    data.sequence++;
    result = eepromWriteBytes(0, reinterpret_cast<uint8_t *>(&data), sizeof(data));
    if (!result) {
        data.sequence--;
    }
    DEBUG_PRINT(DEBUG_ERROR, "write=%u magic=%08x version=%d sequence=%d", result, data.magic, data.version, data.sequence);

    #if EEPROM_VALIDATE_WRITE
        tmp.invalidate();
        result = eepromReadBytes(0, reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp));
        DEBUG_PRINT(DEBUG_DEBUG, "verify=%u magic=%08x version=%d sequence=%d", result, tmp.magic, tmp.version, tmp.sequence);
    #endif
    return result;
}

void EEPROM::updateTemperatureLimits()
{
    mosfet_temperature_limit_adc = ADCConverter::NTC::reverse(data.mosfet_temperature_limit);
    motor_temperature_limit_adc = ADCConverter::NTC::reverse(data.motor_temperature_limit);
}


// === AT24C02C I2C implementation ===

//------------------------------------------------------------------
// Poll the EEPROM with a dummy control-byte write until it ACKs,
// which signals the internal write cycle (tWR, ~5 ms) has finished.
//------------------------------------------------------------------
bool eepromWaitReady(void)
{
    uint32_t retries = EEPROM_WAIT_RETRIES;
    while (!i2c.sendByte(EEPROM_ADDRESS, 0x00, true)) {
        if (--retries == 0) {
            return false;
        }
    }
    return true;
}

//------------------------------------------------------------------
// Single byte write: [dev addr+W][word addr][data]
//------------------------------------------------------------------
bool eepromWriteByte(uint8_t memAddress, uint8_t data)
{
    uint8_t buf[2] = { memAddress, data };

    if (!i2c.sendBytes(EEPROM_ADDRESS, buf, sizeof(buf), true)) {
        return false;
    }
    return eepromWaitReady();
}

//------------------------------------------------------------------
// Single byte read: [dev addr+W][word addr] (repeated start) [dev addr+R][data]
//------------------------------------------------------------------
int16_t eepromReadByte(uint8_t memAddress)
{
    if (!i2c.sendBytes(EEPROM_ADDRESS, &memAddress, 1, false)) {
        return -1;
    }
    return i2c.readByte(EEPROM_ADDRESS);
}

//------------------------------------------------------------------
// Multi byte write, split into AT24C02 page-aligned chunks (8 bytes/page)
//------------------------------------------------------------------
bool eepromWriteBytes(uint16_t memAddress, const uint8_t *data, uint16_t length)
{
    if (memAddress + length > EEPROM_SIZE) {
        return false; // out of range
    }

    while (length > 0) {
        uint8_t pageOffset  = (uint8_t)(memAddress % EEPROM_PAGE_SIZE);
        uint8_t spaceInPage = EEPROM_PAGE_SIZE - pageOffset;
        uint8_t chunk       = (length < spaceInPage) ? (uint8_t)length : spaceInPage;

        uint8_t buf[1 + EEPROM_PAGE_SIZE];
        buf[0] = (uint8_t)memAddress;
        memcpy(&buf[1], data, chunk);

        if (!i2c.sendBytes(EEPROM_ADDRESS, buf, (uint16_t)(chunk + 1), true)) {
            return false;
        }
        if (!eepromWaitReady()) {
            return false;
        }

        memAddress += chunk;
        data       += chunk;
        length     -= chunk;
    }
    return true;
}

//------------------------------------------------------------------
// Multi byte sequential read: word addr write, repeated start, burst read
//------------------------------------------------------------------
bool eepromReadBytes(uint16_t memAddress, uint8_t *data, uint16_t length)
{
    if (memAddress + length > EEPROM_SIZE) {
        return false; // out of range
    }

    uint8_t addr = (uint8_t)memAddress;
    if (!i2c.sendBytes(EEPROM_ADDRESS, &addr, 1, false)) {
        return false;
    }

    return i2c.readBytes(EEPROM_ADDRESS, data, length);
}
