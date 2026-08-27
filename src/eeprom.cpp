/**
  Author: sascha_lammers@gmx.de
*/

#include "eeprom.h"

// === AT24C02CM5/TR prototypes etc... ===

I2CHelper i2c;
EEPROM eeprom;

bool eepromWriteByte(uint8_t memAddress, uint8_t data);
int eepromReadByte(uint8_t memAddress);
bool eepromWriteBytes(uint8_t memAddress, const void *data, uint32_t length);
bool eepromReadBytes(uint8_t memAddress, void *data, uint32_t length);
bool eepromWaitReady(void);

// === EEPROM implementation ===

void EEPROM::init()
{
    SWO::data.EEPROM.address = reinterpret_cast<uint32_t>(&this->data);
    i2c.initI2C1Remapped();
    bool result = i2c.sendBytes(kAddress, nullptr, 0);
    (void)result;
    DEBUG_PRINT(DebugType::INFO, "EEPROM detected=%u", (int)result);
}

void EEPROM::read()
{
    Data tmp;
    tmp.invalidate();
    bool result = eepromReadBytes(kDefaultOffset, &tmp, sizeof(tmp));
    DEBUG_PRINT(DEBUG_LEVEL_RESULT(result), "read=%u magic=%08x version=%d sequence=%d ofs=%u", (int)result, tmp.magic, tmp.version, tmp.sequence, kDefaultOffset);
    if (!result || tmp.magic != kMagic || tmp.version != kVersion || tmp.validateCRC() == kInvalidCRC) {
        if constexpr (kBackupOffset) {
            tmp.invalidate();
            result = eepromReadBytes(kBackupOffset, &tmp, sizeof(tmp));
            DEBUG_PRINT(DEBUG_LEVEL_RESULT(result), "read=%u magic=%08x version=%d sequence=%d ofs=%u (BACKUP)", (int)result, tmp.magic, tmp.version, tmp.sequence, kBackupOffset);
            if (!result || tmp.magic != kMagic || tmp.version != kVersion || tmp.validateCRC() == kInvalidCRC) {
                DEBUG_PRINT(DebugType::ERROR, "EEPROM data invalid, resetting to defaults");
                resetDefaults();
                return;
            }
        }
        else {
            DEBUG_PRINT(DebugType::ERROR, "EEPROM data invalid, resetting to defaults");
            resetDefaults();
            return;
        }
    }
    data = tmp;
    updateTemperatureLimits();
}

bool EEPROM::write()
{
    // read EEPROM and compare with current data to avoid unnecessary writes
    Data tmp;
    tmp.invalidate();
    bool result = eepromReadBytes(kDefaultOffset, &tmp, sizeof(tmp));
    if (result) {
        tmp.validateCRC();
        if (tmp == data) {
            DEBUG_PRINT(DebugType::INFO, "EEPROM write skipped, no changes");
            return false;
        }
    }
    else if (!result) {
        DEBUG_PRINT(DebugType::ERROR, "EEPROM read failed");
    }

    // write data to EEPROM
    data.sequence++;
    data.crc = data.calculateCRC();
    result = eepromWriteBytes(kDefaultOffset, &data, sizeof(data));
    if (!result) {
        data.sequence--;
    }
    DEBUG_PRINT(DEBUG_LEVEL_RESULT(result), "write=%u magic=%08x version=%d sequence=%d ofs=%u", (unsigned)result, data.magic, data.version, data.sequence, kDefaultOffset);

    if constexpr (kBackupOffset) {
        result = eepromWriteBytes(kBackupOffset, &data, sizeof(data));
        DEBUG_PRINT(DEBUG_LEVEL_RESULT(result), "write=%u magic=%08x version=%d sequence=%d ofs=%u (BACKUP)", (unsigned)result, data.magic, data.version, data.sequence, kBackupOffset);
    }

    if constexpr (kValidateWrite) {
        tmp.invalidate();
        result = eepromReadBytes(kDefaultOffset, &tmp, sizeof(tmp));
        DEBUG_PRINT(DebugType::INFO, "verify=%u magic=%08x version=%d sequence=%d crc=%08x ofs=%u", (unsigned)result, tmp.magic, tmp.version, tmp.sequence, tmp.crc, kDefaultOffset);
        if constexpr (kBackupOffset) {
            tmp.invalidate();
            result = eepromReadBytes(kBackupOffset, &tmp, sizeof(tmp));
            tmp.validateCRC();
            DEBUG_PRINT(DebugType::INFO, "verify=%u magic=%08x version=%d sequence=%d crc=%08x ofs=%u (BACKUP)", (unsigned)result, tmp.magic, tmp.version, tmp.sequence, tmp.crc, kBackupOffset);
        }
    }
    return result;
}

void EEPROM::resetDefaults()
{
    data = Data();
    data.crc = data.calculateCRC();
    updateTemperatureLimits();
}

void EEPROM::setMosfetTemperatureLimit(uint8_t value)
{
    data.mosfet_temperature_limit = value;
    mosfet_temperature_limit_adc = ADCConverter::NTC::reverse(value);
}

void EEPROM::setMotorTemperatureLimit(uint8_t value)
{
    data.motor_temperature_limit = value;
    motor_temperature_limit_adc = ADCConverter::NTC::reverse(value);
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
    uint32_t start = HAL_GetTick();
    while(HAL_GetTick() - start <= EEPROM::kWriteCycleWaitTimeoutMs) {
        if (i2c.sendByte(EEPROM::kAddress, 0x00, true)) {
            return true;
        }
    }
    DEBUG_PRINT(DebugType::ERROR, "timeout=%u", (unsigned)(HAL_GetTick() - start));
    return false;
}

//------------------------------------------------------------------
// Single byte write: [dev addr+W][word addr][data]
//------------------------------------------------------------------
bool eepromWriteByte(uint8_t memAddress, uint8_t data)
{
    const uint8_t buf[2] = { memAddress, data };
    if (!i2c.sendBytes(EEPROM::kAddress, buf, sizeof(buf), true)) {
        return false;
    }
    return eepromWaitReady();
}

//------------------------------------------------------------------
// Single byte read: [dev addr+W][word addr] (repeated start) [dev addr+R][data]
//------------------------------------------------------------------
int eepromReadByte(uint8_t memAddress)
{
    if (!i2c.sendBytes(EEPROM::kAddress, &memAddress, 1, false)) {
        return -1;
    }
    return i2c.readByte(EEPROM::kAddress);
}

//------------------------------------------------------------------
// Multi byte write, split into AT24C02 page-aligned chunks (8 bytes/page)
//------------------------------------------------------------------
bool eepromWriteBytes(uint8_t memAddress, const void *data, uint32_t length)
{
    if (memAddress + length > EEPROM::kSize) {
        return false; // out of range
    }

    const uint8_t *src = reinterpret_cast<const uint8_t *>(data);
    while (length > 0) {
        size_t pageOffset  = memAddress % EEPROM::kPageSize;
        size_t spaceInPage = EEPROM::kPageSize - pageOffset;
        size_t chunk       = (length < spaceInPage) ? length : spaceInPage;

        uint8_t buf[1 + EEPROM::kPageSize];
        buf[0] = memAddress;
        memcpy(&buf[1], src, chunk);

        if (!i2c.sendBytes(EEPROM::kAddress, buf, chunk + 1, true)) {
            return false;
        }
        if (!eepromWaitReady()) {
            return false;
        }

        memAddress += chunk;
        src        += chunk;
        length     -= chunk;
    }
    return true;
}

//------------------------------------------------------------------
// Multi byte sequential read: word addr write, repeated start, burst read
//------------------------------------------------------------------
bool eepromReadBytes(uint8_t memAddress, void *data, uint32_t length)
{
    if (memAddress + length > EEPROM::kSize) {
        return false; // out of range
    }
    if (!i2c.sendBytes(EEPROM::kAddress, &memAddress, 1, false)) {
        return false;
    }
    return i2c.readBytes(EEPROM::kAddress, reinterpret_cast<uint8_t *>(data), length);
}
