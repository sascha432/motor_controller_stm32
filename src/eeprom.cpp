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
    i2c.initI2C1Remapped();
    const bool result = i2c.sendBytes(kAddress, nullptr, 0);
    (void)result;
    DEBUG_PRINT(DebugType::INFO, "EEPROM detected=%u", static_cast<int>(result));
}

void EEPROM::read()
{
    Data tmp;
    bool result = readData(tmp, kDefaultOffset);
    if (!result || !tmp.isValid()) {
        for(;;) {
            if constexpr (kBackupOffset) {
                // invalid eeprom data, read backup
                result = readData(tmp, kBackupOffset);
                if (result && tmp.isValid()) {
                    break;
                }
            }
            DEBUG_PRINT(DebugType::ERROR, "EEPROM data invalid, resetting to defaults");
            resetDefaults();
            return;
        }
    }
    else if constexpr (kBackupOffset) {
        Data tmp2;
        result = readData(tmp2, kBackupOffset);
        DEBUG_PRINT(DEBUG_LEVEL_RESULT(result), "EEPROM sequence=%d backup_sequence=%u", tmp.sequence, tmp2.sequence);
        if (result && tmp2.isValid() && (tmp2.sequence > tmp.sequence)) {
            // the backup sequence number is more recent, use the backup instead
            data = tmp2;
            updateTemperatureLimits();
            return;
        }
    }
    data = tmp;
    updateTemperatureLimits();
}

bool EEPROM::write()
{
    // read EEPROM and compare with current data to avoid unnecessary writes
    bool result = hasChanged();
    if (!result) {
        DEBUG_PRINT(DebugType::INFO, "EEPROM write skipped, no changes");
        return false;
    }
    // update sequence and crc
    data.sequence++;
    data.crc = data.calculateCRC();
    // write data to EEPROM
    result = writeData(data, kDefaultOffset);
    if constexpr (kBackupOffset) {
        if (writeData(data, kBackupOffset)) {
            result = true;
        }
    }
    if (!result) {
        // restore sequence and update crc if both writes fail
        data.sequence--;
        data.crc = data.calculateCRC();
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

bool EEPROM::hasChanged() const
{
    Data tmp;
    const bool result = readData(tmp, kDefaultOffset);
    const bool resultValid = result && (tmp == data);
    if constexpr (kBackupOffset) {
        const bool result2 = readData(tmp, kBackupOffset);
        if (!(result2 && tmp == data)) {
            // backup read error or backup data does not match
            return true;
        }
    }
    if (resultValid) {
        return false;
    }
    // read error or data does not match
    return true;
}

bool EEPROM::readData(Data &data, size_t offset) const
{
    data.invalidate();
    const bool result = eepromReadBytes(offset, &data, sizeof(data));
    if (result) {
        data.validateCRC();
    }
    DEBUG_PRINT(DEBUG_LEVEL_RESULT(result), "EEPROM read=%u magic=%08x version=%d sequence=%d ofs=%u", static_cast<int>(result), data.magic, data.version, data.sequence, offset);
    if (!result) {
        data.invalidate();
    }
    return result;
}

bool EEPROM::writeData(const Data &data, size_t offset) const
{
    bool result = eepromWriteBytes(offset, &data, sizeof(data));
    DEBUG_PRINT(DEBUG_LEVEL_RESULT(result), "EEPROM write=%u magic=%08x version=%d sequence=%d ofs=%u", static_cast<unsigned>(result), data.magic, data.version, data.sequence, offset);

    #if DEBUG
        if constexpr (kValidateWrite) {
            Data tmp;
            const bool result2 = readData(tmp, offset);
            (void)result2;
            DEBUG_PRINT(DebugType::INFO, "EEPROM verify=%u magic=%08x version=%d sequence=%d crc=%08x ofs=%u", static_cast<unsigned>(result2), tmp.magic, tmp.version, tmp.sequence, tmp.crc, offset);
        }
    #endif
    return result;
}

// === AT24C02C I2C implementation ===

//------------------------------------------------------------------
// Poll the EEPROM with a dummy control-byte write until it ACKs,
// which signals the internal write cycle (tWR, ~5 ms) has finished.
//------------------------------------------------------------------
bool eepromWaitReady(void)
{
    const uint32_t start = HAL_GetTick();
    while((HAL_GetTick() - start) < EEPROM::kWriteCycleWaitTimeoutMs) {
        if (i2c.sendByte(EEPROM::kAddress, 0x00, true)) {
            return true;
        }
    }
    DEBUG_PRINT(DebugType::ERROR, "EEPROM timeout=%u", static_cast<unsigned>(HAL_GetTick() - start));
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
