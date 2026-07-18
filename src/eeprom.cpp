/**
  Author: sascha_lammers@gmx.de
*/

#include "eeprom.h"
#include "adc.h"

void EEPROM::init() 
{
    // Initialize EEPROM hardware if necessary
}

void EEPROM::read(Data &data) 
{
    // Read EEPROM data from hardware
    // For example, read from I2C or SPI EEPROM
    data = Data();
    updateTemperatureLimits();
}

void EEPROM::write(const Data &data) 
{
    // Write EEPROM data to hardware
    // For example, write to I2C or SPI EEPROM    
}

void EEPROM::updateTemperatureLimits()
{
    mosfet_temperature_limit_adc = ADCTemperatureConverter::reverse(data.mosfet_temperature_limit);
    motor_temperature_limit_adc = ADCTemperatureConverter::reverse(data.motor_temperature_limit);
}
