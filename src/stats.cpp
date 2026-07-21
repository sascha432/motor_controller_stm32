/**
  Author: sascha_lammers@gmx.de
*/

#include "stats.h"
#include "adc_converters.h"

// static range checks to void overflows in internal calculations, need to be updated if constants are changed
static_assert(ADCConverter::Voltage::convert(100) == 966, "static check failed");
static_assert(ADCConverter::Voltage::convert(4095) == 39563, "static check failed, uint32_t overflow?");
static_assert(ADCConverter::Current::convert(100) == 1006, "static check failed");
static_assert(ADCConverter::Current::convert(4095) == 41205, "static check failed");

Stats stats;

void Stats::update()
{
    // get adc values and update stats
    auto v = adc.readAll();
    minMax.vcc.update(v.vsense);
    minMax.motorTemp.update(v.motor_ntc);
    minMax.mosfetTemp.update(v.driver_ntc);
    auto isenseAvg = adc.getISenseAverageValue();
    minMax.current.update(isenseAvg);

    // store converted values for display purposes
    vcc = ADCConverter::Voltage::convert(v.vsense);
    current = ADCConverter::Current::convert(isenseAvg);
    motorTemp = ADCConverter::NTC::convert(v.motor_ntc);
    mosfetTemp = ADCConverter::NTC::convert(v.driver_ntc);
    min.vcc = ADCConverter::Voltage::convert(minMax.vcc.getMin());
    min.current = ADCConverter::Current::convert(minMax.current.getMin());
    min.motorTemp = ADCConverter::NTC::convert(minMax.motorTemp.getMin());
    min.mosfetTemp = ADCConverter::NTC::convert(minMax.mosfetTemp.getMin());
    max.vcc = ADCConverter::Voltage::convert(minMax.vcc.getMax());
    max.current = ADCConverter::Current::convert(minMax.current.getMax());
    max.motorTemp = ADCConverter::NTC::convert(minMax.motorTemp.getMax());
    max.mosfetTemp = ADCConverter::NTC::convert(minMax.mosfetTemp.getMax());
}
