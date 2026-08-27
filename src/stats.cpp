/**
  Author: sascha_lammers@gmx.de
*/

#include "stats.h"

// static range checks to void overflows in internal calculations, need to be updated if constants are changed
static_assert(ADCConverter::Voltage::convert(100) == 966, "static check failed");
static_assert(ADCConverter::Voltage::reverse(996) == 103, "static check failed");
static_assert(ADCConverter::Voltage::convert(4095) == 39563, "static check failed");
static_assert(ADCConverter::Voltage::reverse(39563) == 4095, "static check failed");
static_assert(ADCConverter::Current::convert(100) == 1006, "static check failed");
static_assert(ADCConverter::Current::reverse(1010) == 100, "static check failed");
static_assert(ADCConverter::Current::convert(4095) == 41205, "static check failed");
static_assert(ADCConverter::Current::reverse(41205) == 4083, "static check failed");

Stats stats;

void Stats::update()
{
    // get adc values
    const uint16_t vSense = adc.getVSenseValue();
    const uint16_t iSenseAvg = adc.getISenseAverageValue();
    const uint16_t motorTempFiltered = adc.getMotorTemperatureFiltered();
    const uint16_t mosfetTempFiltered = adc.getMosfetTemperatureFiltered();

    // update stats
    minMax.vcc.update(adc.getAndClearVSenseMinValue(), adc.getAndClearVSenseMaxValue(), vSense);

    minMax.motorTemp.update(motorTempFiltered);
    minMax.mosfetTemp.update(mosfetTempFiltered);
    minMax.current.update(iSenseAvg);

    // store converted values for display purposes
    vcc = ADCConverter::Voltage::convert(vSense);
    current = ADCConverter::Current::convert(iSenseAvg);
    motorTemp = ADCConverter::NTC::convert(motorTempFiltered);
    mosfetTemp = ADCConverter::NTC::convert(mosfetTempFiltered);
    min.vcc = ADCConverter::Voltage::convert(minMax.vcc.getMin());
    min.current = ADCConverter::Current::convert(minMax.current.getMin());
    max.vcc = ADCConverter::Voltage::convert(minMax.vcc.getMax());
    max.current = ADCConverter::Current::convert(minMax.current.getMax());
    if constexpr (ADCConverter::NTC::kHighSideNTC) {
        min.motorTemp = ADCConverter::NTC::convert(minMax.motorTemp.getMin());
        min.mosfetTemp = ADCConverter::NTC::convert(minMax.mosfetTemp.getMin());
        max.motorTemp = ADCConverter::NTC::convert(minMax.motorTemp.getMax());
        max.mosfetTemp = ADCConverter::NTC::convert(minMax.mosfetTemp.getMax());
    }
    else {
        // min/max swapped for NTC because the ADC value is inverted, lower ADC value = higher temperature
        min.motorTemp = ADCConverter::NTC::convert(minMax.motorTemp.getMax());
        min.mosfetTemp = ADCConverter::NTC::convert(minMax.mosfetTemp.getMax());
        max.motorTemp = ADCConverter::NTC::convert(minMax.motorTemp.getMin());
        max.mosfetTemp = ADCConverter::NTC::convert(minMax.mosfetTemp.getMin());
    }
}
