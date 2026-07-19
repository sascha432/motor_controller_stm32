# TEST TEST

## This document contains the test protocol

### Rev 1.0

- 3.3V supply produces 3.27-3.29V, tested up to 200ma
- LED cc driver produces 122mA for a 12V LED, shorted with mA range (50R?) 127mA
- LED driver 27.09V disconnected max. voltage
- USB connector and diode checked, TODO check 5.1k to data line and data to mcu
- MCU working
- DRV8701 no shorts
- TFT, knob, buttons are working
- TFT backlight working
- LED lightning: added pulldown to pwm to avoid flashing during reset
- INA K connections fixed and output filter added
- INA alert# triggers interrupt
- DRV8701 powers up without faults after providing 3.3v on nSLEEP
- Motor LEDs working, floating seems to cause minor glow
- 4 ADC channels working current, voltage, 2x temp.
- DAC outputs working DRV8701 vref and INA381 comparator vref
- AT24C02CM5 EEPROM is working
- TODO DRV8701 fault after turning PWM on

### Rev 1.1

Untested