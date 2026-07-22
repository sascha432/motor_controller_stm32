# TEST TEST

## This document contains the test protocol

### Rev 1.0

- 3.3V supply produces 3.27-3.29V, tested up to 200ma
- LED cc driver produces 122mA for a 12V LED, shorted with mA range (50R?) 127mA
- LED driver 27.09V disconnected max. voltage
- USB connector and diode checked
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
- DRV8701 fault after turning PWM on.. fixed swapped GL1 GH1 pins
- motor is running with PWM
- PID controller is regulating
- DRV8701 current limit and INA OCP working
- USB not detecting any device: TODO USB-C CC resistors: both CC1 and CC2 need 5.1k to GND for a device-only port.
D+ and D- continuity from connector to PA12 and PA11.
VBUS presence at the MCU board when connected to PC.
Data-capable cable and direct PC port (no hub).
D+ and D- not swapped.
- MT6701 analog sensor working as counter to check if the motor turns

### Rev 1.1

Untested
