# Changelog

## [Unreleased 1.0.2]

- Settings Tuning enabled for Open Loop Mode
- Added current limit strength (Low - Very High)
- Added LVGL double buffering with DMA interrupts to improve display performance
- Terminal connected detection to avoid USB timeouts and lag
- Using ADC injection group and interrupt to handle over current and voltage
- Device controlled USB D+ pull-up

## 1.0.1

- USB debug output disabled inside ISRs
- Improved display performance by removing the big endian conversion before transferring data via DMA
- Added support for USB Device Id as serial port to motor_config.py

## 1.0.0

- Initial version
