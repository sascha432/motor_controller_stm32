# DC Motor Controller Firmware for STM32F107/DRV8701

## TODOs

 - Implement USB CDC device
 - PID tuning over USB and/or UART

## Software Features

- Start/dashboard workflow for quick motor start-stop and live runtime status
- Selectable control mode: PWM open-loop or RPM-based PID closed-loop
- On-device tuning for speed, brake strength, and stall timeout
- Configurable protection limits for input current, motor current, MOSFET temperature, and motor temperature
- Advanced setup for RPM range, motor direction, and sensor direction
- PID parameter tuning (Kp, Ki, Kd, anti-windup reduction) with live parameter apply
- Diagnostics screen with firmware/PCB info and live min/max telemetry (VCC, current, temperatures, RPM/PWM)
- EEPROM-backed settings with save/restore-defaults from the menu
- LVGL TFT UI with rotary encoder plus dedicated start/back/knob buttons
- Python script for PID parameter tuning over SWD connection

## Hardware Features

- 8-36V, 20A continuous, 40A peak
- Adjustable cycle by cycle motor current limit (0.5-40A)
- Adjustable current limit with signal LED (0.5-40A)
- 1.14" 135x240 TFT display, rotary encoder and 3 buttons for easy navigation
- Support for magnetic encoder MT6701 in A/B mode up to 1024PPR (55000 RPM)
- Dimmable LED with CC driver 3-26V/350mA/5W
- Current, voltage and temperature monitoring

## Controller schematics and PCB

[Schematics & PCB](https://oshwlab.com/sascha23095123423/project_dzierqaj)

[Schematics PDF](docs/schematic_dc-motor-controller_rev11.pdf)

<a href="docs/schematic_dc-motor-controller_rev11.pdf">
	<img src="docs/schematics_rev11.png" alt="Schematic thumbnail" width="640" />
</a>

## STL files for motor mount and enclosure

[STL files](stl/STL.md)

## Some pictures

![Enclosure and motor](stl/enclosure_and_motor.jpg)

![PCB front](stl/pcb_front.jpg)

![PCB back](stl/pcb_back.jpg)

![STL files and MT6701](stl/stl_files.png)

![PID tuning](stl/pid_tuning.png)
