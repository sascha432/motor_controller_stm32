<style>
table {
	width: 100%;
	table-layout: fixed;
}
th:nth-child(1), td:nth-child(1) { width: 26%; }
th:nth-child(2), td:nth-child(2) { width: 14%; }
th:nth-child(3), td:nth-child(3) { width: 14%; }
th:nth-child(4), td:nth-child(4) { width: 46%; }
</style>

# STM32F107VCT6 - pin assignments

# Revision: PCB Rev1.0 and Rev1.1

### DAC output

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| DRVOCP_VREF | PA4 | DAC | DAC1_OUT1 |
| OCP_VREF | PA5 | DAC | DAC1_OUT2 |

### ADC input

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| EXT_NTC | PC4 | ADC | ADC12_IN14 |
| DRV_TEMP | PC5 | ADC | ADC12_IN15 |
| ISENSE | PA2 | ADC | ADC12_IN2 |
| VSENSE | PA3 | ADC | ADC12_IN3 |
| ENC1_ANALOG | PA1 | ADC | ADC12_IN1/TIM5_CH2 |

### QDEC - rotary encoder ENC2

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| ENC2_A | PA6 | QDEC | TIM3_CH1 |
| ENC2_B | PA7 | QDEC | TIM3_CH2 |

### QDEC - MT6701 magnetic encoder ENC1

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| ENC1_SCL | PB6 | QDEC/I2C | TIM4_CH1 / I2C1_SCL |
| ENC1_SDA | PB7 | QDEC/I2C | TIM4_CH2 / I2C1_SDA |

### PWM output - motor 40 kHz

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| DRV_IN2 | PA9 | PWM | TIM1_CH2 |
| DRV_IN1 | PA8 | PWM | TIM1_CH1 |

### PWM output - 1 kHz

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| LED_PWM | PB10 | PWM | TIM2_CH3 (remap) - CC LED driver |
| TFT_PWM | PB11 | PWM | TIM2_CH4 (remap) - TFT backlight |

### SPI - TFT display

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| TFT_DC | PC7 | DOUT SPI | GPIO data/command |
| TFT_RST | PC6 | DOUT SPI | GPIO reset |
| TFT_CS | PD15 | DOUT SPI | GPIO chip select |
| TFT_MOSI | PB15 | SPI2 | SPI2_MOSI |
| TFT_CLK | PB13 | SPI2 | SPI2_SCK |

### I2C

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| SDA1 | PB9 | I2C1 | I2C1_SDA (remap) - 5.1 kOhm pull-up |
| SCL1 | PB8 | I2C1 | I2C1_SCL (remap) - 5.1 kOhm pull-up |

### USB

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| USB_DP | PA12 | USB | USB FS D+ |
| USB_DN | PA11 | USB | USB FS D- |

### Debug / programming - TagConnect 6-pin

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| SWCLK | PA14 | SWD |  |
| SWDIO | PA13 | SWD |  |
| SWO | PB3 | SWD |  |

### Digital output

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| ENC1_I2C_EN | PD7 | DOUT | Enables MT6701 I2C mode |
| OCP_LED | PD12 | DOUT | 220 Ohm series resistor |

### Digital input - fast interrupt

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| DRV_SNSOUT | PD11 | DIN IRQ | Current sense output |
| DRV_FAULT# | PB14 | DIN IRQ | Active-low - DRV8701 fault |
| OCP_INT# | PB12 | DIN IRQ | Active-low - INA381 ALERT# |

## Digital input - interrupt

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| BTN_3 | PD10 | DIN IRQ | Start Button, 100 nF debounce cap |
| BTN_2 | PD9 | DIN IRQ | Back Button, 100 nF debounce cap |
| BTN_1 | PD8 | DIN IRQ | Rotary Knob, 100 nF debounce cap |

## Revision: PCB Rev1.1 only

### UART

| Signal | STM32 pin | Type | Notes |
|---|---|---|---|
| TX4 | PC10 | SERIAL | UART4 TX |
| RX4 | PC11 | SERIAL | UART4 RX |
