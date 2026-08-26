/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#ifndef PID_INTERVAL
// PID interval in milliseconds
#define PID_INTERVAL                                2.0f
#endif

#define VERSION_MAJOR                               1
#define VERSION_MINOR                               0
#define VERSION_PATCH                               2
#define VERSION_FULL                                ((VERSION_MAJOR << 16) | (VERSION_MINOR << 8) | (VERSION_PATCH))

#ifndef PCB_REV_MAJOR
#define PCB_REV_MAJOR                               1
#endif
#ifndef PCB_REV_MINOR
#define PCB_REV_MINOR                               2
#endif
#define PCB_REV_FULL                                (PCB_REV_MAJOR << 8) | (PCB_REV_MINOR)

#ifndef F_CPU
#define F_CPU                                       72000000UL
#endif

// PIN macros for the TFT driver
#define TFT_PIN_RS_HIGH()                           (TFT_DC_GPIO_Port->BSRR = TFT_DC_Pin)
#define TFT_PIN_RS_LOW()                            (TFT_DC_GPIO_Port->BRR = TFT_DC_Pin)
#define TFT_PIN_CS_HIGH()                           (TFT_CS_GPIO_Port->BSRR = TFT_CS_Pin)
#define TFT_PIN_CS_LOW()                            (TFT_CS_GPIO_Port->BRR = TFT_CS_Pin)
#define TFT_PIN_RST_HIGH()                          (TFT_RST_GPIO_Port->BSRR = TFT_RST_Pin)
#define TFT_PIN_RST_LOW()                           (TFT_RST_GPIO_Port->BRR = TFT_RST_Pin)

// motor driver pins are connected to channels 1 and 2 of TIM1 (PA8 and PA9)
#define PID_MOTOR_PWM_TIMER                         TIM1
#define PID_WRITE_MOTOR_PWM_DRV_IN1(level)          (PID_MOTOR_PWM_TIMER->CCR1 = (level))
#define PID_WRITE_MOTOR_PWM_DRV_IN2(level)          (PID_MOTOR_PWM_TIMER->CCR2 = (level))
#define PID_READ_MOTOR_PWM_DRV_IN1()                ((uint16_t)PID_MOTOR_PWM_TIMER->CCR1)
#define PID_READ_MOTOR_PWM_DRV_IN2()                ((uint16_t)PID_MOTOR_PWM_TIMER->CCR2)
#define PID_WRITE_MOTOR_PWM_FORWARD(level)          (PID_MOTOR_PWM_TIMER->CCR1 = (level), PID_MOTOR_PWM_TIMER->CCR2 = 0)
#define PID_WRITE_MOTOR_PWM_REVERSE(level)          (PID_MOTOR_PWM_TIMER->CCR1 = 0, PID_MOTOR_PWM_TIMER->CCR2 = (level))
#define PID_WRITE_MOTOR_PWM_ON(level, dir)          ((dir == EEPROM::MotorDirection::Reverse) ? PID_WRITE_MOTOR_PWM_REVERSE(level) : PID_WRITE_MOTOR_PWM_FORWARD(level))
#define PID_WRITE_MOTOR_PWM_OFF()                   (PID_MOTOR_PWM_TIMER->CCR1 = 0, PID_MOTOR_PWM_TIMER->CCR2 = 0)
#define PID_WRITE_MOTOR_PWM_BRAKE(level)            (PID_MOTOR_PWM_TIMER->CCR1 = (level), PID_MOTOR_PWM_TIMER->CCR2 = (level))

// DAC macros for DRV8701 and INA381 overcurrent protection
#define DAC_SET_MOTOR_CURRENT(value)                (DAC->DHR12R1 = (value) & 0x0fffU)
#define DAC_SET_INPUT_CURRENT(value)                (DAC->DHR12R2 = (value) & 0x0fffU)
#define DAC_GET_MOTOR_CURRENT()                     ((uint16_t)DAC->DHR12R1) // DRVOCP_VREF_DAC_PIN/PA4
#define DAC_GET_INPUT_CURRENT()                     ((uint16_t)DAC->DHR12R2) // OCP_VREF_DAC_PIN/PA5

// macros to read encoder pulse counter and rpm counter
#define PID_READ_ENCODER_COUNTER()                  ((uint16_t)TIM4->CNT) // QDEC on MT6701_I2C_SCL_PIN/MT6701_I2C_SDA_PIN
#define PID_READ_RPM_COUNTER()                      ((uint16_t)TIM5->CNT) // falling edge counter on ENC1_ANALOG_PIN

// rotary encoder macros
#define UI_READ_ROTARY_KNOB_COUNTER()               ((uint16_t)TIM3->CNT) // rotary encoder on ROTARY_ENCODER_PIN_A/ROTARY_ENCODER_PIN_B
#define UI_WRITE_ROTARY_KNOB_COUNTER(value)         (TIM3->CNT = (value))

// PWM for the LED CC driver
#define UI_ILLUMINATION_LED_SET_PWM(value)          (TIM2->CCR3 = (value)) // ILLUMINATION_LED_PIN

// PWM for the TFT backlight
#define UI_TFT_BACKLIGHT_SET_PWM(value)             (TIM2->CCR4 = (value)) // TFT_PIN_LED

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENC1_ANALOG_Pin GPIO_PIN_1
#define ENC1_ANALOG_GPIO_Port GPIOA
#define ISENSE_Pin GPIO_PIN_2
#define ISENSE_GPIO_Port GPIOA
#define VSENSE_Pin GPIO_PIN_3
#define VSENSE_GPIO_Port GPIOA
#define DRVOCP_VREF_Pin GPIO_PIN_4
#define DRVOCP_VREF_GPIO_Port GPIOA
#define OCP_VREF_Pin GPIO_PIN_5
#define OCP_VREF_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_6
#define ENC2_A_GPIO_Port GPIOA
#define ENC2_B_Pin GPIO_PIN_7
#define ENC2_B_GPIO_Port GPIOA
#define EXT_NTC_Pin GPIO_PIN_4
#define EXT_NTC_GPIO_Port GPIOC
#define DRV_TEMP_Pin GPIO_PIN_5
#define DRV_TEMP_GPIO_Port GPIOC
#define LED_PWM_Pin GPIO_PIN_10
#define LED_PWM_GPIO_Port GPIOB
#define TFT_PWM_Pin GPIO_PIN_11
#define TFT_PWM_GPIO_Port GPIOB
#define OCP_INT_Pin GPIO_PIN_12
#define OCP_INT_GPIO_Port GPIOB
#define OCP_INT_EXTI_IRQn EXTI15_10_IRQn
#define TFT_CLK_Pin GPIO_PIN_13
#define TFT_CLK_GPIO_Port GPIOB
#define DRV_FAULT_Pin GPIO_PIN_14
#define DRV_FAULT_GPIO_Port GPIOB
#define DRV_FAULT_EXTI_IRQn EXTI15_10_IRQn
#define TFT_MOSI_Pin GPIO_PIN_15
#define TFT_MOSI_GPIO_Port GPIOB
#define BTN_1_Pin GPIO_PIN_8
#define BTN_1_GPIO_Port GPIOD
#define BTN_1_EXTI_IRQn EXTI9_5_IRQn
#define BTN_2_Pin GPIO_PIN_9
#define BTN_2_GPIO_Port GPIOD
#define BTN_2_EXTI_IRQn EXTI9_5_IRQn
#define BTN_3_Pin GPIO_PIN_10
#define BTN_3_GPIO_Port GPIOD
#define BTN_3_EXTI_IRQn EXTI15_10_IRQn
#define DRV_SNSOUT_Pin GPIO_PIN_11
#define DRV_SNSOUT_GPIO_Port GPIOD
#define DRV_SNSOUT_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_LEDS_Pin GPIO_PIN_12
#define MOTOR_LEDS_GPIO_Port GPIOD
#define TFT_CS_Pin GPIO_PIN_15
#define TFT_CS_GPIO_Port GPIOD
#define TFT_RST_Pin GPIO_PIN_6
#define TFT_RST_GPIO_Port GPIOC
#define TFT_DC_Pin GPIO_PIN_7
#define TFT_DC_GPIO_Port GPIOC
#define DRV_IN1_Pin GPIO_PIN_8
#define DRV_IN1_GPIO_Port GPIOA
#define DRV_IN2_Pin GPIO_PIN_9
#define DRV_IN2_GPIO_Port GPIOA
#define USB_DN_Pin GPIO_PIN_11
#define USB_DN_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PAD_TX4_Pin GPIO_PIN_10
#define PAD_TX4_GPIO_Port GPIOC
#define PAD_RX4_Pin GPIO_PIN_11
#define PAD_RX4_GPIO_Port GPIOC
#define ENC1_I2C_EN_Pin GPIO_PIN_7
#define ENC1_I2C_EN_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENC1_SCL_Pin GPIO_PIN_6
#define ENC1_SCL_GPIO_Port GPIOB
#define ENC1_SDA_Pin GPIO_PIN_7
#define ENC1_SDA_GPIO_Port GPIOB
#define SCL1_Pin GPIO_PIN_8
#define SCL1_GPIO_Port GPIOB
#define SDA1_Pin GPIO_PIN_9
#define SDA1_GPIO_Port GPIOB
#define USB_PU_EN_Pin GPIO_PIN_1
#define USB_PU_EN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
