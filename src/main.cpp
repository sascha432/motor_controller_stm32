/**
  Author: sascha_lammers@gmx.de
*/

#include "i2c.h"
#include "controls.h"
#include "mt6701_encoder.h"
#include "pid_controller.h"
#include "adc.h"
#include "leds.h"
#include "tft_driver.h"
#include "ui.h"
#include "menu.h"
#include "eeprom.h"
#include "stats.h"
#include "debug.h"

// === core setup ===

static void setup()
{
    // Initialize debug output
    debug_init();

    // Initialize and read EEPROM on I2C1 on PB8/9
    eeprom.init();
    eeprom.read();

    // LEDs
    LEDs::init();

    // motor encoder
    motorEncoder.init();

    // buttons
    knobButton.init();
    backButton.init();
    startButton.init();

    // rotary encoder knob
    knob.init();

    // ADC with DMA
    adc.init();
    // DAC
    adc.initDAC();
    // PID controller
    pid.init();

    // Initialize display gpio and SPI
    tft_driver_gpio_init();
    tft_driver_spi_init();
}

// === user setup runs after core setup ===

static void user_setup() 
{
    // Initialize display driver
    tft_driver_init();
    tft_clear_display();

    // Initialize LVGL and register flush callback
    lv_init();
    tft_driver_lvgl_init();

    // Show welcome screen and load main menu
    menu.showWelcomeScreen();
    // Apply settings after welcome screen since it turns the backlight on
    menu.applyEEPROMSettings();

    // program MT6701 PPR via I2C
    if (PidController::kProgramPPR) {
        motorEncoder.programPPR(i2c, PidController::kPPR);
    }

    menu.loadStartScreen();
}

// === main loop ===

static void loop()
{
    // handle buttons
    if (knobButton.isPressed()) {
        menu.handleButtonPress();
    }
    if (backButton.isPressed()) {
        menu.handleBackButtonPress();
    }
    if (startButton.isPressed()) {
        menu.handleStartButtonPress();
    }

    if (pid.faults.count) {
        static uint32_t lastFaultTime = 0;
        if (HAL_GetTick() - lastFaultTime >= 500) {
            lastFaultTime = HAL_GetTick();
            pid.faults.reset();
            LEDs::offLED1and2();
        }
        else {
            if (pid.faults.ocpFault) {
                LEDs::onLED1();
            }
            if (pid.faults.snsoutFault) {
                LEDs::onLED2();
            }
        }
    }

    // handle ui updates and rotary encoder
    static uint32_t lastLvHandler = 0;
    if (HAL_GetTick() - lastLvHandler >= 5) {
        // handle rotary encoder
        int32_t newPosition;
        int32_t delta = knob.getDeltaPosition();
        if (delta) {
            newPosition = menu.updateRotaryValue(delta);
            DEBUG_PRINT(DEBUG_DEBUG, "menu=%d delta=%d", newPosition, delta);
        }
        // handle LVGL updates
        auto &screenFlow = menu.getScreenFlow();
        switch(screenFlow->getId()) {
            case Screen::Type::START:
            case Screen::Type::DASHBOARD:
            case Screen::Type::DIAGNOSTICS:
                stats.update();
                screenFlow->update();
                break;
            default:
                break;
        }
        // check NTC sensors, not time critical and a couple times per seconds is enough
        if (adc.getMotorNTCValue() < eeprom.getMotorTemperatureLimitADC()) {
            if (pid.running) {
                pid.setErrorCode(PidController::ErrorCodeType::MOTOR_OVER_TEMPERATURE);
            }
        }
        if (adc.getMosfetNTCValue() < eeprom.getMosfetTemperatureLimitADC()) {
            if (pid.running) {
                pid.setErrorCode(PidController::ErrorCodeType::MOSFET_OVER_TEMPERATURE);
            }
        }
        
        // update UI
        lv_timer_handler();
        lastLvHandler = HAL_GetTick();
    }

    if (pid.running && eeprom.getPidTuning() != EEPROM::kPidTuningDisabled) {
        // send PID tuning data
        PidController::PidLoopType item;
        while (pid.pidLoopBuffer.pop(item)) {
            DEBUG_PRINT_MSG(DEBUG_DEBUG, "pid_seq=%u rpm=%u pwm=%u U=%u Io=%u I=%u motor=%u mosfet=%u faults=%u drv_fault=%d ocp=%d snsout=%d",
                item.sequence,
                item.rpm,
                item.pwmLevel,
                item.voltage,
                ADCConverter::Current::convert(item.currentOcp),
                ADCConverter::Current::convert(item.currentAverage),
                item.motorTemperature,
                item.mosfetTemperature,
                item.errorCount,
                item.drv8701Fault ? 1 : 0,
                item.ocpFault ? 1 : 0,
                item.snsoutFault ? 1 : 0
            );
        }
    }

    if (false) {
        static uint32_t lastTime37 = 0;
        if (HAL_GetTick() - lastTime37 >= 100) {
            lastTime37 = HAL_GetTick();
            // extern volatile uint32_t rpm_counter;
            // DEBUG_PRINT(DEBUG_DEBUG, "RPM_COUNTER=%u", rpm_counter);
            // DEBUG_PRINT(DEBUG_DEBUG, "TIM5=%u", TIM5->CNT);
        }
    }

    if (false) { // print faults
        static uint32_t lastTime3 = 0;
        if (HAL_GetTick() - lastTime3 >= 1000) {
            lastTime3 = HAL_GetTick();
            static uint32_t lastCounter = 0;
            if (pid.faults.count != lastCounter) {
                lastCounter = pid.faults.count;
                pid.debugPrintFaults();
            }
        }
    }
}

// === interrupt handlers ===

static TIM_HandleTypeDef tim6;

extern "C" void TIM6_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&tim6);
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) { // every 5ms
        static uint32_t timer6Counter = 0;
        if (++timer6Counter >= 5) { 
            timer6Counter = 0;
            knob.isr(); // every 25ms
        }
        pid.isr();
    }
}

extern "C" void EXTI9_5_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1 << 8) | (1 << 9));
    EXTI->PR = pending; // clear flags
    if (pending & (1 << 8)) {
        // KNOB_BUTTON_PIN/PD8 changed
        knobButton.isr(GPIOD->IDR);
    }
    if (pending & (1 << 9)) {
        // BACK_BUTTON_PIN/PD9 changed
        backButton.isr(GPIOD->IDR);
    }
}

extern "C" void EXTI15_10_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1 << 10) | (1 << 11) | (1 << 12) | (1 << 14));
    EXTI->PR = pending; // clear flags
    if (pending & (1 << 10)) {
        // START_BUTTON_PIN/PD10 changed
        startButton.isr(GPIOD->IDR);
    }
    if (pending & (1 << 11)) {
        // DRV_SNSOUT_PIN/PD11 changed
        pid.faults.snsoutFault = true;
        pid.faults.count++;
    }
    if (pending & (1 << 12)) {
        // OCP_INT_PIN/PB12 changed
        if (adc.getISenseOcpAverageValue() > pid.faults.isenseMax) {
            pid.faults.ocpFault = true;
            pid.faults.count++;
            // disable PWM until the PID loop turns it on again
            PID_WRITE_MOTOR_PWM_OFF();
        }
    }
    if (pending & (1 << 14)) {
        // DRV8701_FAULT_PIN/PB14 changed
        pid.faults.drv8701Fault = true;
        pid.faults.count++;
    }
}

// === interrupt handlers initialization ===

static void EXTI_Init()
{
    // EXTI8-11 -> Port D
    AFIO->EXTICR[2] =
        (0x3 << 0) |    // EXTI8 PD8
        (0x3 << 4) |    // EXTI9 PD9
        (0x3 << 8) |    // EXTI10 PD10
        (0x3 << 12);    // EXTI11 PD11

    // EXTI12, EXTI14 -> Port B
    AFIO->EXTICR[3] =
        (0x1 << 0) |    // EXTI12 PB12
        (0x1 << 8);     // EXTI14 PB14

    // Clear pending flags
    EXTI->PR =  
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10) |   // PD10 BTN_3
        (1U<<11) |   // PD11 DRV_SNSOUT
        (1U<<12) |   // PB12 OCP_INT
        (1U<<14);    // PB14 DRV_FAULT

    // Enable interrupt lines
    EXTI->IMR |= 
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10) |   // PD10 BTN_3
        (1U<<11) |   // PD11 DRV_SNSOUT
        (1U<<12) |   // PB12 OCP_INT
        (1U<<14);    // PB14 DRV_FAULT

    // Rising edge: button change interrupt
    EXTI->RTSR |= 
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10);    // PD10 BTN_3

    // Falling edge: button change + fault inputs
    EXTI->FTSR |= 
        (1U<<8)  |   // PD8  BTN_1
        (1U<<9)  |   // PD9  BTN_2
        (1U<<10) |   // PD10 BTN_3
        (1U<<11) |   // PD11 DRV_SNSOUT
        (1U<<12) |   // PB12 OCP_INT
        (1U<<14);    // PB14 DRV_FAULT

    // Enable NVIC
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void TIM7_TIM6_Init() 
{
    // TIM7 for microsecond delay
    TIM_HandleTypeDef tim7;
    tim7.Instance = TIM7;
    tim7.Init.Prescaler = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    tim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim7.Init.Period = 0xFFFF;
    tim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    __HAL_RCC_TIM7_CLK_ENABLE();
    HAL_TIM_Base_Init(&tim7);
    HAL_TIM_Base_Start(&tim7);

    // TIM6 for periodic interrupts
    tim6.Instance = TIM6;
    tim6.Init.Prescaler = 71; // 72 MHz / 72 = 1 MHz (1 us tick)
    tim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim6.Init.Period = 5000 - 1; // 5000 counts = 5ms
    tim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_TIM_Base_Init(&tim6);
    HAL_TIM_Base_Start_IT(&tim6);
    HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
   
}

// === interrupt handlers ===

/**
  * @brief This function handles System tick timer.
  */
extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
extern "C" void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    PID_WRITE_MOTOR_PWM_OFF();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

/**
  * @brief This function handles Non maskable interrupt.
  */
extern "C" void NMI_Handler(void)
{
    Error_Handler();
}

/**
  * @brief This function handles Hard fault interrupt.
  */
extern "C" void HardFault_Handler(void)
{
    Error_Handler();
}

/**
  * @brief This function handles Memory management fault.
  */
extern "C" void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
extern "C" void BusFault_Handler(void)
{
    Error_Handler();
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
extern "C" void UsageFault_Handler(void)
{
    Error_Handler();
}

// === core clock configuration ===

/**
  * @brief System Clock Configuration
  * @retval None
  */
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

// === USB initialization ===

#define INIT_USB 0

#if INIT_USB

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief PCD MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
extern "C" void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB_OTG_FS)
  {
    /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

    /* USER CODE END USB_OTG_FS_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    /* USB_OTG_FS interrupt Init */
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

    /* USER CODE END USB_OTG_FS_MspInit 1 */

  }

}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
extern "C" void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

#endif

// === main ===

int main(void)
{
    // system init
    HAL_Init();
    SystemClock_Config();
    TIM7_TIM6_Init();
    #if INIT_USB
        MX_USB_OTG_FS_PCD_Init();
    #endif
    setup();
    EXTI_Init();

    // user init
    user_setup();

    // main loop
    while (1) {
        loop();
    }
}

// EOF
