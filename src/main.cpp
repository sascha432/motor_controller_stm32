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
#include "helpers.h"
#include "debug.h"
#if HAVE_USB_DEVICE
#include <usb_device.h>

static uint32_t irqCount = 0;

static void debug_usb_otg_state(const char *tag)
{
    auto *usb = USB_OTG_FS;
    auto *dev = reinterpret_cast<USB_OTG_DeviceTypeDef *>(reinterpret_cast<uint32_t>(usb) + USB_OTG_DEVICE_BASE);
    uint32_t pcgcctl = *reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uint32_t>(usb) + USB_OTG_PCGCCTL_BASE);

    DEBUG_PRINT_MSG(
        DEBUG_DEBUG,
        "USB[%s] irc#=%u GOTGCTL=%08lx GCCFG=%08lx GAHBCFG=%08lx GUSBCFG=%08lx GINTSTS=%08lx GINTMSK=%08lx GRSTCTL=%08lx DSTS=%08lx DCTL=%08lx PCGCCTL=%08lx",
        tag,
        irqCount,
        static_cast<unsigned long>(usb->GOTGCTL),
        static_cast<unsigned long>(usb->GCCFG),
        static_cast<unsigned long>(usb->GAHBCFG),
        static_cast<unsigned long>(usb->GUSBCFG),
        static_cast<unsigned long>(usb->GINTSTS),
        static_cast<unsigned long>(usb->GINTMSK),
        static_cast<unsigned long>(usb->GRSTCTL),
        static_cast<unsigned long>(dev->DSTS),
        static_cast<unsigned long>(dev->DCTL),
        static_cast<unsigned long>(pcgcctl)
    );
}
#endif

// === core setup ===

static void setup()
{
    // Initialize debug output
    SWO::init();
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
    tft_driver_gpio_tim_init();
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

    // start the watchdog after startup is complete
    WatchDog::init();

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
    // feed the dog
    WatchDog::feed();

    // handle buttons
    if (knobButton.isReleased()) {
        menu.handleButtonPress();
    }
    if (backButton.isReleased()) {
        menu.handleBackButtonPress();
    }
    if (startButton.isReleased()) {
        menu.handleStartButtonPress();
    }

    // LED 1 signals fault or error
    if (LEDs::isErrorLEDOn()) {
        // check if fault/errors have cleared
        if (
            !pid.faults.drv8701Fault &&
            (pid.getErrorCode() == PidController::ErrorCodeType::NONE)
        ) {
            // turn LEDs off
            LEDs::off();
        }
    }

    if (pid.running) {
        // check NTC sensors, not time critical and a couple times per seconds is enough
        if (ADCConverter::NTC::compare(adc.getMotorTemperatureFiltered(), eeprom.getMotorTemperatureLimitADC()) > 0) {
            pid.setErrorCode(PidController::ErrorCodeType::MOTOR_OVER_TEMPERATURE);
        }
        else if (ADCConverter::NTC::compare(adc.getMosfetTemperatureFiltered(), eeprom.getMosfetTemperatureLimitADC()) > 0) {
            pid.setErrorCode(PidController::ErrorCodeType::MOSFET_OVER_TEMPERATURE);
        }
    }

    if (SWO::data.EEPROM.commit) {
        // check if the PID tuning requests to save the EEPROM settings
        SWO::data.EEPROM.commit = false;
        bool result = eeprom.write();
        (void)result;
        DEBUG_PRINT(DebugType::INFO, "SWO EEPROM write=%u", result);
        menu.applyEEPROMSettings();
    }

    // handle ui updates and rotary encoder
    static uint32_t lastLvHandler = 0;
    if (HAL_GetTick() - lastLvHandler >= 5) {
        // handle rotary encoder
        int32_t newPosition;
        int32_t delta = knob.getDeltaPosition();
        if (delta) {
            newPosition = menu.updateRotaryValue(delta);
            (void)newPosition;
            DEBUG_PRINT(DebugType::UI, "menu=%d delta=%d", newPosition, delta);
        }
        // handle LVGL updates
        ScreenFlow &screenFlow = menu.getScreenFlow();
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

        // update UI, this might take a couple 100ms
        lv_timer_handler();
        lastLvHandler = HAL_GetTick();
    }

    if (SWO::data.enabled != SWO::EnableState::DISABLED) {
        // send PID tuning data
        PidController::PidLoopType item;
        while (pid.pidLoopBuffer.pop(item)) {
            if (SWO::data.enabled == SWO::EnableState::SWO) {
                static constexpr char kPidFrameMagic[] = {'P', 'I', 'D', '1'};
                if (!SWO::write(1, kPidFrameMagic, sizeof(kPidFrameMagic))) {
                    pid.pidLoopBuffer.clear();
                    break;
                }
                if (!SWO::write(1, item)) {
                    pid.pidLoopBuffer.clear();
                    break;
                }
            }
            else if (SWO::data.enabled == SWO::EnableState::USB) {
                // not implemented yet
            } else if (SWO::data.enabled == SWO::EnableState::SERIAL) {
                // not implemented yet
            }
        }
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
        (1U<<10) |   // PD10 BTN_3
        (1U<<11) |   // PD11 DRV_SNSOUT
        (1U<<14);    // PB14 DRV_FAULT

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

extern TIM_HandleTypeDef tim6;

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
    tim6.Init.Period = 5 - 1; // 5 counts = 5us
    tim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_TIM_Base_Init(&tim6);
    HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
    HAL_TIM_Base_Start_IT(&tim6);
}

#if HAVE_USB_DEVICE

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
extern "C" void OTG_FS_IRQHandler(void)
{
    irqCount++;
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

#endif

#if HAVE_HAL_CRC

CRC_HandleTypeDef hcrc;

void MX_CRC_Init(void)
{
    __HAL_RCC_CRC_CLK_ENABLE();

    hcrc.Instance = CRC;

    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }
}

#endif

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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    /** Configure the Systick interrupt time
     */
    __HAL_RCC_PLLI2S_ENABLE();
}

// === main ===

int main(void)
{
    // system init
    HAL_Init();
    SystemClock_Config();
    #if HAVE_HAL_CRC
        MX_CRC_Init();
    #endif
    TIM7_TIM6_Init();
    #if HAVE_USB_DEVICE
        /*

        AFTER manually disabling vbus sensing in the USB_OTG_FS->GCCFG register, the USB device
        does not detect the cable in one direction, check PCB ... in one direction the cable is detecetd
        and fires 2 interrupts, but does not enumerate.. try different usb hub etc...
        check if tvs diode blocks usb_dp or usb_dm
        check if reverse usb connector pins are not soldered properly = one direction only/could be a bad cable as well

        OUTCOME: the connector had a solder bridge and might have connected vbus to d+ or d- damaging the mcu
        until the MCU is replaced i cannot test this further


        */

        debug_usb_otg_state("pre-init");
        MX_USB_DEVICE_Init();
        /* F107 workaround: disable VBUS sensing when VBUS detect is not wired. */
        hpcd_USB_OTG_FS.Instance->GCCFG &= ~(USB_OTG_GCCFG_VBUSASEN | USB_OTG_GCCFG_VBUSBSEN);
        debug_usb_otg_state("post-init");
        for(;;) {
            debug_usb_otg_state("loop");
            WatchDog::delay(500);
        }
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
