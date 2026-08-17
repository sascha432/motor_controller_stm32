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
#include "tft_driver_screenshot.h"
#include "ui.h"
#include "menu.h"
#include "eeprom.h"
#include "stats.h"
#include "helpers.h"
#include "serial.h"
#include "debug.h"

#if LV_USE_LOG
static void lvgl_log_cb(const char *buf)
{
    DEBUG_PRINT_MSG(DebugType::LVGL, "%s", buf ? buf : "<null>");
}
#endif

// === core setup ===

static void setup()
{
    // Initialize and read EEPROM on I2C1 on PB8/PB9
    eeprom.init();
    eeprom.read();

    // motor encoder
    motorEncoder.init();

    // buttons
    knobButton.init();
    backButton.init(nullptr, [](uint32_t duration) {
        // do hard reset after holding the back button for 5 seconds
        if (duration >= 5000) {
            pid.running = false;
            PID_WRITE_MOTOR_PWM_OFF();
            NVIC_SystemReset();
        }
    });
    startButton.init();

    // rotary encoder knob
    knob.init();

    // ADC with DMA
    adc.init();
    // DAC
    adc.initDAC();
    // PID controller
    pid.init();

    // Initialize display GPIO, PWM timer and SPI
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
    #if LV_USE_LOG
        lv_log_register_print_cb(lvgl_log_cb);
    #endif
    tft_driver_lvgl_init();

    // start the watchdog after startup is complete
    WatchDog::init();

    // Show welcome screen and load main menu
    menu.loadWelcomeScreen();
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
    uint32_t dur;
    if ((dur = knobButton.isReleased()) != 0) {
        menu.handleButtonPress(dur);
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

    #if HAVE_SCREENSHOTS
        bool screenshotRequested = false;
        // check if a screenshot was requested
        if (SWO::data.sendScreenshot) {
            SWO::data.sendScreenshot = false;
            screenshotRequested = screenshot.begin();
            if (screenshotRequested) {
                DEBUG_PRINT(DebugType::INFO, "Screenshot started");
                lv_obj_invalidate(lv_scr_act());
            }
            else {
                DEBUG_PRINT(DebugType::INFO, "Screenshot failed to start");
            }
        }
    #else
        constexpr bool screenshotRequested = false;
    #endif

    // handle ui updates and rotary encoder
    static uint32_t lastLvHandler = 0;
    if ((HAL_GetTick() - lastLvHandler) >= 5 || screenshotRequested)
    {
        // handle rotary encoder
        int32_t delta = knob.getPositionDelta();
        if (delta) {
            int32_t newPosition = menu.updateRotaryValue(delta);
            (void)newPosition;
            DEBUG_PRINT(DebugType::UI, "main() menu=%d rotary_delta=%d", newPosition, delta);
        }
        // handle LVGL updates
        ScreenFlow &screenFlow = menu.getScreenFlow();
        switch(screenFlow->getId()) {
            case Screen::Type::DASHBOARD:
                if (pid.hasErrorCode()) {
                    // return to start screen
                    menu.loadStartScreen();
                    break;
                }
                // fallthrough
            case Screen::Type::START:
            case Screen::Type::DIAGNOSTICS:
                stats.update();
                screenFlow->update();
                break;
            default:
                break;
        }

        // update UI, this might take a couple 100ms
        #if HAVE_DWT_TICK_PROFILER && false
            TickProfiler::start(8);
            screenFlow.refresh();
            TickProfiler::stop();
            DEBUG_PRINT_MSG(DebugType::UI, "%u\n", TickProfiler::getTicks());
        #else
            screenFlow.refresh();
        #endif

        #if HAVE_SCREENSHOTS
            // finalize requested screenshot
            if (screenshotRequested) {
                screenshot.end();
                DEBUG_PRINT(DebugType::INFO, "Screenshot sent");
            }
        #endif
        // DEBUG_PRINT_MSG(DebugType::UI, "lv_timer_handler=%ums\n", HAL_GetTick() - lastLvHandler);
        lastLvHandler = HAL_GetTick();
    }

    #if LV_MEM_DEBUG
        static uint32_t lastMemStats = 0;
        if ((HAL_GetTick() - lastMemStats) >= 1000U) {
            lv_mem_monitor_t monitor;
            lv_mem_monitor(&monitor);
            DEBUG_PRINT(DebugType::MEM,
                "LVGL heap total=%lu free=%lu used=%u%% frag=%u%% largest=%lu max=%lu allocs=%lu free_cnt=%lu",
                static_cast<unsigned long>(monitor.total_size),
                static_cast<unsigned long>(monitor.free_size),
                static_cast<unsigned>(monitor.used_pct),
                static_cast<unsigned>(monitor.frag_pct),
                static_cast<unsigned long>(monitor.free_biggest_size),
                static_cast<unsigned long>(monitor.max_used),
                static_cast<unsigned long>(monitor.used_cnt),
                static_cast<unsigned long>(monitor.free_cnt)
            );
            lastMemStats = HAL_GetTick();
        }
    #endif

    #if HAVE_USB_DEVICE
        // handle USB CDC data
        if (Serial::isConnected()) {
            char buf[128]; // EEPROM::Data is 60 byte, the header is 12 byte + a lot extra space
            Serial::BinaryType type;
            uint32_t crc;

            size_t result = Serial::readBinary(buf, sizeof(buf), type, crc);
            if (result) {
                if (result % sizeof(uint32_t) != 0) {
                    DEBUG_PRINT(DebugType::ERROR, "Serial: invalid binary size=%u type=%u", result, static_cast<uint32_t>(type));
                }
                else {
                    uint32_t newCrc = stm32_CRC(buf, result);
                    if (newCrc != crc) {
                        DEBUG_PRINT(DebugType::ERROR, "Serial: CRC mismatch type=%u size=%u got=0x%08X expected=0x%08X", static_cast<uint32_t>(type), result, crc, newCrc);
                    }
                    else {
                        switch(type) {
                            case Serial::BinaryType::REQUEST_SCREENSHOT: {
                                    DEBUG_PRINT(DebugType::INFO, "Serial: screenshot requested");
                                    SWO::data.sendScreenshot = true;
                                }
                                break;
                            case Serial::BinaryType::TOGGLE_PID: {
                                    uint32_t value = *reinterpret_cast<uint32_t *>(buf);
                                    SWO::data.enabled = value ? SWO::EnableState::SERIAL : SWO::EnableState::DISABLED;
                                    DEBUG_PRINT(DebugType::INFO, "Serial: PID tuning %s", value ? "enabled" : "disabled");
                                    if (value) {
                                        auto params = pid.getPidParameters();
                                        Serial::writeBinary(Serial::BinaryType::PARAMETERS, &params, sizeof(params));
                                    }
                                }
                                break;
                            case Serial::BinaryType::REQUEST_PARAMETERS: {
                                    DEBUG_PRINT(DebugType::INFO, "Serial: PID parameters requested");
                                    auto params = pid.getPidParameters();
                                    Serial::writeBinary(Serial::BinaryType::PARAMETERS, &params, sizeof(params));
                                }
                                break;
                            case Serial::BinaryType::PARAMETERS: {
                                    DEBUG_PRINT(DebugType::INFO, "Serial: PID parameters received");
                                    pid.setPidParameters(*reinterpret_cast<PidController::PidParameters *>(buf));
                                }
                                break;
                            case Serial::BinaryType::REQUEST_EEPROM: {
                                    DEBUG_PRINT(DebugType::INFO, "Serial: EEPROM requested");
                                    EEPROM::Data &data = eeprom.getData();
                                    Serial::writeBinary(Serial::BinaryType::EEPROM, &data, sizeof(data));
                                }
                                break;
                            case Serial::BinaryType::EEPROM: {
                                    eeprom.getData() = *reinterpret_cast<EEPROM::Data *>(buf);
                                    bool result = eeprom.write();
                                    (void)result;
                                    DEBUG_PRINT(DebugType::INFO, "Serial: EEPROM write=%u", result);
                                    menu.applyEEPROMSettings();
                                }
                                break;
                            case Serial::BinaryType::SYSTEM_RESET: {
                                    DEBUG_PRINT(DebugType::INFO, "Serial: system reset requested");
                                    NVIC_SystemReset();
                                }
                                break;
                            default:
                                DEBUG_PRINT(DebugType::ERROR, "Serial: binary type=%u size=%u", static_cast<uint32_t>(type), result);
                                break;
                        }
                    }
                }
            }
        }
    #endif

    if (SWO::data.enabled == SWO::EnableState::SWO) {
        if (SWO::isPortWritable(1)) {
            // send PID tuning data
            PidController::PidLoopType item;
            while (pid.pidLoopBuffer.pop(item)) {
                if (!SWO::writeByteFast(1, sizeof(item)) || !SWO::writeFast(1, item)) {
                    pid.pidLoopBuffer.clear();
                    break;
                }
            }
        }
    }
    #if HAVE_USB_DEVICE || HAVE_SERIAL
    else if (SWO::data.enabled == SWO::EnableState::SERIAL) {
        // send PID tuning data
        PidController::PidLoopType item;
        while (pid.pidLoopBuffer.pop(item)) {
            if (Serial::writeBinary(Serial::BinaryType::PID, &item, sizeof(item)) != sizeof(item)) {
                pid.pidLoopBuffer.clear();
                break;
            }
        }
    }
    #endif
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

TIM_HandleTypeDef tim6;
TIM_HandleTypeDef tim7;

static void TIM7_TIM6_Init()
{
    // TIM7 for microsecond delay
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
    tim6.Init.Period = PidController::kOcpTickInterval - 1; // 20 counts = 20us
    tim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_TIM_Base_Init(&tim6);
    HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
    HAL_TIM_Base_Start_IT(&tim6);
}

CRC_HandleTypeDef hcrc;

void MX_CRC_Init(void)
{
    __HAL_RCC_CRC_CLK_ENABLE();

    hcrc.Instance = CRC;

    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }
}

// === DWT cycle counter ===

static inline void DWT_Init(void)
{
    // Enable trace
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Reset cycle counter
    DWT->CYCCNT = 0;
    // Enable cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
    LEDs::init();
    DWT_Init();
    SystemClock_Config();
    SWO::init();
    debug_init();
    MX_CRC_Init();
    TIM7_TIM6_Init();
    #if HAVE_USB_DEVICE
        MX_USB_DEVICE_Init();
        #if 0
        // test USB stack
        __HAL_RCC_GPIOx_CLK_ENABLE<PC13>();
        // PC13: MODE=10 (2MHz), CNF=00 (push-pull output)
        GPIO_CRx_REG<PC13>() &= ~(0xF << digitalPinShift<PC13>());
        GPIO_CRx_REG<PC13>() |= (0x2 << digitalPinShift<PC13>());
        digitalWriteLow<PC13>();
        bool pc13State = false;
        for(;;) {
            int ch = USBSerial::read();
            if (ch != -1) {
                DEBUG_PRINT(DebugType::INFO, "USB read=%d ('%c')", ch, ch);
            }
            DEBUG_PRINT(DebugType::INFO, "USB time=%u connected=%u", (unsigned)HAL_GetTick(), (unsigned)USBSerial::isConnected());
            char buf[32];
            snprintf(buf, sizeof(buf), "time=%u\r\n", (unsigned)HAL_GetTick());
            USBSerial::write(buf, strlen(buf));
            pc13State = !pc13State;
            if (pc13State) {
                digitalWriteHigh<PC13>();
            }
            else {
                digitalWriteLow<PC13>();
            }
            WatchDog::delay(500);
        }
        #endif
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
