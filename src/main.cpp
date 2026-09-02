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

/**
 * @brief Initialize interrupt handlers
 *
 */
static inline void EXTI_Init()
{
    // Route EXTI8-EXTI11 to GPIO port D
    AFIO->EXTICR[2] =
        AFIO_EXTICR3_EXTI8_PD  |    // EXTI8  PD8
        AFIO_EXTICR3_EXTI9_PD  |    // EXTI9  PD9
        AFIO_EXTICR3_EXTI10_PD |    // EXTI10 PD10
        AFIO_EXTICR3_EXTI11_PD;     // EXTI11 PD11

    // Route EXTI14 to GPIO port B
    AFIO->EXTICR[3] =
        AFIO_EXTICR4_EXTI14_PB;     // EXTI14 PB14

    // Clear pending flags
    EXTI->PR =
        BTN_1_Pin       |   // PD8  BTN_1
        BTN_2_Pin       |   // PD9  BTN_2
        BTN_3_Pin       |   // PD10 BTN_3
        DRV_SNSOUT_Pin  |   // PD11 DRV_SNSOUT
        DRV_FAULT_Pin;      // PB14 DRV_FAULT

    // Enable interrupt lines
    EXTI->IMR |=
        BTN_1_Pin       |   // PD8  BTN_1
        BTN_2_Pin       |   // PD9  BTN_2
        BTN_3_Pin       |   // PD10 BTN_3
        DRV_SNSOUT_Pin  |   // PD11 DRV_SNSOUT
        DRV_FAULT_Pin;      // PB14 DRV_FAULT

    // Rising edge: button change interrupt
    EXTI->RTSR |=
        BTN_1_Pin       |   // PD8  BTN_1
        BTN_2_Pin       |   // PD9  BTN_2
        BTN_3_Pin       |   // PD10 BTN_3
        DRV_SNSOUT_Pin  |   // PD11 DRV_SNSOUT
        DRV_FAULT_Pin;      // PB14 DRV_FAULT

    // Falling edge: button change + fault inputs
    EXTI->FTSR |=
        BTN_1_Pin       |   // PD8  BTN_1
        BTN_2_Pin       |   // PD9  BTN_2
        BTN_3_Pin       |   // PD10 BTN_3
        DRV_SNSOUT_Pin  |   // PD11 DRV_SNSOUT
        DRV_FAULT_Pin;      // PB14 DRV_FAULT

    // Enable NVIC
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(TIM6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
}

/**
 * @brief Disable PID, PWM and invoke system reset
 *
 */
static inline void invoke_system_reset()
{
    pid.running = false;
    PID_WRITE_MOTOR_PWM_OFF();
    HAL_NVIC_SystemReset();
}

/**
 * @brief Core setup
 *
 */
static inline void setup()
{
    // Initialize and read EEPROM on I2C1 on PB8/PB9
    eeprom.init();
    eeprom.read();

    // motor encoder
    motorEncoder.init();

    // buttons
    knobButton.init();
    backButton.init(nullptr, [](uint32_t duration) {
        // do hard reset after holding the back button
        if (duration >= UIConstants::kResetFirmwareLongPressTimeout) {
            invoke_system_reset();
        }
    });
    startButton.init();

    // ADC with DMA
    adc.init();
    // PID controller
    pid.init();
    // apply parameters
    pid.applyPIDParams();

    // Initialize display GPIO and SPI
    tft_driver_gpio_init();
    tft_driver_spi_init();
}

/**
 * @brief User setup
 *
 */
static inline void user_setup()
{
    // Initialize display driver
    tft_driver_init();
    tft_clear_display();

    // start the watchdog after startup is complete
    WatchDog::init();
    MX_WWDG_Init();

    // Initialize LVGL and register flush callback
    lv_init();
    #if LV_USE_LOG
        lv_log_register_print_cb(lvgl_log_cb);
    #endif
    tft_driver_lvgl_init();

    // Show welcome screen and load main menu
    menu.loadWelcomeScreen();
    // Apply settings after welcome screen since it turns the backlight on
    menu.applyEEPROMSettings();

    // program MT6701 PPR via I2C
    if constexpr (PidController::kProgramPPR) {
        motorEncoder.programPPR(PidController::kPPR);
    }

    menu.loadStartScreen();
}

#if HAVE_USB_DEVICE

/**
 * @brief Handle incoming serial data
 *
 */
static inline void handle_serial_data(Serial::BinaryResult result, const char *buf)
{
    switch(result.type) {
        case Serial::BinaryType::REQUEST_SCREENSHOT: {
                DEBUG_PRINT(DebugType::INFO, "Serial: screenshot requested");
                SWO::data.sendScreenshot = true;
            }
            break;
        case Serial::BinaryType::TOGGLE_PID: {
                const uint32_t value = *reinterpret_cast<const uint32_t *>(buf);
                SWO::data.enabled = value ? SWO::EnableState::SERIAL : SWO::EnableState::DISABLED;
                DEBUG_PRINT(DebugType::INFO, "Serial: PID tuning %s", value ? "enabled" : "disabled");
                if (value) {
                    const auto params = pid.getPidParameters();
                    Serial::writeBinary(Serial::BinaryType::PARAMETERS, &params, sizeof(params));
                }
            }
            break;
        case Serial::BinaryType::REQUEST_PARAMETERS: {
                DEBUG_PRINT(DebugType::INFO, "Serial: PID parameters requested");
                const auto params = pid.getPidParameters();
                Serial::writeBinary(Serial::BinaryType::PARAMETERS, &params, sizeof(params));
            }
            break;
        case Serial::BinaryType::PARAMETERS: {
                DEBUG_PRINT(DebugType::INFO, "Serial: PID parameters received");
                pid.setPidParameters(*reinterpret_cast<const PidController::PidParameters *>(buf));
            }
            break;
        case Serial::BinaryType::REQUEST_EEPROM: {
                DEBUG_PRINT(DebugType::INFO, "Serial: EEPROM requested");
                const EEPROM::Data &data = eeprom.getData();
                Serial::writeBinary(Serial::BinaryType::EEPROM, &data, sizeof(data));
            }
            break;
        case Serial::BinaryType::EEPROM: {
                eeprom.getData() = *reinterpret_cast<const EEPROM::Data *>(buf);
                const bool result = eeprom.write();
                (void)result;
                DEBUG_PRINT(DebugType::INFO, "Serial: EEPROM write=%u", static_cast<unsigned>(result));
                menu.applyEEPROMSettings();
            }
            break;
        case Serial::BinaryType::SYSTEM_RESET: {
                DEBUG_PRINT(DebugType::INFO, "Serial: system reset requested");
                invoke_system_reset();
            }
            break;
        default:
            DEBUG_PRINT(DebugType::ERROR, "Serial: binary type=%u size=%u", static_cast<unsigned>(result.type), static_cast<unsigned>(result.size));
            break;
    }
}

#endif

/**
 * @brief Main loop function
 *
 */
static inline void loop()
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

    if (LEDs::isAnyLEDOn()) {
        // check if fault/errors have cleared
        if (
            // check for errors set
            !pid.hasErrorCode() &&
            // check for DRV fault
            !pid.faults.drv8701Fault &&
            // check OCP condition
            !pid.ocp.isActive()
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
        const bool result = eeprom.write();
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
    if ((HAL_GetTick() - lastLvHandler) >= 5 || screenshotRequested) {
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

    #if 0
        static uint32_t lastDebugPrint = 0;
        if ((HAL_GetTick() - lastDebugPrint) >= 100U) {
            DEBUG_PRINT(DebugType::INFO, "abc=%u", 123);
            lastDebugPrint = HAL_GetTick();
        }
    #endif

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
            Serial::BinaryResult result = Serial::readBinary(buf, sizeof(buf));
            if (result.size) {
                uint32_t newCrc;
                if ((result.size % sizeof(uint32_t)) != 0) {
                    DEBUG_PRINT(DebugType::ERROR, "Serial: invalid binary type=%u size=%u", static_cast<unsigned>(result.type), static_cast<unsigned>(result.size));
                }
                else if ((newCrc = stm32_CRC(reinterpret_cast<uint32_t *>(buf), result.size)) != result.crc) {
                    DEBUG_PRINT(DebugType::ERROR, "Serial: CRC expected=0x%08X got=0x%08X type=%u size=%u", result.crc, newCrc, static_cast<unsigned>(result.type), static_cast<unsigned>(result.size));
                }
                else {
                    handle_serial_data(result, buf);
                }
            }
        }
    #endif

    if (SWO::data.enabled == SWO::EnableState::SWO) {
        if (SWO::isPortWritable(SWO::kPidPort)) {
            // send PID tuning data
            PidController::PidLoopType item;
            while (pid.pidLoopBuffer.pop(item)) {
                if (!SWO::writeByteFast(SWO::kPidPort, sizeof(item)) || !SWO::writeFast(SWO::kPidPort, item)) {
                    pid.pidLoopBuffer.clear();
                    break;
                }
            }
        }
    }
    #if HAVE_USB_DEVICE
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

/**
 * @brief Main function
 *
 */
int main(void)
{
    // system init
    HAL_Init();
    LEDs::init();
    DWT_Init();
    SystemClock_Config();
    SWO::init();
    MX_CRC_Init();
    MX_TIM1_Init(); // PWM for motor and ADC injection group trigger
    MX_TIM2_Init(); // PWM for TFT backlight and LED
    MX_TIM3_Init(); // rotary encoder
    MX_TIM4_Init(); // mt6701 encoder
    MX_TIM5_Init(); // analog rpm counter
    MX_TIM6_Init(); // PID timer callback
    MX_TIM7_Init(); // microseconds tick timer
    MX_DAC_Init();
    #if HAVE_USB_DEVICE
        MX_USB_DEVICE_Init();
    #endif
    EXTI_Init();
    setup();
    // user init
    user_setup();

    // main loop
    while (1) {
        loop();
    }
}

// EOF
