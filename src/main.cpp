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

static void button_isr() 
{
    #if defined(STM32F107xC)
        uint32_t idrD = ((GPIO_TypeDef *)GPIOD_BASE)->IDR;
        knobButton.isr(idrD);
        backButton.isr(idrD);
        startButton.isr(idrD);
    #elif defined(STM32F103xC)
        uint32_t idrB = ((GPIO_TypeDef *)GPIOB_BASE)->IDR;
        startButton.isr(idrB);
        uint32_t idrA = ((GPIO_TypeDef *)GPIOA_BASE)->IDR;
        knobButton.isr(idrA);
        backButton.isr(idrA);
        TODO pin macros need to be  chagned and isr too
    #else
        #error missing ISR implementation for this MCU
    #endif
}

static void knob_isr()
{
    knob.isr();
}

static void pid_timer_isr()
{
    pid.isr();
}

static void pid_fault_isr()
{
    pid.fault_isr();
}

#include "stm32f107xc.h"

void setup()
{
    debug_init();

    // Initialize and read EEPROM on I2C1 on PB8/9
    eeprom.init();
    eeprom.read();

    // LEDs
    LEDs::init();

    // motor encoder
    motorEncoder.init();
    if (PidController::kProgramPPR) {
        motorEncoder.programPPR(i2c, PidController::kPPR);
    }

    // buttons
    knobButton.init(button_isr);
    backButton.init(button_isr);
    startButton.init(button_isr);

    // rotary encoder knob
    knob.init();
    knob.enable(knob_isr);

    // ADC with DMA
    adc.init();
    // DAC
    adc.initDAC();
    // PID controller
    pid.init(pid_timer_isr, pid_fault_isr);

    // Initialize display driver
    tft_driver_init();
    tft_clear_display();

    // Initialize LVGL and register flush callback
    lv_init();
    tft_driver_lvgl_init();

#if 0
    // color flashing test loop
    tft_backlight_pwm_set(100);
    uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFFF};// Red, Green, Blue, White
    int c = 0;

    for(;;) {
        uint32_t start = millis();
        tft_clear_display(colors[c++%4]);
        uint32_t dur = millis() - start;
        DEBUG_PRINT(DEBUG_DEBUG, "Clear display took %lu ms", dur);
        delay(250);
    }
#endif

    // Show welcome screen and load main menu
    menu.showWelcomeScreen();
    // Apply settings after welcome screen since it turns the backlight on
    menu.applyEEPROMSettings();

    menu.loadStartScreen();
}

void loop()
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
        if (millis() - lastFaultTime >= 500) {
            lastFaultTime = millis();
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
    if (millis() - lastLvHandler >= 5) {
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
        lastLvHandler = millis();
    }

    if (false) {
        static uint32_t lastTime37 = 0;
        if (millis() - lastTime37 >= 100) {
            lastTime37 = millis();
            // extern volatile uint32_t rpm_counter;
            // DEBUG_PRINT(DEBUG_DEBUG, "RPM_COUNTER=%u", rpm_counter);
            // DEBUG_PRINT(DEBUG_DEBUG, "TIM5=%u", TIM5->CNT);
        }
    }

    if (false) { // print faults
        static uint32_t lastTime3 = 0;
        if (millis() - lastTime3 >= 1000) {
            lastTime3 = millis();
            static uint32_t lastCounter = 0;
            if (pid.faults.count != lastCounter) {
                lastCounter = pid.faults.count;
                pid.debugPrintFaults();
            }
        }
    }
}
