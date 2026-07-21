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

static void button_isr() {
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

void setup()
{
    #ifdef PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
    #if defined(STM32F107xC)
    // STM32F107 USB clock must be 48 MHz for FS enumeration.
    // With PLLCLK=72 MHz this requires OTGFSPRE = /3.
    RCC->CFGR &= ~RCC_CFGR_OTGFSPRE;
    #endif
    Serial.begin(115200);
    #endif
    debug_init();

    // Initialize and read EEPROM on I2C1 on PB8/9
    eeprom.init();
    eeprom.read();

    // LEDs
    LEDs::init();

    // motor encoder
    motorEncoder.init();
    if (0) {
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
        lv_timer_handler();

        // check NTC sensors
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

    if (false) { // print RPM and encoder count
        static uint32_t lastTime6 = 0;
        if (millis() - lastTime6 >= 100) {
            uint32_t idrB = GPIOB->IDR;
            DEBUG_PRINT(DEBUG_DEBUG, "RPM=%d ENCODER=%u A=%u B=%u", pid.lastRpmMeasured, TIM4->CNT, (idrB >> 6) & 1U, (idrB >> 7) & 1U);
            lastTime6 = millis();
        }
    }

    if (false) {
        static uint32_t lastTime5 = 0;
        if (pid.running && millis() - lastTime5 >= 250) {
            if (pid.lastDebugNewData) {
                auto tmpLastRpmMeasured = pid.lastRpmMeasured;
                auto tmpLastPwmLevel = pid.lastPwmLevel;
                pid.lastDebugNewData = false;
                static uint32_t debugStep = 0;
                DEBUG_PRINT(DEBUG_DEBUG, "%lu,%d,%d,%u,%u,%u.%u,%u.%u",
                    debugStep++,
                    tmpLastRpmMeasured,
                    pid.clampPWMLevel(tmpLastPwmLevel),
                    stats.vcc,
                    stats.current,
                    CONVERT_TO_FP1(stats.motorTemp),
                    CONVERT_TO_FP1(stats.mosfetTemp)
                );
                // D,debugStep,lastRpmMeasured,lastPwmLevel,lastCurrent(mA)
                // Serial.print("D,");
                // Serial.print(debugStep++);
                // Serial.print(',');
                // Serial.print(tmpLastRpmMeasured);
                // Serial.print(',');
                // Serial.print(pid.clampPWMLevel(tmpLastPwmLevel));
                // Serial.print(',');
                // Serial.print((uint32_t)((adc.readAll().isense / 4095.0f) * 3300.0f));
                // Serial.println();
                lastTime5 = millis();
            }

        }
    }
}

#if 0
void loop()
{
    #if DEBUG_HUMAN == 0
        if (pid.lastDebugNewData) {
            auto tmpLastRpmMeasured = pid.lastRpmMeasured;
            auto tmpLastPwmLevel = pid.lastPwmLevel;
            pid.lastDebugNewData = false;
            if (debugStep != -1) {
                // D,debugStep,lastRpmMeasured,lastPwmLevel,lastCurrent(mA)
                Serial.print("D,");
                Serial.print(debugStep++);
                Serial.print(',');
                Serial.print(tmpLastRpmMeasured);
                Serial.print(',');
                Serial.print(pid.clampPWMLevel(tmpLastPwmLevel));
                Serial.print(',');
                Serial.print((uint32_t)((adc.readAll().isense / 4095.0f) * 3300.0f));
                // Serial.print(',');
                // Serial.print(pid.getLastError());
                // Serial.print(',');
                // Serial.print(pid.getIntegral());
                // Serial.print(',');
                // Serial.print(pid.getLastError());
                // Serial.print(',');
                // Serial.print(pid.getLastDerivative());
                Serial.println();
            }
        }
    #endif
    static uint32_t lastTime = 0;
    if (millis() - lastTime >= 100) {

        auto knobDelta = knob.getDeltaPositionAndMultiplier();
        if (knobDelta.hasPosition()) {
            pid.setRPM(pid.clampRPM(pid.getRPM() + knobDelta.getPosition()));
            Serial.print("R,");
            Serial.println(pid.getRPM());
        }

        #if DEBUG_HUMAN != 0

            //DEBUG
            static uint32_t lastTime3 = 0;
            if (millis() - lastTime3 >= debugSpeed)
            // if(1)
            {

                Serial.print("PID(");
                Serial.print(cType);
                Serial.print(") ");

                // PID
                Serial.print(pid.Kp, 2);
                Serial.print('(');
                Serial.print(pid.KpPreCalc);
                Serial.print(')');
                Serial.print(' ');
                Serial.print(pid.Ki, 2);
                Serial.print('(');
                Serial.print(pid.KiPreCalc);
                Serial.print(')');
                Serial.print(' ');
                Serial.print(pid.Kd, 2);
                Serial.print('(');
                Serial.print(pid.KdPreCalc);
                Serial.print(')');
                Serial.print(" PV ");
                Serial.print(pid.getLastError());
                Serial.print(' ');
                Serial.print(pid.getIntegral());
                Serial.print(' ');
                Serial.print(pid.getLastDerivative());
                // PLANT
                Serial.print(" RPM ");
                Serial.print(pid.getRPM());
                Serial.print('/');
                Serial.println(pid.avgRPM);

                lastTime3 = millis();
            }

        #endif
    }
}
#endif
