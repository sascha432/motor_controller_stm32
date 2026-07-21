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

MotorEncoder motorEncoder;

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

void motorOff()
{
    PID_WRITE_MOTOR_PWM_OFF();
    __disable_irq();
    if (pid.running) {
        pid.running = false;
        __enable_irq();
        Serial.println("STOP");
        DEBUG_PRINT(DEBUG_DEBUG, "STOP");
    }
    else {
        __enable_irq();
        Serial.println("ERR=NOT_RUNNING");
        DEBUG_PRINT(DEBUG_ERROR, "MOTOR NOT RUNNING");
    }
}

void motorOn()
{
    __disable_irq();
    if (!pid.running) {
        pid.running = true;
        __enable_irq();
        pid.reset();
        pid.setRPM(eeprom.getSpeed());
        Serial.println("START");
        DEBUG_PRINT(DEBUG_DEBUG, "START: rpm=%d", pid.getRPM());
    }
    else {
        __enable_irq();
        Serial.println("ERR=RUNNING");
        DEBUG_PRINT(DEBUG_ERROR, "MOTOR RUNNING");
    }
}

bool toggleMotor()
{
    if (pid.running) {
        motorOff();
        return false;
    }
    else {
        motorOn();
        return true;
    }
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
            newPosition = menu.updateRotaryValue(menu.getValue() + delta);
            // DEBUG_PRINT(DEBUG_DEBUG, "menu=%d delta=%d", newPosition, delta);
        }
        // handle LVGL updates
        auto &screenFlow = menu.getScreenFlow();
        switch(screenFlow->getId()) {
            case Screen::Type::DASHBOARD:
                // TODO
                // if (delta) {
                //     eeprom.setMotorRPM(newPosition);
                //     pid.setRPM(eeprom.getMotorRPM());
                //     DEBUG_PRINT(DEBUG_DEBUG, "DASHBOARD: rpm=%d", (int)eeprom.getMotorRPM());
                // }
                // fallthrough
            case Screen::Type::DIAGNOSTICS:
                stats.update();
                screenFlow->update();
                break;
            default:
                break;
        }
        lv_timer_handler();

        // check NTC sensors
        static bool triggered = false;
        uint32_t overTemperature = 0;
        if (adc.getMotorNTCValue() < eeprom.getMotorTemperatureLimitADC()) {
            overTemperature |= 0x01;
            LEDs::onLED1();
        }
        if (adc.getMosfetNTCValue() < eeprom.getMosfetTemperatureLimitADC()) {
            overTemperature |= 0x02;
            LEDs::onLED2();
        }
        if (overTemperature) {
            auto adcValues = adc.readAll();
            triggered = true;
            if (pid.running) {
                motorOff();
            }
            // DEBUG_PRINT(DEBUG_ERROR, "OVER TEMPERATURE: flag=%02x motor=%d mosfet=%d", overTemperature, (int32_t)adcValues.getMotorTemperature(), (int32_t)adcValues.getMosfetTemperature());
        }
        if (!overTemperature && triggered) {
            triggered = false;
            LEDs::offLED1and2();
        }

        // ADC temperature debugging
        if (0) {
            static uint16_t lastMotorNTCValue = 0;
            static uint16_t lastMosfetNTCValue = 0;
            uint16_t motorNTCValue = adc.getMotorNTCValue() / 64; // only on significant changes
            uint16_t mosfetNTCValue = adc.getMosfetNTCValue() / 64;
            if (motorNTCValue != lastMotorNTCValue || mosfetNTCValue != lastMosfetNTCValue) {
                lastMotorNTCValue = motorNTCValue;
                lastMosfetNTCValue = mosfetNTCValue;
                DEBUG_PRINT(DEBUG_DEBUG, "motor=%u/%u mosfet=%u/%u", adc.getMotorNTCValue(), eeprom.getMotorTemperatureLimitADC(), adc.getMosfetNTCValue(), eeprom.getMosfetTemperatureLimitADC());
            }
        }

        lastLvHandler = millis();
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
