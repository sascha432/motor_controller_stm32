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

static char cType = 'r';
static int debugSpeed = 500;
static uint32_t debugStep = -1;
static uint32_t lastCurrent = 0;

I2CHelper i2c;
Button<KNOB_BUTTON_PIN, false> knobButton;
Button<START_BUTTON_PIN, false> startButton;
Button<BACK_BUTTON_PIN, false> backButton;

RotaryEncoder<ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B> knob;
MT6701Encoder<MT6701_I2C_ENABLE_PIN, false> motorEncoder;
PidController pid;
ADC adc;
Menu menu;

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

static void knob_isr() {
    knob.isr();
}

static void pid_timer_isr() {
    pid.isr();
}

void clear_user_inputs() 
{
    knobButton.clear();
    backButton.clear();
    startButton.clear();
    knob.clear();
}

void apply_eeprom_settings() 
{
    auto &eeprom = EEPROM::getInstance();
    tft_backlight_pwm_set(eeprom.getTFTBrightness());
    LEDs::illuminationLedSetPWM(eeprom.getLEDBrightness());
}

void setup()
{
    debug_init();

    // Initialize and read EEPROM on I2C1 on PB8/9
    i2c.initI2C1Remapped();
    auto &eeprom = EEPROM::getInstance();
    eeprom.init();
    eeprom.read(eeprom.getData());

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

    // PID controller
    pid.init(pid_timer_isr);

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
    apply_eeprom_settings(); 
    menu.loadMainMenu();
}

void motorOff() {
    PID_WRITE_MOTOR_PWM(0);
    __disable_irq();
    if (pid.running) {
        pid.running = false;
        __enable_irq();
        Serial.println("STOP");
        debugStep = -1;
    }
    else {
        __enable_irq();
        Serial.println("ERR=NOT_RUNNING");
    }
}

void motorOn() {
    __disable_irq();
    if (!pid.running) {
        pid.running = true;
        __enable_irq();
        pid.reset();
        Serial.println("START");
        debugStep = 0;
    } else {
        __enable_irq();
        Serial.println("ERR=RUNNING");
    }
}

void toggleMotor() {
    if (pid.running) {
        motorOff();
    }
    else {
        motorOn();
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

    // handle ui updates and rotary encoder
    static uint32_t lastLvHandler = 0;
    if (millis() - lastLvHandler >= 5) {
        int32_t delta = knob.getDeltaPosition();
        if (delta) {
            int32_t newPosition = menu.updateRotaryValue(menu.getValue() + delta);
            DEBUG_PRINT(DEBUG_DEBUG, "knob=%d", newPosition);
        }
        lv_timer_handler();
        lastLvHandler = millis();
    }

#if 1
    // ADC debugoutput and blink motor LEDs
    {
        static uint32_t lastTime = 0;
        static uint32_t counter = 0;
        if (millis() - lastTime >= 100) {
            lastTime = millis();
            adc.debugPrint();
            // ((++counter / 10) & 0x01) ? LEDs::onLED1() : LEDs::onLED2();
        }
    }
#endif

}

#if 0
void loop()
{
    if (startButton.isPressed()) {
        toggleMotor();
    }
    #if 0
    if (backButton.isReleased()) {
        DEBUG_PRINTF("BACK BTN RELEASED");
    }
    #endif
    #if 0
    if (backButton.isPressed()) {
        static uint32_t num = 0;
        switch(num++ % 3) {
            case 0:
                pid.setRPM(200);
                break;
            case 1:
                pid.setRPM(5000);
                break;
        }
    }
    #endif
    #if 1
    if (backButton.isPressed()) {
        static uint32_t num = 0;
        DEBUG_PRINTF("BACK BTN PRESSED %u", num);
        switch(num++ % 3) {
            case 0:
                leds.onLED1();
                break;
            case 1:
                leds.onLED2();
                break;
            case 2:
                leds.off();
                break;
        }
    }
    #endif

    // static uint32_t lastTime3 = 0;
    // if (millis() - lastTime3 >= 100) {
    //     lastTime3 = millis();
    //     adc.debugPrint(Serial);
    // }

    if (Serial.available()) {
        static constexpr auto kRPMChange = 10;
        static constexpr auto kPIDChange = 0.01;
        int ch;
        switch (ch = Serial.read()) {
        case 0x7f: // switch to programming mode if handshake is received
        case 'x': // manual key
            pinMode(PA5, OUTPUT);
            digitalWrite(PA5, LOW);
            delay(100);
            NVIC_SystemReset();
            for(;;) {}
            break;
        case 'c':
            pid.printDebug(Serial);
            break;
        case 'm':
            // motor must be off and pid controller disabled to program the encoder
            if (pid.running) {
                DEBUG_PRINTF("MOTOR OFF...");
                motorOff();
                delay(2000);
            }
            pid.disable();
            motorEncoder.programPPR(i2c, PidController::kPPR);
            // re-init pid after programming
            pid.initQDECpins();
            pid.enable();
            break;
        case 'u':
            switch(pid.getRPM()) {
                case 100:
                    pid.setRPM(2000);
                    break;
                default:
                    pid.setRPM(100);
                    break;
            }
            break;
        case 'P':
            {
                char buf[64];
                buf[Serial.readBytesUntil(',', buf, sizeof(buf) - 1)] = 0;
                if (buf[0] == '=') {
                    if (buf[1]) {
                        pid.setKp(atof(buf + 1));
                    }
                    Serial.print("P");
                    Serial.print(buf);
                    buf[Serial.readBytesUntil(',', buf, sizeof(buf) - 1)] = 0;
                    if (*buf) {
                        pid.setKi(atof(buf));
                    }
                    Serial.print(" I=");
                    Serial.print(buf);
                    buf[Serial.readBytesUntil('\n', buf, sizeof(buf) - 1)] = 0;
                    if (*buf) {
                        pid.setKd(atof(buf));
                    }
                    Serial.print(" D=");
                    Serial.println(buf);
                    // update limits after changing pid values
                    pid.setRPM(pid.getRPM());
                }
                else {
                    Serial.println("INVALID PID");
                }
            }
            break;
        case 'R':
            {
                char buf[32];
                buf[Serial.readBytesUntil('\n', buf, sizeof(buf) - 1)] = 0;
                if (buf[0] == '=') {
                    uint32_t rpm = pid.clampRPM(atoi(buf + 1));
                    pid.setRPM(rpm);
                    if (pid.running) {
                        if (rpm == 0) {
                            motorOff();
                        }
                    }
                }
            }
            break;
        case '+':
            switch(cType) {
                case 'r':
                    pid.setRPM(pid.rpm + kRPMChange);
                    break;
                case 'p':
                    pid.setKp(pid.Kp + kPIDChange);
                    break;
                case 'i':
                    pid.setKi(pid.Ki + kPIDChange);
                    break;
                case 'd':
                    pid.setKd(pid.Kd + kPIDChange);
                    break;
                case 'w':
                    pid.antiWindupReduction = std::clamp(pid.antiWindupReduction + 1, 0, 99);
                    Serial.print("W=");
                    Serial.println(pid.antiWindupReduction);
                    break;
            }
            break;
        case '-':
            switch(cType) {
                case 'r':
                    pid.setRPM(pid.rpm - kRPMChange);
                    break;
                case 'p':
                    pid.setKp(pid.Kp - kPIDChange);
                    break;
                case 'i':
                    pid.setKi(pid.Ki - kPIDChange);
                    break;
                case 'd':
                    pid.setKd(pid.Kd - kPIDChange);
                    break;
                case 'w':
                    pid.antiWindupReduction = std::clamp(pid.antiWindupReduction - 1, 0, 99);
                    Serial.print("W=");
                    Serial.println(pid.antiWindupReduction);
                    break;
            }
            break;
        case 'r':
        case 'p':
        case 'i':
        case 'd':
        case 'w':
            cType = ch;
            break;
        case 't':
            debugSpeed = debugSpeed != 500 ? 500 : 50;
            break;
        case '0':
            motorOff();
            break;
        case '1':
            motorOn();
            break;
        case ' ':
            toggleMotor();
            break;
        }
    }

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