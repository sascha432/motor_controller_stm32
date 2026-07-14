/**
  Author: sascha_lammers@gmx.de
*/

#include "controls.h"
#include "mt6701_encoder.h"
#include "pid_controller.h"
#include "adc.h"
#include "leds.h"

static char cType = 'r';
static int debugSpeed = 500;
static uint32_t debugStep = -1;
static uint32_t lastCurrent = 0;

Button<TOGGLE_PIN, GPIOB_BASE, false> startButton;
Button<BACK_PIN, GPIOA_BASE, false> backButton;
RotaryEncoder<PA6, PA7, GPIOA_BASE> knob;
PidController pid;
MT6701Encoder<MT6701_I2C_PIN, GPIOB_BASE, false> motorEncoder;
ADC adc;
LEDs leds;

static void button_isr() {
    // group by GOIO port
    uint32_t idrb = ((GPIO_TypeDef *)GPIOB_BASE)->IDR;
    startButton.isr(idrb);
    uint32_t idra = ((GPIO_TypeDef *)GPIOA_BASE)->IDR;
    backButton.isr(idra);
}

static void knob_isr() {
    knob.isr();
}

static void pid_timer_isr() {
    pid.isr();
}

void setup()
{
    Serial.begin(115200);

    // === LEDs ===
    leds.init();

    // === motor encoder ===
    motorEncoder.init();
    if (0) {
        motorEncoder.programPPR(PidController::kPPR);
    }

    // === buttons ===
    startButton.init(button_isr);
    backButton.init(button_isr);

    // === rotary encoder knob ===
    knob.init();
    knob.enable(knob_isr);

    // === ADC with DMA ===
    adc.init();

    // === PID controller ===
    pid.init(pid_timer_isr);
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
    if (startButton.isPressed()) {
        toggleMotor();
    }
    #if 0
    if (backButton.isReleased()) {
        Serial.print(millis());
        Serial.print(' ');
        Serial.println("BACK BTN RELEASED");
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
            while(1) {
                Serial.println("ERROR");
                delay(1000);
            }
            break;
        case 'c':
            pid.printDebug(Serial);
            break;
        case 'm':
            // motor must be off and pid controller disabled to program the encoder
            if (pid.running) {
                Serial.println("MOTOR OFF...");
                motorOff();
                delay(2000);
            }
            pid.disable();
            motorEncoder.programPPR(PidController::kPPR);
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
