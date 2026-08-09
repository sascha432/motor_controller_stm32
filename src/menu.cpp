/**
  Author: sascha_lammers@gmx.de

  Menu implementation for the UI
*/

#include <stdio.h>
#include "menu.h"
#include "ui.h"
#include "eeprom.h"
#include "leds.h"
#include "pid_controller.h"

ScreenFlow screenFlow;
Menu menu;

// === menu enums and items ===

enum class MainMenuItemType {
    SPEED = 0,
    CONTROL_MODE,
    LED_BRIGHTNESS,
    CURRENT_LIMITS,
    STALL_TIME,
    MOTOR_BRAKE,
    ADVANCED,
    RESTORE_DEFAULTS,
    SAVE_EXIT
};

static const char *kMainMenuItems[] = {
    "Speed",                    // 0
    "Mode",                     // 1
    "LED Brightness",           // 2
    "Current Limits",           // 3
    "Stall Time",               // 4
    "Motor Brake",              // 5
    "Advanced",                 // 6
    "Restore Defaults",         // 7
    "Save & Exit",              // 8
};

enum class AdvancedMenuItemType {
    PID_PARAMETERS = 0,
    TFT_BRIGHTNESS,
    MOSFET_TEMPERATURE,
    MOTOR_TEMPERATURE,
    MOTOR_RPM_SETTINGS,
    MOTOR_DIRECTION,
    SENSOR_DIRECTION,
    PWM_FREQUENCY,
    OVP_PROTECTION,
    WELCOME_CHIME,
    DIAGNOSTICS,
#if HAVE_IMPERIAL_MARCH
    PLAY_IMPERIAL_MARCH,
#endif
    BACK
};

static const char *kAdvancedMenuItems[] = {
    "PID Parameters",           // 0
    "TFT Brightness",           // 1
    "MOSFET Temperature",       // 2
    "Motor Temperature",        // 3
    "Motor RPM Settings",       // 4
    "Motor Direction",          // 5
    "Sensor Direction",         // 6
    "PWM Frequency",            // 7
    "OVP Protection",           // 8
    "Welcome Chime",            // 9
    "Diagnostics",              // 10
#if HAVE_IMPERIAL_MARCH
    "Play Imperial March",      // 11
#endif
    "Back"
};

enum class PIDParametersItemType {
    KP = 0,
    KI,
    KD,
    ANTI_WINDUP,
    BACK
};

static const char *kPIDParametersItems[] = {
    "Kp",                       // 0
    "Ki",                       // 1
    "Kd",                       // 2
    "Anti-Windup",              // 3
    "Back"                      // 4
};

enum class MotorRPMSettingsItemType {
    MIN_RPM = 0,
    MAX_RPM,
    BACK
};

static const char *kMotorRPMSettingsItems[] = {
    "Minimum RPM",              // 0
    "Maximum RPM",              // 1
    "Back"                      // 2
};

enum class CurrentLimitItemType {
    INPUT_CURRENT_LIMIT = 0,
    MOTOR_CURRENT_LIMIT,
    BACK
};

static const char *kCurrentLimitItems[] = {
    "Input Current Limit",      // 0
    "Motor Current Limit",      // 1
    "Back"                      // 2
};

enum class ControlModeItemType {
    PWM = 0,
    PID,
    BACK
};

static const char *kControlModeItems[] = {
    "PWM / Open Loop",          // 0
    "PID / Closed Loop"         // 1
};

static const char *kDirectionItems[] = {
    "Forward",                  // 0
    "Reverse"                   // 1
};

enum class RestoreDefaultsItemType {
    RESTORE = 0,
    CANCEL
};

static const char *kRestoreDefaultsMenuItems[] = {
    "Restore",                  // 0
    "Cancel"                    // 1
};

#if HAVE_IMPERIAL_MARCH

struct Tone {
    uint8_t frequency;
    uint8_t duration;
};

const Tone imperial_march[] = {
    {65, 125}, {65, 125}, {65, 125}, {51, 93}, {77, 31}, {65, 125}, {51, 93}, {77, 31}, {65, 250},
    {97, 125}, {97, 125}, {97, 125}, {103, 93}, {77, 31}, {65, 125}, {51, 93}, {77, 31}, {65, 250},
    {130, 125}, {65, 93}, {65, 31}, {130, 125}, {123, 93}, {116, 31}, {110, 31}, {103, 31}, {110, 125},
    {69, 62}, {92, 125}, {87, 93}, {82, 31}, {77, 31}, {73, 31}, {77, 125},
    {51, 62}, {61, 125}, {51, 93}, {61, 31}, {77, 125}, {65, 93}, {77, 31}, {97, 250},
    {130, 125}, {65, 93}, {65, 31}, {130, 125}, {123, 93}, {116, 31}, {110, 31}, {103, 31}, {110, 125},
    {69, 62}, {92, 125}, {87, 93}, {82, 31}, {77, 31}, {73, 31}, {77, 125},
    {51, 62}, {61, 125}, {51, 93}, {77, 31}, {65, 125}, {51, 93}, {77, 31}, {65, 250},
};

static void playImperialMarch()
{
    menu.clearUserInput();
    screenFlow.next(ScreenFlow::newScreen<InfoScreen>(Screen::Type::DIAGNOSTICS, "Imperial March", Screen::kInfoScreenLabelFont));
    screenFlow.refresh();
    MotorVibes vibes;
    vibes.init();
    for(uint32_t i = 0; i < sizeof_array(imperial_march); ++i) {
        vibes.playTone(imperial_march[i].frequency << 2);
        WatchDog::delay(imperial_march[i].duration << 2);
        vibes.stopTone();
        WatchDog::delay(20);
        if (menu.hasAnyButtonBeenPressed()) {
            break;
        }
    }
    vibes.stopTone();
    vibes.deinit();
    while (menu.isAnyButtonDown()) {
        WatchDog::feed();
    }
    menu.clearUserInput();
    screenFlow.back();
    screenFlow.refresh();
}

#endif

/**
 * @brief custom format for current (mA) conversion from uint32_t to "n.n A"
 */
static const char *current_format_callback(uint32_t value, char *buf, size_t bufSize)
{
    snprintf(buf, bufSize, SPRINTF_FP1_FMT " A", CONVERT_TO_FP1(value));
    return buf;
}

/**
 * @brief custom format for voltage (mV) conversion from uint32_t to "n.n V"
 *
 */
static const char *voltage_format_callback(uint32_t value, char *buf, size_t bufSize)
{
    snprintf(buf, bufSize, SPRINTF_FP1_FMT " V", CONVERT_TO_FP1(value));
    return buf;
}

/**
 * @brief custom format for anti windup conversion from uint32_t to "n.nn%"
 *
 */
static const char *anti_windup_format_callback(uint32_t value, char *buf, size_t bufSize)
{
    if (value == 0) {
        snprintf(buf, bufSize, "Disabled");
        return buf;
    }
    const uint32_t antiWindup = value * 1000 / UIConstants::kAntiWindupFactor; // multiply by 1000 for convert_to_fp2
    snprintf(buf, bufSize, SPRINTF_FP2_FMT " %%", CONVERT_TO_FP2(antiWindup));
    return buf;
}

/**
 * @brief custom format for PID parameters and conversion from uint32_t to "n.nnnnnn" with trimmed trailing zeros
 *
 */
static const char *pid_parameter_format_callback(uint32_t value, char *buf, size_t bufSize)
{
    FloatToString::convertTrimmed(buf, bufSize, value / UIConstants::kPIDParamFactor, 6);
    return buf;
}

// === menu implementation ===

void Menu::loadWelcomeScreen()
{
    // Show welcome screen for a few seconds
    screenFlow.init();
    screenFlow.setScreen(ScreenFlow::newScreen<WelcomeScreen>());
    screenFlow.refresh();
    tft_backlight_pwm_set(eeprom.getTFTBrightness());

    if (UIConstants::kEnableIlluminationLEDFading) {

        #if HAVE_MOTOR_VIBES
            MotorVibes chime;
            const bool play = eeprom.getMotorChime();
            if (play) {
                chime.init();
            }

            const uint8_t chimeFrequency50msInterval[] = {
                104, 104, 104,  // 520 Hz
                0,
                132, 132, 132,  // 660 Hz
                0,
                157, 157, 157, 157,  // 785 Hz
                0,
                132, 132, 132,  // 660 Hz
                0,
                157, 157, 157, 157,  // 785 Hz
                0,
                209, 209, 209, 209, 209, // 1045 Hz
                0
            };
        #endif

        // gradually increase LED brightness to target value
        constexpr uint32_t kMultiplier = (1 << 23);
        constexpr uint32_t kLoopDelay = 10;
        const uint32_t start = HAL_GetTick();
        uint32_t targetBrightness = eeprom.getLEDBrightness() * kMultiplier;
        uint32_t currentBrightness = 0;
        uint32_t step = targetBrightness / (UIConstants::kWelcomeScreenTimeout / kLoopDelay);
        targetBrightness -= step;
        for(uint32_t i = 0; ; i++) {
            uint32_t elapsed = HAL_GetTick() - start;
            if (elapsed >= UIConstants::kWelcomeScreenTimeout) {
                break;
            }
            if (currentBrightness < targetBrightness) {
                currentBrightness += step;
            }
            LEDs::illuminationLedSetPWM(currentBrightness / (kMultiplier / LEDs::kIlluminationResolution));
            // blink motor LEDs
            ((elapsed / 500) & 0x01) ? LEDs::onLEDError() : LEDs::onLEDWarning();
            #if HAVE_MOTOR_VIBES
                // play chime tone
                if (play) {
                    const uint32_t index = i / 5;
                    chime.playTone((index >= sizeof(chimeFrequency50msInterval)) ? 0 : (chimeFrequency50msInterval[index] * 5));
                }
            #endif
            WatchDog::delay(kLoopDelay);
        }
        LEDs::off();
        #if HAVE_MOTOR_VIBES
            if (play) {
                chime.stopTone();
                chime.deinit();
            }
        #endif
    }
    else {
        WatchDog::delay(UIConstants::kWelcomeScreenTimeout);
    }
    clearUserInput();
}

void Menu::loadMainMenu()
{
    screenFlow.setScreen(ScreenFlow::newScreen<MenuScreen>(
        Screen::Type::MAIN_MENU,
        kMainMenuItems,
        sizeof_array(kMainMenuItems)
    ));
    setValue(0);
    clearUserInput();
}

void Menu::loadAdvancedMenu()
{
    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
        Screen::Type::ADVANCED_MENU,
        kAdvancedMenuItems,
        sizeof_array(kAdvancedMenuItems)
    ));
    setValue(0);
    clearUserInput();
}

void Menu::exitAdvancedMenu()
{
    DEBUG_PRINT(DebugType::UI, "prev. screen=%p", screenFlow.getScreen()->getPrevScreen());
    if (screenFlow.getScreen()->getPrevScreen()) {
        // accessed through main menu
        restorePreviousMenu();
    }
    else {
        // accessed through long press on start screen
        saveEEPROMChanges();
        loadStartScreen();
    }
}

void Menu::loadStartScreen()
{
    screenFlow.setScreen(ScreenFlow::newScreen<StartScreen>());
    setValue(eeprom.getSpeed());
    // save any changes when returning to start screen
    bool success = eeprom.write();
    (void)success;
    DEBUG_PRINT(DebugType::UI, "eeprom.write=%d", success);
    clearUserInput();
}

void Menu::loadDashboardScreen()
{
    screenFlow.setScreen(ScreenFlow::newScreen<DashboardScreen>());
    setValue(eeprom.getSpeed());
    clearUserInput();
}

void Menu::restorePreviousMenu()
{
    DEBUG_PRINT(DebugType::UI, "value=%d", getValue());
    screenFlow.back(); // restore previous screen
    clearUserInput();
}

void Menu::setValue(int32_t value)
{
    screenFlow->setValue(value);
}

int32_t Menu::getValue() const
{
    return screenFlow->getValue();
}

ScreenFlow &Menu::getScreenFlow()
{
    return screenFlow;
}

void Menu::abortableDelay(uint32_t ms)
{
    clearUserInput();
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ms) {
        WatchDog::feed();
        if (hasAnyButtonBeenPressed()) {
            while (isAnyButtonDown()) {
                WatchDog::feed();
            }
            clearUserInput();
            break;
        }
    }
}

bool Menu::isAnyButtonDown() const
{
    return knobButton.isDown() || backButton.isDown() || startButton.isDown();
}

bool Menu::hasAnyButtonBeenPressed() const
{
    return knobButton.isPressed() || backButton.isPressed() || startButton.isPressed();
}

void Menu::clearUserInput()
{
    // clear states
    knobButton.reset();
    backButton.reset();
    startButton.reset();
    knob.reset();
}

void Menu::saveEEPROMChanges()
{
    if (eeprom.write()) {
        screenFlow.setScreen(ScreenFlow::newScreen<InfoScreen>(Screen::Type::EEPROM_SAVED, "Saved"));
        screenFlow.refresh();
        abortableDelay(UIConstants::kInfoScreenTimeout);
    }
}

void Menu::applyEEPROMSettings()
{
    tft_backlight_pwm_set(eeprom.getTFTBrightness());
    LEDs::illuminationLedSetPWM(eeprom.getLEDBrightness() * LEDs::kIlluminationResolution);
    adc.setInputCurrentLimit(eeprom.getInputCurrentLimit());
    adc.setMotorCurrentLimit(eeprom.getMotorCurrentLimit());
    pid.applyPIDParams();
}

void Menu::handleButtonPress(uint32_t duration)
{
    DEBUG_PRINT(DebugType::UI, "enter screen=%p prev=%p id=%d value=%d duration=%d", screenFlow.getScreen(), screenFlow->getPrevScreen(), static_cast<int>(screenFlow->getId()), getValue(), duration);
    Screen *screen;
    // Handle button press based on the current screen
    switch(screenFlow->getId()) {
        // === start screen ===
        case Screen::Type::START:
            if (duration >= UIConstants::kLongPressDuration) {
                screenFlow.destroy();
                loadAdvancedMenu();
            }
            else {
                loadMainMenu();
            }
            break;
        // === main menu ===
        case Screen::Type::MAIN_MENU:
            switch(static_cast<MainMenuItemType>(getValue())) {
                case MainMenuItemType::SPEED:
                    switch(eeprom.getControlMode()) {
                        case EEPROM::ControlMode::PID:
                            screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                                Screen::Type::MOTOR_SPEED,
                                "Motor Speed",
                                eeprom.getMinRPM(),
                                eeprom.getMaxRPM(),
                                "RPM"
                            ));
                            break;
                        case EEPROM::ControlMode::PWM:
                            screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                                Screen::Type::MOTOR_SPEED,
                                "Motor Speed",
                                1,
                                eeprom.getMaxPWM(),
                                "%"
                            ));
                            break;
                    }
                    setValue(eeprom.getSpeed());
                    break;
                case MainMenuItemType::CONTROL_MODE:
                    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
                        Screen::Type::CONTROL_MODE,
                        kControlModeItems,
                        sizeof_array(kControlModeItems)
                    ));
                    setValue(static_cast<uint8_t>(eeprom.getControlMode()));
                    break;
                case MainMenuItemType::LED_BRIGHTNESS:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::LED_BRIGHTNESS,
                        "LED Brightness",
                        UIConstants::kMinLEDBrightness,
                        UIConstants::kMaxLEDBrightness,
                        "%",
                        nullptr,
                        "OFF"
                    ));
                    setValue(eeprom.getLEDBrightness());
                    break;
                case MainMenuItemType::CURRENT_LIMITS:
                    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
                        Screen::Type::CURRENT_LIMITS,
                        kCurrentLimitItems,
                        sizeof_array(kCurrentLimitItems)
                    ));
                    setValue(0);
                    break;
                case MainMenuItemType::STALL_TIME:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::MOTOR_STALL_TIMEOUT,
                        "Motor Stall Timeout",
                        UIConstants::kMinMotorStallTimeout,
                        UIConstants::kMaxMotorStallTimeout,
                        "ms"
                    ));
                    setValue(eeprom.getMotorStallTimeout());
                    break;
                case MainMenuItemType::MOTOR_BRAKE:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::MOTOR_BRAKE,
                        "Motor Brake",
                        0,
                        100,
                        "%",
                        nullptr,
                        "OFF"
                    ));
                    setValue(eeprom.getMotorBrake());
                    break;
                case MainMenuItemType::ADVANCED:
                    loadAdvancedMenu();
                    break;
                case MainMenuItemType::RESTORE_DEFAULTS:
                    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
                        Screen::Type::RESTORE_DEFAULTS_CONFIRMATION,
                        kRestoreDefaultsMenuItems,
                        sizeof_array(kRestoreDefaultsMenuItems)
                    ));
                    setValue(1);
                    break;
                case MainMenuItemType::SAVE_EXIT:
                    saveEEPROMChanges();
                    loadStartScreen();
                    break;
            }
            break;
        // === current limits menu ===
        case Screen::Type::CURRENT_LIMITS:
            switch(static_cast<CurrentLimitItemType>(getValue())) {
                case CurrentLimitItemType::INPUT_CURRENT_LIMIT:
                    screen = ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::INPUT_CURRENT_LIMIT,
                        "Input Current Limit",
                        UIConstants::kMinInputCurrent,
                        UIConstants::kMaxInputCurrent,
                        "A",
                        current_format_callback
                    );
                    screen->setSteps(UIConstants::kStepInputCurrent);
                    screenFlow.next(screen);
                    setValue(eeprom.getInputCurrentLimit());
                    break;
                case CurrentLimitItemType::MOTOR_CURRENT_LIMIT:
                    screen = ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::MOTOR_CURRENT_LIMIT,
                        "Motor Current Limit",
                        UIConstants::kMinMotorCurrent,
                        UIConstants::kMaxMotorCurrent,
                        "A",
                        current_format_callback
                    );
                    screen->setSteps(UIConstants::kStepMotorCurrent);
                    screenFlow.next(screen);
                    setValue(eeprom.getMotorCurrentLimit());
                    break;
                case CurrentLimitItemType::BACK:
                    restorePreviousMenu();
                    break;
            }
            break;
        // === advanced menu ===
        case Screen::Type::ADVANCED_MENU:
            switch(static_cast<AdvancedMenuItemType>(getValue())) {
                case AdvancedMenuItemType::TFT_BRIGHTNESS:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::TFT_BRIGHTNESS,
                        "TFT Brightness",
                        UIConstants::kMinTFTBrightness,
                        UIConstants::kMaxTFTBrightness,
                        "%"
                    ));
                    setValue(eeprom.getTFTBrightness());
                    break;
                case AdvancedMenuItemType::MOSFET_TEMPERATURE:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::MOSFET_TEMPERATURE_LIMIT,
                        "MOSFET Temperature Limit",
                        UIConstants::kMinMosfetTemperature,
                        UIConstants::kMaxMosfetTemperature,
                        DEGREE_UTF8 "C"
                    ));
                    setValue(eeprom.getMosfetTemperatureLimit());
                    break;
                case AdvancedMenuItemType::MOTOR_TEMPERATURE:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::MOTOR_TEMPERATURE_LIMIT,
                        "Motor Temperature Limit",
                        UIConstants::kMinMotorTemperature,
                        UIConstants::kMaxMotorTemperature,
                        DEGREE_UTF8 "C"
                    ));
                    setValue(eeprom.getMotorTemperatureLimit());
                    break;
                case AdvancedMenuItemType::MOTOR_RPM_SETTINGS:
                    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
                        Screen::Type::MOTOR_RPM_SETTINGS,
                        kMotorRPMSettingsItems,
                        sizeof_array(kMotorRPMSettingsItems)
                    ));
                    setValue(0);
                    break;
                case AdvancedMenuItemType::MOTOR_DIRECTION:
                    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
                        Screen::Type::MOTOR_DIRECTION,
                        kDirectionItems,
                        sizeof_array(kDirectionItems)
                    ));
                    setValue(static_cast<uint8_t>(eeprom.getMotorDirection()));
                    break;
                case AdvancedMenuItemType::SENSOR_DIRECTION:
                    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
                        Screen::Type::SENSOR_DIRECTION,
                        kDirectionItems,
                        sizeof_array(kDirectionItems)
                    ));
                    setValue(static_cast<uint8_t>(eeprom.getSensorDirection()));
                    break;
                case AdvancedMenuItemType::PID_PARAMETERS:
                    screenFlow.next(ScreenFlow::newScreen<MenuScreen>(
                        Screen::Type::PID_PARAMETERS,
                        kPIDParametersItems,
                        sizeof_array(kPIDParametersItems)
                    ));
                    setValue(0);
                    break;
                case AdvancedMenuItemType::PWM_FREQUENCY:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::PWM_FREQUENCY,
                        "PWM Frequency",
                        UIConstants::kMinPWMFrequency,
                        UIConstants::kMaxPWMFrequency,
                        "Hz"
                    ));
                    screenFlow->setSteps(UIConstants::kStepPWMFrequency);
                    setValue(eeprom.getPWMFrequency());
                    break;
                case AdvancedMenuItemType::OVP_PROTECTION:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::OVP_PROTECTION,
                        "OVP Protection",
                        UIConstants::kMinOvpProtection,
                        UIConstants::kMaxOvpProtection,
                        "V",
                        voltage_format_callback
                    ));
                    screenFlow->setSteps(UIConstants::kStepOvpProtection);
                    setValue(eeprom.getOvpProtection());
                    break;
                case AdvancedMenuItemType::DIAGNOSTICS:
                    screenFlow.next(ScreenFlow::newScreen<DiagnosticsScreen>(Screen::Type::DIAGNOSTICS));
                    setValue(0);
                    break;
                case AdvancedMenuItemType::WELCOME_CHIME:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::WELCOME_CHIME,
                        "Welcome Chime",
                        0,
                        1,
                        ""
                    ));
                    setValue(eeprom.getMotorChime());
                    break;
                #if HAVE_IMPERIAL_MARCH
                case AdvancedMenuItemType::PLAY_IMPERIAL_MARCH:
                    playImperialMarch();
                    break;
                #endif
                case AdvancedMenuItemType::BACK:
                    exitAdvancedMenu();
                    break;
            }
            break;
        // === motor RPM settings menu ===
        case Screen::Type::MOTOR_RPM_SETTINGS:
            switch(static_cast<MotorRPMSettingsItemType>(getValue())) {
                case MotorRPMSettingsItemType::MIN_RPM:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::MIN_RPM,
                        "Min RPM",
                        UIConstants::kMinRPM,
                        UIConstants::kMaxRPM,
                        "RPM"
                    ));
                    setValue(eeprom.getMinRPM());
                    break;
                case MotorRPMSettingsItemType::MAX_RPM:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::MAX_RPM,
                        "Max RPM",
                        UIConstants::kMinRPM,
                        UIConstants::kMaxRPM,
                        "RPM"
                    ));
                    setValue(eeprom.getMaxRPM());
                    break;
                case MotorRPMSettingsItemType::BACK:
                    restorePreviousMenu();
                    break;
            }
            break;
        // === pid parameters menu ===
        case Screen::Type::PID_PARAMETERS:
            switch(static_cast<PIDParametersItemType>(getValue())) {
                case PIDParametersItemType::KP:
                    screenFlow.next(ScreenFlow::newScreen<PidSliderScreen>(
                        Screen::Type::PID_KP,
                        "Kp Parameter",
                        UIConstants::kMinKp,
                        UIConstants::kMaxKp,
                        pid_parameter_format_callback,
                        UIConstants::kSelectKpSteps
                    ));
                    setValue(EEPROM::kPIDParamToUint32(eeprom.getKp()));
                    break;
                case PIDParametersItemType::KI:
                    screenFlow.next(ScreenFlow::newScreen<PidSliderScreen>(
                        Screen::Type::PID_KI,
                        "Ki Parameter",
                        UIConstants::kMinKi,
                        UIConstants::kMaxKi,
                        pid_parameter_format_callback,
                        UIConstants::kSelectKiSteps
                    ));
                    setValue(EEPROM::kPIDParamToUint32(eeprom.getKi()));
                    break;
                case PIDParametersItemType::KD:
                    screenFlow.next(ScreenFlow::newScreen<PidSliderScreen>(
                        Screen::Type::PID_KD,
                        "Kd Parameter",
                        UIConstants::kMinKd,
                        UIConstants::kMaxKd,
                        pid_parameter_format_callback,
                        UIConstants::kSelectKdSteps
                    ));
                    setValue(EEPROM::kPIDParamToUint32(eeprom.getKd()));
                    break;
                case PIDParametersItemType::ANTI_WINDUP:
                    screenFlow.next(ScreenFlow::newScreen<SliderScreen>(
                        Screen::Type::PID_ANTI_WINDUP,
                        "Anti-Windup",
                        UIConstants::kMinAntiWindup,
                        UIConstants::kMaxAntiWindup,
                        "%",
                        anti_windup_format_callback
                    ));
                    screenFlow->setSteps(UIConstants::kStepsAntiWindup);
                    setValue(eeprom.getAntiWindup());
                    break;
                case PIDParametersItemType::BACK:
                    restorePreviousMenu();
                    break;
            }
            break;
        // === restore defaults confirmation menu ===
        case Screen::Type::RESTORE_DEFAULTS_CONFIRMATION:
            switch(static_cast<RestoreDefaultsItemType>(getValue())) {
                case RestoreDefaultsItemType::RESTORE:
                    eeprom.resetDefaults();
                    eeprom.write();
                    menu.applyEEPROMSettings();
                    screenFlow.next(ScreenFlow::newScreen<InfoScreen>(Screen::Type::EEPROM_RESTORED, "Restored"));
                    screenFlow.refresh();
                    abortableDelay(UIConstants::kInfoScreenTimeout);
                    loadMainMenu();
                    break;
                case RestoreDefaultsItemType::CANCEL:
                    restorePreviousMenu();
                    break;
            }
            break;
        // === dashboard screen ===
        case Screen::Type::DASHBOARD:
            if (eeprom.isPIDMode()) {
                DashboardScreen &dashboard = *reinterpret_cast<DashboardScreen *>(screenFlow.getScreen());
                switch(dashboard.incrSelectedValue()) {
                    case DashboardScreen::SelectedValueType::SPEED:
                    case DashboardScreen::SelectedValueType::SPEED2:
                        dashboard.setMaxAcceleration(eeprom.isPIDMode() ? UIConstants::kStepsRPM : UIConstants::kStepsPWM);
                        screenFlow->setSteps(1);
                        setValue(eeprom.getSpeed());
                        break;
                    case DashboardScreen::SelectedValueType::KP:
                        dashboard.setMaxAcceleration(UIConstants::kMaxPIDParamAcceleration);
                        screenFlow->setSteps(UIConstants::kSelectKpSteps);
                        setValue(EEPROM::kPIDParamToUint32(eeprom.getKp()));
                        break;
                    case DashboardScreen::SelectedValueType::KI:
                        dashboard.setMaxAcceleration(UIConstants::kMaxPIDParamAcceleration);
                        screenFlow->setSteps(UIConstants::kSelectKiSteps);
                        setValue(EEPROM::kPIDParamToUint32(eeprom.getKi()));
                        break;
                    case DashboardScreen::SelectedValueType::KD:
                        dashboard.setMaxAcceleration(UIConstants::kMaxPIDParamAcceleration);
                        screenFlow->setSteps(UIConstants::kSelectKdSteps);
                        setValue(EEPROM::kPIDParamToUint32(eeprom.getKd()));
                        break;
                    case DashboardScreen::SelectedValueType::ANTI_WINDUP:
                        dashboard.setMaxAcceleration(std::sqrt(UIConstants::kMaxAntiWindup - UIConstants::kMinAntiWindup));
                        screenFlow->setSteps(UIConstants::kStepsAntiWindup);
                        setValue(eeprom.getAntiWindup());
                        break;
                    case DashboardScreen::SelectedValueType::MAX:
                        break;
                }
            }
            break;
        // === mixed menus ===
        case Screen::Type::MOTOR_DIRECTION:
        case Screen::Type::SENSOR_DIRECTION:
        case Screen::Type::MIN_RPM:
        case Screen::Type::MAX_RPM:
        case Screen::Type::TFT_BRIGHTNESS:
        case Screen::Type::MOSFET_TEMPERATURE_LIMIT:
        case Screen::Type::MOTOR_TEMPERATURE_LIMIT:
        case Screen::Type::INPUT_CURRENT_LIMIT:
        case Screen::Type::MOTOR_CURRENT_LIMIT:
        case Screen::Type::LED_BRIGHTNESS:
        case Screen::Type::MOTOR_BRAKE:
        case Screen::Type::MOTOR_SPEED:
        case Screen::Type::MOTOR_STALL_TIMEOUT:
        case Screen::Type::CONTROL_MODE:
        case Screen::Type::DIAGNOSTICS:
        case Screen::Type::PID_KP:
        case Screen::Type::PID_KI:
        case Screen::Type::PID_KD:
        case Screen::Type::PID_ANTI_WINDUP:
        case Screen::Type::OVP_PROTECTION:
        case Screen::Type::PWM_FREQUENCY:
        case Screen::Type::WELCOME_CHIME:
            restorePreviousMenu();
            break;
        case Screen::Type::WELCOME:
        case Screen::Type::EEPROM_SAVED:
        case Screen::Type::EEPROM_RESTORED:
            // no button action
            break;
    }
    DEBUG_PRINT(DebugType::UI, "leave screen=%p prev=%p id=%d value=%d", screenFlow.getScreen(), screenFlow->getPrevScreen(), static_cast<int>(screenFlow->getId()), getValue());
}

void Menu::handleBackButtonPress()
{
    DEBUG_PRINT(DebugType::UI, "enter screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
    switch(screenFlow->getId()) {
        case Screen::Type::DASHBOARD:
            pid.motorOff();
            loadStartScreen();
            break;
        case Screen::Type::START:
            pid.toggleMotorDirection();
            break;
        case Screen::Type::WELCOME:
            // no back button available
            break;
        case Screen::Type::MAIN_MENU:
            saveEEPROMChanges();
            loadStartScreen();
            break;
        case Screen::Type::ADVANCED_MENU:
            exitAdvancedMenu();
            break;
        default:
            // default is back
            restorePreviousMenu();
            break;
    }
    DEBUG_PRINT(DebugType::UI, "leave screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
}

void Menu::handleStartButtonPress()
{
    DEBUG_PRINT(DebugType::UI, "enter screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
    switch(screenFlow->getId()) {
        case Screen::Type::START:
            if (pid.motorToggle()) {
                loadDashboardScreen();
            }
            break;
        case Screen::Type::DASHBOARD:
            if (!pid.motorToggle()) {
                loadStartScreen();
            }
            break;
        default:
            // no default action
            break;
    }
    DEBUG_PRINT(DebugType::UI, "leave screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
}

int32_t Menu::updateRotaryValue(int32_t value)
{
    int32_t clampedValue;
    screenFlow->setValue(screenFlow->getValue() + (value * screenFlow->getSteps()));
    switch(screenFlow->getId()) {
        case Screen::Type::DASHBOARD:
            switch(reinterpret_cast<DashboardScreen *>(screenFlow.getScreen())->getSelectedValue()) {
                case DashboardScreen::SelectedValueType::SPEED:
                case DashboardScreen::SelectedValueType::SPEED2:
                    updateSpeedValue();
                    break;
                case DashboardScreen::SelectedValueType::KP:
                    clampedValue = std::clamp<int32_t>(getValue(), UIConstants::kMinKp, UIConstants::kMaxKp); // clamp value, usually done in the slider screen
                    eeprom.setKp(EEPROM::kUint32ToPIDParam(clampedValue));
                    pid.applyPIDParams(); // apply or live tuning otherwise only a reset will apply the new values
                    setValue(clampedValue);
                    break;
                case DashboardScreen::SelectedValueType::KI:
                    clampedValue = std::clamp<int32_t>(getValue(), UIConstants::kMinKi, UIConstants::kMaxKi);
                    eeprom.setKi(EEPROM::kUint32ToPIDParam(clampedValue));
                    pid.applyPIDParams();
                    setValue(clampedValue);
                    break;
                case DashboardScreen::SelectedValueType::KD:
                    clampedValue = std::clamp<int32_t>(getValue(), UIConstants::kMinKd, UIConstants::kMaxKd);
                    eeprom.setKd(EEPROM::kUint32ToPIDParam(clampedValue));
                    pid.applyPIDParams();
                    setValue(clampedValue);
                    break;
                case DashboardScreen::SelectedValueType::ANTI_WINDUP:
                    eeprom.setAntiWindup(clampAntiWindupValue());
                    pid.applyPIDParams();
                    break;
                case DashboardScreen::SelectedValueType::MAX:
                    break;
            }
            break;
        case Screen::Type::START:
            updateSpeedValue();
            break;
        case Screen::Type::TFT_BRIGHTNESS:
            eeprom.setTFTBrightness(getValue());
            tft_backlight_pwm_set(eeprom.getTFTBrightness());
            break;
        case Screen::Type::LED_BRIGHTNESS:
            eeprom.setLEDBrightness(getValue());
            LEDs::illuminationLedSetPWM(eeprom.getLEDBrightness() * LEDs::kIlluminationResolution);
            break;
        case Screen::Type::INPUT_CURRENT_LIMIT:
            eeprom.setInputCurrentLimit(getValue());
            adc.setInputCurrentLimit(eeprom.getInputCurrentLimit());
            break;
        case Screen::Type::MOTOR_CURRENT_LIMIT:
            eeprom.setMotorCurrentLimit(getValue());
            adc.setMotorCurrentLimit(eeprom.getMotorCurrentLimit());
            break;
        case Screen::Type::MOTOR_DIRECTION:
            eeprom.setMotorDirection(static_cast<EEPROM::MotorDirection>(getValue()));
            break;
        case Screen::Type::SENSOR_DIRECTION:
            eeprom.setSensorDirection(static_cast<EEPROM::SensorDirection>(getValue()));
            break;
        case Screen::Type::MOTOR_BRAKE:
            eeprom.setMotorBrake(getValue());
            break;
        case Screen::Type::CONTROL_MODE:
            eeprom.setControlMode(static_cast<EEPROM::ControlMode>(getValue()));
            break;
        case Screen::Type::MIN_RPM:
            eeprom.setMinRPM(getValue());
            break;
        case Screen::Type::MAX_RPM:
            eeprom.setMaxRPM(getValue());
            break;
        case Screen::Type::MOSFET_TEMPERATURE_LIMIT:
            eeprom.setMosfetTemperatureLimit(getValue());
            break;
        case Screen::Type::MOTOR_TEMPERATURE_LIMIT:
            eeprom.setMotorTemperatureLimit(getValue());
            break;
        case Screen::Type::MOTOR_SPEED:
            eeprom.setSpeed(getValue());
            break;
        case Screen::Type::MOTOR_STALL_TIMEOUT:
            eeprom.setMotorStallTimeout(getValue());
            break;
        case Screen::Type::PID_KP:
            eeprom.setKp(EEPROM::kUint32ToPIDParam(getValue()));
            pid.applyPIDParams();
            break;
        case Screen::Type::PID_KI:
            eeprom.setKi(EEPROM::kUint32ToPIDParam(getValue()));
            pid.applyPIDParams();
            break;
        case Screen::Type::PID_KD:
            eeprom.setKd(EEPROM::kUint32ToPIDParam(getValue()));
            pid.applyPIDParams();
            break;
        case Screen::Type::PID_ANTI_WINDUP:
            eeprom.setAntiWindup(clampAntiWindupValue());
            pid.applyPIDParams();
            break;
        case Screen::Type::OVP_PROTECTION:
            eeprom.setOvpProtection(getValue());
            break;
        case Screen::Type::PWM_FREQUENCY:
            eeprom.setPWMFrequency(getValue());
            break;
        case Screen::Type::WELCOME_CHIME:
            eeprom.setMotorChime(getValue());
            break;
        case Screen::Type::WELCOME:
        case Screen::Type::EEPROM_SAVED:
        case Screen::Type::MAIN_MENU:
        case Screen::Type::ADVANCED_MENU:
        case Screen::Type::MOTOR_RPM_SETTINGS:
        case Screen::Type::CURRENT_LIMITS:
        case Screen::Type::RESTORE_DEFAULTS_CONFIRMATION:
        case Screen::Type::EEPROM_RESTORED:
        case Screen::Type::DIAGNOSTICS:
        case Screen::Type::PID_PARAMETERS:
            break;
    }
    return getValue();
}

void Menu::updateSpeedValue()
{
    int32_t clampedValue;
    if (eeprom.isPIDMode()) {
        clampedValue = std::clamp<int32_t>(getValue(), eeprom.getMinRPM(), eeprom.getMaxRPM());
        eeprom.setSpeed(clampedValue);
        pid.setRPM(eeprom.getSpeed());
    }
    else {
        clampedValue = std::clamp<int32_t>(getValue(), 0, eeprom.getMaxPWM() * pid.getPWMLevelMax() / 100);
        eeprom.setSpeed(clampedValue);
    }
    // set clamped value
    setValue(clampedValue);
}

int32_t Menu::clampAntiWindupValue()
{
    int32_t clampedValue = std::clamp<int32_t>(getValue(), UIConstants::kMinAntiWindup, UIConstants::kMaxAntiWindup);
    if (clampedValue < UIConstants::kLowestAntiWindup) {
        if (eeprom.getAntiWindup() == 0 && clampedValue > 0) {
            clampedValue = UIConstants::kLowestAntiWindup; // restore 50% of the value is 0
        } else {
            clampedValue = 0; // set to 0 if value is less than 50%
        }
    }
    setValue(clampedValue);
    return clampedValue;
}
