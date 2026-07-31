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

static const char *kAdvancedMenuItems[] = {
    "TFT Brightness",           // 0
    "MOSFET Temperature",       // 1
    "Motor Temperature",        // 2
    "Motor RPM Settings",       // 3
    "Motor Direction",          // 4
    "Sensor Direction",         // 5
    "PID Parameters",           // 6
    "OVP Protection",           // 7
    "Diagnostics",              // 8
    "Back"                      // 9
};

static const char *kPIDParametersItems[] = {
    "Kp",                       // 0
    "Ki",                       // 1
    "Kd",                       // 2
    "Anti-Windup",              // 3
    "Back"                      // 4
};

static const char *kMotorRPMSettingsItems[] = {
    "Minimum RPM",              // 0
    "Maximum RPM",              // 1
    "Back"                      // 2
};

static const char *kCurrentLimitItems[] = {
    "Input Current Limit",      // 0
    "Motor Current Limit",      // 1
    "Back"                      // 2
};

static const char *kControlModeItems[] = {
    "PWM / Open Loop",          // 0
    "PID / Closed Loop"         // 1
};

static const char *kDirectionItems[] = {
    "Forward",                  // 0
    "Reverse"                   // 1
};

static const char *kRestoreDefaultsMenuItems[] = {
    "Restore",                  // 0
    "Cancel"                    // 1
};

// custom format for current and conversion from uint16_t to float
static const char *current_slider_format_callback(uint32_t value, char *buf, size_t bufSize)
{
    snprintf(buf, bufSize, SPRINTF_FP1_FMT " A", CONVERT_TO_FP1(value));
    return buf;
}

// custom format for current and conversion from uint16_t to float
static const char *anti_windup_format_callback(uint32_t value, char *buf, size_t bufSize)
{
    const uint32_t antiWindup = value * (1000 / UIConstants::kAntiWindupFactor); // value = percentage * 100, so multiply by 10 to get percentage * 1000
    snprintf(buf, bufSize, SPRINTF_FP2_FMT " %%", CONVERT_TO_FP2(antiWindup));
    return buf;
}

// custom format for PID parameters and conversion from uint32_t to float
static const char *pid_parameter_format_callback(uint32_t value, char *buf, size_t bufSize)
{
    FloatToString::convertTrimmed(buf, bufSize, value / UIConstants::kPIDParamFactor, 6);
    return buf;
}

// custom format for OVP protection and conversion from uint32_t to float
static const char *format_ovp_protection_callback(uint32_t value, char *buf, size_t bufSize)
{
    snprintf(buf, bufSize, SPRINTF_FP1_FMT " V", CONVERT_TO_FP1(value));
    return buf;
}

/**
 * @brief Menu constructor
 *
 */
Menu::Menu()
{
}

/**
 * @brief Restore previous menu screen and selected item
 *
 */
void Menu::restorePreviousMenu()
{
    DEBUG_PRINT(DebugType::UI, "value=%d", getValue());
    screenFlow.back(); // restore previous screen
    clearUserInput();
}

/**
 * @brief Show welcome screen for a few seconds
 *
 */
void Menu::showWelcomeScreen()
{
    // Show welcome screen for a few seconds
    screenFlow.init();
    screenFlow.setScreen(new WelcomeScreen());
    lv_timer_handler();
    tft_backlight_pwm_set(eeprom.getTFTBrightness());

    if (UIConstants::kEnableIlluminationLEDFading) {
        // gradually increase LED brightness to target value
        uint32_t start = HAL_GetTick();
        uint8_t targetBrightness = eeprom.getLEDBrightness();
        float currentBrightness = 0;
        float step = targetBrightness / (UIConstants::kWelcomeScreenTimeout / 8.0f);
        targetBrightness -= step;
        for(;;) {
            uint32_t elapsed = HAL_GetTick() - start;
            if (elapsed >= UIConstants::kWelcomeScreenTimeout) {
                break;
            }
            if (currentBrightness < targetBrightness) {
                currentBrightness += step;
            }
            LEDs::illuminationLedSetPWM(currentBrightness);
            // blink motor LEDs
            ((elapsed / 500) & 0x01) ? LEDs::onLEDError() : LEDs::onLEDWarning();
            WatchDog::delay(8);
        }
        LEDs::off();
    }
    else {
        WatchDog::delay(UIConstants::kWelcomeScreenTimeout);
    }

    clearUserInput();
}

/**
 * @brief Load main menu screen
 *
 */
void Menu::loadMainMenu()
{
    screenFlow.setScreen(new MenuScreen(
        Screen::Type::MAIN_MENU,
        kMainMenuItems,
        sizeof_array(kMainMenuItems)
    ));
    setValue(0);
    clearUserInput();
}

/**
 * @brief Start motor screen with some info
 *
 */
void Menu::loadStartScreen()
{
    screenFlow.setScreen(new StartScreen());
    setValue(eeprom.getSpeed());
    // save any changes when returning to start screen
    bool success = eeprom.write();
    (void)success;
    DEBUG_PRINT(DebugType::UI, "eeprom.write=%d", success);
    clearUserInput();
}

/**
 * @brief Show motor dashboard screen with speed and other info while running
 *
 */
void Menu::loadDashboardScreen()
{
    screenFlow.setScreen(new DashboardScreen());
    setValue(eeprom.getSpeed());
    clearUserInput();
}

/**
 * @brief Update rotary encoder value and set screen value
 *
 * @param value
 */
void Menu::setValue(int32_t value)
{
    screenFlow->setValue(value);
}

/**
 * @brief Get screen value
 *
 * @return int32_t
 */
int32_t Menu::getValue() const
{
    return screenFlow->getValue();
}

/**
 * @brief Return reference to the ScreenFlow object
 *
 * @return ScreenFlow&
 */
ScreenFlow &Menu::getScreenFlow()
{
    return screenFlow;
}

void Menu::abortableDelay(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ms) {
        WatchDog::feed();
        if (isAnyButtonDown()) {
            while(isAnyButtonDown()) {
                WatchDog::feed();
            }
            clearUserInput();
            break;
        }
    }
}

bool Menu::isAnyButtonDown()
{
    return knobButton.isDown() || backButton.isDown() || startButton.isDown();
}

void Menu::clearUserInput()
{
    // clear states
    knobButton.reset();
    backButton.reset();
    startButton.reset();
    knob.reset();
}

/**
 * @brief Save changes to EEPROM and display info screen in case of changes written
 *
 */
void Menu::saveEEPROMChanges()
{
    if (eeprom.write()) {
        screenFlow.setScreen(new InfoScreen(Screen::Type::EEPROM_SAVED, "Saved"));
        lv_timer_handler();
        abortableDelay(UIConstants::kInfoScreenTimeout);
    }
}
/**
 * @brief Apply settings from EEPROM to the system after initialization or after restoring defaults
 *
 */
void Menu::applyEEPROMSettings()
{
    tft_backlight_pwm_set(eeprom.getTFTBrightness());
    LEDs::illuminationLedSetPWM(eeprom.getLEDBrightness());
    adc.setInputCurrentLimit(eeprom.getInputCurrentLimit());
    adc.setMotorCurrentLimit(eeprom.getMotorCurrentLimit());
    pid.applyPIDParams();
}

/**
 * @brief Handle main button press based on the current screen and selected item
 *
 */
void Menu::handleButtonPress()
{
    DEBUG_PRINT(DebugType::UI, "enter screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
    Screen *screen;
    // Handle button press based on the current screen
    switch(screenFlow->getId()) {
        // === start screen ===
        case Screen::Type::START:
            loadMainMenu();
            break;
        // === main menu ===
        case Screen::Type::MAIN_MENU:
            switch(getValue()) {
                case 0: // Speed
                    switch(eeprom.getControlMode()) {
                        case EEPROM::kControlModePID:
                            screenFlow.next(new SliderScreen(
                                Screen::Type::MOTOR_SPEED,
                                "Motor Speed",
                                eeprom.getMinRPM(),
                                eeprom.getMaxRPM(),
                                "RPM"
                            ));
                            break;
                        case EEPROM::kControlModePWM:
                            screenFlow.next(new SliderScreen(
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
                case 1: // Control Mode
                    screenFlow.next(new MenuScreen(
                        Screen::Type::CONTROL_MODE,
                        kControlModeItems,
                        sizeof_array(kControlModeItems)
                    ));
                    setValue(eeprom.getControlMode());
                    break;
                case 2: // LED Brightness
                    screenFlow.next(new SliderScreen(
                        Screen::Type::LED_BRIGHTNESS,
                        "LED Brightness",
                        UIConstants::kMinLEDBrightness,
                        UIConstants::kMaxLEDBrightness,
                        "%"
                    ));
                    setValue(eeprom.getLEDBrightness());
                    break;
                case 3: // Current Limits
                    screenFlow.next(new MenuScreen(
                        Screen::Type::CURRENT_LIMITS,
                        kCurrentLimitItems,
                        sizeof_array(kCurrentLimitItems)
                    ));
                    setValue(0);
                    break;
                case 4: // Stall Time
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MOTOR_STALL_TIMEOUT,
                        "Motor Stall Timeout",
                        UIConstants::kMinMotorStallTimeout,
                        UIConstants::kMaxMotorStallTimeout,
                        "ms"
                    ));
                    setValue(eeprom.getMotorStallTimeout());
                    break;
                case 5: // Motor Brake
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MOTOR_BRAKE,
                        "Motor Brake",
                        0,
                        100,
                        "%"
                    ));
                    setValue(eeprom.getMotorBrake());
                    break;
                case 6: // Advanced
                    screenFlow.next(new MenuScreen(
                        Screen::Type::ADVANCED_MENU,
                        kAdvancedMenuItems,
                        sizeof_array(kAdvancedMenuItems)
                    ));
                    setValue(0);
                    break;
                case 7: // Restore Defaults
                    screenFlow.next(new MenuScreen(
                        Screen::Type::RESTORE_DEFAULTS_CONFIRMATION,
                        kRestoreDefaultsMenuItems,
                        sizeof_array(kRestoreDefaultsMenuItems)
                    ));
                    setValue(1);
                    break;
                case 8:
                    saveEEPROMChanges();
                    loadStartScreen();
                    break;
            }
            break;
        // === current limits menu ===
        case Screen::Type::CURRENT_LIMITS:
            switch(getValue()) {
                case 0: // Input Current Limit
                    screen = new SliderScreen(
                        Screen::Type::INPUT_CURRENT_LIMIT,
                        "Input Current Limit",
                        UIConstants::kMinInputCurrent,
                        UIConstants::kMaxInputCurrent,
                        "A",
                        current_slider_format_callback
                    );
                    screen->setSteps(UIConstants::kStepInputCurrent);
                    screenFlow.next(screen);
                    setValue(eeprom.getInputCurrentLimit());
                    break;
                case 1: // Motor Current Limit
                    screen = new SliderScreen(
                        Screen::Type::MOTOR_CURRENT_LIMIT,
                        "Motor Current Limit",
                        UIConstants::kMinMotorCurrent,
                        UIConstants::kMaxMotorCurrent,
                        "A",
                        current_slider_format_callback
                    );
                    screen->setSteps(UIConstants::kStepMotorCurrent);
                    screenFlow.next(screen);
                    setValue(eeprom.getMotorCurrentLimit());
                    break;
                default: // Back
                    restorePreviousMenu();
                    break;
            }
            break;
        // === advanced menu ===
        case Screen::Type::ADVANCED_MENU:
            switch(getValue()) {
                case 0: // TFT Brightness
                    screenFlow.next(new SliderScreen(
                        Screen::Type::TFT_BRIGHTNESS, "TFT Brightness",
                        UIConstants::kMinTFTBrightness,
                        UIConstants::kMaxTFTBrightness,
                        "%"
                    ));
                    setValue(eeprom.getTFTBrightness());
                    break;
                case 1: // MOSFET Temperature
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MOSFET_TEMPERATURE_LIMIT,
                        "MOSFET Temperature Limit",
                        UIConstants::kMinMosfetTemperature,
                        UIConstants::kMaxMosfetTemperature,
                        DEGREE_UTF8 "C"
                    ));
                    setValue(eeprom.getMosfetTemperatureLimit());
                    break;
                case 2: // Motor Temperature
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MOTOR_TEMPERATURE_LIMIT,
                        "Motor Temperature Limit",
                        UIConstants::kMinMotorTemperature,
                        UIConstants::kMaxMotorTemperature,
                        DEGREE_UTF8 "C"
                    ));
                    setValue(eeprom.getMotorTemperatureLimit());
                    break;
                case 3: // Motor RPM Settings
                    screenFlow.next(new MenuScreen(
                        Screen::Type::MOTOR_RPM_SETTINGS,
                        kMotorRPMSettingsItems,
                        sizeof_array(kMotorRPMSettingsItems)
                    ));
                    setValue(0);
                    break;
                case 4: // Motor Direction
                    screenFlow.next(new MenuScreen(
                        Screen::Type::MOTOR_DIRECTION,
                        kDirectionItems,
                        sizeof_array(kDirectionItems)
                    ));
                    setValue(eeprom.getMotorDirection());
                    break;
                case 5: // Sensor Direction
                    screenFlow.next(new MenuScreen(
                        Screen::Type::SENSOR_DIRECTION,
                        kDirectionItems,
                        sizeof_array(kDirectionItems)
                    ));
                    setValue(eeprom.getSensorDirection());
                    break;
                case 6: // PID Parameters
                    screenFlow.next(new MenuScreen(
                        Screen::Type::PID_PARAMETERS,
                        kPIDParametersItems,
                        sizeof_array(kPIDParametersItems)
                    ));
                    setValue(0);
                    break;
                case 7: // OVP Protection
                    screenFlow.next(new SliderScreen(
                        Screen::Type::OVP_PROTECTION,
                        "OVP Protection",
                        UIConstants::kMinOvpProtection,
                        UIConstants::kMaxOvpProtection,
                        "V",
                        format_ovp_protection_callback
                    ));
                    setValue(eeprom.getOvpProtection());
                    break;
                case 8: // Diagnostics
                    screenFlow.next(new DiagnosticsScreen(Screen::Type::DIAGNOSTICS));
                    setValue(0);
                    break;
                default: // Back
                    restorePreviousMenu();
                    break;
            }
            break;
        // === motor RPM settings menu ===
        case Screen::Type::MOTOR_RPM_SETTINGS:
            switch(getValue()) {
                case 0: // Min RPM
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MIN_RPM,
                        "Min RPM",
                        UIConstants::kMinRPM,
                        UIConstants::kMaxRPM,
                        "RPM"
                    ));
                    setValue(eeprom.getMinRPM());
                    break;
                case 1: // Max RPM
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MAX_RPM,
                        "Max RPM",
                        UIConstants::kMinRPM,
                        UIConstants::kMaxRPM,
                        "RPM"
                    ));
                    setValue(eeprom.getMaxRPM());
                    break;
                default: // Back
                    restorePreviousMenu();
                    break;
            }
            break;
        // === pid parameters menu ===
        case Screen::Type::PID_PARAMETERS:
            switch(getValue()) {
                case 0: // Kp
                    screenFlow.next(new PidSliderScreen(
                        Screen::Type::PID_KP,
                        "Kp Parameter",
                        UIConstants::kMinKp,
                        UIConstants::kMaxKp,
                        pid_parameter_format_callback,
                        UIConstants::kSelectKpSteps
                    ));
                    setValue(EEPROM::kPIDParamToUint32(eeprom.getKp()));
                    break;
                case 1: // Ki
                    screenFlow.next(new PidSliderScreen(
                        Screen::Type::PID_KI,
                        "Ki Parameter",
                        UIConstants::kMinKi,
                        UIConstants::kMaxKi,
                        pid_parameter_format_callback,
                        UIConstants::kSelectKiSteps
                    ));
                    setValue(EEPROM::kPIDParamToUint32(eeprom.getKi()));
                    break;
                case 2: // Kd
                    screenFlow.next(new PidSliderScreen(
                        Screen::Type::PID_KD,
                        "Kd Parameter",
                        UIConstants::kMinKd,
                        UIConstants::kMaxKd,
                        pid_parameter_format_callback,
                        UIConstants::kSelectKdSteps
                    ));
                    setValue(EEPROM::kPIDParamToUint32(eeprom.getKd()));
                    break;
                case 3: // Anti-Windup
                    screenFlow.next(new SliderScreen(
                        Screen::Type::PID_ANTI_WINDUP,
                        "Anti-Windup",
                        UIConstants::kMinAntiWindup,
                        UIConstants::kMaxAntiWindup,
                        "%",
                        anti_windup_format_callback
                    ));
                    setValue(eeprom.getAntiWindup());
                    break;
                default: // Back
                    restorePreviousMenu();
                    break;
            }
            break;
        // === restore defaults confirmation menu ===
        case Screen::Type::RESTORE_DEFAULTS_CONFIRMATION:
            switch(getValue()) {
                case 0: // Restore
                    eeprom.resetDefaults();
                    eeprom.write();
                    menu.applyEEPROMSettings();
                    screenFlow.next(new InfoScreen(Screen::Type::EEPROM_RESTORED, "Restored"));
                    lv_timer_handler();
                    abortableDelay(UIConstants::kInfoScreenTimeout);
                    loadMainMenu();
                    break;
                default: // Cancel
                    restorePreviousMenu();
                    break;
            }
            break;
        // === diagnostics screen ===
        case Screen::Type::DASHBOARD:
            {
                DashboardScreen &dashboard = *reinterpret_cast<DashboardScreen *>(screenFlow.getScreen());
                switch(dashboard.incrSelectedValue()) {
                    case DashboardScreen::SelectedValueType::SPEED:
                        dashboard.setMaxAcceleration(eeprom.isPIDMode() ? 50 : 1);
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
                        dashboard.setMaxAcceleration(sqrt(UIConstants::kMaxAntiWindup - UIConstants::kMinAntiWindup));
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
            restorePreviousMenu();
            break;
        default:
            DEBUG_PRINT(DebugType::ERROR, "MainMenu: unhandled id: %d", static_cast<int>(screenFlow->getId()));
            break;
    }
    DEBUG_PRINT(DebugType::UI, "leave screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
}

/**
 * @brief Handle back button press
 *
 */
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
        default:
            restorePreviousMenu();
            break;
    }
    DEBUG_PRINT(DebugType::UI, "leave screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
}

/**
 * @brief Handle start button press
 */
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
            break;
    }
    DEBUG_PRINT(DebugType::UI, "leave screen=%p id=%d value=%d", screenFlow.getScreen(), static_cast<int>(screenFlow->getId()), getValue());
}


/**
 * @brief Call to update menu position from rotary encoder
 *
 * @param value
 */
int32_t Menu::updateRotaryValue(int32_t value)
{
    int32_t clampedValue;
    screenFlow->setValue(screenFlow->getValue() + (value * screenFlow->getSteps()));
    switch(screenFlow->getId()) {
        case Screen::Type::DASHBOARD:
            switch(reinterpret_cast<DashboardScreen *>(screenFlow.getScreen())->getSelectedValue()) {
                case DashboardScreen::SelectedValueType::SPEED:
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
                    clampedValue = std::clamp<int32_t>(getValue(), UIConstants::kMinAntiWindup, UIConstants::kMaxAntiWindup);
                    eeprom.setAntiWindup(clampedValue);
                    pid.applyPIDParams();
                    setValue(clampedValue);
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
            LEDs::illuminationLedSetPWM(eeprom.getLEDBrightness());
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
            eeprom.setMotorDirection(getValue());
            break;
        case Screen::Type::SENSOR_DIRECTION:
            eeprom.setSensorDirection(getValue());
            break;
        case Screen::Type::MOTOR_BRAKE:
            eeprom.setMotorBrake(getValue());
            break;
        case Screen::Type::CONTROL_MODE:
            eeprom.setControlMode(getValue());
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
            eeprom.setKp(EEPROM::kUint32ToPIDParam(value));
            pid.applyPIDParams(); // apply or live tuning otherwise only a reset will apply the new values
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
            eeprom.setAntiWindup(getValue());
            pid.applyPIDParams();
            break;
        case Screen::Type::OVP_PROTECTION:
            eeprom.setOvpProtection(getValue());
            break;
        case Screen::Type::WELCOME:
        case Screen::Type::EEPROM_SAVED:
        case Screen::Type::MAIN_MENU:
        case Screen::Type::ADVANCED_MENU:
        case Screen::Type::MOTOR_RPM_SETTINGS:
        case Screen::Type::CURRENT_LIMITS:
        case Screen::Type::CONTROL_MODE_PWM:
        case Screen::Type::CONTROL_MODE_PID:
        case Screen::Type::RESTORE_DEFAULTS_CONFIRMATION:
        case Screen::Type::EEPROM_RESTORED:
        case Screen::Type::DIAGNOSTICS:
        case Screen::Type::PID_PARAMETERS:
        default:
            break;
    }

    return getValue();
}

/**
 * @brief Update RPM or PWM value
 *
 */
void Menu::updateSpeedValue()
{
    int32_t clampedValue;
    if (eeprom.isPIDMode()) {
        clampedValue = std::clamp<int32_t>(getValue(), eeprom.getMinRPM(), eeprom.getMaxRPM());
        eeprom.setSpeed(clampedValue);
        pid.setRPM(eeprom.getSpeed());
    }
    else {
        clampedValue = std::clamp<int32_t>(getValue(), 0, eeprom.getMaxPWM() * pid.kMaxPWMLevel / 100);
        eeprom.setSpeed(clampedValue);
    }
    // set clamped value
    setValue(clampedValue);
}
