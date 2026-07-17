/**
  Author: sascha_lammers@gmx.de
*/

#include <stdio.h>
#include "menu.h"
#include "ui.h"
#include "eeprom.h"

ScreenFlow screenFlow;
EEPROM eeprom;

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

static const char *kControlModeItems[] = {
    "PWM / Open Loop",          // 0
    "PID / Closed Loop"         // 1
};

static const char *kAdvancedMenuItems[] = {
    "TFT Brightness",           // 0
    "MOSFET Temperature",       // 1
    "Motor Temperature",        // 2
    "Motor RPM Settings",       // 3
    "Motor Direction",          // 4
    "Main Menu"                 // 5
};

static const char *kMotorRPMSettingsItems[] = {
    "Min RPM",                  // 0
    "Max RPM"                   // 1
};

static const char *kMotorDirectionItems[] = {
    "Forward",                  // 0
    "Reverse"                   // 1
};

static const char *kCurrentLimitItems[] = {
    "Input Current Limit",      // 0
    "Motor Current Limit"       // 1
};

// static const char *kYesNoMenuItems[] = {
//     "Yes",                      // 0
//     "No"                        // 1
// };

// TODO remove globals later
void setRotaryMultiplier(uint16_t multiplier);
void setRotaryValue(int32_t value);
int32_t getRotaryValue();
void led_pwm_set(uint8_t value);

// custom format for float
static const char *current_slider_format_callback(uint32_t value, char *buf, size_t bufSize) 
{
    snprintf(buf, bufSize, "%.2f A", EEPROM::kUint16ToCurrent(value));
    return buf;
}

ScreenFlow &Menu::getScreenFlow()
{
    return screenFlow;
}

EEPROM &Menu::getEEPROM()
{
    return eeprom;
}

void Menu::restorePreviousMenu()
{
    screenFlow.back(); // restore previous screen
    setRotaryMultiplier(1); // set rotary multiplier back to 1 for normal operation
    setRotaryValue(screenFlow->getValue()); // restore previous menu position
}

void Menu::handleButtonPress()
{
    // Handle button press based on the current screen
    switch(screenFlow->getId()) {
        // === main menu ===
        case Screen::Type::MAIN_MENU:
            switch(screenFlow->getValue()) {
                case 1: // Control Mode
                    screenFlow.next(new MenuScreen(
                        Screen::Type::CONTROL_MODE, 
                        kControlModeItems, 
                        sizeof(kControlModeItems) / sizeof(kControlModeItems[0])
                    ));
                    setRotaryValue(0);
                    break;
                case 3: // Current Limits
                    screenFlow.next(new MenuScreen(
                        Screen::Type::CURRENT_LIMITS, 
                        kCurrentLimitItems, 
                        sizeof(kCurrentLimitItems) / sizeof(kCurrentLimitItems[0])
                    ));
                    setRotaryValue(0);
                    break;
                case 2: // LED Brightness
                    screenFlow.next(new SliderScreen(
                        Screen::Type::LED_BRIGHTNESS, 
                        "LED Brightness", 
                        UIConstants::kMinLEDBrightness, 
                        UIConstants::kMaxLEDBrightness, 
                        "%"
                    ));
                    setRotaryValue(eeprom.getLEDBrightness());
                    break;
                case 6: // Advanced
                    screenFlow.next(new MenuScreen(
                        Screen::Type::ADVANCED_MENU, 
                        kAdvancedMenuItems, 
                        sizeof(kAdvancedMenuItems) / sizeof(kAdvancedMenuItems[0])
                    ));
                    setRotaryValue(0);
                    break;
                case 8:
                    eeprom.write(eeprom.getData());
                    //TODO return to start screen
                    break;
            }
            break;
        // === control mode menu ===
        case Screen::Type::CONTROL_MODE:
            eeprom.setControlMode(screenFlow->getValue());
            restorePreviousMenu();
            break;
        // === current limits menu ===
        case Screen::Type::CURRENT_LIMITS:
            switch(screenFlow->getValue()) {
                case 0: // Input Current Limit
                    screenFlow.next(new SliderScreen(
                        Screen::Type::INPUT_CURRENT_LIMIT, 
                        "Input Current Limit", 
                        eeprom.kCurrentToUint16(UIConstants::kMinInputCurrent), 
                        eeprom.kCurrentToUint16(UIConstants::kMaxInputCurrent), 
                        "A", 
                        current_slider_format_callback
                    ));
                    setRotaryMultiplier(512);
                    setRotaryValue(eeprom.getInputCurrentLimit());
                    break;
                case 1: // Motor Current Limit
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MOTOR_CURRENT_LIMIT, 
                        "Motor Current Limit", 
                        eeprom.kCurrentToUint16(UIConstants::kMinMotorCurrent), 
                        eeprom.kCurrentToUint16(UIConstants::kMaxMotorCurrent), 
                        "A", 
                        current_slider_format_callback
                    ));
                    setRotaryMultiplier(512);
                    setRotaryValue(eeprom.getMotorCurrentLimit());
                    break;
            }
            break;
        // === input current limits menu ===
        case Screen::Type::INPUT_CURRENT_LIMIT:
            eeprom.setInputCurrentLimit(screenFlow->getValue());
            restorePreviousMenu();
            break;
        // === motor current limits menu ===
        case Screen::Type::MOTOR_CURRENT_LIMIT:
            eeprom.setMotorCurrentLimit(screenFlow->getValue());
            restorePreviousMenu();
            break;
        // === LED brightness menu ===
        case Screen::Type::LED_BRIGHTNESS:
            eeprom.setLEDBrightness(screenFlow->getValue());    // TODO redundant, already set in live update
            led_pwm_set(eeprom.getLEDBrightness());             // TODO redundant, already set in live update
            restorePreviousMenu();
            break;
        // === advanced menu ===
        case Screen::Type::ADVANCED_MENU:
            switch(screenFlow->getValue()) {
                case 0: // TFT Brightness
                    screenFlow.next(new SliderScreen(
                        Screen::Type::TFT_BRIGHTNESS, "TFT Brightness", 
                        UIConstants::kMinTFTBrightness, 
                        UIConstants::kMaxTFTBrightness, 
                        "%"
                    ));
                    setRotaryValue(eeprom.getTFTBrightness());
                    break;
                case 3: // Motor RPM Settings
                    screenFlow.next(new MenuScreen(
                        Screen::Type::MOTOR_RPM_SETTINGS, 
                        kMotorRPMSettingsItems, 
                        sizeof(kMotorRPMSettingsItems) / sizeof(kMotorRPMSettingsItems[0])
                    ));
                    setRotaryValue(0);
                    break;
                case 4: // Motor Direction
                    screenFlow.next(new MenuScreen(
                        Screen::Type::MOTOR_DIRECTION, 
                        kMotorDirectionItems, 
                        sizeof(kMotorDirectionItems) / sizeof(kMotorDirectionItems[0])
                    ));
                    setRotaryValue(0);
                    break;
                case 5: // Main Menu
                    restorePreviousMenu();
                    break;
            }
            break;
        // === TFT brightness menu ===
        case Screen::Type::TFT_BRIGHTNESS:
            eeprom.setTFTBrightness(screenFlow->getValue());    // TODO redundant, already set in live update
            tft_backlight_pwm_set(eeprom.getTFTBrightness());   // TODO redundant, already set in live update
            restorePreviousMenu();
            break;
        // === motor direction menu ===
        case Screen::Type::MOTOR_DIRECTION:
            eeprom.setMotorDirection(screenFlow->getValue());
            restorePreviousMenu();
            break;
        // === motor RPM settings menu ===
        case Screen::Type::MOTOR_RPM_SETTINGS:
            switch(screenFlow->getValue()) {
                case 0: // Min RPM
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MIN_RPM, 
                        "Min RPM", 
                        UIConstants::kMinRPM, 
                        UIConstants::kMaxRPM, 
                        "RPM"
                    ));
                    setRotaryValue(eeprom.getMinRPM());
                    break;
                case 1: // Max RPM
                    screenFlow.next(new SliderScreen(
                        Screen::Type::MAX_RPM, 
                        "Max RPM", 
                        UIConstants::kMinRPM, 
                        UIConstants::kMaxRPM, 
                        "RPM"
                    ));
                    setRotaryValue(eeprom.getMaxRPM());
                    break;
            }
            break;
        default:
            break;
    }
}

void Menu::loadMainMenu()
{
    screenFlow.setScreen(new MenuScreen(
        Screen::Type::MAIN_MENU, 
        kMainMenuItems, 
        sizeof(kMainMenuItems) / sizeof(kMainMenuItems[0])
    ));
    setRotaryValue(0);
}

void Menu::updateRotaryValue(int32_t value)
{
    screenFlow->setValue(value);
    switch(screenFlow->getId()) {
        case Screen::Type::TFT_BRIGHTNESS:
            eeprom.setTFTBrightness(screenFlow->getValue());
            tft_backlight_pwm_set(eeprom.getTFTBrightness());
            break;
        case Screen::Type::LED_BRIGHTNESS:
            eeprom.setLEDBrightness(screenFlow->getValue());
            led_pwm_set(eeprom.getLEDBrightness());
            break;
    }
}
