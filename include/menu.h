/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>

struct ScreenFlow;
struct EEPROM;

// === Menu class ===
struct Menu 
{
    void handleButtonPress();
    void handleBackButtonPress();
    void handleStartButtonPress();

    int32_t updateRotaryValue(int32_t value);
    void setValue(int32_t value);
    int32_t getValue() const;
    void setSteps(int32_t steps);

    void showWelcomeScreen();
    void loadMainMenu();
    void loadStartScreen();
    void restorePreviousMenu();

    static ScreenFlow &getScreenFlow();
    static EEPROM &getEEPROM();

protected:
    void setRotaryValue(int32_t value);
    int32_t getRotaryValue() const;

    int32_t steps;
};
