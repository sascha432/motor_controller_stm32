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

    void updateRotaryValue(int32_t value);

    void loadMainMenu();
    void restorePreviousMenu();

    static ScreenFlow &getScreenFlow();
    static EEPROM &getEEPROM();
};
