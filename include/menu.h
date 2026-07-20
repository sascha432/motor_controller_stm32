/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include <stdint.h>

struct EEPROM;
struct ScreenFlow;

// === Menu class ===
struct Menu 
{
    Menu();

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
    void loadDashboardScreen();
    void restorePreviousMenu();
    void saveEEPROMChanges();

    ScreenFlow &getScreenFlow();
    void abortableDelay(uint32_t ms);

protected:
    int32_t steps;
    EEPROM &eeprom;
};

extern Menu menu;
