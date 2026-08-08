/**
 Author: sascha_lammers@gmx.de
*/

#pragma once

#include "controls.h"

struct EEPROM;
struct ScreenFlow;

// === Menu class ===
struct Menu
{
    /**
     * @brief Set screen value
     *
     * @param value
     */
    void setValue(int32_t value);

    /**
     * @brief Get screen value
     *
     * @return int32_t
     */
    int32_t getValue() const;

    /**
     * @brief Show welcome screen
     *
     */
    void loadWelcomeScreen();

    /**
     * @brief Load main menu screen
     *
     */
    void loadMainMenu();

    /**
     * @brief Load advanced menu screen
     *
     */
    void loadAdvancedMenu();

    /**
     * @brief Handle custom exit behaviour for advanced menu depending on prev. screen
     *
     */
    void exitAdvancedMenu();

    /**
     * @brief Start motor screen with some info
     *
     */
    void loadStartScreen();

    /**
     * @brief Show motor dashboard screen with speed and other info while running
     *
     */
    void loadDashboardScreen();

    /**
     * @brief Restore previous menu screen and selected item
     *
     */
    void restorePreviousMenu();

    /**
     * @brief Return reference to the ScreenFlow object
     *
     * @return ScreenFlow&
     */
    ScreenFlow &getScreenFlow();

    /**
     * @brief Wait for a specified time or any button press
     *
     * @param ms Time in milliseconds
     */
    void abortableDelay(uint32_t ms);

    /**
     * @brief Return if any button is currently pressed
     *
     * @return true
     * @return false
     */
    bool isAnyButtonDown() const;

    /**
     * @brief Return if any button has been pressed
     *
     * @return true if any button has been pressed
     * @return false otherwise
     */

    bool hasAnyButtonBeenPressed() const;

    /**
     * @brief Clear button states and rotary encoder position
     *
     */
    void clearUserInput();

    /**
     * @brief Save changes to EEPROM and display info screen in case of changes written
     *
     */
    void saveEEPROMChanges();

    /**
     * @brief Apply settings from EEPROM to the system after initialization or after restoring defaults
     *
     */
    void applyEEPROMSettings();

    /**
     * @brief Handle main button press based on the current screen and selected item
     *
     */
    void handleButtonPress(uint32_t duration);

    /**
     * @brief Handle back button press
     *
     */
    void handleBackButtonPress();

    /**
     * @brief Handle start button press
     *
     */
    void handleStartButtonPress();

    /**
     * @brief Call to update menu position from rotary encoder
     *
     * @param value The new rotary encoder value
     */
    int32_t updateRotaryValue(int32_t value);

protected:
    /**
     * @brief Update RPM or PWM value
     *
     */
    void updateSpeedValue();

    /**
     * @brief Clamp anti-windup value to the allowed range and handle special case for "Disabled" below 50%
     *
     * @return int32_t Clamped anti-windup value
     */
    int32_t clampAntiWindupValue();
};

extern Menu menu;
