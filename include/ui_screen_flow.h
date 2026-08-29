/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

struct Screen;

// === Screen Flow Manager ===

struct ScreenFlow {

    ScreenFlow();

    /**
     * @brief Initialize the screen flow manager and load empty screen
     *
     */
    void init();

    /**
     * @brief Destroy current screen and load empty screen
     *
     */
    void destroy();

    /**
     * @brief Set a new screen and destroy if the current screen
     *
     * @param newScreen
     */

    void setScreen(Screen *newScreen);

    /**
     * @brief Go back to the previous screen and destroy the current screen
     *
     * If no previous screen is set, this method will not do anything
     */
    void back();

    /**
     * @brief Set a new screen while keeping the current screen in a linked list for back navigation
     *
     * If no previous screen is set, the method behaves like setScreen()
     *
     * @param nextScreen
     */
    void next(Screen *nextScreen);

    /**
     * @brief Return current screen
     *
     * @return Screen* Current screen or nullptr if no screen is set
     */
    Screen *operator->() const;

    /**
     * @brief Return current screen
     *
     * @return Screen* Current screen or nullptr if no screen is set
     */
    Screen *getScreen() const;

    /**
     * @brief Refresh current screen
     *
     */
    static inline void refresh();

protected:
    Screen *screen;
};

inline ScreenFlow::ScreenFlow() : screen(nullptr)
{
}

inline Screen *ScreenFlow::operator->() const
{
    return screen;
}

inline Screen *ScreenFlow::getScreen() const
{
    return screen;
}

inline void ScreenFlow::refresh()
{
    lv_timer_handler();
}
