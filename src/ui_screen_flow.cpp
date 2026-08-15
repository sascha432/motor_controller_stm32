/**
  Author: sascha_lammers@gmx.de

  UI implementation using LVGL library
*/

#include "ui.h"
#include "controls.h"
#include "menu.h"

// === Screen Flow Manager ===

void ScreenFlow::init()
{
    Screen::emptyScreen = lv_obj_create(nullptr);
    lv_scr_load(Screen::emptyScreen);
}

void ScreenFlow::destroy()
{
    DEBUG_PRINT(DebugType::UI, "ScreenFlow::destroy() screen=%p", screen);
    if (screen) {
        lv_scr_load(Screen::emptyScreen);
        delete screen;
    }
    screen = nullptr;
}

void ScreenFlow::setScreen(Screen *newScreen)
{
    DEBUG_PRINT(DebugType::UI, "ScreenFlow::setScreen(Screen *newScreen = %p) old=%p", newScreen, screen);
    destroy();
    screen = newScreen;
    screen->load();
}

void ScreenFlow::back()
{
    DEBUG_PRINT(DebugType::UI, "ScreenFlow::back() prev=%p current=%p", screen->prevScreen, screen);
    if (screen->prevScreen) {
        lv_scr_load(Screen::emptyScreen);
        auto tmp = screen->removePrevScreen();
        delete screen;
        screen = tmp;
        screen->load();
    }
}

void ScreenFlow::next(Screen *nextScreen)
{
    DEBUG_PRINT(DebugType::UI, "ScreenFlow::next(nextScreen = %p)  current=%p", nextScreen, screen);
    if (screen) {
        lv_obj_clean(screen->screen);
    }
    nextScreen->prevScreen = screen;
    screen = nextScreen;
    screen->load();
}
