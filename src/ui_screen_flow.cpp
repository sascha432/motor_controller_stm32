/**
  Author: sascha_lammers@gmx.de

  UI implementation using LVGL library
*/

#include "ui.h"
#include "controls.h"
#include "menu.h"

// === Screen Flow Manager ===

ScreenFlow::ScreenFlow() : screen(nullptr)
{
}

void ScreenFlow::init()
{
    Screen::emptyScreen = lv_obj_create(nullptr);
    lv_scr_load(Screen::emptyScreen);
}

void ScreenFlow::destroy()
{
    DEBUG_PRINT(DEBUG_DEBUG, "screen=%p", screen);
    if (screen) {
        lv_scr_load(Screen::emptyScreen);
        delete screen;
    }
    screen = nullptr;
}

void ScreenFlow::setScreen(Screen *newScreen)
{
    DEBUG_PRINT(DEBUG_DEBUG, "new=%p old=%p", newScreen, screen);
    destroy();
    screen = newScreen;
    screen->load();
}

void ScreenFlow::back()
{
    DEBUG_PRINT(DEBUG_DEBUG, "prev=%p current=%p", screen->prevScreen, screen);
    if (screen->prevScreen) {
        lv_scr_load(Screen::emptyScreen);
        auto tmp = screen->prevScreen;
        delete screen;
        screen = tmp;
        if (kUIKeepScreenObjectsInMemory) {
            lv_scr_load(screen->screen);
            knob.setMaxAcceleration(screen->maxAcceleration);
        }
        else {
            screen->load();
        }
    }
}

void ScreenFlow::next(Screen *nextScreen)
{
    DEBUG_PRINT(DEBUG_DEBUG, "next=%p current=%p", nextScreen, screen);
    nextScreen->prevScreen = screen;
    if (kUIKeepScreenObjectsInMemory == false && nextScreen->prevScreen) {
        lv_obj_clean(nextScreen->prevScreen->screen);
    }
    screen = nextScreen;
    screen->load();
}

Screen *ScreenFlow::operator->() const 
{
    return screen;
}

Screen *ScreenFlow::getScreen() const
{
    return screen;
}
