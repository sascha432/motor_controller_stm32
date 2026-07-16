/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#include "lvgl.h"
#include "tft_driver.h"
#include <stdint.h>

// set to 1 to create previous screens using Screen::load() instead of lv_scr_load(screen->screen)
// more CPU vs memory usage tradeoff
#define RECREATE_PREV_SCREEN 0

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

// === Base Screen class ===

struct Screen
{
    // Screen Type identifier
    enum class Type {
        WELCOME,
        MAIN_MENU,
        CONTROL_MODE,
        LED_BRIGHTNESS
    };

    // Screen style constants
    static constexpr const lv_font_t *kWelcomeScreenLabelFont = &lv_font_montserrat_18;

    static constexpr const lv_font_t *kMenuScreenLabelFont = &lv_font_montserrat_14;
    static constexpr lv_coord_t kMenuScreenVisibleItems = 5;
    static constexpr lv_coord_t kMenuScreenStartX = 10;
    static constexpr lv_coord_t kMenuScreenStartY = 9;
    static constexpr lv_coord_t kMenuScreenItemHeight = 20;
    static constexpr lv_coord_t kMenuScreenItemWidth = TFT_DIM_WIDTH - (2 * kMenuScreenStartX);
    static constexpr uint8_t kMenuScreenCornerRadius = 4;

    static constexpr const lv_font_t *kSliderScreenLabelFont = &lv_font_montserrat_12;
    static constexpr const lv_font_t *kSliderScreenValueFont = &lv_font_montserrat_12;
    static constexpr lv_coord_t kSliderScreenContainerX = 16;
    static constexpr lv_coord_t kSliderScreenContainerY = 20;
    static constexpr lv_coord_t kSliderScreenContainerWidth = TFT_DIM_WIDTH - 24;
    static constexpr lv_coord_t kSliderScreenTitleBottomGap = 14;
    static constexpr lv_coord_t kSliderScreenSliderHeight = 12;
    static constexpr lv_coord_t kSliderScreenSliderBorder = 2;
    static constexpr lv_coord_t kSliderScreenSliderRadius = 6;
    static constexpr lv_coord_t kSliderScreenKnobSize = 18;
    static constexpr lv_coord_t kSliderScreenValueTopGap = 18;

    Screen(Type id);
    virtual ~Screen();

    Type getId() const;
    virtual void setValue(uint32_t value);
    virtual uint32_t getValue() const;

    virtual void load();

    void style_screen(lv_obj_t *screen);
    void fatal_error(const char *msg);

// protected:
    lv_obj_t *screen;
    Screen *prevScreen;
    Type id;

    static lv_obj_t *emptyScreen;
};

// === Welcome Screen ===

struct WelcomeScreen : public Screen
{
    WelcomeScreen() : Screen(Type::WELCOME) 
    {}

    virtual void load() override;
};

// === Menu Screen ===

struct MenuScreen : public Screen
{
    MenuScreen(Type id, const char **itemLabels, size_t itemCount);

    virtual void load() override;
    virtual void setValue(uint32_t index) override;
    virtual uint32_t getValue() const override;

protected:
    void refreshMenuScreen();
    
private:
    uint8_t _first_visible_start_index(uint8_t selected_index);
    void style_menu_row(lv_obj_t *row, bool selected);
    void style_menu_label(lv_obj_t *label, bool selected);
    lv_obj_t *create_menu_label(lv_obj_t *parent, const char *text, bool selected);

protected:
    lv_obj_t *rows[kMenuScreenVisibleItems];
    lv_obj_t *labels[kMenuScreenVisibleItems];
    const char **itemLabels;
    uint8_t count;
    uint8_t selected;
};

// === Slider Screen ===

struct SliderScreen : public Screen
{
    SliderScreen(Type id, const char *label, uint32_t minValue, uint32_t maxValue, const char *unit) : 
        Screen(id),
        value(minValue),
        minValue(minValue),
        maxValue(maxValue),
        label(label),
        unit(unit),
        sliderFill(nullptr),
        sliderKnob(nullptr),
        valueLabel(nullptr)
    {
    }

    virtual void load() override;
    virtual void setValue(uint32_t value) override;
    virtual uint32_t getValue() const override;

private:
    void refreshVisuals();

private:
    uint32_t value;
    uint32_t minValue;
    uint32_t maxValue;
    const char *label;
    const char *unit;
    lv_obj_t *sliderFill;
    lv_obj_t *sliderKnob;
    lv_obj_t *valueLabel;
};

// === Screen Flow Manager ===

struct ScreenFlow {

    ScreenFlow();

    void init();
    void destroy();
    void setScreen(Screen *newScreen);
    Screen *getScreen() const;
    void back();
    void next(Screen *nextScreen);

    Screen *operator->() const;

protected:
    Screen *screen;
};

