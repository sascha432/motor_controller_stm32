/**
  Author: sascha_lammers@gmx.de

  UI implementation using LVGL library
*/

#include "ui.h"
#include "adc.h"
#include <Arduino.h>

// === Base Screen ===

lv_obj_t *Screen::emptyScreen = nullptr;

Screen::Screen(Type id) : 
    screen(nullptr),
    prevScreen(nullptr),
    id(id)
{
    DEBUG_PRINT(DEBUG_DEBUG, "ctor");
}

Screen::~Screen()
{
    DEBUG_PRINT(DEBUG_DEBUG, "screen=%p", screen);
     if (screen) {
        lv_scr_load(emptyScreen);
        lv_obj_del(screen);
    }
}

void Screen::load() 
{
    DEBUG_PRINT(DEBUG_DEBUG, "screen=%p", screen);
    if (screen) {
        auto tmp = screen;
        screen = lv_obj_create(nullptr);
        lv_scr_load(emptyScreen);
        lv_obj_del(tmp);
    }
    else {
        screen = lv_obj_create(nullptr);
    }
    _style_screen(screen);
}

Screen::Type Screen::getId() const 
{
    return id;
}

void Screen::setValue(uint32_t value)
{
    DEBUG_PRINT(DEBUG_DEBUG, "value=%u", value);
}

uint32_t Screen::getValue() const
{
    DEBUG_PRINT(DEBUG_DEBUG, "value=0");
    return 0;
}

void Screen::_fatal_error(const char *msg) 
{
    uint32_t num = 0;
    while (true) {
        digitalWrite(PE5, HIGH);
        delay(100);
        digitalWrite(PE5, LOW);
        delay(100);
        if (num++ % 10 == 0) {
            DEBUG_PRINT(DEBUG_ERROR, "UI ERROR: %s", msg);
        }
    }
}

void Screen::_style_screen(lv_obj_t *screen)
{
    lv_obj_set_style_bg_color(screen, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
}

// === Welcome Screen ===

WelcomeScreen::WelcomeScreen() : 
    InfoScreen(Type::WELCOME, Screen::kWelcomeScreenLabelFont)
{
    char buf[32];
    snprintf(buf, sizeof(buf)- 1, "Version %u.%u.%u", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    message = strdup(buf);
}

// === Info Screen ===

void InfoScreen::load()
{
    DEBUG_PRINT(DEBUG_NOTICE, "message=%s", message ? message : "<NULL>");
    Screen::load();
    lv_obj_t *label = lv_label_create(screen);
    lv_label_set_text(label, message);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_center(label);
    lv_scr_load(screen);
}

// === Menu Screen ===

MenuScreen::MenuScreen(Type id, const char **itemLabels, size_t itemCount) :
    Screen(id),
    rows{},
    labels{},
    itemLabels(itemLabels),
    count(static_cast<uint8_t>(itemCount)),
    selected(0)
{
}

void MenuScreen::load() 
{
    DEBUG_PRINT(DEBUG_NOTICE, "items=%u selected=%u", count, selected);
    Screen::load();
    lv_obj_t *menu = lv_obj_create(screen);

    lv_obj_remove_style_all(menu);
    lv_obj_set_pos(menu, kMenuScreenStartX, kMenuScreenStartY);
    lv_obj_set_size(menu, kMenuScreenItemWidth, kMenuScreenVisibleItems * kMenuScreenItemHeight);
    lv_obj_set_style_bg_opa(menu, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(menu, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(menu, 0, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(menu, true, LV_PART_MAIN);

    for (lv_coord_t i = 0; i < kMenuScreenVisibleItems && i < count; ++i) {
        const bool isSelected = (i == 0);
        rows[i] = lv_obj_create(menu);
        lv_obj_remove_style_all(rows[i]);
        _style_menu_row(rows[i], isSelected);
        lv_obj_set_pos(rows[i], 0, i * kMenuScreenItemHeight);
        labels[i] = _style_create_menu_label(rows[i], itemLabels[i], isSelected);
    }

    _refreshMenuScreen();

    lv_scr_load(screen);
}

uint8_t MenuScreen::_first_visible_start_index(uint8_t selected_index)
{
    int16_t first = static_cast<int16_t>(selected_index) - 2;
    const int16_t last_first = (count > MenuScreen::kMenuScreenVisibleItems) ? 
        (static_cast<int16_t>(count) - static_cast<int16_t>(MenuScreen::kMenuScreenVisibleItems)) : 0;
    return static_cast<uint8_t>(std::clamp<int16_t>(first, 0, last_first));
}

void MenuScreen::_refreshMenuScreen()
{
    const uint8_t first_index = _first_visible_start_index(selected);
    for (uint8_t i = 0; i < kMenuScreenVisibleItems && i < count; ++i) {
        const uint8_t item_index = first_index + i;
        const bool selected = (item_index == this->selected);
        _style_menu_row(rows[i], selected);
        lv_label_set_text(labels[i], itemLabels[item_index]);
        _style_menu_label(labels[i], selected);
    }
}

void MenuScreen::setValue(uint32_t index)
{
    #if 0
        // allows negative values to wrap around the menu items
        selected = (((int32_t)index % count) + count) % count;
    #else
        // no wrapping
        selected = std::clamp<int16_t>(index, 0, count - 1);
    #endif
    _refreshMenuScreen();
}

uint32_t MenuScreen::getValue() const
{
    return selected;
}

void MenuScreen::_style_menu_row(lv_obj_t *row, bool selected)
{
    lv_obj_set_size(row, kMenuScreenItemWidth, kMenuScreenItemHeight);
    lv_obj_set_style_radius(row, kMenuScreenCornerRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(row, selected ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, selected ? lv_color_white() : lv_color_black(), LV_PART_MAIN);
}

void MenuScreen::_style_menu_label(lv_obj_t *label, bool selected)
{
    lv_obj_set_style_text_color(label, selected ? lv_color_black() : lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, kMenuScreenLabelFont, LV_PART_MAIN);
    lv_label_set_long_mode(label, selected ? LV_LABEL_LONG_SCROLL_CIRCULAR : LV_LABEL_LONG_CLIP);
}

lv_obj_t *MenuScreen::_style_create_menu_label(lv_obj_t *parent, const char *text, bool selected)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    _style_menu_label(label, selected);
    // row boundaries and settings for label/clipping scrolling
    lv_obj_set_style_anim_speed(label, kMenuScreenItemScrollSpeed, LV_PART_MAIN);
    lv_obj_set_width(label, kMenuScreenItemWidth - (2 * kMenuScreenItemStartX));
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    // position in the row
    lv_obj_set_pos(label, kMenuScreenItemStartX, kMenuScreenItemStartY);
    return label;
}

// === Slider Screen ===

void SliderScreen::_refreshVisuals()
{
    const uint32_t clampedValue =  std::clamp<uint32_t>(value, minValue, maxValue);
    const uint32_t range = (maxValue > minValue) ? (maxValue - minValue) : 1;
    const uint32_t percent = ((clampedValue - minValue) * 100U) / range;

    const lv_coord_t fillWidth = static_cast<lv_coord_t>((static_cast<uint32_t>(kSliderScreenContainerWidth) * percent) / 100U);
    lv_obj_set_size(sliderFill, fillWidth, kSliderScreenSliderHeight);

    lv_coord_t knobX = fillWidth - (kSliderScreenKnobSize / 2);
    knobX = std::clamp<lv_coord_t>(knobX, -(kSliderScreenKnobSize / 2), kSliderScreenContainerWidth - (kSliderScreenKnobSize / 2));

    lv_obj_set_x(sliderKnob, knobX);

    DEBUG_PRINT(DEBUG_DEBUG, "callback=%p", formatCallback);

    if (formatCallback) {
        char buf[32];
        lv_label_set_text(valueLabel, formatCallback(clampedValue, buf, sizeof(buf) - 1));
    }
    else {
        lv_label_set_text_fmt(valueLabel, "%lu%s", static_cast<unsigned long>(clampedValue), unit);
    }
    
}

void SliderScreen::load() 
{
    DEBUG_PRINT(DEBUG_NOTICE, "range=%u-%u value=%u label=%s unit=%s", minValue, maxValue, value, label, unit ? unit : "<NULL>");
    Screen::load();

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, kSliderScreenContainerX, kSliderScreenContainerY);
    lv_obj_set_size(container, kSliderScreenContainerWidth, kSliderScreenContainerHeight);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    lv_obj_t *titleRow = lv_obj_create(container);
    lv_obj_remove_style_all(titleRow);
    lv_obj_set_size(titleRow, kSliderScreenContainerWidth, lv_font_get_line_height(kSliderScreenLabelFont));
    lv_obj_set_style_bg_opa(titleRow, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(titleRow, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(titleRow, 0, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(titleRow, true, LV_PART_MAIN);
    lv_obj_set_pos(titleRow, 0, 0);

    lv_obj_t *titleObj = lv_label_create(titleRow);
    lv_label_set_text(titleObj, label);
    lv_obj_set_style_text_color(titleObj, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(titleObj, kSliderScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_width(titleObj, kSliderScreenContainerWidth);
    lv_obj_set_style_text_align(titleObj, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_anim_speed(titleObj, kSliderScreenTitleAnimSpeed, LV_PART_MAIN);
    lv_label_set_long_mode(titleObj, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(titleObj, 0, 0);
    lv_label_set_long_mode(titleObj, LV_LABEL_LONG_SCROLL_CIRCULAR);

    lv_obj_t *slider = lv_obj_create(container);
    lv_obj_remove_style_all(slider);
    lv_obj_set_pos(slider, 0, lv_obj_get_height(titleRow) + kSliderScreenTitleBottomGap);
    lv_obj_set_size(slider, kSliderScreenContainerWidth, kSliderScreenSliderHeight);
    lv_obj_set_style_bg_color(slider, lv_palette_darken(LV_PALETTE_GREY, 2), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(slider, kSliderScreenSliderBorder, LV_PART_MAIN);
    lv_obj_set_style_border_color(slider, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(slider, 0, LV_PART_MAIN);

    sliderFill = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderFill);
    lv_obj_set_pos(sliderFill, 0, 0);
    lv_obj_set_size(sliderFill, 0, kSliderScreenSliderHeight);
    lv_obj_set_style_bg_color(sliderFill, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderFill, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderFill, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderFill, 0, LV_PART_MAIN);

    sliderKnob = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderKnob);
    lv_obj_set_size(sliderKnob, kSliderScreenKnobSize, kSliderScreenKnobSize);
    lv_obj_set_style_bg_color(sliderKnob, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderKnob, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderKnob, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderKnob, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(sliderKnob, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_pos(sliderKnob, -(kSliderScreenKnobSize / 2), -((kSliderScreenKnobSize - kSliderScreenSliderHeight) / 2));

    valueLabel = lv_label_create(container);
    lv_obj_set_style_text_color(valueLabel, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(valueLabel, kSliderScreenValueFont, LV_PART_MAIN);
    lv_obj_set_width(valueLabel, kSliderScreenContainerWidth);
    lv_obj_set_style_text_align(valueLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align_to(valueLabel, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

    _refreshVisuals();

    lv_scr_load(screen);
}

void SliderScreen::setValue(uint32_t value)
{
    this->value = std::clamp<int32_t>(value, minValue, maxValue);
    _refreshVisuals();
}

uint32_t SliderScreen::getValue() const
{
    return value;
}

// === Diagnostics Screen ===

void DiagnosticsScreen::load()
{
    Screen::load();

    // Create container for diagnostics information
    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, 10, 10);
    lv_obj_set_size(container, TFT_DIM_WIDTH - 20, TFT_DIM_HEIGHT - 20);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    // Firmware label
    firmwareLabel = lv_label_create(container);
    lv_obj_set_style_text_color(firmwareLabel, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(firmwareLabel, Screen::kDiagnosticsScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_pos(firmwareLabel, 0, 0);
    lv_obj_set_width(firmwareLabel, TFT_DIM_WIDTH - 20);
    lv_label_set_long_mode(firmwareLabel, LV_LABEL_LONG_CLIP);

    // VCC label
    vccLabel = lv_label_create(container);
    lv_obj_set_style_text_color(vccLabel, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(vccLabel, Screen::kDiagnosticsScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_pos(vccLabel, 0, 20);
    lv_obj_set_width(vccLabel, TFT_DIM_WIDTH - 20);
    lv_label_set_long_mode(vccLabel, LV_LABEL_LONG_CLIP);

    // Current label
    currentLabel = lv_label_create(container);
    lv_obj_set_style_text_color(currentLabel, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(currentLabel, Screen::kDiagnosticsScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_pos(currentLabel, 0, 40);
    lv_obj_set_width(currentLabel, TFT_DIM_WIDTH - 20);
    lv_label_set_long_mode(currentLabel, LV_LABEL_LONG_CLIP);

    // Motor temperature label
    motorTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(motorTempLabel, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(motorTempLabel, Screen::kDiagnosticsScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_pos(motorTempLabel, 0, 60);
    lv_obj_set_width(motorTempLabel, TFT_DIM_WIDTH - 20);
    lv_label_set_long_mode(motorTempLabel, LV_LABEL_LONG_CLIP);

    // MOSFET temperature label
    mosfetTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(mosfetTempLabel, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(mosfetTempLabel, Screen::kDiagnosticsScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_pos(mosfetTempLabel, 0, 80);
    lv_obj_set_width(mosfetTempLabel, TFT_DIM_WIDTH - 20);
    lv_label_set_long_mode(mosfetTempLabel, LV_LABEL_LONG_CLIP);

    char buf[64];
    snprintf(buf, sizeof(buf) - 1, "Firmware %u.%u.%u PCB Rev %u.%u",  VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, PCB_REV_MAJOR, PCB_REV_MINOR);
    lv_label_set_text(firmwareLabel, buf);

    _refreshVisuals();

    lv_scr_load(screen);
}

void DiagnosticsScreen::_refreshVisuals()
{
    char buf[64];
    auto adcValues = adc.readAll();

    uint32_t vccRaw = adcValues.getInputVoltage();
    uint32_t currentRaw = adcValues.getInputCurrent();
    int16_t motorTempRaw = adcValues.getMotorTemperature();
    int16_t mosfetTempRaw = adcValues.getMosfetTemperature();

    vcc.update(vccRaw);
    current.update(currentRaw);
    motorTemp.update(motorTempRaw);
    mosfetTemp.update(mosfetTempRaw);

    snprintf(buf, sizeof(buf) - 1, "VCC %u.%uV (%u.%uV/%u.%uV)", CONVERT_TO_FP1(vccRaw), CONVERT_TO_FP1(vcc.min), CONVERT_TO_FP1(vcc.max));
    lv_label_set_text(vccLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "Current %u.%02uA (%u.%02uA/%u.%02uA)", CONVERT_TO_FP2(currentRaw), CONVERT_TO_FP2(current.min), CONVERT_TO_FP2(current.max));
    lv_label_set_text(currentLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "Motor %dC (%dC/%dC)", (int32_t)motorTempRaw, (int32_t)motorTemp.min, (int32_t)motorTemp.max);
    lv_label_set_text(motorTempLabel, buf);

    // snprintf(buf, sizeof(buf) - 1, "MOSFETs %dC (%dC/%dC)", (int32_t)mosfetTempRaw, (int32_t)mosfetTemp.min, (int32_t)mosfetTemp.max);
    pinMode(PB12, INPUT);
    pinMode(PB14, INPUT);
    snprintf(buf, sizeof(buf) - 1, "limits %u %u", digitalRead(PB12), digitalRead(PB14));
    lv_label_set_text(mosfetTempLabel, buf);
}

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
        #if RECREATE_PREV_SCREEN
            screen->load();
        #else
            lv_scr_load(screen->screen);
        #endif
    }
}

void ScreenFlow::next(Screen *nextScreen)
{
    DEBUG_PRINT(DEBUG_DEBUG, "next=%p current=%p", nextScreen, screen);
    nextScreen->prevScreen = screen;
    #if RECREATE_PREV_SCREEN
    if (nextScreen->prevScreen) {
        lv_obj_clean(nextScreen->prevScreen->screen);
    }
    #endif
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
