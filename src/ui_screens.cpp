/**
  Author: sascha_lammers@gmx.de

  UI implementation using LVGL library
*/

#include "ui.h"
#include "adc.h"
#include "pid_controller.h"
#include "stats.h"
#include "controls.h"
#include "menu.h"

// === Helpers ===

inline lv_coord_t diagnostic_screen_get_ypos_for_row(int32_t row)
{
    switch(row) {
        case 0: return 0;
        default:
            break;
    }
    return Screen::kDiagnosticScreenRowHeight * row + (Screen::kDiagnosticScreenRowHeight * 3);
}

inline int32_t diagnostic_screen_content_height()
{
    return diagnostic_screen_get_ypos_for_row(Screen::kDiagnosticScreenRowCount) + 10;
}

inline int32_t diagnostic_screen_scroll_max_lines(int32_t viewportHeight)
{
    const int32_t hiddenPixels = std::max<int32_t>(0, diagnostic_screen_content_height() - viewportHeight);
    return hiddenPixels / Screen::kDiagnosticScreenRowHeight;
}

lv_obj_t *diagnostic_screen_create_label(lv_obj_t *parent, lv_coord_t width, int32_t row)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_color(label, DIAGNOSTICSCREEN_COLOR_TEXT, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, Screen::kDiagnosticsScreenLabelFont, LV_PART_MAIN);
    lv_obj_set_pos(label, 0, diagnostic_screen_get_ypos_for_row(row));
    lv_obj_set_width(label, width);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    return label;
}

inline void diagnostic_screen_set_label_row(lv_obj_t *label, int32_t row, int32_t scrollOffset)
{
    lv_obj_set_y(label, diagnostic_screen_get_ypos_for_row(row) - scrollOffset);
}

void start_screen_update_top_status_labels(lv_obj_t *voltageLabel, lv_obj_t *currentLabel, lv_obj_t *motorTempLabel, lv_obj_t *mosfetTempLabel)
{
    char buf[32];

    snprintf(buf, sizeof(buf) - 1, "%u.%uV (%u.%uV)", CONVERT_TO_FP1(stats.vcc), CONVERT_TO_FP1(stats.max.vcc));
    lv_label_set_text(voltageLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "%u.%uA (%u.%uA)", CONVERT_TO_FP1(stats.current), CONVERT_TO_FP1(stats.max.current));
    lv_label_set_text(currentLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "%d" DEGREE_UTF8 "C", stats.motorTemp);
    lv_label_set_text(motorTempLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "%d" DEGREE_UTF8 "C", stats.mosfetTemp);
    lv_label_set_text(mosfetTempLabel, buf);
}

// === Base Screen ===

lv_obj_t *Screen::emptyScreen = nullptr;

Screen::Screen(Type id) :
    screen(nullptr),
    prevScreen(nullptr),
    id(id),
    maxAcceleration(1),
    steps(1),
    value(0)
{
    DEBUG_PRINT(DebugType::UI, "ctor");
}

Screen::~Screen()
{
    DEBUG_PRINT(DebugType::UI, "screen=%p", screen);
     if (screen) {
        lv_scr_load(emptyScreen);
        lv_obj_del(screen);
    }
}

void Screen::load()
{
    DEBUG_PRINT(DebugType::UI, "screen=%p", screen);
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
    knob.setMaxAcceleration(maxAcceleration);
}

void Screen::update()
{
    DEBUG_PRINT(DebugType::UI, "screen=%p", screen);
}

Screen::Type Screen::getId() const
{
    return id;
}

void Screen::setValue(uint32_t value)
{
    this->value = value;
}

uint32_t Screen::getValue() const
{
    return value;
}

void Screen::_fatal_error(const char *msg)
{
    uint32_t num = 0;
    while (true) {
        WatchDog::delay(100);
        if (num++ % 10 == 0) {
            DEBUG_PRINT(DebugType::ERROR, "UI ERROR: %s", msg);
        }
    }
}

void Screen::_style_screen(lv_obj_t *screen)
{
    lv_obj_set_style_bg_color(screen, SCREEN_COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
}

// === Welcome Screen ===

WelcomeScreen::WelcomeScreen() :
    InfoScreen(Type::WELCOME, kWelcomeScreenLabelFont)
{
    char buf[32];
    snprintf(buf, sizeof(buf) - 1, "Version %u.%u.%u", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
    message = strdup(buf);
}

// === Info Screen ===

void InfoScreen::load()
{
    DEBUG_PRINT(DebugType::UI, "message=%s", message ? message : "<NULL>");
    Screen::load();
    label = lv_label_create(screen);
    if (message) {
        lv_label_set_text(label, message);
        free(message);
        message = nullptr;
    }
    lv_obj_set_style_text_color(label, INFOSCREEN_COLOR_TEXT, LV_PART_MAIN);
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
    steps = -1; // invert for menus
}

void MenuScreen::load()
{
    DEBUG_PRINT(DebugType::UI, "items=%u selected=%u", count, selected);
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
    lv_obj_set_style_bg_color(row, selected ? MENUSCREEN_COLOR_SELECTED_BG : MENUSCREEN_COLOR_BG, LV_PART_MAIN);
}

void MenuScreen::_style_menu_label(lv_obj_t *label, bool selected)
{
    lv_obj_set_style_text_color(label, selected ? MENUSCREEN_COLOR_SELECTED_ITEM : MENUSCREEN_COLOR_ITEM, LV_PART_MAIN);
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

void SliderScreen::load()
{
    DEBUG_PRINT(DebugType::UI, "range=%u-%u value=%u label=%s unit=%s", minValue, maxValue, value, label, unit ? unit : "<NULL>");
    Screen::load();

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, kSliderScreenContainerX, kSliderScreenContainerY);
    lv_obj_set_size(container, kSliderScreenContainerWidth, kSliderScreenContainerHeight);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    auto titleFont = (lv_txt_get_width(label, strlen(label), kSliderScreenLabelFontBig, 0, LV_TEXT_FLAG_NONE) < kSliderScreenContainerWidth) ?
        kSliderScreenLabelFontBig :
        kSliderScreenLabelFont;

    lv_obj_t *titleRow = lv_obj_create(container);
    lv_obj_remove_style_all(titleRow);
    lv_obj_set_size(titleRow, kSliderScreenContainerWidth, lv_font_get_line_height(titleFont));
    lv_obj_set_style_bg_opa(titleRow, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(titleRow, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(titleRow, 0, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(titleRow, true, LV_PART_MAIN);
    lv_obj_set_pos(titleRow, 0, 0);

    lv_obj_t *titleObj = lv_label_create(titleRow);
    lv_label_set_text(titleObj, label);
    lv_obj_set_style_text_color(titleObj, SLIDERSCREEN_COLOR_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_font(titleObj, titleFont, LV_PART_MAIN);
    lv_obj_set_width(titleObj, kSliderScreenContainerWidth);
    lv_obj_set_style_text_align(titleObj, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_style_anim_speed(titleObj, kSliderScreenTitleAnimSpeed, LV_PART_MAIN);
    lv_label_set_long_mode(titleObj, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(titleObj, 0, 0);
    lv_label_set_long_mode(titleObj, LV_LABEL_LONG_SCROLL_CIRCULAR);

    const lv_coord_t sliderVisualHeight = std::max<lv_coord_t>(kSliderScreenSliderHeight, kSliderScreenKnobSize + 4);
    const lv_coord_t sliderFillHeight = std::max<lv_coord_t>(6, kSliderScreenSliderHeight - 8);

    lv_obj_t *slider = lv_obj_create(container);
    lv_obj_remove_style_all(slider);
    lv_obj_set_pos(slider, 0, lv_obj_get_height(titleRow) + kSliderScreenTitleBottomGap);
    lv_obj_set_size(slider, kSliderScreenContainerWidth, sliderVisualHeight);
    lv_obj_set_style_bg_color(slider, SLIDERSCREEN_COLOR_SLIDER_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(slider, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(slider, kSliderScreenSliderBorder, LV_PART_MAIN);
    lv_obj_set_style_border_color(slider, SLIDERSCREEN_COLOR_SLIDER_BORDER, LV_PART_MAIN);
    lv_obj_set_style_pad_all(slider, 0, LV_PART_MAIN);

    sliderFill = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderFill);
    lv_obj_set_pos(sliderFill, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFill, 0, sliderFillHeight);
    lv_obj_set_style_bg_color(sliderFill, SLIDERSCREEN_COLOR_SLIDER_FILL_ACTIVE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderFill, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderFill, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderFill, 0, LV_PART_MAIN);

    sliderFillAfterActive = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderFillAfterActive);
    lv_obj_set_pos(sliderFillAfterActive, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfterActive, 0, sliderFillHeight);
    lv_obj_set_style_bg_color(sliderFillAfterActive, SLIDERSCREEN_COLOR_SLIDER_FILL_ACTIVE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderFillAfterActive, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderFillAfterActive, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderFillAfterActive, 0, LV_PART_MAIN);

    sliderFillAfter = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderFillAfter);
    lv_obj_set_pos(sliderFillAfter, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfter, 0, sliderFillHeight);
    lv_obj_set_style_bg_color(sliderFillAfter, SLIDERSCREEN_COLOR_SLIDER_FILL, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderFillAfter, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderFillAfter, kSliderScreenSliderRadius, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderFillAfter, 0, LV_PART_MAIN);

    sliderKnob = lv_obj_create(slider);
    lv_obj_remove_style_all(sliderKnob);
    lv_obj_set_size(sliderKnob, kSliderScreenKnobSize, kSliderScreenKnobSize);
    lv_obj_set_style_bg_color(sliderKnob, SLIDERSCREEN_COLOR_SLIDER_KNOB, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(sliderKnob, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(sliderKnob, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(sliderKnob, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(sliderKnob, SLIDERSCREEN_COLOR_SLIDER_KNOB_BORDER, LV_PART_MAIN);
    lv_obj_set_pos(sliderKnob, 0, (sliderVisualHeight - kSliderScreenKnobSize) / 2);

    valueLabel = lv_label_create(container);
    lv_obj_set_style_text_color(valueLabel, SLIDERSCREEN_COLOR_VALUE, LV_PART_MAIN);
    lv_obj_set_style_text_font(valueLabel, kSliderScreenValueFont, LV_PART_MAIN);
    lv_obj_set_width(valueLabel, kSliderScreenContainerWidth);
    lv_obj_set_style_text_align(valueLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align_to(valueLabel, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

    _refreshVisuals();

    lv_scr_load(screen);
}

void SliderScreen::_refreshVisuals()
{
    const uint32_t range = (maxValue > minValue) ? (maxValue - minValue) : 1;
    const uint32_t percent = ((value - minValue) * 100U) / range;
    const lv_coord_t sliderVisualHeight = std::max<lv_coord_t>(kSliderScreenSliderHeight, kSliderScreenKnobSize + 4);
    const lv_coord_t sliderFillHeight = std::max<lv_coord_t>(6, kSliderScreenSliderHeight - 8);

    const lv_coord_t knobTravel = std::max<lv_coord_t>(0, kSliderScreenContainerWidth - kSliderScreenKnobSize);
    lv_coord_t knobX = static_cast<lv_coord_t>((static_cast<uint32_t>(knobTravel) * percent) / 100U);

    const lv_coord_t fillWidth = std::min<lv_coord_t>(kSliderScreenContainerWidth, knobX + (kSliderScreenKnobSize / 2));
    lv_obj_set_pos(sliderFill, 0, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFill, fillWidth, sliderFillHeight);

    const lv_coord_t knobRight = std::min<lv_coord_t>(kSliderScreenContainerWidth, knobX + kSliderScreenKnobSize);
    const lv_coord_t trackWidth = kSliderScreenContainerWidth;

    lv_obj_set_x(sliderKnob, knobX - 1);

    // Desired rendering: blue active segment, then knob, then gray remainder.
    const lv_coord_t afterStart = knobRight;
    const lv_coord_t remaining = std::max<lv_coord_t>(0, trackWidth - afterStart);
    const lv_coord_t activeAfterWidth = 0;

    lv_obj_set_pos(sliderFillAfterActive, afterStart, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfterActive, activeAfterWidth, sliderFillHeight);

    lv_obj_set_pos(sliderFillAfter, afterStart + activeAfterWidth - 8, (sliderVisualHeight - sliderFillHeight) / 2);
    lv_obj_set_size(sliderFillAfter, remaining + 8 - 2, sliderFillHeight);

    if (formatCallback) {
        char buf[32];
        lv_label_set_text(valueLabel, formatCallback(value, buf, sizeof(buf) - 1));
    }
    else {
        lv_label_set_text_fmt(valueLabel, "%u%s", static_cast<unsigned>(value), unit);
    }
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

    // Diagnostics container with manual scrolling and a visual scrollbar.
    const lv_coord_t viewportWidth = TFT_DIM_WIDTH - kDiagnosticScreenMargin;
    const lv_coord_t viewportHeight = TFT_DIM_HEIGHT - kDiagnosticScreenMargin;
    const lv_coord_t textWidth = viewportWidth - kDiagnosticScreenScrollbarWidth - 4;

    viewport = lv_obj_create(screen);
    lv_obj_remove_style_all(viewport);
    lv_obj_set_pos(viewport, kDiagnosticScreenViewportX, kDiagnosticScreenViewportY);
    lv_obj_set_size(viewport, viewportWidth, viewportHeight);
    lv_obj_set_style_bg_opa(viewport, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(viewport, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(viewport, 0, LV_PART_MAIN);

    content = viewport;

    scrollbarTrack = lv_obj_create(viewport);
    lv_obj_remove_style_all(scrollbarTrack);
    lv_obj_set_size(scrollbarTrack, kDiagnosticScreenScrollbarWidth, viewportHeight);
    lv_obj_set_pos(scrollbarTrack, viewportWidth - kDiagnosticScreenScrollbarWidth, 0);
    lv_obj_set_style_bg_color(scrollbarTrack, DIAGNOSTICSCREEN_COLOR_SCROLLBAR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scrollbarTrack, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(scrollbarTrack, 2, LV_PART_MAIN);

    scrollbarThumb = lv_obj_create(scrollbarTrack);
    lv_obj_remove_style_all(scrollbarThumb);
    lv_obj_set_size(scrollbarThumb, kDiagnosticScreenScrollbarWidth, 16);
    lv_obj_set_pos(scrollbarThumb, 0, 0);
    lv_obj_set_style_bg_color(scrollbarThumb, DIAGNOSTICSCREEN_COLOR_SCROLLBAR_THUMB, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scrollbarThumb, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(scrollbarThumb, 2, LV_PART_MAIN);

    // Firmware label
    firmwareLabel = diagnostic_screen_create_label(content, textWidth, 0);
    lv_label_set_text_fmt(firmwareLabel, "Firmware %u.%u.%u\nPCB Rev %u.%u\nBuild " __DATE__ " " __TIME__  "\nEEPROM cycle #%u",  VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, PCB_REV_MAJOR, PCB_REV_MINOR, (unsigned)eeprom.getData().sequence);

    // VCC label
    vccLabel = diagnostic_screen_create_label(content, textWidth, 1);

    // Current label
    currentLabel = diagnostic_screen_create_label(content, textWidth, 2);

    // Motor temperature label
    motorTempLabel = diagnostic_screen_create_label(content, textWidth, 3);

    // MOSFET temperature label
    mosfetTempLabel = diagnostic_screen_create_label(content, textWidth, 4);

    // RPM/PWM label
    rpmPwmLabel = diagnostic_screen_create_label(content, textWidth, 5);

    // Last error label
    lastErrorLabel = diagnostic_screen_create_label(content, textWidth, 6);

    scrollMax = diagnostic_screen_scroll_max_lines(viewportHeight);
    scrollOffset = 0;

    lv_scr_load(screen); // update is using lv_obj_get_height() etc, load screen first
    update();
}

void DiagnosticsScreen::setValue(uint32_t value)
{
    const int32_t viewportHeight = lv_obj_get_height(viewport);
    scrollMax = diagnostic_screen_scroll_max_lines(viewportHeight);
    this->value = std::clamp<int32_t>(static_cast<int32_t>(value), 0, scrollMax);
}

void DiagnosticsScreen::update()
{
    _refreshVisuals();

    const int32_t viewportHeight = lv_obj_get_height(viewport);
    scrollMax = diagnostic_screen_scroll_max_lines(viewportHeight);

    const int32_t signedValue = static_cast<int32_t>(value);
    const int32_t scrollLine = std::clamp<int32_t>(signedValue, 0, scrollMax);
    scrollOffset = scrollLine * Screen::kDiagnosticScreenRowHeight;

    // Keep labels in one container and scroll in full line-height steps.
    diagnostic_screen_set_label_row(firmwareLabel, 0, scrollOffset);
    diagnostic_screen_set_label_row(vccLabel, 1, scrollOffset);
    diagnostic_screen_set_label_row(currentLabel, 2, scrollOffset);
    diagnostic_screen_set_label_row(motorTempLabel, 3, scrollOffset);
    diagnostic_screen_set_label_row(mosfetTempLabel, 4, scrollOffset);
    diagnostic_screen_set_label_row(rpmPwmLabel, 5, scrollOffset);
    diagnostic_screen_set_label_row(lastErrorLabel, 6, scrollOffset);

    const int32_t trackHeightRaw = lv_obj_get_height(scrollbarTrack);
    const int32_t trackHeight = std::max<int32_t>(trackHeightRaw, 10);

    // Line-based scrollbar: thumb size and movement are based on scroll steps.
    const int32_t stepCount = scrollMax + 1;
    int32_t thumbHeight = (stepCount > 0) ? (trackHeight / stepCount) : trackHeight;
    thumbHeight = std::clamp<int32_t>(thumbHeight, 8, trackHeight);
    lv_obj_set_height(scrollbarThumb, thumbHeight);

    int32_t thumbY = 0;
    if (scrollMax > 0 && (trackHeight - thumbHeight) > 0) {
        thumbY = (scrollLine * (trackHeight - thumbHeight)) / scrollMax;
    }
    lv_obj_set_y(scrollbarThumb, thumbY);
}

void DiagnosticsScreen::_refreshVisuals()
{
    char buf[64];

    snprintf(buf, sizeof(buf) - 1, "VCC %u.%uV (%u.%uV/%u.%uV)",
        CONVERT_TO_FP1(stats.vcc),
        CONVERT_TO_FP1(stats.min.vcc),
        CONVERT_TO_FP1(stats.max.vcc)
    );
    lv_label_set_text(vccLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "Current %u.%02uA (%u.%02uA/%u.%02uA)",
        CONVERT_TO_FP2(stats.current),
        CONVERT_TO_FP2(stats.min.current),
        CONVERT_TO_FP2(stats.max.current)
    );
    lv_label_set_text(currentLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "Motor %d" DEGREE_UTF8 "C (%d" DEGREE_UTF8 "C/%d" DEGREE_UTF8 "C)",
        stats.motorTemp,
        stats.min.motorTemp,
        stats.max.motorTemp
    );
    lv_label_set_text(motorTempLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "MOSFETs %d" DEGREE_UTF8 "C (%d" DEGREE_UTF8 "C/%d" DEGREE_UTF8 "C)",
        stats.mosfetTemp,
        stats.min.mosfetTemp,
        stats.max.mosfetTemp
    );
    lv_label_set_text(mosfetTempLabel, buf);

    snprintf(buf, sizeof(buf) - 1, "RPM %d/%u",
        (signed)pid.stats.rpm.get(),
        (unsigned)pid.getRPM()
    );
    lv_label_set_text(rpmPwmLabel, buf);

    pid.errorPrintf(buf, sizeof(buf));
    lv_label_set_text_fmt(lastErrorLabel, "Last Error %s", buf);
}

// === Dashboard Screen ===

void DashboardScreen::load()
{
    Screen::load();

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, 8, 6);
    lv_obj_set_size(container, kDashboardScreenContainerWidth, kDashboardScreenContainerHeight);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    voltageLabel = lv_label_create(container);
    lv_obj_set_style_text_color(voltageLabel, DASHBOARDSCREEN_COLOR_VOLTAGE, LV_PART_MAIN);
    lv_obj_set_style_text_font(voltageLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(voltageLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(voltageLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(voltageLabel, 0, 0);

    currentLabel = lv_label_create(container);
    lv_obj_set_style_text_color(currentLabel, DASHBOARDSCREEN_COLOR_CURRENT, LV_PART_MAIN);
    lv_obj_set_style_text_font(currentLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(currentLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(currentLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(currentLabel, 0, 18);

    motorTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(motorTempLabel, DASHBOARDSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(motorTempLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(motorTempLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(motorTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(motorTempLabel, kDashboardScreenContainerWidth - kDashboardScreenColumnWidth, 0);

    mosfetTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(mosfetTempLabel, DASHBOARDSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(mosfetTempLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(mosfetTempLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(mosfetTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(mosfetTempLabel, kDashboardScreenContainerWidth - kDashboardScreenColumnWidth, 18);

    rpmLabel = lv_label_create(container);
    lv_obj_set_style_text_color(rpmLabel, DASHBOARDSCREEN_COLOR_SPEED, LV_PART_MAIN);
    lv_obj_set_style_text_font(rpmLabel, kDashboardScreenBigFont, LV_PART_MAIN);
    lv_obj_set_width(rpmLabel, kDashboardScreenContainerWidth);
    lv_obj_set_style_text_align(rpmLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(rpmLabel, 0, (TFT_DIM_HEIGHT / 2) - 22);

    valueLabel = lv_label_create(container);
    lv_obj_set_style_text_color(valueLabel, DASHBOARDSCREEN_COLOR_PWM_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_font(valueLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(valueLabel, kDashboardScreenContainerWidth);
    lv_obj_set_style_text_align(valueLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(valueLabel, 0, kDashboardScreenContainerHeight - 24);

    _refreshVisuals();

    lv_scr_load(screen);
}

void DashboardScreen::_refreshVisuals()
{
    char buf[32];

    start_screen_update_top_status_labels(voltageLabel, currentLabel, motorTempLabel, mosfetTempLabel);

    // blink any errors
    bool showError;
    if (((HAL_GetTick() / 512) & 0x01) == 0 && pid.hasErrorCode()) {
        showError = true;
        pid.errorPrintf(buf, sizeof(buf) - 1);
        lv_obj_set_style_text_color(rpmLabel, DASHBOARDSCREEN_COLOR_ERROR, LV_PART_MAIN);
    }
    else {
        showError = false;
        lv_obj_set_style_text_color(rpmLabel, DASHBOARDSCREEN_COLOR_SPEED, LV_PART_MAIN);
    }

    if (!showError) {
        if (eeprom.isPIDMode()) {
            snprintf(buf, sizeof(buf) - 1, "%u RPM (%u)", (unsigned)pid.clampRPM(pid.stats.rpm.get()), (unsigned)pid.getRPM());
        }
        else {
            snprintf(buf, sizeof(buf) - 1, "%u RPM", (unsigned)pid.clampRPM(pid.stats.rpm.get()));
        }
    }
    lv_label_set_text(rpmLabel, buf);

    switch(getSelectedValue()) {
        case SelectedValueType::SPEED:
            lv_label_set_text_fmt(valueLabel, "PWM %u%%", (unsigned)((pid.stats.pwm.get() * 100 / pid.kMaxPWMLevel) + 1));
            break;
        case SelectedValueType::KP:
            FloatToString::convertTrimmed(buf, sizeof(buf) - 1, eeprom.getKp(), 6);
            lv_label_set_text_fmt(valueLabel, "Kp %s", buf);
            break;
        case SelectedValueType::KI:
            FloatToString::convertTrimmed(buf, sizeof(buf) - 1, eeprom.getKi(), 6);
            lv_label_set_text_fmt(valueLabel, "Ki %s", buf);
            break;
        case SelectedValueType::KD:
            FloatToString::convertTrimmed(buf, sizeof(buf) - 1, eeprom.getKd(), 6);
            lv_label_set_text_fmt(valueLabel, "Kd %s", buf);
            break;
        case SelectedValueType::ANTI_WINDUP:
            FloatToString::convertTrimmed(buf, sizeof(buf) - 1, eeprom.getAntiWindupReduction() / (float)UIConstants::kAntiWindupFactor, 2);
            lv_label_set_text_fmt(valueLabel, "Anti-windup %s%%", buf);
            break;
        case SelectedValueType::MAX:
            break;
    }

}

// === Start Screen ===

void StartScreen::load()
{
    Screen::load();

    lv_obj_t *container = lv_obj_create(screen);
    lv_obj_remove_style_all(container);
    lv_obj_set_pos(container, 8, 6);
    lv_obj_set_size(container, kDashboardScreenContainerWidth, kDashboardScreenContainerHeight);
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(container, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(container, 0, LV_PART_MAIN);

    voltageLabel = lv_label_create(container);
    lv_obj_set_style_text_color(voltageLabel, STARTSCREEN_COLOR_VOLTAGE, LV_PART_MAIN);
    lv_obj_set_style_text_font(voltageLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(voltageLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(voltageLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(voltageLabel, 0, 0);

    currentLabel = lv_label_create(container);
    lv_obj_set_style_text_color(currentLabel, STARTSCREEN_COLOR_CURRENT, LV_PART_MAIN);
    lv_obj_set_style_text_font(currentLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(currentLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(currentLabel, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_set_pos(currentLabel, 0, 18);

    motorTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(motorTempLabel, STARTSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(motorTempLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(motorTempLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(motorTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(motorTempLabel, kDashboardScreenContainerWidth - kDashboardScreenColumnWidth, 0);

    mosfetTempLabel = lv_label_create(container);
    lv_obj_set_style_text_color(mosfetTempLabel, STARTSCREEN_COLOR_TEMPERATURE, LV_PART_MAIN);
    lv_obj_set_style_text_font(mosfetTempLabel, kDashboardScreenFont, LV_PART_MAIN);
    lv_obj_set_width(mosfetTempLabel, kDashboardScreenColumnWidth);
    lv_obj_set_style_text_align(mosfetTempLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_pos(mosfetTempLabel, kDashboardScreenContainerWidth - kDashboardScreenColumnWidth, 18);

    directionLabel = lv_label_create(container);
    lv_obj_set_style_text_color(directionLabel, STARTSCREEN_COLOR_START_LABEL, LV_PART_MAIN);
    lv_obj_set_style_text_font(directionLabel, kDashboardScreenBigFont, LV_PART_MAIN);
    lv_obj_set_width(directionLabel, kDashboardScreenContainerWidth);
    lv_obj_set_style_text_align(directionLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(directionLabel, 0, (TFT_DIM_HEIGHT / 2) - 24);

    speedLabel = lv_label_create(container);
    lv_obj_set_style_text_color(speedLabel, STARTSCREEN_COLOR_SPEED, LV_PART_MAIN);
    lv_obj_set_style_text_font(speedLabel, kDashboardScreenBigFont, LV_PART_MAIN);
    lv_obj_set_width(speedLabel, kDashboardScreenContainerWidth);
    lv_obj_set_style_text_align(speedLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_pos(speedLabel, 0, (TFT_DIM_HEIGHT / 2) + 10);

    _refreshVisuals();

    lv_scr_load(screen);
}

void StartScreen::_refreshVisuals()
{
    char buf[32];

    start_screen_update_top_status_labels(voltageLabel, currentLabel, motorTempLabel, mosfetTempLabel);

    lv_label_set_text(directionLabel, pid.isForwardMotorDirection() ? "START FORWARD" : "START REVERSE");

    if (eeprom.isPIDMode()) {
        snprintf(buf, sizeof(buf) - 1, "%u RPM", (unsigned)eeprom.getSpeed());
    } else {
        snprintf(buf, sizeof(buf) - 1, "%u%% PWM", (unsigned)eeprom.getSpeed());
    }
    lv_label_set_text(speedLabel, buf);
}

void StartScreen::update()
{
    _refreshVisuals();
}
