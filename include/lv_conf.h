#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* Color and display */
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 135

/* Memory */
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (8U * 1024U)

/* Timing */
#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#define LV_DISP_DEF_REFR_PERIOD 30

/* Keep drawing simple on STM32F103 */
#define LV_DRAW_COMPLEX 0

/* Logging and asserts */
#define LV_USE_LOG 0
#define LV_USE_ASSERT_NULL 0
#define LV_USE_ASSERT_MALLOC 0
#define LV_USE_ASSERT_STYLE 0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ 0

/* Theme/layout */
#define LV_USE_THEME_DEFAULT 0
#define LV_USE_THEME_BASIC 0
#define LV_USE_THEME_MONO 0
#define LV_USE_FLEX 0
#define LV_USE_GRID 0

/* Widgets: keep only bare object infrastructure */
#define LV_USE_ARC 0
#define LV_USE_BAR 0
#define LV_USE_BTN 0
#define LV_USE_BTNMATRIX 0
#define LV_USE_CANVAS 0
#define LV_USE_CHECKBOX 0
#define LV_USE_DROPDOWN 0
#define LV_USE_IMG 0
#define LV_USE_LABEL 1
#define LV_USE_LINE 0
#define LV_USE_ROLLER 0
#define LV_USE_SLIDER 0
#define LV_USE_SWITCH 0
#define LV_USE_TABLE 0
#define LV_USE_TEXTAREA 0

/* Extra widgets */
#define LV_USE_ANIMIMG 0
#define LV_USE_CALENDAR 0
#define LV_USE_CHART 0
#define LV_USE_COLORWHEEL 0
#define LV_USE_IMGBTN 0
#define LV_USE_KEYBOARD 0
#define LV_USE_LED 0
#define LV_USE_LIST 0
#define LV_USE_MENU 0
#define LV_USE_METER 0
#define LV_USE_MSGBOX 0
#define LV_USE_SPAN 0
#define LV_USE_SPINBOX 0
#define LV_USE_SPINNER 0
#define LV_USE_TABVIEW 0
#define LV_USE_TILEVIEW 0
#define LV_USE_WIN 0

/* Others */
#define LV_USE_ANIMATION 0
#define LV_USE_SHADOW 0

/* Image/decoder libs */
#define LV_USE_BMP 0
#define LV_USE_GIF 0
#define LV_USE_PNG 0
#define LV_USE_SJPG 0
#define LV_USE_QRCODE 0

/* Fonts */
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_UNSCII_8 0

//#define LV_FONT_DEFAULT &lv_font_unscii_8
#define LV_FONT_DEFAULT &lv_font_montserrat_12

#endif
