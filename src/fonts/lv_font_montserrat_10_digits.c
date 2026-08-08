/*******************************************************************************
 * Size: 10 px
 * Bpp: 4
 * Opts: --size 10 --bpp 4 --format lvgl --lv-include lvgl.h --font C:\Users\sascha\Documents\PlatformIO\Projects\motor_controller_stm32\src\fonts\Montserrat-Regular.ttf --lv-font-name lv_font_montserrat_10_digits -o C:\Users\sascha\Documents\PlatformIO\Projects\motor_controller_stm32\src\fonts\lv_font_montserrat_10_digits.c --range 32 --range 44 --range 46 --range 48-57
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl.h"
#endif

#ifndef LV_FONT_MONTSERRAT_10_DIGITS
#define LV_FONT_MONTSERRAT_10_DIGITS 1
#endif

#if LV_FONT_MONTSERRAT_10_DIGITS

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */

    /* U+002C "," */
    0x0, 0x2c, 0x36, 0x82, 0x80,

    /* U+002E "." */
    0x0, 0x2b, 0x80,

    /* U+0030 "0" */
    0x4, 0xaa, 0x84, 0x7, 0xea, 0x91, 0x60, 0xfc,
    0x0, 0x21, 0x1, 0x10, 0x4, 0x23, 0x80, 0x21,
    0x13, 0xf0, 0x0, 0x84, 0x7, 0xea, 0x91, 0x60,

    /* U+0031 "1" */
    0x8b, 0x78, 0xf0, 0xf, 0xfe, 0x20,

    /* U+0032 "2" */
    0x4a, 0xa5, 0xa8, 0x35, 0x52, 0xe4, 0x8, 0x3,
    0xf4, 0x48, 0x5, 0x1c, 0xa0, 0x9, 0xd6, 0x0,
    0x3a, 0x64, 0xc8, 0x80,

    /* U+0033 "3" */
    0x69, 0x95, 0x60, 0x34, 0xc8, 0x38, 0x2, 0x6c,
    0x20, 0xb, 0x7d, 0x0, 0x2b, 0x9f, 0x12, 0x0,
    0xe4, 0xaa, 0xb4, 0x40,

    /* U+0034 "4" */
    0x0, 0x95, 0xc0, 0x31, 0xe3, 0x80, 0x45, 0x32,
    0x30, 0x0, 0xcd, 0x84, 0x88, 0x38, 0xc4, 0xbd,
    0xa3, 0x54, 0xc9, 0xed, 0x0, 0x3e,

    /* U+0035 "5" */
    0xd, 0x99, 0x40, 0xa, 0x4c, 0xa0, 0xc, 0x40,
    0x30, 0x9c, 0xc9, 0x40, 0xee, 0x65, 0xe4, 0x40,
    0x8, 0xc5, 0xea, 0x6b, 0x8c,

    /* U+0036 "6" */
    0x3, 0xaa, 0x88, 0x62, 0xaa, 0x24, 0xd0, 0xc,
    0x41, 0x13, 0x2, 0x2d, 0x89, 0xab, 0x71, 0x0,
    0x18, 0x8b, 0xe6, 0x57, 0x20,

    /* U+0037 "7" */
    0xba, 0x99, 0x63, 0x15, 0x4c, 0x9d, 0xa4, 0x0,
    0x66, 0x0, 0xd3, 0x20, 0x8, 0x80, 0x40, 0x2b,
    0x90, 0x8, 0x48, 0xc0, 0x0,

    /* U+0038 "8" */
    0x8, 0x99, 0x50, 0x98, 0xcc, 0xd0, 0x26, 0x0,
    0x71, 0x20, 0xa8, 0x9a, 0x6c, 0xa8, 0xc9, 0x13,
    0x0, 0xb, 0xbb, 0x26, 0x55, 0xe0,

    /* U+0039 "9" */
    0x1a, 0x99, 0x30, 0x44, 0x26, 0x5c, 0x40, 0x20,
    0x2, 0x59, 0xb9, 0x8f, 0xf0, 0x44, 0xc0, 0xf8,
    0x6, 0xf6, 0x1a, 0x9a, 0xf1
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 42, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 34, .box_w = 2, .box_h = 4, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 5, .adv_w = 34, .box_w = 2, .box_h = 2, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 8, .adv_w = 106, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 32, .adv_w = 58, .box_w = 3, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 38, .adv_w = 91, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 58, .adv_w = 90, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 78, .adv_w = 106, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 100, .adv_w = 91, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 121, .adv_w = 97, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 142, .adv_w = 94, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 163, .adv_w = 102, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 185, .adv_w = 97, .box_w = 6, .box_h = 7, .ofs_x = 0, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0xc, 0xe
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 15, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 3, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    },
    {
        .range_start = 48, .range_length = 10, .glyph_id_start = 4,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 2,
    .bpp = 4,
    .kern_classes = 0,
    .bitmap_format = 1,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t lv_font_montserrat_10_digits = {
#else
lv_font_t lv_font_montserrat_10_digits = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 9,          /*The maximum line height required by the font*/
    .base_line = 2,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if LV_FONT_MONTSERRAT_10_DIGITS*/

