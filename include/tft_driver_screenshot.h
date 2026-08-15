/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

#if HAVE_SCREENSHOTS

#include "serial.h"
#include "lvgl.h"

// === Screenshot streaming support ===

struct TFTDriverScreenshot
{
    static constexpr uint8_t kPort = 2;

    TFTDriverScreenshot() : active(false)
    {}

    static constexpr uint32_t kPixelFormatRgb565 = 1U;

    struct FrameHeader {
        uint16_t width;
        uint16_t height;
        uint32_t format: 1;
        uint32_t reserved: 31;
    };
    static_assert(sizeof(FrameHeader) % 4 == 0, "FrameHeader must be 4-byte aligned");

    struct TileHeader {
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        uint32_t byteCount;
    };
    static_assert(sizeof(TileHeader) % 4 == 0, "TileHeader must be 4-byte aligned");

    bool write_tile(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const lv_color_t *color_p);
    bool begin();
    void end();

    #if HAVE_USB_DEVICE || HAVE_SERIAL

        inline size_t write(const void *data, size_t size)
        {
            return Serial::writeBinary(Serial::BinaryType::SCREENSHOT, data, size);
        }

    #else

        inline size_t write(const void *data, size_t size)
        {
            return SWO::write(kPort, data, size);
        }

    #endif

    template<typename T>
    bool write(const T &value)
    {
        return write(&value, sizeof(value)) == sizeof(value);
    }

    bool writeByte(uint8_t value)
    {
        return write(&value, sizeof(value)) == sizeof(value);
    }

    inline bool isActive() const
    {
        return active;
    }

protected:
    bool active;
};

extern TFTDriverScreenshot screenshot;

#endif
