/**
  Author: sascha_lammers@gmx.de
*/

#pragma once

// === USB helper class ===

#if HAVE_USB_DEVICE

#include <stm32f1xx.h>
#include <usb_device.h>
#include <usbd_cdc_if.h>
#include "crc.h"
#include "debug.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;

#endif

struct SerialProtocol
{
   #if HAVE_SCREENSHOTS
        // timeout in milliseconds before giving up on writing to USB
        // tested about ~70-80kb/100ms (size is limited to 64kb per packet) mostly relevant for sending screenshots
        static constexpr uint32_t kTimeoutMs = 100;
    #else
        static constexpr uint32_t kTimeoutMs = 10;
    #endif

    static constexpr uint32_t kMagic = 0xDEADBEEF;

    enum class BinaryType : uint16_t {
        PID,
        TOGGLE_PID,
        SCREENSHOT,
        REQUEST_SCREENSHOT,
        PARAMETERS,
        REQUEST_PARAMETERS,
        EEPROM,
        REQUEST_EEPROM,
    };

    struct BinaryHeader
    {
        uint32_t magic;
        uint16_t size;
        BinaryType type;
        uint32_t crc;

        BinaryHeader(BinaryType type, size_t size = 0, uint32_t crc = 0xffffffff) : magic(kMagic), size(size), type(type), crc(crc) {}
    };

};

#if HAVE_USB_DEVICE

struct Serial : public SerialProtocol
{
     inline static bool isConnected()
    {
        return hUsbDeviceFS.pClassData != nullptr && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
    }

    inline static bool canWrite()
    {
        uint32_t start = HAL_GetTick();
        while(HAL_GetTick() - start < kTimeoutMs) {
            if (reinterpret_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData)->TxState == 0) {
                return true;
            }
        }
        return false;
    }

    static size_t readBinary(void *data, size_t size, BinaryType &type, uint32_t &crc)
    {
        if (size == 0 || !isConnected()) {
            return 0;
        }
        return CDC_ReadBinary_FS(reinterpret_cast<uint8_t *>(data), size, reinterpret_cast<uint16_t *>(&type), &crc);
    }

    static size_t write(const void *data, size_t size)
    {
        if (size == 0 || !isConnected()) {
            return 0;
        }
        if (!canWrite()) {
            return 0;
        }
        return (CDC_Transmit_FS(reinterpret_cast<uint8_t *>(const_cast<void *>(data)), size) == USBD_OK) ? size : 0;
    }

    static size_t writeBinary(BinaryType type, const void *data, size_t size)
    {
        if (size == 0 || !isConnected() || !canWrite()) {
            return 0;
        }
        BinaryHeader hdr(type, size, stm32_CRC(data, size));
        uint8_t result = CDC_Transmit_FS(reinterpret_cast<uint8_t *>(&hdr), sizeof(hdr));
        if (result != USBD_OK || !canWrite()) {
            return 0;
        }
        result = CDC_Transmit_FS(reinterpret_cast<uint8_t *>(const_cast<void *>(data)), size);
        if (result != USBD_OK || !canWrite()) {
            return 0;
        }
        return size;
    }
};

#elif HAVE_SERIAL

//TODO

struct Serial : public SerialProtocol
{
};

#else

// === dummy class ===

struct Serial
{
    static bool isConnected()
    {
        return false;
    }
};

#endif
