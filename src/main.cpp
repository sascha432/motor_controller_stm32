/**
  Author: sascha_lammers@gmx.de
*/

#include "i2c.h"
#include "controls.h"
#include "mt6701_encoder.h"
#include "pid_controller.h"
#include "adc.h"
#include "leds.h"
#include "tft_driver.h"
#include "ui.h"
#include "menu.h"
#include "eeprom.h"
#include "stats.h"
#include "debug.h"

#if defined(USB_LEGACY_BRINGUP)

#if defined(USBCON) && defined(USBD_USE_CDC)
    #include <USBSerial.h>
    #include <usbd_if.h>
    #include <usbd_cdc_if.h>
    extern USBD_HandleTypeDef hUSBD_Device_CDC;
#else
    #error USB CDC library support is required (USBCON + USBD_USE_CDC)
#endif

static inline USB_OTG_DeviceTypeDef *usb_otg_fs_dev_regs()
{
    return (USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_FS + USB_OTG_DEVICE_BASE);
}

#if defined(STM32F107xC)
static void usb_force_pinmux_f107()
{
    // Ensure USB-related control pins are in expected states before CDC init.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    // PA9 (VBUS sense): input floating to avoid loading external sense network.
    GPIOA->CRH &= ~(0xFUL << ((9U - 8U) * 4U));
    GPIOA->CRH |=  (0x4UL << ((9U - 8U) * 4U));

    // PA10 (ID): input with pull-up
    GPIOA->CRH &= ~(0xFUL << ((10U - 8U) * 4U));
    GPIOA->CRH |=  (0x8UL << ((10U - 8U) * 4U));
    GPIOA->ODR |= (1UL << 10U);
}
#endif

extern "C" void usb_legacy_init(void)
{
#if defined(STM32F107xC)
    usb_force_pinmux_f107();

    // Prefer B-device VBUS sensing on PA9 when NOVBUSSENS is not available.
#if defined(USB_OTG_GCCFG_NOVBUSSENS)
    USB_OTG_FS->GCCFG &= ~(USB_OTG_GCCFG_VBUSASEN | USB_OTG_GCCFG_VBUSBSEN);
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
#else
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
#endif
#endif

    // Force peripheral mode in case OTG ID pin state keeps the core in host mode.
    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    delay(5);

    SerialUSB.begin();

    // Trigger a fresh host re-enumeration pulse after CDC core init.
    USBD_reenumerate();

    // Request a B-device session in case session state machine did not start.
    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_SRQ;

    // Some OTG revisions allow software-valid override of session-valid bits.
    // If these bits are read-only on this silicon, readback will remain unchanged.
    USB_OTG_FS->GOTGCTL |= (USB_OTG_GOTGCTL_BSVLD | USB_OTG_GOTGCTL_ASVLD);

    // Force one more disconnect/connect pulse at the OTG device level.
    usb_otg_fs_dev_regs()->DCTL |= USB_OTG_DCTL_SDIS;
    delay(5);
    usb_otg_fs_dev_regs()->DCTL &= ~USB_OTG_DCTL_SDIS;
}

extern "C" void usb_legacy_poll(void)
{
    // Keep RX queue drained during setup-only test to avoid backpressure.
    while (SerialUSB.available() > 0) {
        (void)SerialUSB.read();
    }
}

extern "C" void usb_legacy_write_line(const char *line)
{
    if (!line) {
        return;
    }
    SerialUSB.println(line);
}

static void usb_setup_only_init()
{
    DEBUG_PRINT(DEBUG_NOTICE, "USB setup-only bring-up start");

#if defined(STM32F107xC)
    // Configure OTG FS VBUS/session sensing for device mode.
    RCC->AHBENR |= RCC_AHBENR_OTGFSEN;
    
#if defined(USB_OTG_GCCFG_NOVBUSSENS)
    USB_OTG_FS->GCCFG &= ~(USB_OTG_GCCFG_VBUSASEN | USB_OTG_GCCFG_VBUSBSEN);
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
#else
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
#endif
    USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
    USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    usb_force_pinmux_f107();

    DEBUG_PRINT(DEBUG_NOTICE, "USB OTG pre-init GCCFG=0x%08lx GUSBCFG=0x%08lx GOTGCTL=0x%08lx", USB_OTG_FS->GCCFG, USB_OTG_FS->GUSBCFG, USB_OTG_FS->GOTGCTL);
#endif

    // Initialize legacy USB device stack (weak symbol until library is linked).
    usb_legacy_init();
    delay(20);

    const uint32_t gotgctl = USB_OTG_FS->GOTGCTL;
    const uint32_t gintsts = USB_OTG_FS->GINTSTS;

    DEBUG_PRINT(DEBUG_NOTICE, "USB init st=%u old=%u cfg=%lu addr=%u ep0=%lu conn=%u dtr=%u", hUSBD_Device_CDC.dev_state, hUSBD_Device_CDC.dev_old_state, hUSBD_Device_CDC.dev_config, hUSBD_Device_CDC.dev_address, hUSBD_Device_CDC.ep0_state, CDC_connected() ? 1u : 0u, SerialUSB.dtr() ? 1u : 0u);
    DEBUG_PRINT(DEBUG_NOTICE, "USB regs GCCFG=%08lx GUSBCFG=%08lx GOTGCTL=%08lx GAHBCFG=%08lx GINTMSK=%08lx GINTSTS=%08lx DCFG=%08lx DCTL=%08lx DSTS=%08lx", USB_OTG_FS->GCCFG, USB_OTG_FS->GUSBCFG, gotgctl, USB_OTG_FS->GAHBCFG, USB_OTG_FS->GINTMSK, gintsts, usb_otg_fs_dev_regs()->DCFG, usb_otg_fs_dev_regs()->DCTL, usb_otg_fs_dev_regs()->DSTS);
    DEBUG_PRINT(DEBUG_NOTICE, "USB bits CIDSTS=%u BSVLD=%u SRQ=%u SRQINT=%u USBRST=%u ENUMDNE=%u RXFLVL=%u PA9=%u", (gotgctl & USB_OTG_GOTGCTL_CIDSTS) ? 1u : 0u, (gotgctl & USB_OTG_GOTGCTL_BSVLD) ? 1u : 0u, (gotgctl & USB_OTG_GOTGCTL_SRQ) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_SRQINT) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_USBRST) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_ENUMDNE) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_RXFLVL) ? 1u : 0u, (GPIOA->IDR >> 9U) & 1U);
}

#endif

static void button_isr() 
{
    #if defined(STM32F107xC)
        uint32_t idrD = ((GPIO_TypeDef *)GPIOD_BASE)->IDR;
        knobButton.isr(idrD);
        backButton.isr(idrD);
        startButton.isr(idrD);
    #elif defined(STM32F103xC)
        uint32_t idrB = ((GPIO_TypeDef *)GPIOB_BASE)->IDR;
        startButton.isr(idrB);
        uint32_t idrA = ((GPIO_TypeDef *)GPIOA_BASE)->IDR;
        knobButton.isr(idrA);
        backButton.isr(idrA);
        TODO pin macros need to be  chagned and isr too
    #else
        #error missing ISR implementation for this MCU
    #endif
}

static void knob_isr()
{
    knob.isr();
}

static void pid_timer_isr()
{
    pid.isr();
}

static void pid_fault_isr()
{
    pid.fault_isr();
}

void setup()
{
    debug_init();

#if defined(USB_LEGACY_BRINGUP)
    usb_setup_only_init();

    for(;;) {
        delay(1000);
        usb_legacy_poll();
        const uint32_t gotgctl = USB_OTG_FS->GOTGCTL;
        const uint32_t gintsts = USB_OTG_FS->GINTSTS;
        DEBUG_PRINT(DEBUG_DEBUG, "USB hb t=%lu st=%u old=%u cfg=%lu addr=%u ep0=%lu conn=%u dtr=%u", millis(), hUSBD_Device_CDC.dev_state, hUSBD_Device_CDC.dev_old_state, hUSBD_Device_CDC.dev_config, hUSBD_Device_CDC.dev_address, hUSBD_Device_CDC.ep0_state, CDC_connected() ? 1u : 0u, SerialUSB.dtr() ? 1u : 0u);
        DEBUG_PRINT(DEBUG_DEBUG, "USB hb regs GOTGCTL=%08lx GAHBCFG=%08lx GINTMSK=%08lx GINTSTS=%08lx DCFG=%08lx DCTL=%08lx DSTS=%08lx", gotgctl, USB_OTG_FS->GAHBCFG, USB_OTG_FS->GINTMSK, gintsts, usb_otg_fs_dev_regs()->DCFG, usb_otg_fs_dev_regs()->DCTL, usb_otg_fs_dev_regs()->DSTS);
        DEBUG_PRINT(DEBUG_DEBUG, "USB hb bits CIDSTS=%u BSVLD=%u SRQ=%u SRQINT=%u USBRST=%u ENUMDNE=%u RXFLVL=%u PA9=%u", (gotgctl & USB_OTG_GOTGCTL_CIDSTS) ? 1u : 0u, (gotgctl & USB_OTG_GOTGCTL_BSVLD) ? 1u : 0u, (gotgctl & USB_OTG_GOTGCTL_SRQ) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_SRQINT) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_USBRST) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_ENUMDNE) ? 1u : 0u, (gintsts & USB_OTG_GINTSTS_RXFLVL) ? 1u : 0u, (GPIOA->IDR >> 9U) & 1U);

        char usbLine[96];
        snprintf(usbLine, sizeof(usbLine), "[USB-SETUP] t=%lu GCCFG=0x%08lx", millis(), (unsigned long)USB_OTG_FS->GCCFG);
        usb_legacy_write_line(usbLine);
    }
#endif

    // Initialize and read EEPROM on I2C1 on PB8/9
    eeprom.init();
    eeprom.read();

    // LEDs
    LEDs::init();

    // motor encoder
    motorEncoder.init();
    if (PidController::kProgramPPR) {
        motorEncoder.programPPR(i2c, PidController::kPPR);
    }

    // buttons
    knobButton.init(button_isr);
    backButton.init(button_isr);
    startButton.init(button_isr);

    // rotary encoder knob
    knob.init();
    knob.enable(knob_isr);

    // ADC with DMA
    adc.init();
    // DAC
    adc.initDAC();
    // PID controller
    pid.init(pid_timer_isr, pid_fault_isr);

    // Initialize display driver
    tft_driver_init();
    tft_clear_display();

    // Initialize LVGL and register flush callback
    lv_init();
    tft_driver_lvgl_init();

#if 0
    // color flashing test loop
    tft_backlight_pwm_set(100);
    uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFFF};// Red, Green, Blue, White
    int c = 0;

    for(;;) {
        uint32_t start = millis();
        tft_clear_display(colors[c++%4]);
        uint32_t dur = millis() - start;
        DEBUG_PRINT(DEBUG_DEBUG, "Clear display took %lu ms", dur);
        delay(250);
    }
#endif

    // Show welcome screen and load main menu
    menu.showWelcomeScreen();
    // Apply settings after welcome screen since it turns the backlight on
    menu.applyEEPROMSettings();

    menu.loadStartScreen();
}

void loop()
{
    // handle buttons
    if (knobButton.isPressed()) {
        menu.handleButtonPress();
    }
    if (backButton.isPressed()) {
        menu.handleBackButtonPress();
    }
    if (startButton.isPressed()) {
        menu.handleStartButtonPress();
    }

    if (pid.faults.count) {
        static uint32_t lastFaultTime = 0;
        if (millis() - lastFaultTime >= 500) {
            lastFaultTime = millis();
            pid.faults.reset();
            LEDs::offLED1and2();
        }
        else {
            if (pid.faults.ocpFault) {
                LEDs::onLED1();
            }
            if (pid.faults.snsoutFault) {
                LEDs::onLED2();
            }
        }
    }

    // handle ui updates and rotary encoder
    static uint32_t lastLvHandler = 0;
    if (millis() - lastLvHandler >= 5) {
        // handle rotary encoder
        int32_t newPosition;
        int32_t delta = knob.getDeltaPosition();
        if (delta) {
            newPosition = menu.updateRotaryValue(delta);
            DEBUG_PRINT(DEBUG_DEBUG, "menu=%d delta=%d", newPosition, delta);
        }
        // handle LVGL updates
        auto &screenFlow = menu.getScreenFlow();
        switch(screenFlow->getId()) {
            case Screen::Type::DASHBOARD:
            case Screen::Type::DIAGNOSTICS:
                stats.update();
                screenFlow->update();
                break;
            default:
                break;
        }
        // check NTC sensors, not time critical and a couple times per seconds is enough
        if (adc.getMotorNTCValue() < eeprom.getMotorTemperatureLimitADC()) {
            if (pid.running) {
                pid.setErrorCode(PidController::ErrorCodeType::MOTOR_OVER_TEMPERATURE);
            }
        }
        if (adc.getMosfetNTCValue() < eeprom.getMosfetTemperatureLimitADC()) {
            if (pid.running) {
                pid.setErrorCode(PidController::ErrorCodeType::MOSFET_OVER_TEMPERATURE);
            }
        }
        
        // update UI
        lv_timer_handler();
        lastLvHandler = millis();
    }

    if (false) {
        static uint32_t lastTime37 = 0;
        if (millis() - lastTime37 >= 100) {
            lastTime37 = millis();
            // extern volatile uint32_t rpm_counter;
            // DEBUG_PRINT(DEBUG_DEBUG, "RPM_COUNTER=%u", rpm_counter);
            // DEBUG_PRINT(DEBUG_DEBUG, "TIM5=%u", TIM5->CNT);
        }
    }

    if (false) { // print faults
        static uint32_t lastTime3 = 0;
        if (millis() - lastTime3 >= 1000) {
            lastTime3 = millis();
            static uint32_t lastCounter = 0;
            if (pid.faults.count != lastCounter) {
                lastCounter = pid.faults.count;
                pid.debugPrintFaults();
            }
        }
    }
}
