/**
  Author: sascha_lammers@gmx.de

  Baremetal SPI DMA driver
*/

#include <stm32f1xx.h>
#include "tft_driver.h"
#include "tft_driver_screenshot.h"

lv_disp_draw_buf_t s_lvgl_draw_buf;
lv_color_t s_lvgl_buf_1[LV_BUFFER_SIZE];
lv_disp_drv_t s_lvgl_disp_drv;
TIM_HandleTypeDef tim2;

/**
 * @brief Calculate the DMA transfer timeout based on the max. number of bytes to transfer and the SPI clock speed
 *
 * @param bytes Number of bytes to transfer
 * @param spiClockHz SPI clock speed in Hz (default: 18 MHz)
 * @return constexpr uint32_t Calculated timeout in milliseconds
 */
static constexpr uint32_t kCalculateDMATimeoutMs(uint16_t bytes, uint32_t spiClockHz = 18000000)
{
    return (((bytes * 8000) + (spiClockHz - 1)) / spiClockHz);
}

static constexpr uint32_t kDMATransferTimeoutMillis = kCalculateDMATimeoutMs(sizeof(s_lvgl_buf_1));         // DMA transfer timeout in milliseconds
static constexpr uint32_t kSPISyncTimeoutMillis = 3;                                                        // SPI synchronization timeout in milliseconds

/**
 * @brief init GPIO pins and timers for the SPI display and backlight PWM
 *
 */
void tft_driver_gpio_tim_init(void)
{
    // Enable GPIOC and GPIOD clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // TFT_RST/PC6, TFT_DC/PC7 (CRL)
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = TFT_RST_Pin | TFT_DC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TFT_RST_GPIO_Port, &GPIO_InitStruct);

    // TFT_CS/PD15 (CRH)
    GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = TFT_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TFT_CS_GPIO_Port, &GPIO_InitStruct);

    // set pins to default state
    TFT_PIN_CS_HIGH();
    TFT_PIN_RS_HIGH();

    // backlight PWM on TFT_PIN_LED/PB11
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_AFIO_REMAP_TIM2_PARTIAL_2();

    // TFT_PWM/PB11 and LED_PWM/PB10 = Alternate Function Push-Pull, 50 MHz (CRH)
    GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = TFT_PWM_Pin | LED_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TFT_PWM_GPIO_Port, &GPIO_InitStruct);

    // TIM2 setup
    tim2.Instance = TIM2;
    tim2.Init.Prescaler = 71;          // 72MHz / (71+1) = 1MHz timer clock
    tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim2.Init.Period = 999;            // 1MHz / (999+1) = 1kHz PWM
    tim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&tim2);

    // PWM mode 1 CH3 + CH4
    TIM_OC_InitTypeDef sConfigOC = {};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&tim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&tim2, &sConfigOC, TIM_CHANNEL_4);

    // Start PWM outputs
    HAL_TIM_PWM_Start(&tim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&tim2, TIM_CHANNEL_4);
}

/**
 * Initialize SPI2 for display communication
 */
void tft_driver_spi_init(void)
{
    // Enable clocks
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure TFT_CLK/PB13 and TFT_MOSI/PB15 as alternate function push-pull
    GPIO_InitTypeDef GPIO_InitStruct = {};
    GPIO_InitStruct.Pin = TFT_CLK_Pin | TFT_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TFT_CLK_GPIO_Port, &GPIO_InitStruct);

    // Disable SPI first
    SPI2->CR1 = 0;

    // Configure SPI2 - BR=0 gives ~18MHz on APB1=36MHz
    SPI2->CR1 = (0 << SPI_CR1_BR_Pos)   // Baud rate: BR=0 (36MHz/2 = 18MHz)
            | SPI_CR1_MSTR              // Master mode
            | SPI_CR1_SSM               // Software slave management
            | SPI_CR1_SSI               // Internal slave select
            | SPI_CR1_CPOL              // Clock polarity = 1
            | SPI_CR1_CPHA;             // Clock phase = 1
    SPI2->CR2 = 0;  // DMA disabled by default, enabled per transfer

    // Enable SPI2
    SPI2->CR1 |= SPI_CR1_SPE;

    // initialize DMA1 Channel 5 for SPI2 TX
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    TFT_DMA_CH->CCR = 0;
    TFT_DMA_CH->CPAR = reinterpret_cast<const uint32_t>(&SPI2->DR);
}

/**
 * Send data buffer via SPI2 TX DMA with CS already asserted.
 */
void tft_driver_spi_send_buffer_dma_raw(const void *data, uint16_t len)
{
    TFT_DMA_CH->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;

    // Configure DMA transfer source and parameters
    TFT_DMA_CH->CMAR = reinterpret_cast<const uint32_t>(data);  // Source address (data buffer)
    TFT_DMA_CH->CNDTR = len;                                    // Number of bytes to transfer
    // Channel control: MINC=memory increment mode, DIR=memory-to-peripheral, PL_1=medium priority
    TFT_DMA_CH->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_1;

    // === DMA Transmission ===
    // Enable SPI2 to accept DMA requests on TX and start the DMA transfer
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
    TFT_DMA_CH->CCR |= DMA_CCR_EN;

    // Wait for DMA transfer to complete with timeout protection
    uint32_t start = HAL_GetTick();
    while (!(DMA1->ISR & DMA_ISR_TCIF5) && (HAL_GetTick() - start <= kDMATransferTimeoutMillis)) {
    }

    // === Post-Transfer Cleanup ===
    // Disable DMA channel after transfer
    TFT_DMA_CH->CCR &= ~DMA_CCR_EN;
    // Disable SPI TX DMA requests
    SPI2->CR2 &= ~SPI_CR2_TXDMAEN;
    // Clear all DMA flags again
    DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5;

    // === SPI Synchronization ===
    // Wait for TX FIFO to empty (TXE flag set) - all bytes shifted into shift register
    start = HAL_GetTick();
    while (((SPI2->SR & SPI_SR_TXE) == 0U) && (HAL_GetTick() - start <= kSPISyncTimeoutMillis)) {
    }

    // Wait for SPI to finish transmitting (BSY flag clear) - shift register emptied on wire
    start = HAL_GetTick();
    while ((SPI2->SR & SPI_SR_BSY) && (HAL_GetTick() - start <= kSPISyncTimeoutMillis)) {
    }

    // === RX FIFO Cleanup ===
    // Clear any data in RX FIFO (SPI receives during full-duplex transmission)
    while (SPI2->SR & SPI_SR_RXNE) {
        (void)SPI2->DR;  // Read and discard RX data
    }
    // Read SR register to clear any pending status flags
    (void)SPI2->SR;
}

/**
 * Send single SPI byte (DMA)
 */
void tft_driver_spi_send_byte(uint8_t byte)
{
    /* Pull CS low before transfer */
    TFT_PIN_CS_LOW();
    tft_driver_delay();

    tft_driver_spi_send_buffer_dma_raw(&byte, 1);

    /* Pull CS high after transfer */
    tft_driver_delay();
    TFT_PIN_CS_HIGH();
}

/**
 * Send data buffer via SPI (DMA)
 */
void tft_driver_spi_send_buffer(const void *data, uint16_t len)
{
    TFT_PIN_CS_LOW();
    tft_driver_delay();
    tft_driver_spi_send_buffer_dma_raw(data, len);
    tft_driver_delay();
    TFT_PIN_CS_HIGH();
}

/**
 * Send SPI command byte (with RS=0)
 */
void tft_driver_send_command(uint8_t cmd)
{
    TFT_PIN_RS_LOW();
    tft_driver_delay();
    tft_driver_spi_send_byte(cmd);
    tft_driver_delay();
    TFT_PIN_RS_HIGH();
}

/**
 * Send SPI data bytes (with RS=1)
 */
void tft_driver_send_data(const void *data, uint16_t len)
{
    TFT_PIN_RS_HIGH();
    tft_driver_spi_send_buffer(data, len);
}

/**
 * Flush callback for LVGL
 */
void tft_driver_lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    const uint16_t x0 = area->x1;
    const uint16_t y0 = area->y1;
    const uint16_t x1 = area->x2;
    const uint16_t y1 = area->y2;
    const uint32_t px = static_cast<uint32_t>(x1 - x0 + 1U) * static_cast<uint32_t>(y1 - y0 + 1U);

    tft_write_window_pixels(x0, y0, x1, y1, reinterpret_cast<const uint16_t *>(color_p), px);

    #if HAVE_SCREENSHOTS
    if (screenshot.isActive()) {
        screenshot.write_tile(x0, y0, x1, y1, color_p);
    }
    #endif

    lv_disp_flush_ready(disp_drv);
}

/**
 * Initialize LVGL display driver and register callback
 */
void tft_driver_lvgl_init(void)
{
    lv_disp_draw_buf_init(&s_lvgl_draw_buf, s_lvgl_buf_1, nullptr, LV_BUFFER_SIZE);
    lv_disp_drv_init(&s_lvgl_disp_drv);
    s_lvgl_disp_drv.hor_res = LV_HOR_RES_MAX;
    s_lvgl_disp_drv.ver_res = LV_VER_RES_MAX;
    s_lvgl_disp_drv.flush_cb = tft_driver_lvgl_flush_cb;
    s_lvgl_disp_drv.draw_buf = &s_lvgl_draw_buf;
    lv_disp_drv_register(&s_lvgl_disp_drv);
}


#if HAVE_SCREENSHOTS

// === Screenshot streaming support ===

TFTDriverScreenshot screenshot;

bool TFTDriverScreenshot::write_tile(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const lv_color_t *color_p)
{
    if (!active) {
        return false;
    }
    if (!isPortWritable()) {
        active = false;
        return false;
    }
    const uint32_t width = static_cast<uint32_t>(x1 - x0 + 1U);
    const uint32_t height = static_cast<uint32_t>(y1 - y0 + 1U);
    const uint32_t byte_count = width * height * sizeof(uint16_t);
    TileHeader header = TileHeader(x0, y0, width, height, byte_count);

    if (!writeByte(sizeof(header))) {
        active = false;
        return false;
    }
    if (!write(header)) {
        active = false;
        return false;
    }
    if (!write(color_p, byte_count)) {
        active = false;
        return false;
    }
    return true;
}

bool TFTDriverScreenshot::begin()
{
    if (active) {
        return true;
    }
    if (!isPortWritable()) {
        return false;
    }
    FrameHeader header = FrameHeader(LV_HOR_RES_MAX, LV_VER_RES_MAX, kPixelFormat);
    if (!writeByte(sizeof(header))) {
        return false;
    }
    if (!write(header)) {
        return false;
    }
    active = true;
    return true;
}

void TFTDriverScreenshot::end()
{
    if (!active) {
        return;
    }
    if (!isPortWritable()) {
        return;
    }
    writeByte(0);
    active = false;
}

#endif
