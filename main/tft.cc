#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_types.h>

extern "C"
{
#include "lvgl.h"
}

#include "tft.h"

static const char *TAG_TFT = "tft";

// Globals only used inside TFT module
static esp_lcd_panel_io_handle_t s_panel_io = nullptr;
static SemaphoreHandle_t s_flush_sem = nullptr;

// Backlight control
static void tft_bl_on_internal(void)
{
#if defined(TFT_BL)
    gpio_config_t io{.pin_bit_mask = 1ULL << TFT_BL, .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io);
    gpio_set_level(TFT_BL, TFT_BACKLIGHT_ON ? 1 : 0);
#endif
}

// HW reset
static void tft_hw_reset_internal()
{
    if (TFT_RST != GPIO_NUM_NC)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << TFT_RST,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(TFT_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(TFT_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static inline void tft_send_cmd(uint8_t cmd, const void *data, size_t len)
{
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, data, len));
}

// ST7735 minimal init
static void st7735_init()
{
    uint8_t cmd;
    cmd = 0x11; // Sleep out
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t colmod[] = {0x05}; // 16-bit
    cmd = 0x3A;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, colmod, sizeof(colmod)));

    uint8_t madctl[] = {0x08}; // BGR
    cmd = 0x36;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, madctl, sizeof(madctl)));

#if TFT_HAS_INVERSION
    cmd = 0x20; // INVOFF (no inversion)
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));
#endif
    cmd = 0x13; // Normal display on
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));
    cmd = 0x29; // Display on
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));
}

// GC9D01 minimal init
static void gc9d01_init()
{
    // Sleep out
    tft_send_cmd(0x11, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    tft_send_cmd(0xFE, nullptr, 0);
    tft_send_cmd(0xEF, nullptr, 0);

    uint8_t d;

    d = 0xFF;
    tft_send_cmd(0x80, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x81, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x82, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x83, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x84, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x85, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x86, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x87, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x88, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x89, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x8A, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x8B, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x8C, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x8D, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x8E, &d, 1);
    d = 0xFF;
    tft_send_cmd(0x8F, &d, 1);

    d = 0x05;
    tft_send_cmd(0x3A, &d, 1); // COLMOD: 16bpp
    d = 0x01;
    tft_send_cmd(0xEC, &d, 1);

    {
        uint8_t p[] = {0x02, 0x0E, 0, 0, 0, 0, 0};
        tft_send_cmd(0x74, p, sizeof(p));
    }
    d = 0x3E;
    tft_send_cmd(0x98, &d, 1);
    d = 0x3E;
    tft_send_cmd(0x99, &d, 1);
    {
        uint8_t p[] = {0x0D, 0x0D};
        tft_send_cmd(0xB5, p, sizeof(p));
    }

    {
        uint8_t p[] = {0x38, 0x0F, 0x79, 0x67};
        tft_send_cmd(0x60, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x38, 0x11, 0x79, 0x67};
        tft_send_cmd(0x61, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x38, 0x17, 0x71, 0x5F, 0x79, 0x67};
        tft_send_cmd(0x64, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x38, 0x13, 0x71, 0x5B, 0x79, 0x67};
        tft_send_cmd(0x65, p, sizeof(p));
    }

    {
        uint8_t p[] = {0x00, 0x00};
        tft_send_cmd(0x6A, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x22, 0x02, 0x22, 0x02, 0x22, 0x22, 0x50};
        tft_send_cmd(0x6C, p, sizeof(p));
    }

    {
        uint8_t p[] = {
            0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x0F, 0x0F,
            0x0D, 0x0D, 0x0B, 0x0B, 0x09, 0x09, 0x00, 0x00,
            0x00, 0x00, 0x0A, 0x0A, 0x0C, 0x0C, 0x0E, 0x0E,
            0x10, 0x10, 0x00, 0x00, 0x02, 0x02, 0x04, 0x04};
        tft_send_cmd(0x6E, p, sizeof(p));
    }

    d = 0x01;
    tft_send_cmd(0xBF, &d, 1);
    d = 0x40;
    tft_send_cmd(0xF9, &d, 1);

    d = 0x3B;
    tft_send_cmd(0x9B, &d, 1);
    {
        uint8_t p[] = {0x33, 0x7F, 0x00};
        tft_send_cmd(0x93, p, sizeof(p));
    }

    d = 0x30;
    tft_send_cmd(0x7E, &d, 1);
    {
        uint8_t p[] = {0x0D, 0x02, 0x08, 0x0D, 0x02, 0x08};
        tft_send_cmd(0x70, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x0D, 0x02, 0x08};
        tft_send_cmd(0x71, p, sizeof(p));
    }

    {
        uint8_t p[] = {0x0E, 0x09};
        tft_send_cmd(0x91, p, sizeof(p));
    }

    d = 0x19;
    tft_send_cmd(0xC3, &d, 1);
    d = 0x19;
    tft_send_cmd(0xC4, &d, 1);
    d = 0x3C;
    tft_send_cmd(0xC9, &d, 1);

    {
        uint8_t p[] = {0x53, 0x15, 0x0A, 0x04, 0x00, 0x3E};
        tft_send_cmd(0xF0, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x53, 0x15, 0x0A, 0x04, 0x00, 0x3A};
        tft_send_cmd(0xF2, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x56, 0xA8, 0x7F, 0x33, 0x34, 0x5F};
        tft_send_cmd(0xF1, p, sizeof(p));
    }
    {
        uint8_t p[] = {0x52, 0xA4, 0x7F, 0x33, 0x34, 0xDF};
        tft_send_cmd(0xF3, p, sizeof(p));
    }

    // Orientation / color order
    // Use BGR here (0x08) to avoid software R/B twiddling in the flush.
    d = 0x08;
    tft_send_cmd(0x36, &d, 1);

    // Inversion off (matches your Arduino)
    tft_send_cmd(0x20, nullptr, 0);

    // Display ON
    tft_send_cmd(0x29, nullptr, 0);
    // Optional RAMWR priming
    tft_send_cmd(0x2C, nullptr, 0);

    uint8_t colmod[] = {0x05}; //{0x55}; // RGB565

    ESP_LOGI(TAG_TFT, "Setting color mode to: %02X", colmod[0]);

    tft_send_cmd(0x3A, colmod, sizeof(colmod));

    //uint8_t madctl[] = {0x08}; // BGR
    uint8_t madctl[] = {0x00}; // RGB
    tft_send_cmd(0x36, madctl, sizeof(madctl));

    tft_send_cmd(0x35, nullptr, 0);

#if TFT_HAS_INVERSION
    tft_send_cmd(0x21, nullptr, 0); // INVON
#else
    tft_send_cmd(0x20, nullptr, 0); // INVOFF
#endif

    // Display on
    tft_send_cmd(0x29, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static inline void tft_init_panel()
{
#if defined(DISPLAY_DRIVER_ST7735)
    st7735_init();
#elif defined(DISPLAY_DRIVER_GC9D01)
    gc9d01_init();
#else
    st7735_init();
#endif
}

// Flush callback: push a rectangular area to the panel
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (!s_panel_io)
    {
        lv_display_flush_ready(disp);
        return;
    }
    const int x1 = area->x1 + TFT_OFFSET_X;
    const int y1 = area->y1 + TFT_OFFSET_Y;
    const int x2 = area->x2 + TFT_OFFSET_X;
    const int y2 = area->y2 + TFT_OFFSET_Y;

    uint8_t cmd = 0x2A; // Column address
    uint8_t col_param[4] = {
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF),
        (uint8_t)(x2 >> 8), (uint8_t)(x2 & 0xFF)};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, col_param, sizeof(col_param)));

    cmd = 0x2B; // Row address
    uint8_t row_param[4] = {
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF),
        (uint8_t)(y2 >> 8), (uint8_t)(y2 & 0xFF)};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, row_param, sizeof(row_param)));

    cmd = 0x2C; // Memory write
#if defined(TFT_SWAP_BYTES) && TFT_SWAP_BYTES
    lv_draw_sw_rgb565_swap(px_map, (x2 - x1 + 1) * (y2 - y1 + 1));
#endif
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(s_panel_io, cmd, px_map,
                                              (x2 - x1 + 1) * (y2 - y1 + 1) * 2));

    if (s_flush_sem)
    {
        if (xSemaphoreTake(s_flush_sem, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
            ESP_LOGW(TAG_TFT, "flush wait timeout - SPI transfer may be stuck");
        }
    }
    lv_display_flush_ready(disp);
}

// SPI transfer done ISR context
static bool on_color_trans_done(esp_lcd_panel_io_handle_t /*panel_io*/,
                                esp_lcd_panel_io_event_data_t * /*edata*/,
                                void * /*user_ctx*/)
{
    BaseType_t hp_task_woken = pdFALSE;
    if (s_flush_sem)
    {
        xSemaphoreGiveFromISR(s_flush_sem, &hp_task_woken);
    }
    return hp_task_woken == pdTRUE;
}

#ifdef CONFIG_BOARD_TFT071
// Simple color band test for debugging
static void tft_test_pattern_internal()
{
    if (!s_panel_io)
        return;
    ESP_LOGI(TAG_TFT, "Drawing red test pattern");
    const uint16_t color = 0x00F8; // Red with byte swap
    const int band_px = LCD_H_RES * LCD_V_RES;
    uint16_t *buf = (uint16_t *)heap_caps_malloc(band_px * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buf)
        return;
    for (int i = 0; i < band_px; ++i)
        buf[i] = color;
#if defined(TFT_SWAP_BYTES) && TFT_SWAP_BYTES
    lv_draw_sw_rgb565_swap((uint8_t *)buf, band_px);
#endif
    // Set window to full screen
    uint8_t cmd = 0x2A;
    int x1 = 0, x2 = LCD_H_RES - 1;
    uint8_t col_param[4] = {(uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, col_param, sizeof(col_param)));
    cmd = 0x2B;
    int y1 = 0, y2 = LCD_V_RES - 1;
    uint8_t row_param[4] = {(uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, row_param, sizeof(row_param)));
    cmd = 0x2C;
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(s_panel_io, cmd, buf, band_px * 2));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Show for 1 second
    heap_caps_free(buf);
}
#endif

void tft_init_all()
{
    // Initialize LVGL before any LVGL API usage
    lv_init();
    // Backlight on first
    tft_bl_on_internal();

    // SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = TFT_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = TFT_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * 40 * sizeof(lv_color_t) + 8;
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Panel IO
    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = TFT_DC;
    io_config.cs_gpio_num = TFT_CS;
    io_config.pclk_hz = LCD_SPI_CLOCK_HZ;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = TFT_SPI_MODE;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = on_color_trans_done;
    io_config.user_ctx = nullptr;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_HOST, &io_config, &io_handle));
    s_panel_io = io_handle;

    // Reset and init panel
    tft_hw_reset_internal();
    tft_init_panel();

#ifdef CONFIG_BOARD_TFT071
    // Optional quick sanity test
    tft_test_pattern_internal();
#endif

    // LVGL display setup
    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_default(disp);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

    const uint32_t lines = 40;
    const uint32_t buf_size = LCD_H_RES * lines * 2; // RGB565
    void *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    void *buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    assert(buf1 && buf2);
    lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    // Create flush semaphore
    s_flush_sem = xSemaphoreCreateBinary();

    ESP_LOGI(TAG_TFT, "TFT initialized (mode=%d, clk=%u)", (int)TFT_SPI_MODE, (unsigned)LCD_SPI_CLOCK_HZ);
}
