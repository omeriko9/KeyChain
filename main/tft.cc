#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_types.h>

#include <WiFi.h>

#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <algorithm>
#include <random>

extern "C"
{
#include "lvgl.h"
}

#include "tft.h"

static const char *TAG_TFT = "tft";
// Warm image rendering toggle (can later be exposed via Kconfig or runtime config)
static bool g_warm_images = true; // set false to disable warm tone adjustment

// Globals only used inside TFT module
static esp_lcd_panel_io_handle_t s_panel_io = nullptr;
static SemaphoreHandle_t s_flush_sem = nullptr;

// LVGL related globals
static int g_image_display_sec = 10; // default image display time
static const char *IMG_DIR = "/spiffs/img";

// Helper functions
static std::vector<std::string> get_image_list();
static void create_demo_ui();
static void btn_event_handler(lv_event_t *e);
static void slider_event_handler(lv_event_t *e);
static void tft_clear(uint16_t color);

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
    // Use BGR here
    uint8_t madctl[] = {0x00}; // BGR
    tft_send_cmd(0x36, madctl, sizeof(madctl));

    // Inversion off (matches your Arduino)
    tft_send_cmd(0x20, nullptr, 0);

    // Display ON
    tft_send_cmd(0x29, nullptr, 0);
    // Optional RAMWR priming
    tft_send_cmd(0x2C, nullptr, 0);

    uint8_t colmod[] = {0x05}; //{0x55}; // RGB565

    ESP_LOGI(TAG_TFT, "Setting color mode to: %02X", colmod[0]);

    tft_send_cmd(0x3A, colmod, sizeof(colmod));

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

    tft_clear(0x0000); // black
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
    const uint16_t color = 0xF800; // Red in BGR format
    const int band_px = LCD_H_RES * LCD_V_RES;
    uint16_t *buf = (uint16_t *)heap_caps_malloc(band_px * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!buf)
        return;
    for (int i = 0; i < band_px; ++i)
        buf[i] = color;
    // No byte swapping needed for BGR MADCTL
    // #if defined(TFT_SWAP_BYTES) && TFT_SWAP_BYTES
    //     lv_draw_sw_rgb565_swap((uint8_t *)buf, band_px);
    // #endif
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

// =====================
// LVGL filesystem driver for SPIFFS
// =====================
void tft_init_lvgl_filesystem()
{
    static lv_fs_drv_t fs_drv;
    lv_fs_drv_init(&fs_drv);
    fs_drv.letter = 'A';
    fs_drv.open_cb = [](lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) -> void *
    {
        FILE *f = fopen(path, "rb");
        if (!f)
            ESP_LOGE(TAG_TFT, "Failed to open file: %s", path);
        return (void *)f;
    };
    fs_drv.close_cb = [](lv_fs_drv_t *drv, void *file_p) -> lv_fs_res_t
    {
        ESP_LOGD(TAG_TFT, "LVGL FS close");
        fclose((FILE *)file_p);
        return LV_FS_RES_OK;
    };
    fs_drv.read_cb = [](lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br) -> lv_fs_res_t
    {
        size_t n = fread(buf, 1, btr, (FILE *)file_p);
        *br = (uint32_t)n;
        ESP_LOGD(TAG_TFT, "LVGL FS read: requested %u, read %u", (unsigned)btr, (unsigned)*br);
        if (n < btr)
        {
            if (feof((FILE *)file_p))
                return LV_FS_RES_OK; // EOF is not an error
            if (ferror((FILE *)file_p))
            {
                ESP_LOGE(TAG_TFT, "LVGL FS read error");
                clearerr((FILE *)file_p);
                return LV_FS_RES_FS_ERR;
            }
        }
        return LV_FS_RES_OK;
    };
    fs_drv.seek_cb = [](lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence) -> lv_fs_res_t
    {
        int origin = (whence == LV_FS_SEEK_SET) ? SEEK_SET : (whence == LV_FS_SEEK_CUR) ? SEEK_CUR
                                                                                        : SEEK_END;
        if (fseek((FILE *)file_p, pos, origin) == 0)
            return LV_FS_RES_OK;
        ESP_LOGE(TAG_TFT, "LVGL FS seek failed");
        return LV_FS_RES_UNKNOWN;
    };
    fs_drv.tell_cb = [](lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p) -> lv_fs_res_t
    {
        long pos = ftell((FILE *)file_p);
        if (pos >= 0)
        {
            *pos_p = pos;
            return LV_FS_RES_OK;
        }
        ESP_LOGE(TAG_TFT, "LVGL FS tell failed");
        return LV_FS_RES_UNKNOWN;
    };
    lv_fs_drv_register(&fs_drv);
    ESP_LOGI(TAG_TFT, "LVGL FS driver registered for SPIFFS");
}

// =====================
// LVGL tick callback
// =====================
void tft_lvgl_tick_inc(uint32_t tick_period_ms)
{
    lv_tick_inc(tick_period_ms);
}

// =====================
// Image display duration setter
// =====================
void tft_set_image_display_seconds(int seconds)
{
    if (seconds >= 1 && seconds <= 3600)
    {
        g_image_display_sec = seconds;
        ESP_LOGI(TAG_TFT, "Image display duration set to %d seconds", g_image_display_sec);
    }
}

// =====================
// Helper functions
// =====================
static std::vector<std::string> get_image_list()
{
    std::vector<std::string> files;
    ESP_LOGI(TAG_TFT, "Scanning directory: %s", IMG_DIR);
    DIR *d = opendir(IMG_DIR);
    if (!d)
    {
        ESP_LOGE(TAG_TFT, "Failed to open directory: %s", IMG_DIR);
        return files;
    }
    struct dirent *de;
    while ((de = readdir(d)) != nullptr)
    {
        if (de->d_name[0] == '.')
            continue;
        ESP_LOGI(TAG_TFT, "Found file: %s", de->d_name);
        files.push_back(std::string(de->d_name));
    }
    closedir(d);
    ESP_LOGI(TAG_TFT, "Total files found: %d", files.size());
    return files;
}

// =====================
// Demo UI Functions and Event Callbacks
// =====================

// Event callback for the button press
static void btn_event_handler(lv_event_t *e)
{
    if (lv_event_get_code(e) == LV_EVENT_CLICKED)
    {
        lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e); // Cast applied to fix lint error
        // Change the button color to green on click
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00FF00), LV_PART_MAIN);
    }
}

// Event callback for slider value change
static void slider_event_handler(lv_event_t *e)
{
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e); // Cast applied to fix lint error
    int32_t value = lv_slider_get_value(slider);
    lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
    char buf[8];
    sprintf(buf, "%ld", value);
    lv_label_set_text(label, buf);
}

// Function to create a demo UI with a title, a button, and a slider
static void create_demo_ui()
{
    lv_obj_t *scr = lv_scr_act();

    // Create title label
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "LVGL UI Demo");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Create a button with event callback
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, -80, 0);
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Press Me");
    lv_obj_center(btn_label);
    lv_obj_add_event_cb(btn, btn_event_handler, LV_EVENT_ALL, NULL);

    // Create a slider
    lv_obj_t *slider = lv_slider_create(scr);
    lv_obj_set_width(slider, 200);
    lv_obj_align(slider, LV_ALIGN_CENTER, 80, 0);
    lv_slider_set_range(slider, 0, 100);

    // Create label to display slider value
    lv_obj_t *slider_label = lv_label_create(scr);
    lv_label_set_text(slider_label, "0");
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    // Attach slider event with slider_label as user data
    lv_obj_add_event_cb(slider, slider_event_handler, LV_EVENT_VALUE_CHANGED, slider_label);
}

// =====================
// Main LVGL task implementation
// =====================
void tft_lvgl_run_task()
{
    // Add this task to the Task Watchdog Timer to allow resets
    esp_task_wdt_add(NULL);

    // We'll optionally create either a captive-portal label or the color test pattern container
    lv_obj_t *label = nullptr; // used for portal and IP messages
    lv_obj_t *color_pattern = nullptr;

    // State tracking for interruptions
    bool portal_shown = false;
    bool portal_message_done = false;
    int64_t portal_start_time = 0;
    bool ip_shown = false;
    int64_t ip_start_time = 0;

    auto create_color_test_pattern = []() -> lv_obj_t * {
        lv_obj_t *parent = lv_obj_create(lv_screen_active());
        lv_obj_set_size(parent, LCD_H_RES, LCD_V_RES);
        lv_obj_set_pos(parent, 0, 0);
        lv_obj_set_style_bg_color(parent, lv_color_black(), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(parent, 0, LV_PART_MAIN);

        // Title
        lv_obj_t *title = lv_label_create(parent);
        lv_label_set_text(title, "COLOR TEST");
        lv_obj_set_style_text_color(title, lv_color_white(), LV_PART_MAIN);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

        // We'll draw a grid of hue variants + grayscale bar + primaries
        const int grid_cols = 12; // 12 hues across (30-degree steps)
        const int grid_rows = 4;  // different brightness levels
        const int margin_top = 20;
        const int cell_w = LCD_H_RES / grid_cols;
        const int cell_h = (LCD_V_RES - margin_top - 28) / (grid_rows + 1); // +1 row for grayscale

        auto hsv_to_rgb565 = [](float h, float s, float v) -> lv_color_t {
            float c = v * s;
            float hh = fmodf(h / 60.0f, 6.0f);
            float x = c * (1 - fabsf(fmodf(hh, 2.0f) - 1));
            float r = 0, g = 0, b = 0;
            if (0 <= hh && hh < 1) { r = c; g = x; }
            else if (1 <= hh && hh < 2) { r = x; g = c; }
            else if (2 <= hh && hh < 3) { g = c; b = x; }
            else if (3 <= hh && hh < 4) { g = x; b = c; }
            else if (4 <= hh && hh < 5) { r = x; b = c; }
            else { r = c; b = x; }
            float m = v - c;
            r += m; g += m; b += m;
            uint8_t R = (uint8_t)(r * 255.0f);
            uint8_t G = (uint8_t)(g * 255.0f);
            uint8_t B = (uint8_t)(b * 255.0f);
            return lv_color_make(R, G, B);
        };

        // Hue grid
        for (int row = 0; row < grid_rows; ++row) {
            float v = 1.0f - row * 0.18f; // descending brightness
            float s = 1.0f - row * 0.05f; // slightly desaturated lower rows
            for (int col = 0; col < grid_cols; ++col) {
                float h = (360.0f / grid_cols) * col;
                lv_color_t c = hsv_to_rgb565(h, s, v);
                lv_obj_t *box = lv_obj_create(parent);
                lv_obj_remove_style_all(box);
                lv_obj_set_size(box, cell_w, cell_h);
                lv_obj_set_style_bg_color(box, c, LV_PART_MAIN);
                lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
                lv_obj_set_pos(box, col * cell_w, margin_top + row * cell_h);
            }
        }

        // Grayscale row
        for (int col = 0; col < grid_cols; ++col) {
            float g = (float)col / (grid_cols - 1);
            uint8_t v8 = (uint8_t)(g * 255.0f);
            lv_color_t c = lv_color_make(v8, v8, v8);
            lv_obj_t *box = lv_obj_create(parent);
            lv_obj_remove_style_all(box);
            lv_obj_set_size(box, cell_w, cell_h);
            lv_obj_set_style_bg_color(box, c, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_pos(box, col * cell_w, margin_top + grid_rows * cell_h);
        }

        // Primary / secondary bar (bottom)
        const lv_color_t primaries[] = {
            lv_palette_main(LV_PALETTE_RED),
            lv_palette_main(LV_PALETTE_ORANGE),
            lv_palette_main(LV_PALETTE_YELLOW),
            lv_palette_main(LV_PALETTE_GREEN),
            lv_palette_main(LV_PALETTE_BLUE),
            lv_palette_main(LV_PALETTE_PURPLE)
        };
        int pcount = sizeof(primaries)/sizeof(primaries[0]);
        int p_w = LCD_H_RES / pcount;
        for (int i = 0; i < pcount; ++i) {
            lv_obj_t *box = lv_obj_create(parent);
            lv_obj_remove_style_all(box);
            lv_obj_set_size(box, p_w, cell_h - 2);
            lv_obj_set_style_bg_color(box, primaries[i], LV_PART_MAIN);
            lv_obj_set_style_bg_opa(box, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_set_pos(box, i * p_w, margin_top + (grid_rows + 1) * cell_h + 2);
        }

        return parent;
    };

    // Handle LVGL tasks in a loop
    ESP_LOGI(TAG_TFT, "Entering LVGL loop (color test phase)");
    int64_t start_time = esp_timer_get_time();
    bool showing_pattern = false; // whether color test currently displayed
    
    while (true)
    {
        // Check if SoftAP (captive portal) is active
        bool softap_active = (WiFi.getMode() & WIFI_AP) != 0;
        bool wifi_connected = WiFi.status() == WL_CONNECTED;
        
        if (softap_active && !wifi_connected) {
            // If captive portal active, show message (replace pattern if needed)
            if (color_pattern) { lv_obj_del(color_pattern); color_pattern = nullptr; showing_pattern = false; }
            if (!label) { label = lv_label_create(lv_screen_active()); lv_obj_center(label); }
            lv_label_set_text(label, "Connect to\nKeyChain-Setup\nWiFi");
            start_time = esp_timer_get_time();
        } else {
            // Not captive portal: show color test pattern for first 5 seconds
            if (!showing_pattern) {
                if (label) { lv_obj_del(label); label = nullptr; }
                color_pattern = create_color_test_pattern();
                showing_pattern = true;
                start_time = esp_timer_get_time();
            }
        }

        lv_timer_handler();
        esp_task_wdt_reset();          // Reset watchdog to prevent timeout
        vTaskDelay(pdMS_TO_TICKS(10)); // Increased delay to 10ms for better responsiveness

        // Check if 5 seconds have passed and we're not in SoftAP mode
        if (showing_pattern && !softap_active && esp_timer_get_time() - start_time >= 1 * 1000000LL) {
            ESP_LOGI(TAG_TFT, "Color test complete, starting slideshow");
            if (color_pattern) { lv_obj_del(color_pattern); color_pattern = nullptr; }
            break;
        }
    }

    // Start slideshow of images
    while (true)
    {
        // Check for captive portal interruption
        bool softap_active = (WiFi.getMode() & WIFI_AP) != 0;
        bool wifi_connected = WiFi.status() == WL_CONNECTED;

        if ((!softap_active || wifi_connected) && portal_shown) {
            if (label) { lv_obj_del(label); label = nullptr; }
            portal_shown = false;
        }
        if (!softap_active) {
            portal_message_done = false;
        }

        if (softap_active && !wifi_connected && !portal_message_done) {
            if (!portal_shown) {
                // Show portal message
                if (label) lv_obj_del(label);
                label = lv_label_create(lv_screen_active());
                lv_obj_center(label);
                lv_label_set_text(label, "Connect to\nKeyChain-Setup\nWiFi");
                portal_shown = true;
                portal_start_time = esp_timer_get_time();
            }
            if (esp_timer_get_time() - portal_start_time >= 10 * 1000000LL) {
                // Hide after 10 seconds and resume slideshow
                if (label) { lv_obj_del(label); label = nullptr; }
                portal_shown = false;
                portal_message_done = true;
            } else {
                // Keep showing portal message
                lv_timer_handler();
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
        }

        // Check for WiFi connection interruption
        if (wifi_connected && !ip_shown) {
            // Show IP message
            if (label) lv_obj_del(label);
            label = lv_label_create(lv_screen_active());
            lv_obj_center(label);
            lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(label, LCD_H_RES - 20);
            char buf[128];
            sprintf(buf, "Upload Images at:\n\nhttp://%s:8080", WiFi.localIP().toString().c_str());
            lv_label_set_text(label, buf);
            ip_shown = true;
            ip_start_time = esp_timer_get_time();
            // Wait 5 seconds
            while (esp_timer_get_time() - ip_start_time < 5 * 1000000LL) {
                lv_timer_handler();
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Hide
            if (label) { lv_obj_del(label); label = nullptr; }
            // Do not reset ip_shown to false - show only once per boot
        }

        std::vector<std::string> images = get_image_list();
        ESP_LOGI(TAG_TFT, "Found %d images in slideshow", images.size());

        // Shuffle the images for random order
        if (!images.empty())
        {
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(images.begin(), images.end(), g);
            ESP_LOGI(TAG_TFT, "Images shuffled for random order");
        }

        if (images.empty())
        {
            ESP_LOGI(TAG_TFT, "No images found, showing black screen");
            // No images, show black screen
            lv_obj_t *black_rect = lv_obj_create(lv_screen_active());
            lv_obj_set_size(black_rect, LCD_H_RES, LCD_V_RES);
            lv_obj_set_pos(black_rect, 0, 0);
            lv_obj_set_style_bg_color(black_rect, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_style_border_width(black_rect, 0, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(black_rect, LV_OPA_COVER, LV_PART_MAIN);
            lv_refr_now(NULL);

            // Display black screen for configured seconds, resetting WDT periodically
            int64_t img_start = esp_timer_get_time();
            while (esp_timer_get_time() - img_start < (int64_t)g_image_display_sec * 1000000LL)
            {
                lv_timer_handler();
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100)); // Short delay to allow WDT reset
            }

            lv_obj_del(black_rect);
        }
        else
        {
            ESP_LOGI(TAG_TFT, "Starting slideshow with images");
            for (const auto &img_name : images)
            {
                //ESP_LOGI(TAG_TFT, "Displaying image: %s", img_name.c_str());
                // Clear screen with black before displaying image
                lv_obj_t *bg = lv_obj_create(lv_screen_active());
                lv_obj_set_size(bg, LCD_H_RES, LCD_V_RES);
                lv_obj_set_pos(bg, 0, 0);
                lv_obj_set_style_bg_color(bg, lv_color_black(), LV_PART_MAIN);
                lv_obj_set_style_border_width(bg, 0, LV_PART_MAIN);
                lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, LV_PART_MAIN);

                std::string img_path = std::string("A:") + IMG_DIR + "/" + img_name;
                //ESP_LOGI(TAG_TFT, "Image path: %s", img_path.c_str());

                // Check file size first
                struct stat st;
                if (stat((IMG_DIR + std::string("/") + img_name).c_str(), &st) == 0)
                {
                    //ESP_LOGI(TAG_TFT, "File size: %ld bytes", st.st_size);
                    if (st.st_size > 100000)
                    { // 100KB limit
                        ESP_LOGW(TAG_TFT, "File too large, skipping");
                        continue;
                    }
                }
                else
                {
                    ESP_LOGW(TAG_TFT, "Could not stat file");
                }

                // Quick JPEG header validation
                FILE *test_f = fopen((IMG_DIR + std::string("/") + img_name).c_str(), "rb");
                if (test_f)
                {
                    uint8_t header[4];
                    size_t read_bytes = fread(header, 1, 4, test_f);
                    fclose(test_f);
                    if (read_bytes == 4 && header[0] == 0xFF && header[1] == 0xD8 && header[2] == 0xFF)
                    {
                        //ESP_LOGI(TAG_TFT, "Valid JPEG header detected");
                    }
                    else
                    {
                        ESP_LOGW(TAG_TFT, "Invalid JPEG header, skipping file");
                        continue;
                    }
                }
                else
                {
                    ESP_LOGW(TAG_TFT, "Could not open file for validation");
                    continue;
                }

                // Reset WDT just before heavy decode
                esp_task_wdt_reset();

                lv_obj_t *img = lv_image_create(lv_screen_active());
                lv_image_set_src(img, img_path.c_str());
                lv_obj_center(img);
                // Apply warm tone via recolor (subtle orange overlay) if enabled
                if (g_warm_images) {
                    // Choose a gentle warm color; adjust mix strength via recolor_opa
                    lv_color_t warm = lv_color_make(255, 180, 90); // soft warm light
                    lv_obj_set_style_image_recolor(img, warm, LV_PART_MAIN);
                    lv_obj_set_style_image_recolor_opa(img, LV_OPA_30, LV_PART_MAIN); // ~30% influence
                    // Optional slight brightness lift
                    // (If LVGL is built with image opa support; otherwise this is ignored.)
                    // Future enhancement: custom decode transform for per-pixel gamma shift.
                }
                lv_refr_now(NULL);
                //ESP_LOGI(TAG_TFT, "Image displayed, waiting %d seconds", g_image_display_sec);

                // Display for IMAGE_DISPLAY_TIME_SEC seconds
                int64_t img_start = esp_timer_get_time();
                while (esp_timer_get_time() - img_start < (int64_t)g_image_display_sec * 1000000LL)
                {
                    lv_timer_handler();
                    esp_task_wdt_reset();
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                lv_obj_del(img);
                lv_obj_del(bg);
            }
        }
        // After all images, wait a bit before restarting
        ESP_LOGI(TAG_TFT, "Slideshow cycle complete, restarting in 1 second");
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second pause
    }
}

static void tft_clear(uint16_t color)
{
    uint8_t cmd;
    // set window = full screen with offsets applied
    int x1 = 0 + TFT_OFFSET_X, x2 = LCD_H_RES - 1 + TFT_OFFSET_X;
    int y1 = 0 + TFT_OFFSET_Y, y2 = LCD_V_RES - 1 + TFT_OFFSET_Y;

    cmd = 0x2A;
    uint8_t col_param[4] = {(uint8_t)(x1 >> 8), (uint8_t)x1, (uint8_t)(x2 >> 8), (uint8_t)x2};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, col_param, sizeof(col_param)));
    cmd = 0x2B;
    uint8_t row_param[4] = {(uint8_t)(y1 >> 8), (uint8_t)y1, (uint8_t)(y2 >> 8), (uint8_t)y2};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, row_param, sizeof(row_param)));
    cmd = 0x2C;

    const int pixels = LCD_H_RES * LCD_V_RES;
    static uint16_t *line = nullptr;
    if (!line)
        line = (uint16_t *)heap_caps_malloc(LCD_H_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    for (int i = 0; i < LCD_H_RES; i++)
        line[i] = color;
    for (int y = 0; y < LCD_V_RES; y++)
    {
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(s_panel_io, cmd, line, LCD_H_RES * 2));
    }
}
