#include <esp_system.h>
#include <esp_sleep.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <unistd.h>
#include <vector>

// Minimal LVGL (v9) + esp_lcd SPI setup for an ST7789 TFT on ESP32-C3
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_types.h>

#include <esp_task_wdt.h>

extern "C"
{
#include "lvgl.h"
}

// Networking & FS
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <esp_spiffs.h>
#include <esp_mac.h>

// Pin mapping (per user)
#define TFT_MOSI GPIO_NUM_4
#define TFT_SCLK GPIO_NUM_3
#define TFT_CS GPIO_NUM_2
#define TFT_DC GPIO_NUM_0
#define TFT_RST GPIO_NUM_5
#define TFT_MISO GPIO_NUM_NC

// Display resolution: adjust if your panel differs
#ifndef LCD_H_RES
#define LCD_H_RES 128
#endif
#ifndef LCD_V_RES
#define LCD_V_RES 128
#endif

#define LCD_HOST SPI2_HOST
#define LCD_SPI_CLOCK_HZ (40 * 1000 * 1000) // 40MHz, lower if unstable
#define LVGL_TICK_PERIOD_MS 2

// Configurable image display time (seconds), load from NVS at boot
static int g_image_display_sec = 10; // default

static const char *TAG = "app";

// Globals needed for flush sync
static esp_lcd_panel_io_handle_t s_panel_io = nullptr;
static SemaphoreHandle_t s_flush_sem = nullptr;

// ST7735 panel specifics (GREENTAB3)
#define ST7735_OFFSET_X 2
#define ST7735_OFFSET_Y 3

// Forward decls (LVGL v9 types)
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
static void lv_tick_cb(void *arg);
static void create_demo_ui();
static void wifi_start_station();
static bool wifi_connect_or_ap();

// Web server forward decls
static esp_err_t http_start();
static void wifi_start_softap();
static esp_err_t mount_spiffs();
static void lvgl_task(void *arg);
static esp_err_t handle_upload_fn(httpd_req_t *req);
static esp_err_t handle_settings_get(httpd_req_t *req);
static esp_err_t handle_settings_put(httpd_req_t *req);
static esp_err_t handle_scan(httpd_req_t *req);
static esp_err_t handle_wifi_save(httpd_req_t *req);

// Helper to get list of image filenames
static std::vector<std::string> get_image_list();

static const char *FS_BASE = "/spiffs";
static const char *IMG_DIR = "/spiffs/img";

// Wi-Fi provisioning state
static EventGroupHandle_t s_wifi_event_group = nullptr;
#define WIFI_CONNECTED_BIT BIT0
static bool s_is_softap = false; // true when running captive portal

// Simple NVS-backed store for multiple Wi-Fi credentials
#define WIFI_MAX_SAVED 8
struct WifiCred
{
    char ssid[32];
    char password[64];
};
struct WifiStore
{
    uint32_t magic;
    uint32_t count;
    WifiCred list[WIFI_MAX_SAVED];
};
static const uint32_t WIFI_STORE_MAGIC = 0x57494649; // 'WIFI'

static void wifi_store_load(WifiStore &out)
{
    memset(&out, 0, sizeof(out));
    nvs_handle_t nvh;
    if (nvs_open("cfg", NVS_READONLY, &nvh) == ESP_OK)
    {
        size_t sz = sizeof(out);
        if (nvs_get_blob(nvh, "wifi", &out, &sz) != ESP_OK || out.magic != WIFI_STORE_MAGIC)
        {
            memset(&out, 0, sizeof(out));
        }
        nvs_close(nvh);
    }
}

static bool wifi_store_save(const WifiStore &s)
{
    nvs_handle_t nvh;
    if (nvs_open("cfg", NVS_READWRITE, &nvh) != ESP_OK)
        return false;
    esp_err_t r = nvs_set_blob(nvh, "wifi", &s, sizeof(s));
    if (r == ESP_OK)
        r = nvs_commit(nvh);
    nvs_close(nvh);
    return r == ESP_OK;
}

static bool wifi_store_upsert(const char *ssid, const char *pass)
{
    if (!ssid || !*ssid)
        return false;
    WifiStore s;
    wifi_store_load(s);
    // find existing
    int idx = -1;
    for (uint32_t i = 0; i < s.count; ++i)
    {
        if (strncmp(s.list[i].ssid, ssid, sizeof(s.list[i].ssid)) == 0)
        {
            idx = (int)i;
            break;
        }
    }
    if (idx < 0)
    {
        if (s.count >= WIFI_MAX_SAVED)
            idx = (int)(s.count - 1); // overwrite last
        else
            idx = (int)(s.count++);
    }
    memset(&s.list[idx], 0, sizeof(WifiCred));
    strncpy(s.list[idx].ssid, ssid, sizeof(s.list[idx].ssid) - 1);
    if (pass)
        strncpy(s.list[idx].password, pass, sizeof(s.list[idx].password) - 1);
    s.magic = WIFI_STORE_MAGIC;
    return wifi_store_save(s);
}

static void st7735_hw_reset()
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

static void st7735_init()
{
    // Minimal init sequence for ST7735 (RGB565, normal display, inversion off)
    uint8_t cmd;
    // Sleep out
    cmd = 0x11; // EXIT_SLEEP_MODE
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));
    vTaskDelay(pdMS_TO_TICKS(120));

    // Color mode 16-bit
    uint8_t colmod[] = {0x05};
    cmd = 0x3A; // SET_PIXEL_FORMAT
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, colmod, sizeof(colmod)));

    // MADCTL: set BGR bit (0x08) to match most ST77xx panels' color filter
    // If colors appear swapped (bluish/reddish), toggle this bit.
    uint8_t madctl[] = {0x08};
    cmd = 0x36; // SET_ADDRESS_MODE
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, madctl, sizeof(madctl)));

    // Inversion off
    cmd = 0x20; // EXIT_INVERT_MODE
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));

    // Normal display on
    cmd = 0x13; // ENTER_NORMAL_MODE
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));

    // Display on
    cmd = 0x29; // SET_DISPLAY_ON
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, nullptr, 0));
}

extern "C" void app_main(void)
{
    // 0) Init NVS (for Wi‑Fi)
    ESP_ERROR_CHECK(nvs_flash_init());

    // Load settings from NVS (image display seconds)
    {
        nvs_handle_t nvh;
        esp_err_t r = nvs_open("cfg", NVS_READONLY, &nvh);
        if (r == ESP_OK)
        {
            uint32_t sec = 0;
            if (nvs_get_u32(nvh, "img_secs", &sec) == ESP_OK)
            {
                if (sec >= 1 && sec <= 3600)
                    g_image_display_sec = (int)sec;
            }
            nvs_close(nvh);
        }
        ESP_LOGI(TAG, "image display seconds: %d", g_image_display_sec);
    }

    // 1) Initialize LVGL
    lv_init();

    // 2) SPI bus for LCD
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = TFT_MOSI;
    buscfg.miso_io_num = -1; // not used
    buscfg.sclk_io_num = TFT_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = LCD_H_RES * 40 /*lines*/ * sizeof(lv_color_t) + 8;
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 3) Panel IO over SPI
    esp_lcd_panel_io_handle_t io_handle = nullptr;
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = TFT_DC;
    io_config.cs_gpio_num = TFT_CS;
    io_config.pclk_hz = LCD_SPI_CLOCK_HZ;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = on_color_trans_done;
    io_config.user_ctx = nullptr; // we use a global semaphore
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_HOST, &io_config, &io_handle));
    s_panel_io = io_handle;

    // 4) Reset/init the panel and set orientation as needed
    st7735_hw_reset();
    st7735_init();

    // 6) LVGL display + buffers (v9 API)
    lv_display_t *disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_default(disp);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

    const uint32_t lines = 40;                       // tune per RAM & speed
    const uint32_t buf_size = LCD_H_RES * lines * 2; // RGB565 = 2 bytes/px
    void *buf1 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    void *buf2 = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    assert(buf1 && buf2);
    lv_display_set_buffers(disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

    // 7) Set LVGL flush callback
    lv_display_set_flush_cb(disp, lvgl_flush_cb);

    // 8) LVGL tick via esp_timer
    esp_timer_handle_t tick_timer = nullptr;
    const esp_timer_create_args_t tick_timer_args = {
        .callback = &lv_tick_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lv_tick"};
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000)); // us

    // Create a semaphore to sync SPI color transfers
    s_flush_sem = xSemaphoreCreateBinary();

    // 9) Start LVGL processing task (larger stack for JPEG decode)
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 12288, nullptr, 5, nullptr, tskNO_AFFINITY);

    // 10) Mount SPIFFS (for index.html and images)
    ESP_ERROR_CHECK(mount_spiffs());
    // Ensure image directory exists
    struct stat st{};
    if (stat(IMG_DIR, &st) != 0)
    {
        int r = mkdir(IMG_DIR, 0777);
        if (r != 0)
            ESP_LOGW(TAG, "mkdir %s failed (maybe exists)", IMG_DIR);
    }

    // Register SPIFFS file system with LVGL after mounting
    static lv_fs_drv_t fs_drv;
    lv_fs_drv_init(&fs_drv);
    fs_drv.letter = 'A';
    fs_drv.open_cb = [](lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode) -> void *
    {
        ESP_LOGI(TAG, "LVGL FS open: %s", path);
        FILE *f = fopen(path, "rb");
        if (!f)
            ESP_LOGE(TAG, "Failed to open file: %s", path);
        return (void *)f;
    };
    fs_drv.close_cb = [](lv_fs_drv_t *drv, void *file_p) -> lv_fs_res_t
    {
        ESP_LOGD(TAG, "LVGL FS close");
        fclose((FILE *)file_p);
        return LV_FS_RES_OK;
    };
    fs_drv.read_cb = [](lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br) -> lv_fs_res_t
    {
        size_t n = fread(buf, 1, btr, (FILE *)file_p);
        *br = (uint32_t)n;
        ESP_LOGD(TAG, "LVGL FS read: requested %u, read %u", (unsigned)btr, (unsigned)*br);
        if (n < btr)
        {
            if (feof((FILE *)file_p))
                return LV_FS_RES_OK; // EOF is not an error
            if (ferror((FILE *)file_p))
            {
                ESP_LOGE(TAG, "LVGL FS read error");
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
        ESP_LOGE(TAG, "LVGL FS seek failed");
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
        ESP_LOGE(TAG, "LVGL FS tell failed");
        return LV_FS_RES_UNKNOWN;
    };
    lv_fs_drv_register(&fs_drv);
    ESP_LOGI(TAG, "LVGL FS driver registered for SPIFFS");

    // 11) Bring up networking and HTTP server
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_event_group = xEventGroupCreate();
    // Try to connect to saved SSIDs, otherwise start SoftAP portal
    bool sta_ok = wifi_connect_or_ap();
    // Start HTTP server (works in STA, AP, or APSTA)
    ESP_ERROR_CHECK(http_start());
    if (sta_ok)
    {
        ESP_LOGI(TAG, "Networking: connected to STA");
    }
    else
    {
        ESP_LOGI(TAG, "Networking: SoftAP portal active");
    }
}

// LVGL flush: push a rectangular area to the panel
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (!s_panel_io)
    {
        lv_display_flush_ready(disp);
        return;
    }
    const int x1 = area->x1 + ST7735_OFFSET_X;
    const int y1 = area->y1 + ST7735_OFFSET_Y;
    const int x2 = area->x2 + ST7735_OFFSET_X;
    const int y2 = area->y2 + ST7735_OFFSET_Y;

    // Set column address (0x2A)
    uint8_t cmd = 0x2A;
    uint8_t col_param[4] = {
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF),
        (uint8_t)(x2 >> 8), (uint8_t)(x2 & 0xFF)};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, col_param, sizeof(col_param)));

    // Set row address (0x2B)
    cmd = 0x2B;
    uint8_t row_param[4] = {
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF),
        (uint8_t)(y2 >> 8), (uint8_t)(y2 & 0xFF)};
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(s_panel_io, cmd, row_param, sizeof(row_param)));

    // Memory write (0x2C). Send pixels (RGB565) via DMA
    cmd = 0x2C;
    // If colors look swapped, enable byte swap here:
    lv_draw_sw_rgb565_swap(px_map, (x2 - x1 + 1) * (y2 - y1 + 1));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(s_panel_io, cmd, px_map,
                                              (x2 - x1 + 1) * (y2 - y1 + 1) * 2));

    // Wait for the SPI transfer to finish (signaled from ISR)
    if (s_flush_sem)
    {
        if (xSemaphoreTake(s_flush_sem, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
            ESP_LOGW(TAG, "flush wait timeout - SPI transfer may be stuck");
        }
        else
        {
            ESP_LOGD(TAG, "flush completed");
        }
    }
    lv_display_flush_ready(disp);
}

// Called when color transfer completes; notify LVGL the flush is done
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

static void lv_tick_cb(void * /*arg*/)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// =====================
// Helpers
// =====================
static std::vector<std::string> get_image_list()
{
    std::vector<std::string> files;
    ESP_LOGI(TAG, "Scanning directory: %s", IMG_DIR);
    DIR *d = opendir(IMG_DIR);
    if (!d)
    {
        ESP_LOGE(TAG, "Failed to open directory: %s", IMG_DIR);
        return files;
    }
    struct dirent *de;
    while ((de = readdir(d)) != nullptr)
    {
        if (de->d_name[0] == '.')
            continue;
        ESP_LOGI(TAG, "Found file: %s", de->d_name);
        files.push_back(std::string(de->d_name));
    }
    closedir(d);
    ESP_LOGI(TAG, "Total files found: %d", files.size());
    return files;
}

// Demo UI Functions and Event Callbacks

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
// LVGL task
// =====================
static void lvgl_task(void * /*arg*/)
{
    // Add this task to the Task Watchdog Timer to allow resets
    esp_task_wdt_add(NULL);

    // Create a minimal LVGL widget to show something
    lv_obj_t *label = lv_label_create(lv_screen_active());
    char buf[32];
    sprintf(buf, "Hello :)");
    lv_label_set_text(label, buf);
    lv_obj_center(label);

    // Handle LVGL tasks in a simple loop for 5 seconds, then start slideshow
    ESP_LOGI(TAG, "Entering LVGL loop for 5 seconds");
    int64_t start_time = esp_timer_get_time();
    while (true)
    {
        lv_timer_handler();
        esp_task_wdt_reset();          // Reset watchdog to prevent timeout
        vTaskDelay(pdMS_TO_TICKS(10)); // Increased delay to 10ms for better responsiveness

        // Check if 5 seconds have passed
        if (esp_timer_get_time() - start_time >= 5 * 1000000LL)
        { // 5 seconds in microseconds
            ESP_LOGI(TAG, "5 seconds elapsed");
            // Delete the label
            lv_obj_del(label);
            break;
        }
    }

    // Start slideshow of images
    while (true)
    {
        std::vector<std::string> images = get_image_list();
        ESP_LOGI(TAG, "Found %d images in slideshow", images.size());
        if (images.empty())
        {
            ESP_LOGI(TAG, "No images found, showing black screen");
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
            ESP_LOGI(TAG, "Starting slideshow with images");
            for (const auto &img_name : images)
            {
                ESP_LOGI(TAG, "Displaying image: %s", img_name.c_str());
                // Clear screen with black before displaying image
                lv_obj_t *bg = lv_obj_create(lv_screen_active());
                lv_obj_set_size(bg, LCD_H_RES, LCD_V_RES);
                lv_obj_set_pos(bg, 0, 0);
                lv_obj_set_style_bg_color(bg, lv_color_black(), LV_PART_MAIN);
                lv_obj_set_style_border_width(bg, 0, LV_PART_MAIN);
                lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, LV_PART_MAIN);

                std::string img_path = std::string("A:") + IMG_DIR + "/" + img_name;
                ESP_LOGI(TAG, "Image path: %s", img_path.c_str());

                // Check file size first
                struct stat st;
                if (stat((IMG_DIR + std::string("/") + img_name).c_str(), &st) == 0)
                {
                    ESP_LOGI(TAG, "File size: %ld bytes", st.st_size);
                    if (st.st_size > 100000)
                    { // 100KB limit
                        ESP_LOGW(TAG, "File too large, skipping");
                        continue;
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Could not stat file");
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
                        ESP_LOGI(TAG, "Valid JPEG header detected");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Invalid JPEG header, skipping file");
                        continue;
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Could not open file for validation");
                    continue;
                }

                // Reset WDT just before heavy decode
                esp_task_wdt_reset();

                lv_obj_t *img = lv_image_create(lv_screen_active());
                ESP_LOGI(TAG, "Created image object: %p", img);

                lv_image_set_src(img, img_path.c_str());
                ESP_LOGI(TAG, "Set image source");

                lv_obj_center(img);
                lv_refr_now(NULL);
                ESP_LOGI(TAG, "Image displayed, waiting %d seconds", g_image_display_sec);

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
                ESP_LOGI(TAG, "Image deleted, next");
            }
        }
        // After all images, wait a bit before restarting
        ESP_LOGI(TAG, "Slideshow cycle complete, restarting in 1 second");
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second pause
    }
}
// =====================
// SPIFFS
// =====================
static esp_err_t mount_spiffs()
{
    esp_vfs_spiffs_conf_t conf = {};
    conf.base_path = FS_BASE;
    conf.partition_label = "storage";
    conf.max_files = 8;
    conf.format_if_mount_failed = false; // Don't auto-format to preserve data
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s - data may be preserved but inaccessible", esp_err_to_name(ret));
        return ret;
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "SPIFFS size total=%u used=%u", (unsigned)total, (unsigned)used);
    }
    return ESP_OK;
}

// =====================
// Wi‑Fi Station (connect to existing network)
// =====================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Disconnected from Wi-Fi, retrying...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Webserver available at http://" IPSTR ":8080", IP2STR(&event->ip_info.ip));
        if (s_wifi_event_group)
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_start_station()
{
    // Intentionally left for compatibility; not used in new flow
    // Use wifi_connect_or_ap() instead
}

// Attempt to connect to saved SSIDs; return true if connected, else start SoftAP and return false
static bool wifi_connect_or_ap()
{
    // Create STA netif and init Wi-Fi (once)
    esp_netif_create_default_wifi_sta();
    static bool s_wifi_inited = false;
    if (!s_wifi_inited)
    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        // Register event handlers
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
        s_wifi_inited = true;
    }

    WifiStore store;
    wifi_store_load(store);
    if (store.count == 0)
    {
        ESP_LOGW(TAG, "No saved Wi-Fi credentials");
    }
    else
    {
        for (uint32_t i = 0; i < store.count; ++i)
        {
            const WifiCred &c = store.list[i];
            if (!c.ssid[0])
                continue;
            wifi_config_t sta_cfg = {};
            strncpy((char *)sta_cfg.sta.ssid, c.ssid, sizeof(sta_cfg.sta.ssid) - 1);
            strncpy((char *)sta_cfg.sta.password, c.password, sizeof(sta_cfg.sta.password) - 1);
            sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
            ESP_ERROR_CHECK(esp_wifi_start());
            ESP_LOGI(TAG, "Trying SSID: %s", (char *)sta_cfg.sta.ssid);

            if (s_wifi_event_group)
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            // Wait up to 20s for connection
            EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
            if (bits & WIFI_CONNECTED_BIT)
            {
                s_is_softap = false;
                return true; // connected
            }
            ESP_LOGW(TAG, "SSID '%s' timed out, trying next...", c.ssid);
            esp_wifi_stop();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    // None succeeded -> SoftAP portal
    wifi_start_softap();
    return false;
}

// =====================
// Helpers
// =====================
static bool safe_name(const char *name)
{
    if (!name || !*name)
        return false;
    // reject path traversal or separators
    for (const char *p = name; *p; ++p)
    {
        if (*p == '/' || *p == '\\')
            return false;
    }
    if (strstr(name, ".."))
        return false;
    if (strlen(name) > 64)
        return false;
    return true;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return 10 + c - 'A';
    if (c >= 'a' && c <= 'f')
        return 10 + c - 'a';
    return -1;
}
static void url_decode(char *s)
{
    char *o = s;
    char *p = s;
    while (*p)
    {
        if (*p == '%' && hexval(p[1]) >= 0 && hexval(p[2]) >= 0)
        {
            *o++ = (char)((hexval(p[1]) << 4) | hexval(p[2]));
            p += 3;
        }
        else if (*p == '+')
        {
            *o++ = ' ';
            p++;
        }
        else
        {
            *o++ = *p++;
        }
    }
    *o = 0;
}

static std::string path_join(const char *dir, const char *name)
{
    std::string s(dir);
    s += "/";
    s += name;
    return s;
}

// =====================
// HTTP Handlers
// =====================
extern const uint8_t index_html_start[] asm("_binary_embedded_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_embedded_index_html_end");

static const char PORTAL_HTML[] = R"(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Wi‑Fi Setup</title><style>body{font-family:system-ui,Segoe UI,Arial;margin:2rem;}button{padding:.5rem 1rem}input,select{padding:.4rem;margin:.25rem 0;width:100%;max-width:360px}#list li{margin:.25rem 0}</style></head><body>
<h2>Wi‑Fi setup</h2><p>This device is in setup mode. Select a network and enter its password, then Save. The device will reboot and try to connect automatically.</p>
<button id=scan>Scan networks</button><ul id=list></ul>
<label>SSID<select id=ssidSel></select></label>
<label>Password<input id=pw type=password></label>
<button id=save>Save & Reboot</button>
<script>const sel=document.getElementById('ssidSel');const list=document.getElementById('list');document.getElementById('scan').onclick=async()=>{list.textContent='Scanning...';try{const r=await fetch('/api/scan');const arr=await r.json();list.textContent='';sel.innerHTML='';arr.sort((a,b)=>b.rssi-a.rssi);arr.forEach(ap=>{const li=document.createElement('li');li.textContent=`${ap.ssid} (${ap.rssi})`;list.appendChild(li);const o=document.createElement('option');o.value=o.textContent=ap.ssid;sel.appendChild(o);});}catch(e){list.textContent='Scan failed';}};document.getElementById('save').onclick=async()=>{const ssid=sel.value||prompt('SSID');const password=document.getElementById('pw').value||'';if(!ssid){alert('SSID required');return;}try{await fetch('/api/wifi/save',{method:'PUT',headers:{'content-type':'application/json'},body:JSON.stringify({ssid,password})});alert('Saved. Device will reboot. Reconnect to your router.');}catch(e){alert('Save failed');}}</script>
</body></html>
)";
static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    if (s_is_softap)
    {
        return httpd_resp_send(req, PORTAL_HTML, HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        // Serve embedded index.html to ensure it updates with firmware flashes.
        const size_t len = index_html_end - index_html_start;
        return httpd_resp_send(req, (const char *)index_html_start, len);
    }
}

static esp_err_t handle_list(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    DIR *d = opendir(IMG_DIR);
    if (!d)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "dir open failed");
    httpd_resp_sendstr_chunk(req, "[");
    bool first = true;
    struct dirent *de;
    while ((de = readdir(d)) != nullptr)
    {
        if (de->d_name[0] == '.')
            continue;
        if (!first)
            httpd_resp_sendstr_chunk(req, ",");
        first = false;
        httpd_resp_sendstr_chunk(req, "\"");
        httpd_resp_sendstr_chunk(req, de->d_name);
        httpd_resp_sendstr_chunk(req, "\"");
    }
    closedir(d);
    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t handle_img_get(httpd_req_t *req)
{
    const char *uri = req->uri; // /img/<name>
    const char *name = uri + 5; // skip '/img/'
    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(IMG_DIR, name_dec);
    FILE *f = fopen(full.c_str(), "rb");
    if (!f)
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
    httpd_resp_set_type(req, "image/jpeg");
    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
    {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK)
        {
            fclose(f);
            return ESP_FAIL;
        }
    }
    fclose(f);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t handle_img_put(httpd_req_t *req)
{
    const char *uri = req->uri; // /img/<name>
    const char *name = uri + 5;
    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(IMG_DIR, name_dec);
    FILE *f = fopen(full.c_str(), "wb");
    if (!f)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed");
    char buf[1024];
    int remaining = req->content_len;
    while (remaining > 0)
    {
        int r = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? sizeof(buf) : remaining);
        if (r <= 0)
        {
            fclose(f);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv err");
        }
        fwrite(buf, 1, r, f);
        remaining -= r;
    }
    fclose(f);
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t handle_img_delete(httpd_req_t *req)
{
    const char *uri = req->uri;
    const char *name;

    // Handle both /img/* and /api/delete/* patterns
    if (strncmp(uri, "/api/delete/", 12) == 0)
    {
        name = uri + 12; // skip '/api/delete/'
    }
    else if (strncmp(uri, "/img/", 5) == 0)
    {
        name = uri + 5; // skip '/img/'
    }
    else
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid uri");
    }

    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(IMG_DIR, name_dec);
    int r = unlink(full.c_str());
    if (r != 0)
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t http_start()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;
    config.max_uri_handlers = 16;                   // Increase from default (usually 8) to handle all our endpoints
    config.uri_match_fn = httpd_uri_match_wildcard; // Restore wildcard matcher for wildcard URIs
    httpd_handle_t server = nullptr;
    ESP_ERROR_CHECK(httpd_start(&server, &config));
    httpd_uri_t uri_root = {};
    uri_root.uri = "/";
    uri_root.method = HTTP_GET;
    uri_root.handler = handle_root;
    uri_root.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_root));

    httpd_uri_t uri_idx = {};
    uri_idx.uri = "/index.html";
    uri_idx.method = HTTP_GET;
    uri_idx.handler = handle_root;
    uri_idx.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_idx));

    httpd_uri_t uri_list = {};
    uri_list.uri = "/api/list";
    uri_list.method = HTTP_GET;
    uri_list.handler = handle_list;
    uri_list.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_list));

    httpd_uri_t uri_img_get = {};
    uri_img_get.uri = "/img/*";
    uri_img_get.method = HTTP_GET;
    uri_img_get.handler = handle_img_get;
    uri_img_get.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_img_get));

    httpd_uri_t uri_img_put = {};
    uri_img_put.uri = "/img/*";
    uri_img_put.method = HTTP_PUT;
    uri_img_put.handler = handle_img_put;
    uri_img_put.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_img_put));

    httpd_uri_t uri_img_del = {};
    uri_img_del.uri = "/img/*";
    uri_img_del.method = HTTP_DELETE;
    uri_img_del.handler = handle_img_delete;
    uri_img_del.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_img_del));

    httpd_uri_t uri_api_del = {};
    uri_api_del.uri = "/api/delete/*";
    uri_api_del.method = HTTP_DELETE;
    uri_api_del.handler = handle_img_delete;
    uri_api_del.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_api_del));

    httpd_uri_t uri_upload = {};
    uri_upload.uri = "/api/upload/*";
    uri_upload.method = HTTP_PUT;
    uri_upload.handler = handle_upload_fn;
    uri_upload.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_upload));

    // Settings endpoints
    httpd_uri_t uri_settings_get = {};
    uri_settings_get.uri = "/api/settings";
    uri_settings_get.method = HTTP_GET;
    uri_settings_get.handler = handle_settings_get;
    uri_settings_get.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_settings_get));

    httpd_uri_t uri_settings_put = {};
    uri_settings_put.uri = "/api/settings";
    uri_settings_put.method = HTTP_PUT;
    uri_settings_put.handler = handle_settings_put;
    uri_settings_put.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_settings_put));

    // Wi‑Fi provisioning endpoints
    httpd_uri_t uri_scan = {};
    uri_scan.uri = "/api/scan";
    uri_scan.method = HTTP_GET;
    uri_scan.handler = handle_scan;
    uri_scan.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_scan));

    httpd_uri_t uri_wifi_save = {};
    uri_wifi_save.uri = "/api/wifi/save";
    uri_wifi_save.method = HTTP_PUT;
    uri_wifi_save.handler = handle_wifi_save;
    uri_wifi_save.user_ctx = nullptr;
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_wifi_save));

    ESP_LOGI(TAG, "HTTP server started on :%d", config.server_port);
    return ESP_OK;
}

// Standalone function to handle PUT uploads
static esp_err_t handle_upload_fn(httpd_req_t *req)
{
    // URI is /api/upload/<name>
    const char *uri = req->uri;
    const char *name = uri + 12; // skip '/api/upload/'
    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(IMG_DIR, name_dec);
    FILE *f = fopen(full.c_str(), "wb");
    if (!f)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "open failed");
    char buf[1024];
    int remaining = req->content_len;
    while (remaining > 0)
    {
        int r = httpd_req_recv(req, buf, remaining > (int)sizeof(buf) ? sizeof(buf) : remaining);
        if (r <= 0)
        {
            fclose(f);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv err");
        }
        fwrite(buf, 1, r, f);
        remaining -= r;
    }
    fclose(f);
    return httpd_resp_sendstr(req, "OK");
}

// =====================
// Settings (GET/PUT JSON)
// =====================
static esp_err_t handle_settings_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "{\"image_display_seconds\":%d}", g_image_display_sec);
    if (n < 0)
        return ESP_FAIL;
    return httpd_resp_send(req, buf, n);
}

static esp_err_t handle_settings_put(httpd_req_t *req)
{
    // Read small JSON body
    int len = req->content_len;
    if (len <= 0 || len > 256)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad len");
    char body[257];
    int got = 0;
    while (got < len)
    {
        int r = httpd_req_recv(req, body + got, len - got);
        if (r <= 0)
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv err");
        got += r;
    }
    body[got] = '\0';

    // Very simple parse: find first integer in payload
    const char *p = body;
    long val = -1;
    while (*p)
    {
        if ((*p >= '0' && *p <= '9') || (*p == '-' && p[1] >= '0' && p[1] <= '9'))
        {
            val = strtol(p, nullptr, 10);
            break;
        }
        ++p;
    }
    if (val < 1 || val > 3600)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "range 1..3600");

    g_image_display_sec = (int)val;

    // Persist to NVS
    nvs_handle_t nvh;
    if (nvs_open("cfg", NVS_READWRITE, &nvh) == ESP_OK)
    {
        uint32_t u = (uint32_t)g_image_display_sec;
        nvs_set_u32(nvh, "img_secs", u);
        nvs_commit(nvh);
        nvs_close(nvh);
    }

    return httpd_resp_sendstr(req, "OK");
}

// =====================
// Wi‑Fi Provisioning APIs
// =====================
static esp_err_t handle_scan(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    wifi_scan_config_t sc = {};
    // Active scan, short times for responsiveness
    sc.show_hidden = false;
    esp_err_t r = esp_wifi_scan_start(&sc, true);
    if (r != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "scan start failed");
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    wifi_ap_record_t *aps = (wifi_ap_record_t *)malloc(n * sizeof(wifi_ap_record_t));
    if (!aps && n)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    esp_wifi_scan_get_ap_records(&n, aps);
    httpd_resp_sendstr_chunk(req, "[");
    for (uint16_t i = 0; i < n; ++i)
    {
        if (i)
            httpd_resp_sendstr_chunk(req, ",");
        char line[160];
        snprintf(line, sizeof(line), "{\"ssid\":\"%s\",\"rssi\":%d}", (char *)aps[i].ssid, aps[i].rssi);
        httpd_resp_sendstr_chunk(req, line);
    }
    free(aps);
    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t handle_wifi_save(httpd_req_t *req)
{
    // Expect small JSON {"ssid":"...","password":"..."}
    int len = req->content_len;
    if (len <= 0 || len > 256)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad len");
    char body[257];
    int got = 0;
    while (got < len)
    {
        int r = httpd_req_recv(req, body + got, len - got);
        if (r <= 0)
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv err");
        got += r;
    }
    body[got] = '\0';
    auto find_val = [&](const char *key, char *out, size_t out_sz)
    {
        const char *p = strstr(body, key); if (!p) return false; p = strchr(p, ':'); if (!p) return false; p++; while (*p==' '||*p=='\t') p++; if (*p!='\"') return false; p++; size_t i=0; while (*p && *p!='\"' && i+1<out_sz) { out[i++]=*p++; } out[i]='\0'; return true; };
    char ssid[33] = {0}, pass[65] = {0};
    if (!find_val("\"ssid\"", ssid, sizeof(ssid)))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid missing");
    find_val("\"password\"", pass, sizeof(pass));
    if (!wifi_store_upsert(ssid, pass))
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "save failed");
    httpd_resp_sendstr(req, "OK");
    // Give the response time to flush then reboot
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// =====================
// SoftAP start
// =====================
static void wifi_start_softap()
{
    ESP_LOGW(TAG, "Starting SoftAP portal...");
    // Ensure AP netif exists
    esp_netif_create_default_wifi_ap();
    // Ensure Wi-Fi is initialized (done earlier) and stopped
    esp_wifi_stop();

    wifi_config_t ap_cfg = {};
    strcpy((char *)ap_cfg.ap.ssid, "ESP32-Setup");
    ap_cfg.ap.ssid_len = strlen("ESP32-Setup");
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN; // Open portal; change if needed
    ap_cfg.ap.max_connection = 4;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    s_is_softap = true;
}
