#include <esp_system.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <unistd.h>

// Minimal LVGL (v9) + esp_lcd SPI setup for an ST7789 TFT on ESP32-C3
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>

#include <esp_task_wdt.h>

// Networking & FS
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_spiffs.h>
#include <esp_mac.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

// TFT configuration (pins, driver, SPI clocks, offsets) per-board
#include "tft.h"

// Webserver
#include "webserver.h"

#if defined(DISPLAY_DRIVER_ST7735)
#define BUTTON_PIN GPIO_NUM_10
#elif defined(DISPLAY_DRIVER_GC9D01)
#define BUTTON_PIN GPIO_NUM_7
#endif

#define LCD_HOST SPI2_HOST
#ifndef LCD_SPI_CLOCK_HZ
#define LCD_SPI_CLOCK_HZ (40 * 1000 * 1000) // fallback, usually set by tft.h
#endif
#define LVGL_TICK_PERIOD_MS 2

// Configurable image display time (seconds), load from NVS at boot
static int g_image_display_sec = 10; // default

static const char *TAG = "app";

// Button interrupt handling
static QueueHandle_t button_queue = NULL;
static const int64_t DEBOUNCE_MS = 50; // Debounce time
static volatile int64_t last_interrupt_time = 0;

// Forward decls
static esp_err_t mount_spiffs();
static void simple_lvgl_task(void *arg);

// Button monitoring task
static void button_task(void *arg);

// ISR handler for button
static void IRAM_ATTR button_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time() / 1000;
    if (now - last_interrupt_time > DEBOUNCE_MS)
    {
        last_interrupt_time = now;
        int level = gpio_get_level(BUTTON_PIN);
        xQueueSendFromISR(button_queue, &level, NULL);
    }
}

static const char *FS_BASE = "/spiffs";
static const char *IMG_DIR = "/spiffs/img";

// Simple WiFi connect function
static void wifi_connect() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Disable Wi-Fi power save to improve connection stability
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, "[YOUR SSID]");  // Replace with your SSID
    strcpy((char*)wifi_config.sta.password, "[YOUR PASSWORD]");  // Replace with your password

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set maximum TX power after WiFi is started
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80)); // 80 = 20dBm max

    // Register event handler for connection
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, [](void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
        wifi_event_sta_disconnected_t* disc = (wifi_event_sta_disconnected_t*)event_data;
        ESP_LOGI(TAG, "WiFi disconnected, reason: %d, reconnecting...", disc->reason);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before reconnecting
        esp_wifi_connect();
    }, NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, [](void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }, NULL));

    ESP_ERROR_CHECK(esp_wifi_connect());
}

// Webserver callbacks
static int get_image_display_seconds_cb() { return g_image_display_sec; }
static void set_image_display_seconds_cb(int seconds)
{
    if (seconds < 1)
        seconds = 1;
    if (seconds > 3600)
        seconds = 3600;
    g_image_display_sec = seconds;
    // Persist to NVS
    nvs_handle_t nvh;
    if (nvs_open("cfg", NVS_READWRITE, &nvh) == ESP_OK)
    {
        uint32_t u = (uint32_t)g_image_display_sec;
        nvs_set_u32(nvh, "img_secs", u);
        nvs_commit(nvh);
        nvs_close(nvh);
    }
    // Inform TFT module
    tft_set_image_display_seconds(g_image_display_sec);
}

static bool is_softap_active_cb() { return false; }  // Not using SoftAP
static bool save_wifi_credentials_cb(const char *ssid, const char *password)
{
    // Dummy: since we're using hardcoded WiFi, just log
    ESP_LOGI(TAG, "Ignoring save WiFi credentials: %s", ssid);
    return true;
}

extern "C" void app_main(void)
{

    esp_log_level_set("tft", ESP_LOG_ERROR);
    // 0) Init NVS (for Wi‑Fi)
    ESP_ERROR_CHECK(nvs_flash_init());

    // TFT: initialize everything (SPI bus, panel IO, panel, LVGL display/flush)
    tft_init_all();

    // Configure button GPIO with interrupts
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on both edges
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // Create queue for ISR communication
    button_queue = xQueueCreate(10, sizeof(int));

    // Install ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

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

    // LVGL is initialized inside tft_init_all()

    // 2) LVGL tick via esp_timer
    esp_timer_handle_t tick_timer = nullptr;
    const esp_timer_create_args_t tick_timer_args = {
        .callback = [](void *)
        { tft_lvgl_tick_inc(LVGL_TICK_PERIOD_MS); },
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lv_tick",
        .skip_unhandled_events = false};
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000)); // us

    // 9) Start LVGL processing task (larger stack for JPEG decode)
    xTaskCreatePinnedToCore(simple_lvgl_task, "lvgl", 24576, nullptr, 5, nullptr, tskNO_AFFINITY);

    // 10) Start button monitoring task
    xTaskCreatePinnedToCore(button_task, "button", 2048, nullptr, 4, nullptr, tskNO_AFFINITY);

    // 11) Mount SPIFFS (for index.html and images)
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
    tft_init_lvgl_filesystem();

    // Load and set image display duration from NVS
    tft_set_image_display_seconds(g_image_display_sec);

    // Networking and webserver will be started on button press
    ESP_LOGI(TAG, "Ready, press button to start WiFi and webserver");
}

// =====================
// Simple LVGL task (now just calls TFT module)
// =====================
static void simple_lvgl_task(void * /*arg*/)
{
    tft_lvgl_run_task(); // Delegate to TFT module
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
// Button monitoring task
// =====================
static void button_task(void *arg)
{
    static bool wifi_started = false;
    int level;

    while (true)
    {
        // Wait for button interrupt
        if (xQueueReceive(button_queue, &level, portMAX_DELAY))
        {
            if (level == 0)
            { // Button pressed (low)
                if (!wifi_started)
                {
                    wifi_started = true;
                    ESP_LOGI(TAG, "Starting WiFi...");
                    wifi_connect();
                    ESP_LOGI(TAG, "Starting webserver...");
                    WebServerConfig wcfg{};
                    wcfg.img_dir = IMG_DIR;
                    wcfg.is_softap_active = &is_softap_active_cb;
                    wcfg.save_wifi_credentials = &save_wifi_credentials_cb;
                    wcfg.get_image_display_seconds = &get_image_display_seconds_cb;
                    wcfg.set_image_display_seconds = &set_image_display_seconds_cb;
                    ESP_ERROR_CHECK(webserver_start(&wcfg));
                    ESP_LOGI(TAG, "Webserver started");
                }
            }
        }
    }
}
