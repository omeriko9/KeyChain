#include "Arduino.h"
#include <WiFi.h>
#include <wifimanager/WiFiManager.h>
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
#include <lwip/arch.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

// TFT configuration (pins, driver, SPI clocks, offsets) per-board
#include "tft.h"

// Webserver
#include "webserver.h"

// #undef INADDR_NONE

#include <esp_diagnostics.h>

extern "C" void __wrap_esp_log_writev(esp_log_level_t level, const char *tag, const char *format, va_list args)
{
    esp_diag_log_write(level, tag, format, args);
    esp_log_writev(level, tag, format, args);
}

#if defined(DISPLAY_DRIVER_ST7735)
#define BUTTON_PIN GPIO_NUM_10
#elif defined(DISPLAY_DRIVER_GC9D01)
#define BUTTON_PIN GPIO_NUM_9
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
static void start_wifi(bool auto_start);


// Button monitoring task
static void button_task(void *arg);

// GPIO logging task - for finding out your button's GPIO
static void gpio_logging_task(void *arg);

// ISR handler for button
static void IRAM_ATTR button_isr_handler(void *arg)
{
    ESP_LOGI(TAG, "ISR triggered");
    // Removed debounce for testing
    int level = gpio_get_level(BUTTON_PIN);
    ESP_LOGI(TAG, "ISR: sending level %d to queue", level);
    xQueueSendFromISR(button_queue, &level, NULL);
}

static const char *FS_BASE = "/spiffs";
static const char *IMG_DIR = "/spiffs/img";

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

static bool is_softap_active_cb() { return false; } // Not using SoftAP
static bool save_wifi_credentials_cb(const char *ssid, const char *password)
{
    // Dummy: since we're using hardcoded WiFi, just log
    ESP_LOGI(TAG, "Ignoring save WiFi credentials: %s", ssid);
    return true;
}

extern "C" void app_main(void)
{

    initArduino();

    esp_log_level_set("tft", ESP_LOG_ERROR);
    // 0) Init NVS (for Wi‑Fi)
    ESP_ERROR_CHECK(nvs_flash_init());

    // TFT: initialize everything (SPI bus, panel IO, panel, LVGL display/flush)
    tft_init_all();

    // For TFT071 (GC9D01), automatically start WiFi and webserver
    #if defined(DISPLAY_DRIVER_GC9D01)
    start_wifi(true);
    #endif

    // Reset and configure button GPIO with interrupts (after TFT init to override any conflicts)
    gpio_reset_pin(BUTTON_PIN);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on both edges
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Enable pull-down for button
    gpio_config(&io_conf);

    // Create queue for ISR communication
    button_queue = xQueueCreate(10, sizeof(int));

    // Install ISR handler
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL));

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

    // 11) Start GPIO logging task (optional)
    // xTaskCreatePinnedToCore(gpio_logging_task, "gpio_log", 2048, nullptr, 3, nullptr, tskNO_AFFINITY);

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

static void start_wifi(bool auto_start)
{
    xTaskCreate([](void *pvParameters)
                {
    bool auto_start = (int)pvParameters != 0;
    WiFi.mode(WIFI_STA);          // WiFiManager will switch to AP+STA when needed
    WiFiManager wm;

    // Optional: portal timeout so it doesn’t block forever
    wm.setConfigPortalTimeout(300); // seconds
    // Set connection timeout to prevent infinite auth loops
    wm.setConnectTimeout(15); // 15 seconds timeout for connection attempts
    
    // Reset WiFi settings if button is pressed during startup (for recovery)
    // This is a simple way to clear invalid credentials
    if (gpio_get_level(BUTTON_PIN) == 0) { // Button pressed (active low)
        ESP_LOGI(TAG, "Button pressed during startup - resetting WiFi settings");
        wm.resetSettings();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Give time for reset
    }

    if (wm.autoConnect("KeyChain-Setup")) {
      printf("[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
      printf("[WiFi] Config portal failed or timed out\n");
      // Force start config portal if autoConnect failed
      wm.startConfigPortal("KeyChain-Setup");
    }

    ESP_LOGI(TAG, "Starting webserver...");
    WebServerConfig wcfg{};
    wcfg.img_dir = IMG_DIR;              
    wcfg.get_image_display_seconds = &get_image_display_seconds_cb;
    wcfg.set_image_display_seconds = &set_image_display_seconds_cb;
    ESP_ERROR_CHECK(webserver_start(&wcfg));
    ESP_LOGI(TAG, "Webserver started");

    if (auto_start) {
        // Keep the task running for 5 minutes to maintain WiFi and webserver
        for (int i = 0; i < 300; ++i) {  // 300 seconds = 5 minutes
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGI(TAG, "WiFi task ending after 5 minutes");
        vTaskDelete(nullptr);
    } else {
        // Keep the task running indefinitely
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } }, "wifi_mgr", 8192, (void*)(auto_start ? 1 : 0), 5, nullptr);
}

// =====================
// Button monitoring task
// =====================
static void button_task(void *arg)
{
    ESP_LOGI(TAG, "Button task started");
    int level;

    while (true)
    {
        // Wait for button interrupt
        if (xQueueReceive(button_queue, &level, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Button Clicked! Level: %d", level);

            if (level == 1)
            { // Button pressed (high with pull-down)
                ESP_LOGI(TAG, "Starting WiFi...");
                start_wifi(false);
            }
            else
            {
                ESP_LOGI(TAG, "Button released (low)");
            }
        }
    }
}

// =====================
// GPIO logging task
// =====================
static void gpio_logging_task(void *arg)
{
    while (true)
    {
        // Log all GPIOs that are high
        std::string high_pins = "";
        for (int pin = 0; pin <= 21; ++pin)
        {
            if (gpio_get_level((gpio_num_t)pin) == 1)
            {
                if (!high_pins.empty())
                    high_pins += ", ";
                high_pins += std::to_string(pin);
            }
        }
        ESP_LOGI(TAG, "High GPIOs: %s", high_pins.c_str());
        // Log every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
