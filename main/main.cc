#include <esp_system.h>
#include <esp_sleep.h>
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
#include <freertos/event_groups.h>

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

#define BUTTON_PIN GPIO_NUM_10

#define LCD_HOST SPI2_HOST
#ifndef LCD_SPI_CLOCK_HZ
#define LCD_SPI_CLOCK_HZ (40 * 1000 * 1000) // fallback, usually set by tft.h
#endif
#define LVGL_TICK_PERIOD_MS 2

// Configurable image display time (seconds), load from NVS at boot
static int g_image_display_sec = 10; // default

static const char *TAG = "app";

// Forward decls
static void wifi_start_station();
static bool wifi_connect_or_ap();

// Web server forward decls
static void wifi_start_softap();
static esp_err_t mount_spiffs();
static void simple_lvgl_task(void *arg);

// Button monitoring task
static void button_task(void *arg);

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

// =============== Webserver wiring ===============
#include "webserver.h"

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

static bool is_softap_active_cb() { return s_is_softap; }
static bool save_wifi_credentials_cb(const char *ssid, const char *password)
{
    return wifi_store_upsert(ssid, password);
}

extern "C" void app_main(void)
{
    // 0) Init NVS (for Wi‑Fi)
    ESP_ERROR_CHECK(nvs_flash_init());

    // TFT: initialize everything (SPI bus, panel IO, panel, LVGL display/flush)
    tft_init_all();

    // Check wake-up cause
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_GPIO)
    {
        ESP_LOGI(TAG, "Woken up by GPIO button press");
    }

    // Configure button GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // No interrupt, using polling
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // No ISR, using task

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

    // 11) Bring up networking and HTTP server
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_event_group = xEventGroupCreate();
    // Try to connect to saved SSIDs, otherwise start SoftAP portal
    bool sta_ok = wifi_connect_or_ap();
    // Start HTTP server (works in STA, AP, or APSTA)
    WebServerConfig wcfg{};
    wcfg.img_dir = IMG_DIR;
    wcfg.is_softap_active = &is_softap_active_cb;
    wcfg.save_wifi_credentials = &save_wifi_credentials_cb;
    wcfg.get_image_display_seconds = &get_image_display_seconds_cb;
    wcfg.set_image_display_seconds = &set_image_display_seconds_cb;
    ESP_ERROR_CHECK(webserver_start(&wcfg));
    if (sta_ok)
    {
        ESP_LOGI(TAG, "Networking: connected to STA");
    }
    else
    {
        ESP_LOGI(TAG, "Networking: SoftAP portal active");
    }
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
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG, "Disconnected from Wi-Fi, reason=%d, retrying...", disc ? disc->reason : -1);
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

static void wifi_start_station() { /* kept for compatibility */ }

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
        // Disable Wi-Fi power save to avoid timing sensitivity during auth/handshake
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
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
            // Prefer WPA2; allow WPA3 on AP if present but don't require PMF
            sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
            sta_cfg.sta.pmf_cfg.capable = true;
            sta_cfg.sta.pmf_cfg.required = false;
            sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
            sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
            sta_cfg.sta.bssid_set = 0; // don't pin unless you want to lock AP
            sta_cfg.sta.channel = 0;   // scan all
            sta_cfg.sta.listen_interval = 3;

            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
            // Explicit protocols (2.4 GHz)
            ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
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

static void wifi_start_softap()
{
    ESP_LOGW(TAG, "Starting SoftAP portal...");
    // Ensure AP netif exists
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Configure DHCP server to provide our IP as DNS server
    esp_netif_dhcp_status_t dhcp_status;
    esp_netif_dhcps_get_status(ap_netif, &dhcp_status);
    if (dhcp_status != ESP_NETIF_DHCP_STARTED)
    {
        esp_netif_dhcps_start(ap_netif);
    }

    // Set DNS server to our own IP
    esp_netif_dns_info_t dns_info = {};
    dns_info.ip.u_addr.ip4.addr = esp_ip4addr_aton("192.168.4.1");
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    esp_netif_set_dns_info(ap_netif, ESP_NETIF_DNS_MAIN, &dns_info);

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

// =====================
// Button monitoring task
// =====================
static void button_task(void *arg)
{
    bool button_pressed = false;
    int64_t press_start = 0;
    const int64_t DEEP_SLEEP_HOLD_MS = 2000; // Hold for 2 seconds to enter deep sleep

    while (true)
    {
        int level = gpio_get_level(BUTTON_PIN);
        if (level == 0)
        { // Button pressed (low)
            if (!button_pressed)
            {
                button_pressed = true;
                press_start = esp_timer_get_time() / 1000; // ms
                ESP_LOGI(TAG, "Button pressed");
            }
            else
            {
                int64_t now = esp_timer_get_time() / 1000;
                if (now - press_start >= DEEP_SLEEP_HOLD_MS)
                {
                    ESP_LOGI(TAG, "Button held for 2 seconds, entering deep sleep");
                    // Wait for button release
                    while (gpio_get_level(BUTTON_PIN) == 0)
                    {
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    ESP_LOGI(TAG, "Button released, enabling wake-up and entering deep sleep");
                    // Enable wake-up on GPIO low level
                    esp_deep_sleep_enable_gpio_wakeup(BIT(BUTTON_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
                    // Enter deep sleep
                    esp_deep_sleep_start();
                }
            }
        }
        else
        {
            if (button_pressed)
            {
                ESP_LOGI(TAG, "Button released");
                button_pressed = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Poll every 100ms
    }
}
