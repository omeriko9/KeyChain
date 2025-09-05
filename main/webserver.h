#pragma once

#include <esp_err.h>

// Minimal interface to start the HTTP server and related request handlers.
// Main provides callbacks so webserver stays decoupled from app state.
struct WebServerConfig {
    // Directory path where images are stored (e.g., "/spiffs/img").
    const char* img_dir;

    // Returns true when device is in SoftAP provisioning mode (captive portal active).
    bool (*is_softap_active)();

    // Save Wi‑Fi credentials (persist to NVS etc.). Return true on success.
    bool (*save_wifi_credentials)(const char* ssid, const char* password);

    // Get/Set image display seconds setting.
    int  (*get_image_display_seconds)();
    void (*set_image_display_seconds)(int seconds);
};

// Start HTTP server and register all endpoints. Non-blocking. Returns ESP_OK on success.
esp_err_t webserver_start(const WebServerConfig* cfg);
