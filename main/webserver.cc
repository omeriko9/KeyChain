#include "webserver.h"

#include <string.h>
#include <string>
#include <sys/stat.h>
#include <dirent.h>

#include <esp_log.h>
#include <esp_http_server.h>

#include <lwip/sockets.h>
#include <lwip/netdb.h>

static const char *TAG_WS = "websrv";

// Keep a copy of configuration
static WebServerConfig g_cfg{};

// Embedded portal/index handling
extern const uint8_t index_html_start[] asm("_binary_embedded_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_embedded_index_html_end");

// Captive portal HTML when in SoftAP mode
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

// Helpers
static bool safe_name(const char *name)
{
    if (!name || !*name)
        return false;
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

// HTTP Handlers (use g_cfg and helpers)
static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    const size_t len = index_html_end - index_html_start;
    return httpd_resp_send(req, (const char *)index_html_start, len);
}

static esp_err_t handle_list(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    DIR *d = opendir(g_cfg.img_dir);
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
    const char *name = uri + 5;
    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(g_cfg.img_dir, name_dec);
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
    const char *uri = req->uri;
    const char *name = uri + 5;
    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(g_cfg.img_dir, name_dec);
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
    if (strncmp(uri, "/api/delete/", 12) == 0)
        name = uri + 12;
    else if (strncmp(uri, "/img/", 5) == 0)
        name = uri + 5;
    else
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid uri");
    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(g_cfg.img_dir, name_dec);
    int r = unlink(full.c_str());
    if (r != 0)
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found");
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t handle_delete_all(httpd_req_t *req)
{
    DIR *d = opendir(g_cfg.img_dir);
    if (!d)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "dir open failed");

    struct dirent *de;
    int deleted_count = 0;
    while ((de = readdir(d)) != nullptr)
    {
        if (de->d_name[0] == '.')
            continue;
        std::string full = path_join(g_cfg.img_dir, de->d_name);
        if (unlink(full.c_str()) == 0)
            deleted_count++;
    }
    closedir(d);

    char response[64];
    snprintf(response, sizeof(response), "Deleted %d images", deleted_count);
    return httpd_resp_sendstr(req, response);
}

static esp_err_t handle_upload_fn(httpd_req_t *req)
{
    const char *uri = req->uri;
    const char *name = uri + 12; // /api/upload/
    if (!safe_name(name))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad name");
    char name_dec[96];
    snprintf(name_dec, sizeof(name_dec), "%s", name);
    url_decode(name_dec);
    std::string full = path_join(g_cfg.img_dir, name_dec);
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

static esp_err_t handle_settings_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (!g_cfg.get_image_display_seconds)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cfg err");
    int secs = g_cfg.get_image_display_seconds();
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "{\"image_display_seconds\":%d}", secs);
    if (n < 0)
        return ESP_FAIL;
    return httpd_resp_send(req, buf, n);
}

static esp_err_t handle_settings_put(httpd_req_t *req)
{
    if (!g_cfg.set_image_display_seconds)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cfg err");
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
    g_cfg.set_image_display_seconds((int)val);
    return httpd_resp_sendstr(req, "OK");
}

esp_err_t webserver_start(const WebServerConfig *cfg)
{
    if (!cfg || !cfg->img_dir)
        return ESP_ERR_INVALID_ARG;
    g_cfg = *cfg; // shallow copy of function pointers and consts

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;
    config.max_uri_handlers = 17;
    config.uri_match_fn = httpd_uri_match_wildcard;
    httpd_handle_t server = nullptr;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t uri_root = {.uri = "/", .method = HTTP_GET, .handler = handle_root, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_root);
    httpd_uri_t uri_idx = {.uri = "/index.html", .method = HTTP_GET, .handler = handle_root, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_idx);
    httpd_uri_t uri_list = {.uri = "/api/list", .method = HTTP_GET, .handler = handle_list, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_list);
    httpd_uri_t uri_img_get = {.uri = "/img/*", .method = HTTP_GET, .handler = handle_img_get, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_img_get);
    httpd_uri_t uri_img_put = {.uri = "/img/*", .method = HTTP_PUT, .handler = handle_img_put, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_img_put);
    httpd_uri_t uri_img_del = {.uri = "/img/*", .method = HTTP_DELETE, .handler = handle_img_delete, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_img_del);
    httpd_uri_t uri_api_del = {.uri = "/api/delete/*", .method = HTTP_DELETE, .handler = handle_img_delete, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_api_del);
    httpd_uri_t uri_delete_all = {.uri = "/api/delete-all", .method = HTTP_DELETE, .handler = handle_delete_all, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_delete_all);
    httpd_uri_t uri_upload = {.uri = "/api/upload/*", .method = HTTP_PUT, .handler = handle_upload_fn, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_upload);
    httpd_uri_t uri_settings_get = {.uri = "/api/settings", .method = HTTP_GET, .handler = handle_settings_get, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_settings_get);
    httpd_uri_t uri_settings_put = {.uri = "/api/settings", .method = HTTP_PUT, .handler = handle_settings_put, .user_ctx = nullptr};
    httpd_register_uri_handler(server, &uri_settings_put);

    ESP_LOGI(TAG_WS, "HTTP server started on :%d", config.server_port);

    return ESP_OK;
}
