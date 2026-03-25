#include "WifiService.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include <string>
#include <vector>
#include <cstring>

static const char *TAG = "WifiService";

// --------------------------------------------------------------------------------
// NVS helpers
// --------------------------------------------------------------------------------
static bool nvs_set_string(const char *ns, const char *key, const std::string &v)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(ns, NVS_READWRITE, &h);
    if (e != ESP_OK) return false;
    e = nvs_set_str(h, key, v.c_str());
    if (e == ESP_OK) nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
}

static bool nvs_get_string(const char *ns, const char *key, std::string &out)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(ns, NVS_READONLY, &h);
    if (e != ESP_OK) return false;
    size_t required = 0;
    e = nvs_get_str(h, key, nullptr, &required);
    if (e != ESP_OK) { nvs_close(h); return false; }
    out.resize(required);
    e = nvs_get_str(h, key, out.data(), &required);
    nvs_close(h);
    if (e != ESP_OK) return false;
    if (!out.empty() && out.back() == '\0') out.resize(out.size() - 1);
    return true;
}

// --------------------------------------------------------------------------------
// WifiService implementation (STA-only)
// --------------------------------------------------------------------------------

void WifiService::init()
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    // These may already be initialized (e.g. by BLE provisioning WiFi scan)
    {
        esp_err_t r = esp_netif_init();
        if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(r);
        }
    }
    {
        esp_err_t r = esp_event_loop_create_default();
        if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
            ESP_ERROR_CHECK(r);
        }
    }

    // STA only - no AP netif
    if (!sta_netif) sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t wifi_err = esp_wifi_init(&cfg);
    if (wifi_err != ESP_OK && wifi_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(wifi_err);
    }

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    registerEvents();
    ESP_LOGI(TAG, "WifiService initialized");
}

bool WifiService::autoConnect()
{
    loadCredentials();
    if (sta_ssid.empty())
    {
        ESP_LOGW(TAG, "autoConnect: No saved credentials found");
        return false;
    }
    ESP_LOGI(TAG, "autoConnect: SSID: %s", sta_ssid.c_str());
    startSTA();
    return true;
}

void WifiService::connectWithCredentials(const char *ssid, const char *pass)
{
    if (!ssid) return;

    ESP_LOGI(TAG, "connectWithCredentials: %s", ssid);
    sta_ssid = ssid;
    sta_pass = pass ? pass : "";
    saveCredentials(sta_ssid.c_str(), sta_pass.c_str());
    startSTA();
}

void WifiService::disconnect()
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    wifi_started = false;
    connected = false;
    if (status_cb) status_cb(0);
    ESP_LOGI(TAG, "WiFi disconnected");
}

void WifiService::disableAutoConnect()
{
    auto_connect_enabled = false;
}

std::string WifiService::getIp() const
{
    if (!sta_netif) return {};
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(sta_netif, &info) == ESP_OK)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&info.ip));
        return std::string(buf);
    }
    return {};
}

std::vector<WifiInfo> WifiService::scanNetworks()
{
    std::vector<WifiInfo> out;

    if (!wifi_started)
    {
        ESP_LOGW(TAG, "Scan blocked: wifi not started");
        return {};
    }

    wifi_scan_config_t cfg = {};
    cfg.show_hidden = true;

    esp_err_t e = esp_wifi_scan_start(&cfg, true);
    if (e != ESP_OK)
    {
        ESP_LOGW(TAG, "scan start failed: %d", e);
        return out;
    }

    uint16_t ap_num = 0;
    esp_wifi_scan_get_ap_num(&ap_num);
    if (ap_num == 0) return out;

    std::vector<wifi_ap_record_t> records(ap_num);
    esp_wifi_scan_get_ap_records(&ap_num, records.data());

    for (uint16_t i = 0; i < ap_num; i++)
    {
        WifiInfo wi;
        wi.ssid = reinterpret_cast<const char *>(records[i].ssid);
        wi.rssi = records[i].rssi;
        out.push_back(wi);
    }
    return out;
}

void WifiService::ensureStaStarted()
{
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    if (mode != WIFI_MODE_STA)
    {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    }
    if (!wifi_started)
    {
        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_started = true;
    }
}

// --------------------------------------------------------------------------------
// Internal
// --------------------------------------------------------------------------------

void WifiService::loadCredentials()
{
    std::string s, p;
    if (nvs_get_string("storage", "ssid", s)) sta_ssid = s;
    if (nvs_get_string("storage", "pass", p)) sta_pass = p;
    ESP_LOGI(TAG, "loadCredentials: SSID: %s, Pass: %s",
             sta_ssid.c_str(), sta_pass.empty() ? "<empty>" : "<set>");
}

void WifiService::saveCredentials(const char *ssid, const char *pass)
{
    nvs_set_string("storage", "ssid", ssid ? ssid : "");
    nvs_set_string("storage", "pass", pass ? pass : "");
}

void WifiService::startSTA()
{
    ESP_LOGI(TAG, "startSTA: Configuring WiFi STA mode (SSID: %s, Pass: %s)",
             sta_ssid.c_str(), sta_pass.empty() ? "<empty>" : "<set>");

    wifi_config_t cfg = {};
    strncpy(reinterpret_cast<char *>(cfg.sta.ssid), sta_ssid.c_str(), sizeof(cfg.sta.ssid) - 1);
    strncpy(reinterpret_cast<char *>(cfg.sta.password), sta_pass.c_str(), sizeof(cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    wifi_started = true;
    ESP_LOGI(TAG, "WiFi STA started. Connecting to SSID: %s (password: %s)",
             sta_ssid.c_str(), sta_pass.empty() ? "<empty>" : "<set>");
    esp_wifi_connect();

    if (status_cb) status_cb(1); // CONNECTING
}

void WifiService::registerEvents()
{
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
        ESP_EVENT_ANY_ID, &WifiService::wifiEventHandlerStatic, this, &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
        IP_EVENT_STA_GOT_IP, &WifiService::ipEventHandlerStatic, this, &instance_got_ip));
}

void WifiService::wifiEventHandlerStatic(void *arg, esp_event_base_t base,
                                         int32_t id, void *data)
{
    auto *s = static_cast<WifiService *>(arg);
    if (s) s->wifiEventHandler(base, id, data);
}

void WifiService::ipEventHandlerStatic(void *arg, esp_event_base_t base,
                                       int32_t id, void *data)
{
    auto *s = static_cast<WifiService *>(arg);
    if (s) s->ipEventHandler(base, id, data);
}

void WifiService::wifiEventHandler(esp_event_base_t base, int32_t id, void *data)
{
    switch (id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        connected = false;
        if (status_cb) status_cb(0);
        if (auto_connect_enabled && !sta_ssid.empty())
        {
            esp_wifi_connect();
        }
        break;
    default:
        break;
    }
}

void WifiService::ipEventHandler(esp_event_base_t base, int32_t id, void *event)
{
    if (id == IP_EVENT_STA_GOT_IP)
    {
        connected = true;
        if (status_cb) status_cb(2);
        ESP_LOGI(TAG, "Got IP - WiFi connected");
    }
}
