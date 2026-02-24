#include "network/WifiService.h"
#include "config/AppConfig.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs.h"
#include <cstring>

static const char* TAG = "WifiService";

WifiService::WifiService() {}
WifiService::~WifiService() {}

void WifiService::init() {
    if (initialized_) return;

    event_group_ = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiService::eventHandler, this, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiService::eventHandler, this, nullptr));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    initialized_ = true;
    ESP_LOGI(TAG, "WiFi initialized (STA mode)");
}

bool WifiService::autoConnect() {
    std::string ssid, pass;
    if (!loadCredentials(ssid, pass) || ssid.empty()) {
        ESP_LOGW(TAG, "No saved credentials");
        return false;
    }
    connectWithCredentials(ssid.c_str(), pass.c_str());
    return true;
}

void WifiService::connectWithCredentials(const char* ssid, const char* pass) {
    wifi_config_t cfg = {};
    strncpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid) - 1);
    strncpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password) - 1);
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    retry_count_ = 0;
    xEventGroupClearBits(event_group_, BIT_CONNECTED | BIT_FAIL);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_connect());

    if (status_cb_) status_cb_(1);  // connecting
    ESP_LOGI(TAG, "Connecting to '%s'...", ssid);

    EventBits_t bits = xEventGroupWaitBits(
        event_group_, BIT_CONNECTED | BIT_FAIL,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(app_cfg::WIFI_CONNECT_TIMEOUT));

    if (bits & BIT_CONNECTED) {
        ESP_LOGI(TAG, "Connected, IP: %s", ip_addr_.c_str());
        saveCredentials(ssid, pass);
    } else {
        ESP_LOGE(TAG, "Connection failed");
        if (status_cb_) status_cb_(0);  // disconnected
    }
}

void WifiService::disconnect() {
    connected_ = false;
    esp_wifi_disconnect();
}

void WifiService::eventHandler(void* arg, esp_event_base_t base,
                                int32_t id, void* data) {
    auto* self = static_cast<WifiService*>(arg);

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        self->handleWifiDisconnect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        self->handleGotIp(static_cast<ip_event_got_ip_t*>(data));
    }
}

void WifiService::handleWifiDisconnect() {
    connected_ = false;
    if (retry_count_ < app_cfg::WIFI_MAX_RETRY) {
        retry_count_++;
        ESP_LOGW(TAG, "Disconnected, retry %d/%d", retry_count_, app_cfg::WIFI_MAX_RETRY);
        vTaskDelay(pdMS_TO_TICKS(app_cfg::WIFI_RETRY_INTERVAL));
        esp_wifi_connect();
    } else {
        ESP_LOGE(TAG, "Max retries reached");
        xEventGroupSetBits(event_group_, BIT_FAIL);
    }
    if (status_cb_) status_cb_(0);
}

void WifiService::handleGotIp(ip_event_got_ip_t* event) {
    char buf[16];
    snprintf(buf, sizeof(buf), IPSTR, IP2STR(&event->ip_info.ip));
    ip_addr_ = buf;
    connected_ = true;
    retry_count_ = 0;
    xEventGroupSetBits(event_group_, BIT_CONNECTED);
    if (status_cb_) status_cb_(2);  // got IP
}

void WifiService::saveCredentials(const char* ssid, const char* pass) {
    nvs_handle_t h;
    if (nvs_open(app_cfg::NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, app_cfg::NVS_KEY_SSID, ssid);
        nvs_set_str(h, app_cfg::NVS_KEY_PASS, pass);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool WifiService::loadCredentials(std::string& ssid, std::string& pass) {
    nvs_handle_t h;
    if (nvs_open(app_cfg::NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    char buf[65] = {};
    size_t len = sizeof(buf);
    bool ok = false;
    if (nvs_get_str(h, app_cfg::NVS_KEY_SSID, buf, &len) == ESP_OK) {
        ssid = buf;
        len = sizeof(buf);
        if (nvs_get_str(h, app_cfg::NVS_KEY_PASS, buf, &len) == ESP_OK) {
            pass = buf;
            ok = true;
        }
    }
    nvs_close(h);
    return ok;
}
