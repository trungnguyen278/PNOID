#pragma once

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <functional>
#include <string>

// ============================================================================
// WiFi Station service - ported from PTalk WifiService
// ============================================================================

class WifiService {
public:
    WifiService();
    ~WifiService();

    void init();
    bool autoConnect();  // Try saved credentials, returns false if no creds
    void connectWithCredentials(const char* ssid, const char* pass);
    void disconnect();

    bool isConnected() const { return connected_; }
    const std::string& getIp() const { return ip_addr_; }

    // Save/load from NVS
    void saveCredentials(const char* ssid, const char* pass);
    bool loadCredentials(std::string& ssid, std::string& pass);

    // Status callback: 0=disconnected, 1=connecting, 2=got_ip
    void onStatus(std::function<void(int)> cb) { status_cb_ = std::move(cb); }

private:
    static void eventHandler(void* arg, esp_event_base_t base,
                             int32_t id, void* data);

    void handleWifiDisconnect();
    void handleGotIp(ip_event_got_ip_t* event);

    std::function<void(int)> status_cb_;
    EventGroupHandle_t event_group_ = nullptr;
    std::string ip_addr_;
    int retry_count_ = 0;
    bool initialized_ = false;
    bool connected_   = false;

    static constexpr int BIT_CONNECTED = BIT0;
    static constexpr int BIT_FAIL      = BIT1;
};
