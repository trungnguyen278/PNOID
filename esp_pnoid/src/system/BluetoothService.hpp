#pragma once

#include <string>
#include <functional>
#include <vector>
#include <cstdint>

#include "Version.hpp"

struct WifiInfo;

// BLE GATT provisioning service using NimBLE stack.
// ESP32-C5 BLE 5.0 LE - compatible with PNoid V1 mobile app.
class BluetoothService
{
public:
    struct ConfigData
    {
        std::string device_name = "PNoid";
        uint8_t volume = 60;
        uint8_t brightness = 100;
        std::string ssid;
        std::string pass;
        std::string ws_url;
        std::string mqtt_url;
        std::string mqtt_user;
        std::string mqtt_pass;
        ConfigData() = default;
    };

    using OnConfigComplete = std::function<void(const ConfigData &)>;

    BluetoothService();
    ~BluetoothService();

    bool init(const std::string &adv_name, const std::vector<WifiInfo> &cached_networks = {}, const ConfigData *current_config = nullptr);
    bool start();
    void stop();
    void deinit();  // Full teardown: stop + release BLE controller memory

    void onConfigComplete(OnConfigComplete cb) { config_cb_ = cb; }

    // Same UUIDs as V1 for mobile app compatibility
    static constexpr uint16_t SVC_UUID_CONFIG = 0xFF01;
    static constexpr uint16_t CHR_UUID_DEVICE_NAME = 0xFF02;
    static constexpr uint16_t CHR_UUID_VOLUME = 0xFF03;
    static constexpr uint16_t CHR_UUID_BRIGHTNESS = 0xFF04;
    static constexpr uint16_t CHR_UUID_WIFI_SSID = 0xFF05;
    static constexpr uint16_t CHR_UUID_WIFI_PASS = 0xFF06;
    static constexpr uint16_t CHR_UUID_APP_VERSION = 0xFF07;
    static constexpr uint16_t CHR_UUID_BUILD_INFO = 0xFF08;
    static constexpr uint16_t CHR_UUID_SAVE_CMD = 0xFF09;
    static constexpr uint16_t CHR_UUID_DEVICE_ID = 0xFF0A;
    static constexpr uint16_t CHR_UUID_WIFI_LIST = 0xFF0B;
    static constexpr uint16_t CHR_UUID_WS_URL = 0xFF0C;
    static constexpr uint16_t CHR_UUID_MQTT_URL = 0xFF0D;
    static constexpr uint16_t CHR_UUID_MQTT_USER = 0xFF0E;
    static constexpr uint16_t CHR_UUID_MQTT_PASS = 0xFF0F;

    // NimBLE callbacks need access to instance
    static BluetoothService *s_instance;

    ConfigData temp_cfg_;
    OnConfigComplete config_cb_ = nullptr;
    bool url_unlocked_ = false;
    static constexpr const char *WS_URL_AUTH_TOKEN = "PNoid_OK";

    std::string device_id_str_;
    std::vector<WifiInfo> wifi_networks_;
    size_t wifi_read_index_ = 0;

private:
    std::string adv_name_;
    bool started_ = false;
};
