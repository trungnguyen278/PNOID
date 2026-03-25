#include "DeviceProfile.hpp"
#include "AppController.hpp"

// System managers
#include "system/NetworkManager.hpp"
#include "system/SpiBridge.hpp"
#include "system/UartBridge.hpp"
#include "system/BluetoothService.hpp"
#include "system/StateManager.hpp"
#include "system/StateTypes.hpp"

// Pin config
#include "config/PinConfig.hpp"

#include "WifiService.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"

static const char* TAG = "DeviceProfile";

// =============================================================================
// User settings from NVS
// =============================================================================
namespace user_cfg {
    struct UserSettings {
        std::string device_name = "PNoid";
        uint8_t  volume = 30;
        std::string wifi_ssid;
        std::string wifi_pass;
        std::string ws_url;
        std::string mqtt_url;
        std::string user_id;
        std::string tx_key;
    };

    static std::string get_string(nvs_handle_t h, const char* key) {
        size_t required = 0;
        if (nvs_get_str(h, key, nullptr, &required) != ESP_OK || required == 0) return {};
        std::string tmp; tmp.resize(required);
        if (nvs_get_str(h, key, tmp.data(), &required) != ESP_OK) return {};
        if (!tmp.empty() && tmp.back() == '\0') tmp.pop_back();
        return tmp;
    }

    static uint8_t get_u8(nvs_handle_t h, const char* key, uint8_t def_val) {
        uint8_t v = def_val;
        nvs_get_u8(h, key, &v);
        return v;
    }

    static UserSettings load() {
        UserSettings cfg;
        nvs_handle_t h;
        if (nvs_open("storage", NVS_READONLY, &h) != ESP_OK) {
            ESP_LOGI(TAG, "NVS storage not found, using defaults");
            return cfg;
        }
        cfg.device_name = get_string(h, "device_name");
        if (cfg.device_name.empty()) cfg.device_name = "PNoid";
        cfg.wifi_ssid = get_string(h, "ssid");
        cfg.wifi_pass = get_string(h, "pass");
        cfg.ws_url    = get_string(h, "ws_url");
        cfg.mqtt_url  = get_string(h, "mqtt_url");
        cfg.user_id   = get_string(h, "user_id");
        cfg.tx_key    = get_string(h, "tx_key");
        cfg.volume    = get_u8(h, "volume", cfg.volume);
        nvs_close(h);
        return cfg;
    }
}

// URL normalization helpers
static std::string normalize_ws_url(std::string val) {
    while (!val.empty() && (val.front() == ' ' || val.front() == '\n')) val.erase(val.begin());
    while (!val.empty() && (val.back() == ' ' || val.back() == '\n')) val.pop_back();
    if (val.empty()) return {};
    if (val.rfind("ws://", 0) == 0 || val.rfind("wss://", 0) == 0) {
        if (val.find('/', val.find("://") + 3) == std::string::npos) val += "/ws";
        return val;
    }
    return "ws://" + val + "/ws";
}

static std::string normalize_mqtt_url(std::string val) {
    while (!val.empty() && (val.front() == ' ' || val.front() == '\n')) val.erase(val.begin());
    while (!val.empty() && (val.back() == ' ' || val.back() == '\n')) val.pop_back();
    if (val.empty()) return {};
    if (val.rfind("mqtt://", 0) == 0 || val.rfind("mqtts://", 0) == 0) return val;
    return "mqtt://" + val;
}

// =============================================================================
// Setup
// =============================================================================
bool DeviceProfile::setup(AppController& app)
{
    ESP_LOGI(TAG, "DeviceProfile setup begin (ESP32-C5), free heap: %lu",
             (unsigned long)esp_get_free_heap_size());

    // Init NVS
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    auto user = user_cfg::load();

    // =========================================================
    // 1. BLE PROVISIONING (if no WiFi credentials in NVS)
    // =========================================================
    if (user.wifi_ssid.empty()) {
        ESP_LOGI(TAG, "No WiFi credentials found - starting BLE provisioning");

        esp_netif_init();
        esp_event_loop_create_default();
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&wifi_cfg);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        vTaskDelay(pdMS_TO_TICKS(500));

        wifi_scan_config_t scan_cfg = {};
        esp_wifi_scan_start(&scan_cfg, true);
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        std::vector<wifi_ap_record_t> ap_records(ap_count);
        esp_wifi_scan_get_ap_records(&ap_count, ap_records.data());

        std::vector<WifiInfo> networks;
        for (int i = 0; i < ap_count; i++) {
            WifiInfo info;
            info.ssid = (const char*)ap_records[i].ssid;
            info.rssi = ap_records[i].rssi;
            networks.push_back(info);
        }

        esp_wifi_stop();
        esp_wifi_deinit();
        ESP_LOGI(TAG, "Scanned %d WiFi networks for BLE listing", (int)networks.size());

        BluetoothService::ConfigData ble_cfg;
        ble_cfg.device_name = user.device_name;
        ble_cfg.volume = user.volume;
        ble_cfg.ws_url = user.ws_url;
        ble_cfg.mqtt_url = user.mqtt_url;
        ble_cfg.mqtt_user = user.user_id;
        ble_cfg.mqtt_pass = user.tx_key;

        static BluetoothService ble;
        if (!ble.init("PNoid", networks, &ble_cfg)) {
            ESP_LOGE(TAG, "BLE init failed");
            return false;
        }

        static volatile bool ble_config_received = false;
        static BluetoothService::ConfigData received_cfg;

        ble.onConfigComplete([](const BluetoothService::ConfigData& cfg) {
            received_cfg = cfg;
            ble_config_received = true;
            ESP_LOGI(TAG, "BLE config received: ssid='%s'", cfg.ssid.c_str());
        });

        ble.start();
        ESP_LOGI(TAG, "Waiting for BLE provisioning...");

        while (!ble_config_received) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        ble.deinit();
        vTaskDelay(pdMS_TO_TICKS(200));

        nvs_handle_t h;
        if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
            if (!received_cfg.ssid.empty())
                nvs_set_str(h, "ssid", received_cfg.ssid.c_str());
            if (!received_cfg.pass.empty())
                nvs_set_str(h, "pass", received_cfg.pass.c_str());
            if (!received_cfg.ws_url.empty())
                nvs_set_str(h, "ws_url", received_cfg.ws_url.c_str());
            if (!received_cfg.mqtt_url.empty())
                nvs_set_str(h, "mqtt_url", received_cfg.mqtt_url.c_str());
            if (!received_cfg.mqtt_user.empty())
                nvs_set_str(h, "user_id", received_cfg.mqtt_user.c_str());
            if (!received_cfg.mqtt_pass.empty())
                nvs_set_str(h, "tx_key", received_cfg.mqtt_pass.c_str());
            if (!received_cfg.device_name.empty())
                nvs_set_str(h, "device_name", received_cfg.device_name.c_str());
            nvs_set_u8(h, "volume", received_cfg.volume);
            nvs_commit(h);
            nvs_close(h);
        }

        user.wifi_ssid = received_cfg.ssid;
        user.wifi_pass = received_cfg.pass;
        if (!received_cfg.ws_url.empty()) user.ws_url = received_cfg.ws_url;
        if (!received_cfg.mqtt_url.empty()) user.mqtt_url = received_cfg.mqtt_url;
        if (!received_cfg.mqtt_user.empty()) user.user_id = received_cfg.mqtt_user;
        if (!received_cfg.mqtt_pass.empty()) user.tx_key = received_cfg.mqtt_pass;
        user.device_name = received_cfg.device_name;
        user.volume = received_cfg.volume;

        ESP_LOGI(TAG, "BLE provisioning complete, continuing with WiFi connect");
    }

    // =========================================================
    // 2. NETWORK (WiFi + WebSocket + MQTT)
    // =========================================================
    auto network_mgr = std::make_unique<NetworkManager>();

    const std::string default_ws   = "10.170.75.193:8000";
    const std::string default_mqtt = "10.170.75.193:1883";

    NetworkManager::Config net_cfg{};
    net_cfg.wifi_ssid = user.wifi_ssid;
    net_cfg.wifi_pass = user.wifi_pass;
    net_cfg.ws_url    = normalize_ws_url(user.ws_url.empty() ? default_ws : user.ws_url);
    net_cfg.mqtt_url  = normalize_mqtt_url(user.mqtt_url.empty() ? default_mqtt : user.mqtt_url);
    net_cfg.user_id   = user.user_id;
    net_cfg.tx_key    = user.tx_key;

    ESP_LOGI(TAG, "Before NetworkManager init, free heap: %lu",
             (unsigned long)esp_get_free_heap_size());
    if (!network_mgr->init(net_cfg)) {
        ESP_LOGE(TAG, "NetworkManager init failed");
        return false;
    }
    ESP_LOGI(TAG, "After NetworkManager init, free heap: %lu",
             (unsigned long)esp_get_free_heap_size());

    // =========================================================
    // 3. SPI BRIDGE (Communication with S3 via SPI2 slave)
    // =========================================================
    auto spi_bridge = std::make_unique<SpiBridge>();
    SpiBridge::Config spi_cfg{
        .pin_mosi      = (gpio_num_t)PinConfig::SPI_MOSI,
        .pin_miso      = (gpio_num_t)PinConfig::SPI_MISO,
        .pin_sclk      = (gpio_num_t)PinConfig::SPI_SCLK,
        .pin_cs        = (gpio_num_t)PinConfig::SPI_CS,
        .pin_handshake = (gpio_num_t)PinConfig::SPI_HANDSHAKE,
    };

    if (!spi_bridge->init(spi_cfg)) {
        ESP_LOGE(TAG, "SpiBridge init failed");
        return false;
    }

    // Wire SPI to NetworkManager (audio only)
    network_mgr->setSpiBridge(spi_bridge.get());

    // Flow control: SPI EMPTY frames include WS tx_buffer free space (KB)
    // so S3 can throttle audio uplink when C5's WS buffer is full
    NetworkManager* net_ptr = network_mgr.get();
    spi_bridge->setUplinkSpaceQuery([net_ptr]() -> size_t {
        return net_ptr->getWsTxFreeSpace();
    });
    spi_bridge->onAudioUplink([net_ptr](const uint8_t* data, size_t len) {
        net_ptr->sendBinary(data, len);
    });

    // =========================================================
    // 4. UART BRIDGE (Control/status with S3 via UART1)
    // =========================================================
    auto uart_bridge = std::make_unique<UartBridge>();
    UartBridge::Config uart_cfg{
        .pin_tx = (gpio_num_t)PinConfig::UART_TX,
        .pin_rx = (gpio_num_t)PinConfig::UART_RX,
    };

    if (!uart_bridge->init(uart_cfg)) {
        ESP_LOGE(TAG, "UartBridge init failed");
        return false;
    }

    // UART control: S3 sends START/END/config commands via UART
    uart_bridge->onControlCmd([net_ptr](uart_proto::ControlCmd cmd, const uint8_t* data, size_t len) {
        switch (cmd) {
        case uart_proto::ControlCmd::START:
            net_ptr->endSpeakingSession();
            net_ptr->sendText("START");
            StateManager::instance().setInteractionState(
                state::InteractionState::LISTENING, state::InputSource::BUTTON);
            break;
        case uart_proto::ControlCmd::END:
            net_ptr->waitTxDrain(1000);
            net_ptr->sendText("END");
            StateManager::instance().setInteractionState(
                state::InteractionState::IDLE, state::InputSource::BUTTON);
            break;
        case uart_proto::ControlCmd::SET_VOLUME:
            if (len >= 1) {
                ESP_LOGI(TAG, "Volume command from S3: %d", data[0]);
            }
            break;
        case uart_proto::ControlCmd::REBOOT:
            esp_restart();
            break;
        case uart_proto::ControlCmd::WIFI_CONFIG: {
            if (len < 2) break;
            size_t pos = 0;
            uint8_t ssid_len = data[pos++];
            if (pos + ssid_len > len) break;
            std::string ssid(reinterpret_cast<const char*>(&data[pos]), ssid_len);
            pos += ssid_len;
            if (pos >= len) break;
            uint8_t pass_len = data[pos++];
            if (pos + pass_len > len) break;
            std::string pass(reinterpret_cast<const char*>(&data[pos]), pass_len);
            ESP_LOGI(TAG, "UART: WiFi config from S3: ssid='%s'", ssid.c_str());
            nvs_handle_t h;
            if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
                nvs_set_str(h, "ssid", ssid.c_str());
                nvs_set_str(h, "pass", pass.c_str());
                nvs_commit(h);
                nvs_close(h);
            }
            net_ptr->setCredentials(ssid, pass);
            break;
        }
        default:
            break;
        }
    });

    // Forward emotion state changes to S3 via UART
    UartBridge* uart_ptr = uart_bridge.get();
    StateManager::instance().subscribeEmotion([uart_ptr](state::EmotionState e) {
        uart_ptr->sendStatusUpdate(
            (uint8_t)StateManager::instance().getInteractionState(),
            (uint8_t)StateManager::instance().getConnectivityState(),
            (uint8_t)StateManager::instance().getSystemState(),
            (uint8_t)e);
    });

    // Forward connectivity state changes to S3 via UART
    StateManager::instance().subscribeConnectivity([uart_ptr](state::ConnectivityState c) {
        uart_ptr->sendStatusUpdate(
            (uint8_t)StateManager::instance().getInteractionState(),
            (uint8_t)c,
            (uint8_t)StateManager::instance().getSystemState(),
            (uint8_t)StateManager::instance().getEmotionState());
    });

    // =========================================================
    // 5. ATTACH MODULES -> APP CONTROLLER
    // =========================================================
    app.attachModules(
        std::move(network_mgr),
        std::move(spi_bridge),
        std::move(uart_bridge));

    ESP_LOGI(TAG, "DeviceProfile setup OK (ESP32-C5)");
    return true;
}
