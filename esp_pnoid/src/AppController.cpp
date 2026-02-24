#include "AppController.h"
#include "system/StateManager.h"
#include "config/AppConfig.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char* TAG = "AppController";

AppController& AppController::instance() {
    static AppController inst;
    return inst;
}

void AppController::init() {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  PNOID Robot - Connectivity Module");
    ESP_LOGI(TAG, "  ESP32-C5 Co-Processor");
    ESP_LOGI(TAG, "========================================");

    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS truncated, erasing...");
        nvs_flash_erase();
        nvs_flash_init();
    }

    StateManager::instance().setSystem(SystemState::BOOTING);

    // Subscribe to connectivity changes
    StateManager::instance().subscribeConnectivity(
        [this](ConnectivityState s) { onConnectivityChanged(s); });

    // Init modules
    uart_.init();
    network_.init();
    ble_.init();

    // Wire server → STM32 data forwarding
    network_.onWsText([this](const std::string& msg) {
        uart_.sendString(msg + "\n");
    });
    network_.onWsBinary([this](const uint8_t* data, size_t len) {
        uart_.send(data, len);
    });
    network_.onMqttMessage([this](const std::string& topic, const std::string& payload) {
        // Forward as "MQTT:<topic>:<payload>\n"
        uart_.sendString("MQTT:" + topic + ":" + payload + "\n");
    });

    // Wire STM32 → server data forwarding
    uart_.onReceive([this](const uint8_t* data, size_t len) {
        std::string msg((const char*)data, len);
        // Forward to server via MQTT telemetry
        if (network_.mqtt().isConnected()) {
            network_.mqtt().publish(
                std::string(app_cfg::MQTT_TOPIC_PREFIX) + "telemetry", msg);
        }
        // Also forward via WebSocket if connected
        if (network_.ws().isConnected()) {
            network_.ws().sendBinary(data, len);
        }
    });

    // Wire BLE config → WiFi reconnect (like PTalk)
    ble_.onConfigComplete([this](const BleConfig& cfg) {
        ESP_LOGI(TAG, "BLE config received, saving & reconnecting");
        network_.wifi().saveCredentials(cfg.ssid.c_str(), cfg.password.c_str());
        ble_.stopAdvertising();
        wifi_failed_ = false;

        StateManager::instance().setConnectivity(ConnectivityState::CONNECTING_WIFI);
        network_.wifi().connectWithCredentials(cfg.ssid.c_str(), cfg.password.c_str());
    });
}

void AppController::start() {
    ESP_LOGI(TAG, "Starting...");
    StateManager::instance().setSystem(SystemState::RUNNING);

    uart_.startReceiveTask();
    network_.start();

    ESP_LOGI(TAG, "All modules started");
}

void AppController::stop() {
    network_.stop();
    ESP_LOGI(TAG, "Stopped");
}

void AppController::onConnectivityChanged(ConnectivityState s) {
    ESP_LOGI(TAG, "Connectivity -> %s", toString(s));

    // Notify STM32 about state changes
    notifyStm32(std::string("STATE:") + toString(s) + "\n");

    switch (s) {
        case ConnectivityState::OFFLINE:
            if (!wifi_failed_) {
                wifi_failed_ = true;
                // WiFi failed → start BLE provisioning (like PTalk fallback)
                ESP_LOGW(TAG, "WiFi offline -> starting BLE provisioning");
                StateManager::instance().setConnectivity(ConnectivityState::CONFIG_BLE);
                ble_.startAdvertising();
            }
            break;

        case ConnectivityState::ONLINE:
            wifi_failed_ = false;
            ble_.stopAdvertising();
            ESP_LOGI(TAG, "Fully connected!");
            break;

        default:
            break;
    }
}

void AppController::notifyStm32(const std::string& msg) {
    uart_.sendString(msg);
}

// ============================================================================
// Entry point (called from main.c)
// ============================================================================

extern "C" void app_init() {
    auto& app = AppController::instance();
    app.init();
    app.start();
}
