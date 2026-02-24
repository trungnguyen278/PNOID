#include "system/NetworkManager.h"
#include "system/StateManager.h"
#include "config/AppConfig.h"
#include "esp_log.h"

static const char* TAG = "NetworkManager";

NetworkManager::NetworkManager() {}
NetworkManager::~NetworkManager() {}

void NetworkManager::init() {
    // Wire up WiFi status → connection sequence
    wifi_.onStatus([this](int s) { onWifiStatus(s); });

    // Wire up WebSocket callbacks
    ws_.onStatus([this](int s) { onWsStatus(s); });
    ws_.onText([this](const std::string& msg) {
        ESP_LOGD(TAG, "WS text: %.*s", (int)msg.size(), msg.c_str());
        if (ws_text_cb_) ws_text_cb_(msg);
    });
    ws_.onBinary([this](const uint8_t* data, size_t len) {
        ESP_LOGD(TAG, "WS binary: %d bytes", (int)len);
        if (ws_binary_cb_) ws_binary_cb_(data, len);
    });

    // Wire up MQTT callbacks
    mqtt_.onConnected([this]() { setupMqttSubscriptions(); });
    mqtt_.onMessage([this](const std::string& topic, const std::string& payload) {
        ESP_LOGD(TAG, "MQTT [%s]: %.*s", topic.c_str(), (int)payload.size(), payload.c_str());
        if (mqtt_msg_cb_) mqtt_msg_cb_(topic, payload);
    });

    wifi_.init();
    ws_.init();

    ESP_LOGI(TAG, "NetworkManager initialized");
}

void NetworkManager::start() {
    ESP_LOGI(TAG, "Starting network...");
    StateManager::instance().setConnectivity(ConnectivityState::CONNECTING_WIFI);

    // Try saved credentials, otherwise use defaults
    if (!wifi_.autoConnect()) {
        wifi_.connectWithCredentials(app_cfg::WIFI_DEFAULT_SSID, app_cfg::WIFI_DEFAULT_PASS);
    }
}

void NetworkManager::stop() {
    mqtt_.stop();
    ws_.close();
    wifi_.disconnect();
    StateManager::instance().setConnectivity(ConnectivityState::OFFLINE);
    ESP_LOGI(TAG, "Network stopped");
}

void NetworkManager::onWifiStatus(int status) {
    switch (status) {
        case 0:  // disconnected
            ESP_LOGW(TAG, "WiFi disconnected");
            StateManager::instance().setConnectivity(ConnectivityState::OFFLINE);
            break;

        case 1:  // connecting
            StateManager::instance().setConnectivity(ConnectivityState::CONNECTING_WIFI);
            break;

        case 2:  // got IP
            ESP_LOGI(TAG, "WiFi connected -> starting WebSocket + MQTT");
            StateManager::instance().setConnectivity(ConnectivityState::CONNECTING_WS);
            ws_.connect();
            mqtt_.start();
            break;
    }
}

void NetworkManager::onWsStatus(int status) {
    switch (status) {
        case 0:  // closed
            ESP_LOGW(TAG, "WebSocket closed");
            // Don't downgrade if MQTT still connected
            break;

        case 2:  // open
            ESP_LOGI(TAG, "WebSocket connected");
            // Check if MQTT also connected
            if (mqtt_.isConnected()) {
                StateManager::instance().setConnectivity(ConnectivityState::ONLINE);
            }
            break;
    }
}

void NetworkManager::setupMqttSubscriptions() {
    ESP_LOGI(TAG, "MQTT connected -> subscribing");

    std::string prefix = app_cfg::MQTT_TOPIC_PREFIX;
    mqtt_.subscribe(prefix + "cmd");
    mqtt_.subscribe(prefix + "config");

    // If WS also connected, we're fully online
    if (ws_.isConnected()) {
        StateManager::instance().setConnectivity(ConnectivityState::ONLINE);
    }
}
