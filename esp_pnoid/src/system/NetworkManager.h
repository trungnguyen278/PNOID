#pragma once

#include "network/WifiService.h"
#include "network/WebSocketClient.h"
#include "network/MqttClient.h"
#include <memory>

// ============================================================================
// Network orchestrator - ported from PTalk NetworkManager
// Manages: WiFi → WebSocket → MQTT connection sequence
// ============================================================================

class NetworkManager {
public:
    NetworkManager();
    ~NetworkManager();

    void init();
    void start();
    void stop();

    WifiService&      wifi()  { return wifi_; }
    WebSocketClient&  ws()    { return ws_; }
    MqttClient&       mqtt()  { return mqtt_; }

    // Data forwarding callbacks (from server → STM32)
    void onWsText(std::function<void(const std::string&)> cb) { ws_text_cb_ = std::move(cb); }
    void onWsBinary(std::function<void(const uint8_t*, size_t)> cb) { ws_binary_cb_ = std::move(cb); }
    void onMqttMessage(std::function<void(const std::string&, const std::string&)> cb) { mqtt_msg_cb_ = std::move(cb); }

private:
    void onWifiStatus(int status);
    void onWsStatus(int status);
    void setupMqttSubscriptions();

    WifiService      wifi_;
    WebSocketClient  ws_;
    MqttClient       mqtt_;

    std::function<void(const std::string&)> ws_text_cb_;
    std::function<void(const uint8_t*, size_t)> ws_binary_cb_;
    std::function<void(const std::string&, const std::string&)> mqtt_msg_cb_;
};
