#pragma once

#include "esp_event.h"
#include <functional>
#include <string>

// Forward declare to avoid name collision with ESP-IDF mqtt_client.h
typedef struct esp_mqtt_client* esp_mqtt_client_handle_t;

// ============================================================================
// MQTT client wrapper - ported from PTalk MqttClient
// ============================================================================

class MqttClient {
public:
    MqttClient();
    ~MqttClient();

    void setUri(const std::string& uri) { uri_ = uri; }
    void setClientId(const std::string& id) { client_id_ = id; }

    void start();
    void stop();
    bool isConnected() const { return connected_; }

    bool publish(const std::string& topic, const std::string& data, int qos = 0);
    bool subscribe(const std::string& topic, int qos = 0);

    // Callbacks (same pattern as PTalk)
    void onConnected(std::function<void()> cb) { conn_cb_ = std::move(cb); }
    void onDisconnected(std::function<void()> cb) { disc_cb_ = std::move(cb); }
    void onMessage(std::function<void(const std::string&, const std::string&)> cb) { msg_cb_ = std::move(cb); }

private:
    static void eventHandler(void* arg, esp_event_base_t base,
                             int32_t id, void* data);

    esp_mqtt_client_handle_t client_ = nullptr;
    std::string uri_;
    std::string client_id_;
    bool connected_ = false;

    std::function<void()> conn_cb_;
    std::function<void()> disc_cb_;
    std::function<void(const std::string&, const std::string&)> msg_cb_;
};
