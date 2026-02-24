#include "network/MqttClient.h"
#include "config/AppConfig.h"
#include <mqtt_client.h>   // ESP-IDF header (angle brackets to avoid collision)
#include "esp_log.h"
#include <cstring>

static const char* TAG = "MqttClient";

MqttClient::MqttClient()
    : uri_(app_cfg::MQTT_DEFAULT_URI)
    , client_id_(app_cfg::MQTT_CLIENT_ID) {}

MqttClient::~MqttClient() { stop(); }

void MqttClient::start() {
    if (client_) { stop(); }

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = uri_.c_str();
    cfg.credentials.client_id = client_id_.c_str();
    cfg.session.keepalive = app_cfg::MQTT_KEEPALIVE;

    client_ = esp_mqtt_client_init(&cfg);
    if (!client_) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    esp_mqtt_client_register_event(client_, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                   &MqttClient::eventHandler, this);
    esp_mqtt_client_start(client_);
    ESP_LOGI(TAG, "Connecting to %s", uri_.c_str());
}

void MqttClient::stop() {
    if (!client_) return;
    connected_ = false;
    esp_mqtt_client_stop(client_);
    esp_mqtt_client_destroy(client_);
    client_ = nullptr;
}

bool MqttClient::publish(const std::string& topic, const std::string& data, int qos) {
    if (!client_ || !connected_) return false;
    return esp_mqtt_client_publish(client_, topic.c_str(), data.c_str(), 0, qos, 0) >= 0;
}

bool MqttClient::subscribe(const std::string& topic, int qos) {
    if (!client_ || !connected_) return false;
    return esp_mqtt_client_subscribe(client_, topic.c_str(), qos) >= 0;
}

void MqttClient::eventHandler(void* arg, esp_event_base_t base,
                               int32_t id, void* data) {
    auto* self = static_cast<MqttClient*>(arg);
    auto* evt = static_cast<esp_mqtt_event_handle_t>(data);

    switch (id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected");
            self->connected_ = true;
            if (self->conn_cb_) self->conn_cb_();
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected");
            self->connected_ = false;
            if (self->disc_cb_) self->disc_cb_();
            break;

        case MQTT_EVENT_DATA:
            if (self->msg_cb_ && evt->topic_len > 0) {
                std::string topic(evt->topic, evt->topic_len);
                std::string payload(evt->data, evt->data_len);
                self->msg_cb_(topic, payload);
            }
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error type: %d", evt->error_handle->error_type);
            break;

        default:
            break;
    }
}
