#include "network/WebSocketClient.h"
#include "config/AppConfig.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "WebSocketClient";

WebSocketClient::WebSocketClient() : url_(app_cfg::WS_DEFAULT_URL) {}
WebSocketClient::~WebSocketClient() { close(); }

void WebSocketClient::init() {
    ESP_LOGI(TAG, "WebSocket client ready");
}

void WebSocketClient::connect() {
    if (client_) {
        esp_websocket_client_destroy(client_);
        client_ = nullptr;
    }

    esp_websocket_client_config_t cfg = {};
    cfg.uri = url_.c_str();
    cfg.reconnect_timeout_ms = app_cfg::WS_RECONNECT_TIMEOUT;

    client_ = esp_websocket_client_init(&cfg);
    if (!client_) {
        ESP_LOGE(TAG, "Init failed");
        return;
    }

    esp_websocket_register_events(client_, WEBSOCKET_EVENT_ANY,
                                  &WebSocketClient::eventHandler, this);

    if (status_cb_) status_cb_(1);  // connecting
    esp_websocket_client_start(client_);
    ESP_LOGI(TAG, "Connecting to %s", url_.c_str());
}

void WebSocketClient::close() {
    if (!client_) return;
    connected_ = false;
    esp_websocket_client_close(client_, pdMS_TO_TICKS(2000));
    esp_websocket_client_destroy(client_);
    client_ = nullptr;
    if (status_cb_) status_cb_(0);
}

bool WebSocketClient::sendText(const std::string& msg) {
    if (!client_ || !connected_) return false;
    return esp_websocket_client_send_text(client_, msg.c_str(), msg.size(),
                                          pdMS_TO_TICKS(1000)) >= 0;
}

bool WebSocketClient::sendBinary(const uint8_t* data, size_t len) {
    if (!client_ || !connected_) return false;
    return esp_websocket_client_send_bin(client_, (const char*)data, len,
                                         pdMS_TO_TICKS(1000)) >= 0;
}

void WebSocketClient::eventHandler(void* arg, esp_event_base_t base,
                                   int32_t id, void* data) {
    auto* self = static_cast<WebSocketClient*>(arg);
    auto* evt = static_cast<esp_websocket_event_data_t*>(data);

    switch (id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected");
            self->connected_ = true;
            if (self->status_cb_) self->status_cb_(2);  // open
            break;

        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected");
            self->connected_ = false;
            if (self->status_cb_) self->status_cb_(0);  // closed
            break;

        case WEBSOCKET_EVENT_DATA:
            if (evt->op_code == 0x01 && self->text_cb_ && evt->data_len > 0) {
                self->text_cb_(std::string(evt->data_ptr, evt->data_len));
            } else if (evt->op_code == 0x02 && self->binary_cb_ && evt->data_len > 0) {
                self->binary_cb_((const uint8_t*)evt->data_ptr, evt->data_len);
            }
            break;

        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "Error");
            break;

        default:
            break;
    }
}
