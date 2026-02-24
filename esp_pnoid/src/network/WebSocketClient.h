#pragma once

#include "esp_websocket_client.h"
#include <functional>
#include <string>

// ============================================================================
// WebSocket client wrapper - ported from PTalk WebSocketClient
// ============================================================================

class WebSocketClient {
public:
    WebSocketClient();
    ~WebSocketClient();

    void init();
    void connect();
    void close();
    bool isConnected() const { return connected_; }

    void setUrl(const std::string& url) { url_ = url; }

    bool sendText(const std::string& msg);
    bool sendBinary(const uint8_t* data, size_t len);

    // Callbacks (same pattern as PTalk)
    void onStatus(std::function<void(int)> cb) { status_cb_ = std::move(cb); }
    void onText(std::function<void(const std::string&)> cb) { text_cb_ = std::move(cb); }
    void onBinary(std::function<void(const uint8_t*, size_t)> cb) { binary_cb_ = std::move(cb); }

private:
    static void eventHandler(void* arg, esp_event_base_t base,
                             int32_t id, void* data);

    esp_websocket_client_handle_t client_ = nullptr;
    std::string url_;
    bool connected_ = false;

    std::function<void(int)> status_cb_;
    std::function<void(const std::string&)> text_cb_;
    std::function<void(const uint8_t*, size_t)> binary_cb_;
};
