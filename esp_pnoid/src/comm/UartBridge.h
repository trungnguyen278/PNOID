#pragma once

#include "driver/uart.h"
#include <functional>
#include <string>
#include <cstdint>

// ============================================================================
// UART bridge to STM32 master
// Forwards server data ↔ STM32 commands
// ============================================================================

class UartBridge {
public:
    UartBridge();
    ~UartBridge();

    void init();
    void startReceiveTask();

    void send(const uint8_t* data, size_t len);
    void sendString(const std::string& str);

    // Callback when data received from STM32
    void onReceive(std::function<void(const uint8_t*, size_t)> cb) {
        recv_cb_ = std::move(cb);
    }

private:
    static void receiveTask(void* param);

    std::function<void(const uint8_t*, size_t)> recv_cb_;
    bool initialized_ = false;
};
