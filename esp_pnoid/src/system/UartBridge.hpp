#pragma once
#include <functional>
#include <atomic>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "UartProtocol.hpp"

// UartBridge for ESP32-C5: sends STATUS_UPDATE to S3, receives CONTROL_CMD from S3.
// Uses UART1 at 460800 baud. Lightweight alternative to SPI for control messages.
class UartBridge {
public:
    struct Config {
        gpio_num_t pin_tx;
        gpio_num_t pin_rx;
        uart_port_t uart_num = UART_NUM_1;
        int baud_rate = 460800;
    };

    using ControlCmdCb = std::function<void(uart_proto::ControlCmd cmd,
                                             const uint8_t* data, size_t len)>;

    UartBridge() = default;
    ~UartBridge();

    bool init(const Config& cfg);
    void start();
    void stop();

    // Send status update to S3 (non-blocking)
    bool sendStatusUpdate(uint8_t interaction, uint8_t connectivity,
                          uint8_t system_state, uint8_t emotion);

    // Callback for control commands received from S3
    void onControlCmd(ControlCmdCb cb) { control_cb_ = std::move(cb); }

private:
    static void rxTaskEntry(void* arg);
    void rxLoop();

    Config cfg_{};
    std::atomic<bool> started_{false};
    TaskHandle_t rx_task_ = nullptr;
    ControlCmdCb control_cb_;
};
