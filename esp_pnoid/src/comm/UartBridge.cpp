#include "comm/UartBridge.h"
#include "config/AppConfig.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "UartBridge";

UartBridge::UartBridge() {}
UartBridge::~UartBridge() {}

void UartBridge::init() {
    if (initialized_) return;

    uart_config_t cfg = {
        .baud_rate  = app_cfg::UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(app_cfg::UART_PORT,
                    app_cfg::UART_BUF_SIZE * 2, app_cfg::UART_BUF_SIZE * 2,
                    20, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(app_cfg::UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(app_cfg::UART_PORT,
                    app_cfg::UART_TX_PIN, app_cfg::UART_RX_PIN,
                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    initialized_ = true;
    ESP_LOGI(TAG, "UART initialized (port %d, baud %d)", app_cfg::UART_PORT, app_cfg::UART_BAUD);
}

void UartBridge::startReceiveTask() {
    xTaskCreate(UartBridge::receiveTask, "uart_rx",
                app_cfg::TASK_STACK_DEFAULT, this,
                app_cfg::TASK_PRIO_DEFAULT, nullptr);
}

void UartBridge::send(const uint8_t* data, size_t len) {
    if (!initialized_) return;
    uart_write_bytes(app_cfg::UART_PORT, data, len);
}

void UartBridge::sendString(const std::string& str) {
    send((const uint8_t*)str.c_str(), str.size());
}

void UartBridge::receiveTask(void* param) {
    auto* self = static_cast<UartBridge*>(param);
    uint8_t buf[app_cfg::UART_BUF_SIZE];

    ESP_LOGI(TAG, "Receive task started");
    while (true) {
        int len = uart_read_bytes(app_cfg::UART_PORT, buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(100));
        if (len > 0 && self->recv_cb_) {
            self->recv_cb_(buf, len);
        }
    }
}
