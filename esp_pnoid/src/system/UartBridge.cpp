#include "UartBridge.hpp"
#include "esp_log.h"

static const char* TAG = "UartBridge";

UartBridge::~UartBridge()
{
    stop();
}

bool UartBridge::init(const Config& cfg)
{
    cfg_ = cfg;

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = cfg_.baud_rate;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(cfg_.uart_num, 512, 256, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_param_config(cfg_.uart_num, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_set_pin(cfg_.uart_num, cfg_.pin_tx, cfg_.pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "UartBridge init OK (TX=%d RX=%d baud=%d)",
             cfg_.pin_tx, cfg_.pin_rx, cfg_.baud_rate);
    return true;
}

void UartBridge::start()
{
    if (started_) return;
    started_ = true;

    xTaskCreatePinnedToCore(&UartBridge::rxTaskEntry, "UartBridge", 3072, this, 3, &rx_task_, 0);
    ESP_LOGI(TAG, "UartBridge started");
}

void UartBridge::stop()
{
    if (!started_) return;
    started_ = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    rx_task_ = nullptr;
    uart_driver_delete(cfg_.uart_num);
    ESP_LOGI(TAG, "UartBridge stopped");
}

bool UartBridge::sendStatusUpdate(uint8_t interaction, uint8_t connectivity,
                                   uint8_t system_state, uint8_t emotion)
{
    uart_proto::StatusPayload sp;
    sp.interaction   = interaction;
    sp.connectivity  = connectivity;
    sp.system_state  = system_state;
    sp.emotion       = emotion;

    uint8_t frame[uart_proto::MAX_FRAME_SIZE];
    size_t frame_len = uart_proto::buildFrame(frame, uart_proto::MsgType::STATUS_UPDATE,
                                               (const uint8_t*)&sp, sizeof(sp));
    if (frame_len == 0) return false;

    int written = uart_write_bytes(cfg_.uart_num, frame, frame_len);
    return written == (int)frame_len;
}

void UartBridge::rxTaskEntry(void* arg)
{
    static_cast<UartBridge*>(arg)->rxLoop();
}

void UartBridge::rxLoop()
{
    ESP_LOGI(TAG, "RX task started");

    uart_proto::FrameParser parser;
    uint8_t byte;

    while (started_) {
        int len = uart_read_bytes(cfg_.uart_num, &byte, 1, pdMS_TO_TICKS(50));
        if (len <= 0) continue;

        if (parser.feed(byte)) {
            uint8_t type = parser.getType();
            const uint8_t* payload = parser.getPayload();
            uint8_t payload_len = parser.getPayloadLen();

            if (type == (uint8_t)uart_proto::MsgType::CONTROL_CMD && payload_len >= 1) {
                auto cmd = static_cast<uart_proto::ControlCmd>(payload[0]);
                ESP_LOGI(TAG, "RX CONTROL_CMD: cmd=%d len=%d", payload[0], payload_len);
                if (control_cb_) {
                    control_cb_(cmd, &payload[1], payload_len - 1);
                }
            }
        }
    }

    ESP_LOGW(TAG, "RX task ended");
    vTaskDelete(nullptr);
}
