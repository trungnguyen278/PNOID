#pragma once

#include <cstdint>
#include "driver/uart.h"

// ============================================================================
// PNOID Co-Processor Configuration
// Ported from PTalk - adapted for ESP32-C5 connectivity module
// ============================================================================

namespace app_cfg {

// -- WiFi --
constexpr const char* WIFI_DEFAULT_SSID     = "PNOID_AP";
constexpr const char* WIFI_DEFAULT_PASS     = "pnoid1234";
constexpr int         WIFI_MAX_RETRY        = 5;
constexpr int         WIFI_RETRY_INTERVAL   = 3000;  // ms
constexpr int         WIFI_CONNECT_TIMEOUT  = 15000; // ms
constexpr const char* WIFI_AP_SSID          = "PNOID-Config";

// -- WebSocket --
constexpr const char* WS_DEFAULT_URL        = "ws://192.168.1.100:8080/ws";
constexpr int         WS_RECONNECT_TIMEOUT  = 5000;  // ms

// -- MQTT --
constexpr const char* MQTT_DEFAULT_URI      = "mqtt://broker.hivemq.com";
constexpr const char* MQTT_CLIENT_ID        = "pnoid_esp32c5";
constexpr int         MQTT_KEEPALIVE        = 30;     // sec
constexpr const char* MQTT_TOPIC_PREFIX     = "pnoid/";

// -- BLE --
constexpr const char* BLE_DEVICE_NAME       = "PNOID-Robot";
constexpr uint16_t    BLE_PROV_SVC_UUID     = 0xFF01;
constexpr uint16_t    BLE_CHAR_SSID         = 0xFF05;
constexpr uint16_t    BLE_CHAR_PASS         = 0xFF06;
constexpr uint16_t    BLE_CHAR_WS_URL       = 0xFF0C;
constexpr uint16_t    BLE_CHAR_MQTT_URL     = 0xFF0D;
constexpr uint16_t    BLE_CHAR_SAVE         = 0xFF09;
constexpr uint16_t    BLE_CHAR_DEVICE_NAME  = 0xFF02;

// -- UART (STM32 Bridge) --
constexpr uart_port_t UART_PORT             = UART_NUM_1;
constexpr int         UART_BAUD             = 115200;
constexpr int         UART_TX_PIN           = 4;
constexpr int         UART_RX_PIN           = 5;
constexpr int         UART_BUF_SIZE         = 1024;

// -- NVS --
constexpr const char* NVS_NAMESPACE         = "pnoid_cfg";
constexpr const char* NVS_KEY_SSID          = "wifi_ssid";
constexpr const char* NVS_KEY_PASS          = "wifi_pass";
constexpr const char* NVS_KEY_WS_URL        = "ws_url";
constexpr const char* NVS_KEY_MQTT_URL      = "mqtt_url";

// -- Tasks --
constexpr uint32_t    TASK_STACK_DEFAULT    = 4096;
constexpr UBaseType_t TASK_PRIO_DEFAULT     = 5;

}  // namespace app_cfg
