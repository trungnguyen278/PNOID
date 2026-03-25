#pragma once
#include <memory>
#include <atomic>
#include <cstdint>
#include "system/StateManager.hpp"
#include "system/StateTypes.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace event {
    enum class AppEvent : uint8_t {
        SLEEP_REQUEST,
        WAKE_REQUEST,
        REBOOT_REQUEST
    };
}

class NetworkManager;
class SpiBridge;
class UartBridge;

// AppController for ESP32-C5: network-only orchestrator.
// Audio, button, and display are now on S3. C5 handles WiFi/WS/MQTT and
// relays data to/from S3 via SPI.
class AppController {
public:
    static AppController& instance();

    bool init();
    void start();
    void stop();
    void reboot();

    void postEvent(event::AppEvent evt);

    void attachModules(std::unique_ptr<NetworkManager> networkIn,
                       std::unique_ptr<SpiBridge> spiIn,
                       std::unique_ptr<UartBridge> uartIn);

    static state::EmotionState parseEmotionCode(const std::string& code);

private:
    AppController() = default;
    ~AppController();
    AppController(const AppController&) = delete;
    AppController& operator=(const AppController&) = delete;

    static void controllerTask(void* param);
    void processQueue();

    void onInteractionStateChanged(state::InteractionState, state::InputSource);
    void onConnectivityStateChanged(state::ConnectivityState);
    void onSystemStateChanged(state::SystemState);

    int sub_inter_id = -1;
    int sub_conn_id  = -1;
    int sub_sys_id   = -1;

    std::unique_ptr<NetworkManager> network;
    std::unique_ptr<SpiBridge>      spi;
    std::unique_ptr<UartBridge>     uart;

    QueueHandle_t app_queue = nullptr;
    TaskHandle_t  app_task  = nullptr;
    std::atomic<bool> started{false};
};
