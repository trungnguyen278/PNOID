#include "AppController.hpp"
#include "system/NetworkManager.hpp"
#include "system/SpiBridge.hpp"
#include "system/UartBridge.hpp"

#include "esp_log.h"
#include <utility>

static const char* TAG = "AppController";

struct AppMessage {
    enum class Type : uint8_t {
        INTERACTION,
        CONNECTIVITY,
        SYSTEM,
        APP_EVENT
    } type;

    state::InteractionState  interaction_state;
    state::InputSource       interaction_source;
    state::ConnectivityState connectivity_state;
    state::SystemState       system_state;
    event::AppEvent          app_event;
};

// === Singleton ===

AppController& AppController::instance()
{
    static AppController inst;
    return inst;
}

AppController::~AppController()
{
    stop();
    auto& sm = StateManager::instance();
    if (sub_inter_id != -1) sm.unsubscribeInteraction(sub_inter_id);
    if (sub_conn_id != -1)  sm.unsubscribeConnectivity(sub_conn_id);
    if (sub_sys_id != -1)   sm.unsubscribeSystem(sub_sys_id);
    if (app_queue) { vQueueDelete(app_queue); app_queue = nullptr; }
}

// === Module attachment ===

void AppController::attachModules(std::unique_ptr<NetworkManager> networkIn,
                                   std::unique_ptr<SpiBridge> spiIn,
                                   std::unique_ptr<UartBridge> uartIn)
{
    if (started.load()) return;
    network = std::move(networkIn);
    spi     = std::move(spiIn);
    uart    = std::move(uartIn);
}

// === Lifecycle ===

bool AppController::init()
{
    ESP_LOGI(TAG, "init()");

    app_queue = xQueueCreate(16, sizeof(AppMessage));
    if (!app_queue) {
        ESP_LOGE(TAG, "Failed to create queue");
        return false;
    }

    auto& sm = StateManager::instance();

    sub_inter_id = sm.subscribeInteraction(
        [this](state::InteractionState s, state::InputSource src) {
            AppMessage msg{}; msg.type = AppMessage::Type::INTERACTION;
            msg.interaction_state = s; msg.interaction_source = src;
            if (app_queue) xQueueSend(app_queue, &msg, 0);
        });

    sub_conn_id = sm.subscribeConnectivity(
        [this](state::ConnectivityState s) {
            AppMessage msg{}; msg.type = AppMessage::Type::CONNECTIVITY;
            msg.connectivity_state = s;
            if (app_queue) xQueueSend(app_queue, &msg, 0);
        });

    sub_sys_id = sm.subscribeSystem(
        [this](state::SystemState s) {
            AppMessage msg{}; msg.type = AppMessage::Type::SYSTEM;
            msg.system_state = s;
            if (app_queue) xQueueSend(app_queue, &msg, 0);
        });

    return true;
}

void AppController::start()
{
    if (started.load()) return;
    started.store(true);

    xTaskCreatePinnedToCore(&AppController::controllerTask, "AppCtrl", 4096, this, 4, &app_task, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    if (network) network->start();
    if (spi) spi->start();
    if (uart) uart->start();

    ESP_LOGI(TAG, "AppController started");
}

void AppController::stop()
{
    if (!started.load()) return;
    started.store(false);

    if (uart) uart->stop();
    if (network) network->stop();
    if (spi) spi->stop();

    vTaskDelay(pdMS_TO_TICKS(100));
    app_task = nullptr;
    ESP_LOGI(TAG, "AppController stopped");
}

void AppController::reboot()
{
    ESP_LOGW(TAG, "Reboot requested");
    esp_restart();
}

void AppController::postEvent(event::AppEvent evt)
{
    if (!app_queue) return;
    AppMessage msg{}; msg.type = AppMessage::Type::APP_EVENT; msg.app_event = evt;
    xQueueSend(app_queue, &msg, 0);
}

// === Emotion parsing ===

state::EmotionState AppController::parseEmotionCode(const std::string& code)
{
    return NetworkManager::parseEmotionCode(code);
}

// === Task & Queue ===

void AppController::controllerTask(void* param)
{
    static_cast<AppController*>(param)->processQueue();
}

void AppController::processQueue()
{
    ESP_LOGI(TAG, "Controller task started");
    AppMessage msg{};

    while (started.load()) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY) == pdTRUE) {
            switch (msg.type) {
            case AppMessage::Type::INTERACTION:
                onInteractionStateChanged(msg.interaction_state, msg.interaction_source);
                break;
            case AppMessage::Type::CONNECTIVITY:
                onConnectivityStateChanged(msg.connectivity_state);
                break;
            case AppMessage::Type::SYSTEM:
                onSystemStateChanged(msg.system_state);
                break;
            case AppMessage::Type::APP_EVENT:
                switch (msg.app_event) {
                case event::AppEvent::REBOOT_REQUEST:
                    reboot();
                    break;
                case event::AppEvent::SLEEP_REQUEST:
                case event::AppEvent::WAKE_REQUEST:
                    break;
                }
                break;
            }
        }
    }

    vTaskDelete(nullptr);
}

// === State callbacks ===

void AppController::onInteractionStateChanged(state::InteractionState s, state::InputSource src)
{
    ESP_LOGI(TAG, "Interaction: %d (src=%d)", (int)s, (int)src);

    // Forward state to S3 via UART
    if (uart) {
        uart->sendStatusUpdate(
            (uint8_t)s,
            (uint8_t)StateManager::instance().getConnectivityState(),
            (uint8_t)StateManager::instance().getSystemState(),
            (uint8_t)StateManager::instance().getEmotionState());
    }

    // WS immune mode during SPEAKING
    if (network) {
        network->setWSImmuneMode(s == state::InteractionState::SPEAKING);
    }
}

void AppController::onConnectivityStateChanged(state::ConnectivityState s)
{
    ESP_LOGI(TAG, "Connectivity: %d", (int)s);

    // Forward to S3 via UART
    if (uart) {
        uart->sendStatusUpdate(
            (uint8_t)StateManager::instance().getInteractionState(),
            (uint8_t)s,
            (uint8_t)StateManager::instance().getSystemState(),
            (uint8_t)StateManager::instance().getEmotionState());
    }
}

void AppController::onSystemStateChanged(state::SystemState s)
{
    ESP_LOGI(TAG, "System: %d", (int)s);
}
