#include "system/StateManager.h"
#include "esp_log.h"
#include <algorithm>

static const char* TAG = "StateManager";

// ============================================================================
// toString helpers
// ============================================================================

const char* toString(ConnectivityState s) {
    switch (s) {
        case ConnectivityState::OFFLINE:         return "OFFLINE";
        case ConnectivityState::CONNECTING_WIFI:  return "CONNECTING_WIFI";
        case ConnectivityState::WIFI_PORTAL:      return "WIFI_PORTAL";
        case ConnectivityState::CONFIG_BLE:       return "CONFIG_BLE";
        case ConnectivityState::CONNECTING_WS:    return "CONNECTING_WS";
        case ConnectivityState::ONLINE:           return "ONLINE";
        default:                                  return "UNKNOWN";
    }
}

const char* toString(SystemState s) {
    switch (s) {
        case SystemState::BOOTING:           return "BOOTING";
        case SystemState::RUNNING:           return "RUNNING";
        case SystemState::ERROR:             return "ERROR";
        case SystemState::UPDATING_FIRMWARE: return "UPDATING_FIRMWARE";
        default:                             return "UNKNOWN";
    }
}

// ============================================================================
// StateManager
// ============================================================================

StateManager::StateManager() {
    mutex_ = xSemaphoreCreateMutex();
}

StateManager& StateManager::instance() {
    static StateManager inst;
    return inst;
}

void StateManager::setConnectivity(ConnectivityState s) {
    std::vector<ConnSub> subs;
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        if (connectivity_ == s) { xSemaphoreGive(mutex_); return; }
        ESP_LOGI(TAG, "Connectivity: %s -> %s", toString(connectivity_), toString(s));
        connectivity_ = s;
        subs = conn_subs_;  // copy under lock
        xSemaphoreGive(mutex_);
    }
    // Call callbacks OUTSIDE lock (prevents deadlock, same as PTalk)
    for (auto& sub : subs) {
        sub.cb(s);
    }
}

void StateManager::setSystem(SystemState s) {
    std::vector<SysSub> subs;
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        if (system_ == s) { xSemaphoreGive(mutex_); return; }
        ESP_LOGI(TAG, "System: %s -> %s", toString(system_), toString(s));
        system_ = s;
        subs = sys_subs_;
        xSemaphoreGive(mutex_);
    }
    for (auto& sub : subs) {
        sub.cb(s);
    }
}

ConnectivityState StateManager::getConnectivity() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    auto s = connectivity_;
    xSemaphoreGive(mutex_);
    return s;
}

SystemState StateManager::getSystem() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    auto s = system_;
    xSemaphoreGive(mutex_);
    return s;
}

int StateManager::subscribeConnectivity(ConnectivityCb cb) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    int id = next_id_++;
    conn_subs_.push_back({id, std::move(cb)});
    xSemaphoreGive(mutex_);
    return id;
}

int StateManager::subscribeSystem(SystemCb cb) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    int id = next_id_++;
    sys_subs_.push_back({id, std::move(cb)});
    xSemaphoreGive(mutex_);
    return id;
}

void StateManager::unsubscribeConnectivity(int id) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    conn_subs_.erase(
        std::remove_if(conn_subs_.begin(), conn_subs_.end(),
            [id](const ConnSub& s) { return s.id == id; }),
        conn_subs_.end());
    xSemaphoreGive(mutex_);
}

void StateManager::unsubscribeSystem(int id) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    sys_subs_.erase(
        std::remove_if(sys_subs_.begin(), sys_subs_.end(),
            [id](const SysSub& s) { return s.id == id; }),
        sys_subs_.end());
    xSemaphoreGive(mutex_);
}
