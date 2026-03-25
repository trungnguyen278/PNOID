#include "StateManager.hpp"
#include "esp_log.h"
#include <algorithm>

static const char *TAG = "StateManager";

StateManager& StateManager::instance() {
    static StateManager inst;
    return inst;
}

// === Interaction ===

void StateManager::setInteractionState(state::InteractionState s, state::InputSource src) {
    std::vector<std::pair<int, InteractionCb>> callbacks;
    {
        std::lock_guard<std::mutex> lk(mtx);
        if (s == interaction_state && src == interaction_source) return;
        ESP_LOGI(TAG, "Interaction: %d -> %d (source=%d)",
                 (int)interaction_state, (int)s, (int)src);
        interaction_state = s;
        interaction_source = src;
        callbacks = interaction_cbs;
    }
    for (auto &p : callbacks) { if (p.second) p.second(s, src); }
}

state::InteractionState StateManager::getInteractionState() {
    std::lock_guard<std::mutex> lk(mtx);
    return interaction_state;
}

state::InputSource StateManager::getInteractionSource() {
    std::lock_guard<std::mutex> lk(mtx);
    return interaction_source;
}

int StateManager::subscribeInteraction(InteractionCb cb) {
    std::lock_guard<std::mutex> lk(mtx);
    int id = next_sub_id++;
    interaction_cbs.emplace_back(id, std::move(cb));
    return id;
}

void StateManager::unsubscribeInteraction(int id) {
    std::lock_guard<std::mutex> lk(mtx);
    interaction_cbs.erase(
        std::remove_if(interaction_cbs.begin(), interaction_cbs.end(),
            [id](auto &p){ return p.first == id; }),
        interaction_cbs.end());
}

// === Connectivity ===

void StateManager::setConnectivityState(state::ConnectivityState s) {
    std::vector<std::pair<int, ConnectivityCb>> callbacks;
    {
        std::lock_guard<std::mutex> lk(mtx);
        if (s == connectivity_state) return;
        ESP_LOGI(TAG, "Connectivity: %d -> %d", (int)connectivity_state, (int)s);
        connectivity_state = s;
        callbacks = connectivity_cbs;
    }
    for (auto &p : callbacks) { if (p.second) p.second(s); }
}

state::ConnectivityState StateManager::getConnectivityState() {
    std::lock_guard<std::mutex> lk(mtx);
    return connectivity_state;
}

int StateManager::subscribeConnectivity(ConnectivityCb cb) {
    std::lock_guard<std::mutex> lk(mtx);
    int id = next_sub_id++;
    connectivity_cbs.emplace_back(id, std::move(cb));
    return id;
}

void StateManager::unsubscribeConnectivity(int id) {
    std::lock_guard<std::mutex> lk(mtx);
    connectivity_cbs.erase(
        std::remove_if(connectivity_cbs.begin(), connectivity_cbs.end(),
            [id](auto &p){ return p.first == id; }),
        connectivity_cbs.end());
}

// === System ===

void StateManager::setSystemState(state::SystemState s) {
    std::vector<std::pair<int, SystemCb>> callbacks;
    {
        std::lock_guard<std::mutex> lk(mtx);
        if (s == system_state) return;
        ESP_LOGI(TAG, "System: %d -> %d", (int)system_state, (int)s);
        system_state = s;
        callbacks = system_cbs;
    }
    for (auto &p : callbacks) { if (p.second) p.second(s); }
}

state::SystemState StateManager::getSystemState() {
    std::lock_guard<std::mutex> lk(mtx);
    return system_state;
}

int StateManager::subscribeSystem(SystemCb cb) {
    std::lock_guard<std::mutex> lk(mtx);
    int id = next_sub_id++;
    system_cbs.emplace_back(id, std::move(cb));
    return id;
}

void StateManager::unsubscribeSystem(int id) {
    std::lock_guard<std::mutex> lk(mtx);
    system_cbs.erase(
        std::remove_if(system_cbs.begin(), system_cbs.end(),
            [id](auto &p){ return p.first == id; }),
        system_cbs.end());
}

// === Emotion ===

void StateManager::setEmotionState(state::EmotionState s) {
    std::vector<std::pair<int, EmotionCb>> callbacks;
    {
        std::lock_guard<std::mutex> lk(mtx);
        emotion_state = s;
        ESP_LOGI(TAG, "Emotion: %d", (int)s);
        callbacks = emotion_cbs;
    }
    for (auto &p : callbacks) { if (p.second) p.second(s); }
}

state::EmotionState StateManager::getEmotionState() {
    std::lock_guard<std::mutex> lk(mtx);
    return emotion_state;
}

int StateManager::subscribeEmotion(EmotionCb cb) {
    std::lock_guard<std::mutex> lk(mtx);
    int id = next_sub_id++;
    emotion_cbs.emplace_back(id, std::move(cb));
    return id;
}

void StateManager::unsubscribeEmotion(int id) {
    std::lock_guard<std::mutex> lk(mtx);
    emotion_cbs.erase(
        std::remove_if(emotion_cbs.begin(), emotion_cbs.end(),
            [id](auto &p){ return p.first == id; }),
        emotion_cbs.end());
}
