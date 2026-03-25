#pragma once
#include <functional>
#include <vector>
#include <mutex>
#include "StateTypes.hpp"

// Central state hub for ESP32-C5.
// Manages interaction, connectivity, system, and emotion state.
// Power state is managed by S3 and received via UART.
class StateManager {
public:
    using InteractionCb  = std::function<void(state::InteractionState, state::InputSource)>;
    using ConnectivityCb = std::function<void(state::ConnectivityState)>;
    using SystemCb       = std::function<void(state::SystemState)>;
    using EmotionCb      = std::function<void(state::EmotionState)>;

    static StateManager& instance();

    // Setters
    void setInteractionState(state::InteractionState s, state::InputSource src = state::InputSource::UNKNOWN);
    void setConnectivityState(state::ConnectivityState s);
    void setSystemState(state::SystemState s);
    void setEmotionState(state::EmotionState s);

    // Getters
    state::InteractionState  getInteractionState();
    state::InputSource       getInteractionSource();
    state::ConnectivityState getConnectivityState();
    state::SystemState       getSystemState();
    state::EmotionState      getEmotionState();

    // Subscribe (returns subscription id)
    int subscribeInteraction(InteractionCb cb);
    int subscribeConnectivity(ConnectivityCb cb);
    int subscribeSystem(SystemCb cb);
    int subscribeEmotion(EmotionCb cb);

    // Unsubscribe
    void unsubscribeInteraction(int id);
    void unsubscribeConnectivity(int id);
    void unsubscribeSystem(int id);
    void unsubscribeEmotion(int id);

private:
    StateManager() = default;
    StateManager(const StateManager&) = delete;
    StateManager& operator=(const StateManager&) = delete;

    std::mutex mtx;

    state::InteractionState  interaction_state  = state::InteractionState::IDLE;
    state::InputSource       interaction_source = state::InputSource::UNKNOWN;
    state::ConnectivityState connectivity_state = state::ConnectivityState::OFFLINE;
    state::SystemState       system_state       = state::SystemState::BOOTING;
    state::EmotionState      emotion_state      = state::EmotionState::NEUTRAL;

    int next_sub_id = 1;
    std::vector<std::pair<int, InteractionCb>>  interaction_cbs;
    std::vector<std::pair<int, ConnectivityCb>> connectivity_cbs;
    std::vector<std::pair<int, SystemCb>>       system_cbs;
    std::vector<std::pair<int, EmotionCb>>      emotion_cbs;
};
