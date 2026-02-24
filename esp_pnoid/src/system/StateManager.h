#pragma once

#include "system/StateTypes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <functional>
#include <vector>

// ============================================================================
// Thread-safe state hub with pub/sub callbacks
// Pattern ported from PTalk StateManager
// ============================================================================

using ConnectivityCb = std::function<void(ConnectivityState)>;
using SystemCb       = std::function<void(SystemState)>;

class StateManager {
public:
    static StateManager& instance();

    // -- Setters (notify subscribers if changed) --
    void setConnectivity(ConnectivityState s);
    void setSystem(SystemState s);

    // -- Getters --
    ConnectivityState getConnectivity();
    SystemState       getSystem();

    // -- Subscribe (returns ID for unsubscribe) --
    int subscribeConnectivity(ConnectivityCb cb);
    int subscribeSystem(SystemCb cb);

    void unsubscribeConnectivity(int id);
    void unsubscribeSystem(int id);

private:
    StateManager();
    ~StateManager() = default;
    StateManager(const StateManager&) = delete;
    StateManager& operator=(const StateManager&) = delete;

    SemaphoreHandle_t mutex_;

    ConnectivityState connectivity_ = ConnectivityState::OFFLINE;
    SystemState       system_       = SystemState::BOOTING;

    struct ConnSub { int id; ConnectivityCb cb; };
    struct SysSub  { int id; SystemCb cb; };

    std::vector<ConnSub> conn_subs_;
    std::vector<SysSub>  sys_subs_;
    int next_id_ = 1;
};
