#pragma once

#include "system/StateTypes.h"
#include "system/NetworkManager.h"
#include "system/BluetoothService.h"
#include "comm/UartBridge.h"

// ============================================================================
// Central orchestrator - ported from PTalk AppController
// Manages lifecycle: init → start → run → stop
// Wires modules: Network ↔ BLE ↔ UART ↔ StateManager
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif
void app_init();
#ifdef __cplusplus
}
#endif

class AppController {
public:
    static AppController& instance();

    void init();
    void start();
    void stop();

private:
    AppController() = default;
    ~AppController() = default;
    AppController(const AppController&) = delete;
    AppController& operator=(const AppController&) = delete;

    void onConnectivityChanged(ConnectivityState s);
    void notifyStm32(const std::string& msg);

    NetworkManager    network_;
    BluetoothService  ble_;
    UartBridge        uart_;

    bool wifi_failed_ = false;
};
