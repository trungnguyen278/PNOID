#pragma once

#include "esp_err.h"
#include <functional>
#include <string>

// ============================================================================
// BLE GATT provisioning service - ported from PTalk BluetoothService
// Receives WiFi credentials + server URLs via BLE from phone app
// ============================================================================

struct BleConfig {
    std::string ssid;
    std::string password;
    std::string ws_url;
    std::string mqtt_url;
};

class BluetoothService {
public:
    BluetoothService();
    ~BluetoothService();

    esp_err_t init();
    esp_err_t startAdvertising();
    esp_err_t stopAdvertising();
    esp_err_t deinit();

    bool isConnected() const { return connected_; }

    // Called when user saves config via BLE app (like PTalk onConfigComplete)
    void onConfigComplete(std::function<void(const BleConfig&)> cb) {
        config_cb_ = std::move(cb);
    }

    // NimBLE callbacks (must be public for C struct initializers)
    static int gapEventHandler(struct ble_gap_event* event, void* arg);
    static int gattAccessHandler(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt* ctxt, void* arg);

private:
    static void onSync();
    static void onReset(int reason);
    static void hostTask(void* param);
    void beginAdvertising();

    bool initialized_ = false;
    bool connected_    = false;
    uint16_t conn_handle_ = 0xFFFF;

    BleConfig pending_cfg_;
    std::function<void(const BleConfig&)> config_cb_;
};
