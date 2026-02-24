#pragma once

// ============================================================================
// State enumerations - ported from PTalk StateTypes.hpp
// Simplified for connectivity co-processor role
// ============================================================================

// Network connectivity state (matches PTalk ConnectivityState)
enum class ConnectivityState {
    OFFLINE,          // No WiFi
    CONNECTING_WIFI,  // WiFi connecting
    WIFI_PORTAL,      // Captive portal mode
    CONFIG_BLE,       // BLE provisioning mode
    CONNECTING_WS,    // WebSocket connecting
    ONLINE,           // Fully connected (WiFi + WS + MQTT)
};

// System lifecycle state
enum class SystemState {
    BOOTING,
    RUNNING,
    ERROR,
    UPDATING_FIRMWARE,
};

const char* toString(ConnectivityState s);
const char* toString(SystemState s);
