#pragma once
#include <cstdint>

namespace state
{

    // ---------- Interaction (User -> Voice UX / Input trigger) ----------
    enum class InteractionState : uint8_t
    {
        IDLE,       // ready, nothing active
        TRIGGERED,  // input detected (button)
        LISTENING,  // mic capturing + upstream enabled
        PROCESSING, // waiting server AI / LLM / ASR / TTS
        SPEAKING,   // speaker output
        CANCELLING, // cancel by user / timeout
        MUTED,      // input disabled (privacy mode)
        SLEEPING    // system UX off but alive
    };

    // ---------- Connectivity (WiFi / Websocket) ----------
    enum class ConnectivityState : uint8_t
    {
        OFFLINE,         // No network
        CONNECTING_WIFI, // Trying to connect to WiFi
        WIFI_CONNECTED,  // WiFi connected, WS not yet
        CONNECTING_WS,   // Connecting to WebSocket server
        ONLINE           // Connected and operational
    };

    // ---------- System ----------
    enum class SystemState : uint8_t
    {
        BOOTING,
        RUNNING,
        ERROR,
        MAINTENANCE
    };

    // ---------- Input Source (what triggered the interaction) ----------
    enum class InputSource : uint8_t
    {
        BUTTON,         // Physical button
        SERVER_COMMAND, // remote trigger
        SYSTEM,         // system-triggered
        UNKNOWN         // fallback
    };

    // ---------- Emotion (forwarded to S3 via UART for display) ----------
    enum class EmotionState : uint8_t
    {
        NEUTRAL,
        HAPPY,
        SAD,
        ANGRY,
        CONFUSED,
        EXCITED,
        CALM,
        THINKING
    };

} // namespace state
