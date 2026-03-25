#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

// =============================================================================
// SPI Protocol between ESP32-S3 (master) and ESP32-C5 (slave)
// Fixed 256-byte full-duplex transactions.
// Frame: [0xA5 magic][type:1][len:2 LE][payload:250][seq:1][crc8:1]
// =============================================================================

namespace spi_proto {

static constexpr uint8_t FRAME_MAGIC   = 0xA5;
static constexpr size_t  FRAME_SIZE    = 256;   // Fixed transaction size
static constexpr size_t  HEADER_SIZE   = 4;     // magic(1) + type(1) + len(2)
static constexpr size_t  FOOTER_SIZE   = 2;     // seq(1) + crc8(1)
static constexpr size_t  MAX_PAYLOAD   = FRAME_SIZE - HEADER_SIZE - FOOTER_SIZE; // 250

// --- Message types: S3 -> C5 (MOSI) ---
enum class MsgFromS3 : uint8_t {
    AUDIO_UPLINK   = 0x01,  // Opus frames from mic
    CONTROL_CMD    = 0x02,  // START, END, config, volume
    HEARTBEAT      = 0x03,  // Poll / keep-alive
};

// --- Message types: C5 -> S3 (MISO) ---
enum class MsgFromC5 : uint8_t {
    EMPTY          = 0x00,  // No data
    AUDIO_DOWNLINK = 0x81,  // Opus frames from server
    STATUS_UPDATE  = 0x82,  // Connectivity, emotion state
};

// --- Control command sub-types ---
enum class ControlCmd : uint8_t {
    START          = 0x01,  // Begin listening
    END            = 0x02,  // Stop listening
    SET_VOLUME     = 0x03,  // +1 byte volume (0-100)
    REBOOT         = 0x04,
    WIFI_CONFIG    = 0x10,  // {ssid_len:1, ssid:N, pass_len:1, pass:N}
    MQTT_CONFIG    = 0x11,  // {uri_len:1, uri:N, user_len:1, user:N, pass_len:1, pass:N}
    WS_CONFIG      = 0x12,  // {url_len:1, url:N}
};

// --- Status update fields ---
struct StatusPayload {
    uint8_t interaction;    // InteractionState enum
    uint8_t connectivity;   // ConnectivityState enum
    uint8_t system_state;   // SystemState enum
    uint8_t emotion;        // EmotionState enum
};

// CRC-8 (simple XOR, same as UART protocol)
inline uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
    }
    return crc;
}

// Build a 256-byte frame. Payload is zero-padded to fill MAX_PAYLOAD.
// Returns true on success.
inline bool buildFrame(uint8_t* out, uint8_t msg_type,
                       const uint8_t* payload, uint16_t payload_len,
                       uint8_t seq)
{
    if (payload_len > MAX_PAYLOAD) return false;

    memset(out, 0, FRAME_SIZE);

    out[0] = FRAME_MAGIC;
    out[1] = msg_type;
    out[2] = (uint8_t)(payload_len & 0xFF);
    out[3] = (uint8_t)((payload_len >> 8) & 0xFF);

    if (payload_len > 0 && payload) {
        memcpy(&out[HEADER_SIZE], payload, payload_len);
    }

    out[FRAME_SIZE - 2] = seq;
    // CRC over type + len + payload area + seq (everything except magic and crc)
    out[FRAME_SIZE - 1] = crc8(&out[1], FRAME_SIZE - 2);

    return true;
}

// Parse a 256-byte frame. Returns true if valid (magic + CRC match).
inline bool parseFrame(const uint8_t* in, uint8_t& msg_type,
                       const uint8_t*& payload, uint16_t& payload_len,
                       uint8_t& seq)
{
    if (in[0] != FRAME_MAGIC) return false;

    uint8_t expected_crc = crc8(&in[1], FRAME_SIZE - 2);
    if (in[FRAME_SIZE - 1] != expected_crc) return false;

    msg_type    = in[1];
    payload_len = in[2] | ((uint16_t)in[3] << 8);
    payload     = &in[HEADER_SIZE];
    seq         = in[FRAME_SIZE - 2];

    if (payload_len > MAX_PAYLOAD) return false;

    return true;
}

// Build an EMPTY frame (no data, type=0x00)
inline void buildEmptyFrame(uint8_t* out, uint8_t seq) {
    buildFrame(out, (uint8_t)MsgFromC5::EMPTY, nullptr, 0, seq);
}

} // namespace spi_proto
