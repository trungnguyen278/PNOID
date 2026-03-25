#pragma once
#include <cstdint>
#include <cstddef>

// =============================================================================
// UART Protocol between ESP32-C5 and ESP32-S3
// Variable-length frames for control/status messages.
// Frame: [0x55 start][type:1][len:1][payload:0-250][crc8:1]
//
// STATUS_UPDATE: C5 -> S3 (interaction, connectivity, system_state, emotion)
// CONTROL_CMD:   S3 -> C5 (START, END, SET_VOLUME, REBOOT, config...)
// =============================================================================

namespace uart_proto {

static constexpr uint8_t FRAME_START   = 0x55;
static constexpr size_t  MAX_PAYLOAD   = 250;
static constexpr size_t  HEADER_SIZE   = 3;   // start(1) + type(1) + len(1)
static constexpr size_t  FOOTER_SIZE   = 1;   // crc8(1)
static constexpr size_t  MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD + FOOTER_SIZE;

// Message types
enum class MsgType : uint8_t {
    STATUS_UPDATE  = 0x01,  // C5 -> S3: state snapshot
    CONTROL_CMD    = 0x02,  // S3 -> C5: command
};

// Control command sub-types (same as SPI)
enum class ControlCmd : uint8_t {
    START          = 0x01,
    END            = 0x02,
    SET_VOLUME     = 0x03,
    REBOOT         = 0x04,
    WIFI_CONFIG    = 0x10,
    MQTT_CONFIG    = 0x11,
    WS_CONFIG      = 0x12,
};

// Status payload (same as SPI)
struct StatusPayload {
    uint8_t interaction;
    uint8_t connectivity;
    uint8_t system_state;
    uint8_t emotion;
};

// CRC-8 (XOR)
inline uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) crc ^= data[i];
    return crc;
}

// Build a variable-length frame into `out`. Returns total frame size.
// out must have at least HEADER_SIZE + payload_len + FOOTER_SIZE bytes.
inline size_t buildFrame(uint8_t* out, MsgType type,
                         const uint8_t* payload, uint8_t payload_len)
{
    if (payload_len > MAX_PAYLOAD) return 0;

    out[0] = FRAME_START;
    out[1] = (uint8_t)type;
    out[2] = payload_len;

    if (payload_len > 0 && payload) {
        for (uint8_t i = 0; i < payload_len; ++i)
            out[HEADER_SIZE + i] = payload[i];
    }

    // CRC over type + len + payload
    out[HEADER_SIZE + payload_len] = crc8(&out[1], 2 + payload_len);

    return HEADER_SIZE + payload_len + FOOTER_SIZE;
}

// Frame parser state machine
class FrameParser {
public:
    // Feed one byte. Returns true when a complete valid frame is ready.
    bool feed(uint8_t byte) {
        switch (state_) {
        case State::WAIT_START:
            if (byte == FRAME_START) {
                state_ = State::WAIT_TYPE;
            }
            break;

        case State::WAIT_TYPE:
            type_ = byte;
            state_ = State::WAIT_LEN;
            break;

        case State::WAIT_LEN:
            len_ = byte;
            if (len_ > MAX_PAYLOAD) {
                state_ = State::WAIT_START;
                break;
            }
            idx_ = 0;
            state_ = (len_ > 0) ? State::PAYLOAD : State::WAIT_CRC;
            break;

        case State::PAYLOAD:
            payload_[idx_++] = byte;
            if (idx_ >= len_) {
                state_ = State::WAIT_CRC;
            }
            break;

        case State::WAIT_CRC: {
            state_ = State::WAIT_START;
            // Verify CRC: compute over type + len + payload
            uint8_t buf[2 + MAX_PAYLOAD];
            buf[0] = type_;
            buf[1] = len_;
            for (uint8_t i = 0; i < len_; ++i) buf[2 + i] = payload_[i];
            if (crc8(buf, 2 + len_) == byte) {
                return true;  // Frame ready
            }
            break;
        }
        }
        return false;
    }

    uint8_t       getType()       const { return type_; }
    uint8_t       getPayloadLen() const { return len_; }
    const uint8_t* getPayload()   const { return payload_; }

private:
    enum class State : uint8_t {
        WAIT_START, WAIT_TYPE, WAIT_LEN, PAYLOAD, WAIT_CRC
    };

    State   state_ = State::WAIT_START;
    uint8_t type_  = 0;
    uint8_t len_   = 0;
    uint8_t idx_   = 0;
    uint8_t payload_[MAX_PAYLOAD] = {};
};

} // namespace uart_proto
