#pragma once
#include <functional>
#include <atomic>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "SpiProtocol.hpp"

// SpiBridge for ESP32-C5: SPI2 slave communicating with ESP32-S3 master.
// Audio-only: control/status messages now go through UartBridge.
// Sets HANDSHAKE GPIO HIGH when it has audio data to send.
// Blocks on SPI slave receive, returns response in same transaction.
//
// Audio downlink uses a stream buffer so that:
//   - WS callback can push without blocking (no frame building in callback context)
//   - Slave loop reads [len][opus] frames from stream buffer and builds SPI frames
//   - More memory-efficient than queueing pre-built 256-byte frames
class SpiBridge {
public:
    struct Config {
        gpio_num_t pin_mosi;      // SPI MOSI (input from S3)
        gpio_num_t pin_miso;      // SPI MISO (output to S3)
        gpio_num_t pin_sclk;      // SPI SCLK (input from S3)
        gpio_num_t pin_cs;        // SPI CS (input from S3)
        gpio_num_t pin_handshake; // Output to S3 (HIGH = C5 has data)
    };

    using AudioUplinkCb  = std::function<void(const uint8_t* data, size_t len)>;

    SpiBridge() = default;
    ~SpiBridge();

    bool init(const Config& cfg);
    void start();
    void stop();

    // Push audio downlink into stream buffer (non-blocking, called from WS callback)
    // Data must be a complete [2-byte LE len][opus data] frame.
    bool sendAudioDownlink(const uint8_t* data, size_t len);

    // Callback for audio uplink received from S3
    void onAudioUplink(AudioUplinkCb cb)  { audio_ul_cb_ = std::move(cb); }

    // Set query function for WS tx_buffer free space (for flow control in EMPTY frames)
    void setUplinkSpaceQuery(std::function<size_t()> fn) { ul_space_query_ = std::move(fn); }

private:
    static void slaveTaskEntry(void* arg);
    void slaveLoop();

    void prepareTxFrame(uint8_t* tx_buf);
    void handleRxFrame(const uint8_t* rx_buf);

    Config cfg_{};
    std::atomic<bool> started_{false};
    TaskHandle_t slave_task_ = nullptr;

    // Audio downlink: stream buffer for [len][opus] frames from WS
    StreamBufferHandle_t dl_audio_sb_ = nullptr;

    uint8_t tx_seq_ = 0;

    AudioUplinkCb audio_ul_cb_;
    std::function<size_t()> ul_space_query_;  // Returns WS tx_buffer free bytes
};
