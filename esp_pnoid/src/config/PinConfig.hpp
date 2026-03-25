#pragma once

// =============================================================================
// PNoid V2 - ESP32-C5 Pin Configuration
// =============================================================================
// ESP32-C5 has GPIO0-GPIO14 available.
// Responsibilities: WiFi, WebSocket, MQTT, SPI slave bridge to S3
// Audio, button, and display are now on S3.
// =============================================================================

namespace PinConfig {

// --- SPI Slave (S3=Master, C5=Slave) ---
static constexpr int SPI_MOSI      = 2;   // C5 GPIO2  <- S3 GPIO35  (MOSI)
static constexpr int SPI_MISO      = 3;   // C5 GPIO3  -> S3 GPIO36  (MISO)
static constexpr int SPI_SCLK      = 4;   // C5 GPIO4  <- S3 GPIO37  (SCLK)
static constexpr int SPI_CS        = 5;   // C5 GPIO5  <- S3 GPIO38  (CS)
static constexpr int SPI_HANDSHAKE = 8;   // C5 GPIO8  -> S3 GPIO39  (HIGH = C5 has data)

// --- SPI Configuration ---
static constexpr int SPI_NUM       = 2;   // SPI2

// --- UART Bridge to S3 (control/status messages) ---
static constexpr int UART_TX       = 6;   // C5 GPIO6  -> S3 GPIO17  (RX)
static constexpr int UART_RX       = 7;   // C5 GPIO7  <- S3 GPIO18  (TX)

// --- Free GPIOs ---
// GPIO 9, 10 are available for future use

} // namespace PinConfig
