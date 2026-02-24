#ifndef ISPI_HPP
#define ISPI_HPP

#include <cstdint>

// ============================================================================
// SPI Interface - Abstract interface for SPI communication
// ============================================================================
class ISPI {
public:
    virtual ~ISPI() = default;
    
    // Transmit data (blocking)
    virtual void transmit(const uint8_t* data, uint16_t size) = 0;
    
    // Transmit 16-bit data array (blocking)
    virtual void transmit16(const uint16_t* data, uint16_t size) = 0;
    
    // Transmit data using DMA (non-blocking)
    virtual void transmitDMA(const uint8_t* data, uint16_t size) = 0;
    
    // Check if SPI is busy
    virtual bool isBusy() const = 0;
    
    // Wait for transmission to complete
    virtual void waitComplete() const {
        while (isBusy()) {}
    }
};

#endif // ISPI_HPP
