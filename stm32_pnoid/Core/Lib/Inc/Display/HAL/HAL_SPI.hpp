#ifndef HAL_SPI_HPP
#define HAL_SPI_HPP

#include "../Interface/ISPI.hpp"
#include "stm32f4xx_hal.h"

// ============================================================================
// HAL SPI Implementation
// ============================================================================
class HAL_SPI : public ISPI {
private:
    SPI_HandleTypeDef* hspi_;
    uint32_t timeout_;
    
public:
    explicit HAL_SPI(SPI_HandleTypeDef* hspi, uint32_t timeout = HAL_MAX_DELAY) 
        : hspi_(hspi), timeout_(timeout) {}
    
    void transmit(const uint8_t* data, uint16_t size) override {
        HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(data), size, timeout_);
    }
    
    void transmit16(const uint16_t* data, uint16_t size) override {
        HAL_SPI_Transmit(hspi_, reinterpret_cast<uint8_t*>(const_cast<uint16_t*>(data)), 
                         size * 2, timeout_);
    }
    
    void transmitDMA(const uint8_t* data, uint16_t size) override {
        HAL_SPI_Transmit_DMA(hspi_, const_cast<uint8_t*>(data), size);
    }
    
    bool isBusy() const override {
        return HAL_SPI_GetState(hspi_) == HAL_SPI_STATE_BUSY_TX;
    }
    
    // Get HAL SPI handle
    SPI_HandleTypeDef* getHandle() const { return hspi_; }
    
    // Set timeout
    void setTimeout(uint32_t timeout) { timeout_ = timeout; }
};

#endif // HAL_SPI_HPP
