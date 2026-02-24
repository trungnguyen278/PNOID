#ifndef SOFT_QSPI_HPP
#define SOFT_QSPI_HPP

#include "../Interface/IQSPI.hpp"
#include "../Common/GPIOPin.hpp"
#include "stm32f4xx_hal.h"

// ============================================================================
// Software QSPI Implementation (Bit-banging)
// For MCUs without hardware QSPI (like STM32F411)
// ============================================================================

struct SoftQSPI_Pins {
    GPIOPin clk;    // Clock
    GPIOPin cs;     // Chip Select
    GPIOPin d0;     // Data 0 (MOSI in SPI mode)
    GPIOPin d1;     // Data 1 (MISO in SPI mode)
    GPIOPin d2;     // Data 2 (WP in SPI mode)
    GPIOPin d3;     // Data 3 (HOLD in SPI mode)
    
    SoftQSPI_Pins() = default;
};

class SoftQSPI : public IQSPI {
private:
    SoftQSPI_Pins pins_;
    uint32_t delayNs_;
    
    void delay() const {
        // Simple delay - adjust based on your clock speed
        for (volatile uint32_t i = 0; i < delayNs_; i++) { __NOP(); }
    }
    
    void clockPulse() {
        pins_.clk.high();
        delay();
        pins_.clk.low();
        delay();
    }
    
    void selectCS() { pins_.cs.low(); }
    void deselectCS() { pins_.cs.high(); }
    
    // Send 1 byte in single line mode (standard SPI)
    void sendByte1Line(uint8_t byte) {
        for (int i = 7; i >= 0; i--) {
            if (byte & (1 << i)) {
                pins_.d0.high();
            } else {
                pins_.d0.low();
            }
            clockPulse();
        }
    }
    
    // Send 1 byte in quad mode (4 lines)
    void sendByte4Lines(uint8_t byte) {
        // High nibble first
        pins_.d0.write(byte & 0x10);
        pins_.d1.write(byte & 0x20);
        pins_.d2.write(byte & 0x40);
        pins_.d3.write(byte & 0x80);
        clockPulse();
        
        // Low nibble
        pins_.d0.write(byte & 0x01);
        pins_.d1.write(byte & 0x02);
        pins_.d2.write(byte & 0x04);
        pins_.d3.write(byte & 0x08);
        clockPulse();
    }
    
    // Send 1 byte in dual mode (2 lines)
    void sendByte2Lines(uint8_t byte) {
        // 4 clock cycles for 8 bits (2 bits per cycle)
        for (int i = 3; i >= 0; i--) {
            uint8_t twoBits = (byte >> (i * 2)) & 0x03;
            pins_.d0.write(twoBits & 0x01);
            pins_.d1.write(twoBits & 0x02);
            clockPulse();
        }
    }
    
    void sendByte(uint8_t byte, QSPIMode mode) {
        switch (mode) {
            case QSPIMode::Mode_1Line:
                sendByte1Line(byte);
                break;
            case QSPIMode::Mode_2Lines:
                sendByte2Lines(byte);
                break;
            case QSPIMode::Mode_4Lines:
                sendByte4Lines(byte);
                break;
        }
    }
    
    void setDataPinsOutput() {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        
        if (pins_.d0.isValid()) {
            GPIO_InitStruct.Pin = pins_.d0.pin;
            HAL_GPIO_Init(pins_.d0.port, &GPIO_InitStruct);
        }
        if (pins_.d1.isValid()) {
            GPIO_InitStruct.Pin = pins_.d1.pin;
            HAL_GPIO_Init(pins_.d1.port, &GPIO_InitStruct);
        }
        if (pins_.d2.isValid()) {
            GPIO_InitStruct.Pin = pins_.d2.pin;
            HAL_GPIO_Init(pins_.d2.port, &GPIO_InitStruct);
        }
        if (pins_.d3.isValid()) {
            GPIO_InitStruct.Pin = pins_.d3.pin;
            HAL_GPIO_Init(pins_.d3.port, &GPIO_InitStruct);
        }
    }
    
    void setDataPinsInput() {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        
        if (pins_.d0.isValid()) {
            GPIO_InitStruct.Pin = pins_.d0.pin;
            HAL_GPIO_Init(pins_.d0.port, &GPIO_InitStruct);
        }
        if (pins_.d1.isValid()) {
            GPIO_InitStruct.Pin = pins_.d1.pin;
            HAL_GPIO_Init(pins_.d1.port, &GPIO_InitStruct);
        }
        if (pins_.d2.isValid()) {
            GPIO_InitStruct.Pin = pins_.d2.pin;
            HAL_GPIO_Init(pins_.d2.port, &GPIO_InitStruct);
        }
        if (pins_.d3.isValid()) {
            GPIO_InitStruct.Pin = pins_.d3.pin;
            HAL_GPIO_Init(pins_.d3.port, &GPIO_InitStruct);
        }
    }
    
public:
    SoftQSPI(const SoftQSPI_Pins& pins, uint32_t delayNs = 1)
        : pins_(pins), delayNs_(delayNs) {}
    
    void init() {
        pins_.cs.high();
        pins_.clk.low();
        setDataPinsOutput();
    }
    
    void sendCommand(uint8_t cmd) override {
        selectCS();
        sendByte1Line(cmd);
        deselectCS();
    }
    
    void sendCommand(const QSPICommand& cmd, const uint8_t* data, uint32_t size) override {
        selectCS();
        
        // Send instruction
        sendByte(cmd.instruction, cmd.instructionMode);
        
        // Send address if present
        if (cmd.hasAddress && cmd.addressSize > 0) {
            for (int i = cmd.addressSize - 1; i >= 0; i--) {
                sendByte((cmd.address >> (i * 8)) & 0xFF, cmd.addressMode);
            }
        }
        
        // Dummy cycles
        for (uint8_t i = 0; i < cmd.dummyCycles; i++) {
            clockPulse();
        }
        
        // Send data
        if (cmd.hasData && data != nullptr && size > 0) {
            setDataPinsOutput();
            for (uint32_t i = 0; i < size; i++) {
                sendByte(data[i], cmd.dataMode);
            }
        }
        
        deselectCS();
    }
    
    void receiveCommand(const QSPICommand& cmd, uint8_t* data, uint32_t size) override {
        selectCS();
        
        // Send instruction
        sendByte(cmd.instruction, cmd.instructionMode);
        
        // Send address if present
        if (cmd.hasAddress && cmd.addressSize > 0) {
            for (int i = cmd.addressSize - 1; i >= 0; i--) {
                sendByte((cmd.address >> (i * 8)) & 0xFF, cmd.addressMode);
            }
        }
        
        // Dummy cycles
        for (uint8_t i = 0; i < cmd.dummyCycles; i++) {
            clockPulse();
        }
        
        // Receive data (simplified - single line mode)
        setDataPinsInput();
        for (uint32_t i = 0; i < size; i++) {
            data[i] = 0;
            for (int bit = 7; bit >= 0; bit--) {
                pins_.clk.high();
                delay();
                if (pins_.d1.read()) {  // D1 is MISO
                    data[i] |= (1 << bit);
                }
                pins_.clk.low();
                delay();
            }
        }
        setDataPinsOutput();
        
        deselectCS();
    }
    
    void transmit(const uint8_t* data, uint32_t size, QSPIMode mode = QSPIMode::Mode_4Lines) override {
        setDataPinsOutput();
        for (uint32_t i = 0; i < size; i++) {
            sendByte(data[i], mode);
        }
    }
    
    void transmit16(const uint16_t* data, uint32_t size, QSPIMode mode = QSPIMode::Mode_4Lines) override {
        setDataPinsOutput();
        for (uint32_t i = 0; i < size; i++) {
            sendByte(data[i] >> 8, mode);    // High byte
            sendByte(data[i] & 0xFF, mode);  // Low byte
        }
    }
    
    bool isBusy() const override {
        return false;  // Software implementation is always synchronous
    }
    
    void setDelay(uint32_t delayNs) { delayNs_ = delayNs; }
};

#endif // SOFT_QSPI_HPP
