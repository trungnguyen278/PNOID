/**
 * @file    i2s_in.hpp
 * @brief   Low-level I2S receive driver for I2S microphone (C++ OOP)
 * @note    Uses I2S1 (SPI1) full-duplex mode, SDI on PB4.
 *          Blocking receive works immediately.
 *          DMA receive requires SPI1_RX DMA stream configured in CubeMX.
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class I2SIn {
public:
    enum class Status {
        OK = 0,
        ErrInit,
        ErrBusy,
        ErrDMA,
        ErrTimeout,
    };

    using Callback = void (*)(void);

    explicit I2SIn(I2S_HandleTypeDef &hi2s);

    Status init();

    Status receive(uint16_t *data, uint16_t samples, uint32_t timeout);
    Status receiveDMA(uint16_t *data, uint16_t samples);
    Status stop();
    Status pause();
    Status resume();

    bool isRecording() const;

    void rxCompleteHandler();
    void rxHalfCompleteHandler();
    void registerCallback(Callback cb) { cb_ = cb; }

private:
    I2S_HandleTypeDef &hi2s_;
    Callback cb_ = nullptr;
    volatile bool recording_ = false;
};
