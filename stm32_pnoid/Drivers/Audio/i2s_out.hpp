/**
 * @file    i2s_out.hpp
 * @brief   Low-level I2S transmit driver for PCM5102 DAC (C++ OOP)
 * @note    Uses I2S1 (SPI1) full-duplex master mode, TX via DMA1_Stream1
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class I2SOut {
public:
    enum class Status {
        OK = 0,
        ErrInit,
        ErrBusy,
        ErrDMA,
        ErrTimeout,
    };

    using Callback = void (*)(void);

    explicit I2SOut(I2S_HandleTypeDef &hi2s);

    Status init();

    Status transmit(const uint16_t *data, uint16_t samples, uint32_t timeout);
    Status transmitDMA(const uint16_t *data, uint16_t samples);
    Status stop();
    Status pause();
    Status resume();

    Status setSampleRate(uint32_t freq);

    bool isPlaying() const;

    void txCompleteHandler();
    void txHalfCompleteHandler();
    void registerCallback(Callback cb) { cb_ = cb; }

private:
    I2S_HandleTypeDef &hi2s_;
    Callback cb_ = nullptr;
    volatile bool playing_ = false;
};
