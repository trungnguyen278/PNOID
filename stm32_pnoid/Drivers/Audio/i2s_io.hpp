/**
 * @file    i2s_io.hpp
 * @brief   Full-duplex I2S driver for PCM5102 DAC + I2S Microphone
 * @note    Uses I2S1 (SPI1) full-duplex master mode.
 *          TX: PB5 (SDO) → PCM5102 DAC
 *          RX: PB4 (SDI) ← I2S Mic
 *          CK: PB3, WS: PA15
 *
 *          Blocking: HAL_I2SEx_TransmitReceive (TX+RX simultaneously)
 *          DMA:      HAL_I2SEx_TransmitReceive_DMA (requires both TX+RX DMA)
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class I2SIO {
public:
    enum class Status {
        OK = 0,
        ErrInit,
        ErrBusy,
        ErrDMA,
        ErrTimeout,
    };

    using Callback = void (*)(void);

    explicit I2SIO(I2S_HandleTypeDef &hi2s);

    Status init();

    /**
     * @brief  Full-duplex blocking transfer: TX + RX simultaneously
     * @param  txData  Data to transmit (or nullptr → sends zeros/silence)
     * @param  rxData  Buffer to receive into (or nullptr → discard RX)
     * @param  samples Number of 16-bit samples to transfer
     * @param  timeout HAL timeout in ms
     */
    Status transmitReceive(const uint16_t *txData, uint16_t *rxData,
                           uint16_t samples, uint32_t timeout);

    /**
     * @brief  TX-only blocking (sends data on SDO, discards RX)
     */
    Status transmit(const uint16_t *data, uint16_t samples, uint32_t timeout);

    /**
     * @brief  RX-only blocking (sends zeros on SDO, captures RX)
     */
    Status receive(uint16_t *data, uint16_t samples, uint32_t timeout);

    Status stop();
    Status pause();
    Status resume();

    Status setSampleRate(uint32_t freq);

    bool isBusy() const;

    void txRxCompleteHandler();
    void txRxHalfCompleteHandler();
    void registerCallback(Callback cb) { cb_ = cb; }

private:
    I2S_HandleTypeDef &hi2s_;
    Callback cb_ = nullptr;
    volatile bool busy_ = false;

    /* Internal silence/discard buffers for one-direction transfers */
    static constexpr uint16_t kBufSize = 1024;
    static uint16_t silenceBuf_[kBufSize]; /* zeros for TX when RX-only */
    static uint16_t discardBuf_[kBufSize]; /* throw-away for RX when TX-only */
};
