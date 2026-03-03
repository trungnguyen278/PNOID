/**
 * @file    i2s_in.cpp
 * @brief   Low-level I2S receive driver implementation
 */

#include "i2s_in.hpp"
#include "debug_log.h"
#include <cstring>

static const char *TAG = "I2S_IN";

/* ---------- Singleton pointer for HAL callbacks -------------------------- */

static I2SIn *g_rxInstance = nullptr;

/* Silence buffer for TX during full-duplex receive (zeros → DAC outputs nothing) */
static uint16_t txSilence[1024];

/* ---------- Constructor -------------------------------------------------- */

I2SIn::I2SIn(I2S_HandleTypeDef &hi2s) : hi2s_(hi2s) {}

/* ---------- Public API --------------------------------------------------- */

I2SIn::Status I2SIn::init()
{
    g_rxInstance = this;

    if (hi2s_.State == HAL_I2S_STATE_RESET) {
        LOGE(TAG, "Handle not initialized (STATE_RESET)");
        return Status::ErrInit;
    }

    /* Verify full-duplex mode */
    if (hi2s_.Init.Mode != I2S_MODE_MASTER_FULLDUPLEX &&
        hi2s_.Init.Mode != I2S_MODE_SLAVE_FULLDUPLEX) {
        LOGW(TAG, "I2S not in full-duplex mode (Mode=0x%08lX)", hi2s_.Init.Mode);
    }

    bool hasDmaRx = (hi2s_.hdmarx != nullptr);
    LOGI(TAG, "Init OK (AudioFreq=%lu, DMA_RX=%s)",
         hi2s_.Init.AudioFreq, hasDmaRx ? "yes" : "no");

    return Status::OK;
}

I2SIn::Status I2SIn::receive(uint16_t *data, uint16_t samples, uint32_t timeout)
{
    /*
     * Full-duplex master: TX must run to generate WS+CK clocks.
     * Use HAL_I2SEx_TransmitReceive — TX sends zeros (silence on DAC),
     * RX captures mic data simultaneously.
     * Process in chunks of sizeof(txSilence)/sizeof(uint16_t) to limit stack.
     */
    const uint16_t chunkMax = sizeof(txSilence) / sizeof(uint16_t);
    uint16_t remaining = samples;
    uint16_t *pRx = data;

    recording_ = true;

    while (remaining > 0) {
        uint16_t chunk = (remaining > chunkMax) ? chunkMax : remaining;

        std::memset(txSilence, 0, chunk * sizeof(uint16_t));
        HAL_StatusTypeDef st = HAL_I2SEx_TransmitReceive(&hi2s_,
                                                          txSilence, pRx,
                                                          chunk, timeout);
        if (st != HAL_OK) {
            recording_ = false;
            if (st == HAL_BUSY)    return Status::ErrBusy;
            if (st == HAL_TIMEOUT) return Status::ErrTimeout;
            return Status::ErrDMA;
        }

        pRx       += chunk;
        remaining -= chunk;
    }

    recording_ = false;
    return Status::OK;
}

I2SIn::Status I2SIn::receiveDMA(uint16_t *data, uint16_t samples)
{
    if (hi2s_.hdmarx == nullptr) {
        LOGE(TAG, "RX DMA not configured — add SPI1_RX DMA in CubeMX");
        return Status::ErrDMA;
    }

    HAL_StatusTypeDef st = HAL_I2S_Receive_DMA(&hi2s_, data, samples);
    if (st == HAL_BUSY) return Status::ErrBusy;
    if (st != HAL_OK)   return Status::ErrDMA;

    recording_ = true;
    return Status::OK;
}

I2SIn::Status I2SIn::stop()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAStop(&hi2s_);
    recording_ = false;
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SIn::Status I2SIn::pause()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAPause(&hi2s_);
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SIn::Status I2SIn::resume()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAResume(&hi2s_);
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

bool I2SIn::isRecording() const
{
    return recording_;
}

void I2SIn::rxCompleteHandler()
{
    recording_ = false;
    if (cb_) cb_();
}

void I2SIn::rxHalfCompleteHandler()
{
    /* Reserved for double-buffering — user can override via callback */
}

/* ---------- HAL Callbacks (weak overrides) ------------------------------- */

extern "C" {

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_rxInstance) g_rxInstance->rxCompleteHandler();
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_rxInstance) g_rxInstance->rxHalfCompleteHandler();
}

} /* extern "C" */
