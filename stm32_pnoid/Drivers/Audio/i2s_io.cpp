/**
 * @file    i2s_io.cpp
 * @brief   Full-duplex I2S driver implementation
 */

#include "i2s_io.hpp"
#include "debug_log.h"
#include <cstring>

static const char *TAG = "I2S";

/* ---------- Static buffers ----------------------------------------------- */

uint16_t I2SIO::silenceBuf_[I2SIO::kBufSize];
uint16_t I2SIO::discardBuf_[I2SIO::kBufSize];

/* ---------- Singleton pointer for HAL callbacks -------------------------- */

static I2SIO *g_instance = nullptr;

/* ---------- Constructor -------------------------------------------------- */

I2SIO::I2SIO(I2S_HandleTypeDef &hi2s) : hi2s_(hi2s) {}

/* ---------- Public API --------------------------------------------------- */

I2SIO::Status I2SIO::init()
{
    g_instance = this;

    if (hi2s_.State == HAL_I2S_STATE_RESET) {
        LOGE(TAG, "Handle not initialized (STATE_RESET)");
        return Status::ErrInit;
    }

    /* Verify full-duplex mode */
    if (hi2s_.Init.Mode != I2S_MODE_MASTER_FULLDUPLEX &&
        hi2s_.Init.Mode != I2S_MODE_SLAVE_FULLDUPLEX) {
        LOGW(TAG, "I2S not in full-duplex mode (Mode=0x%08lX)", hi2s_.Init.Mode);
    }

    /* Zero out silence buffer once */
    std::memset(silenceBuf_, 0, sizeof(silenceBuf_));

    bool hasDmaTx = (hi2s_.hdmatx != nullptr);
    bool hasDmaRx = (hi2s_.hdmarx != nullptr);
    LOGI(TAG, "Init OK (AudioFreq=%lu, DMA_TX=%s, DMA_RX=%s)",
         hi2s_.Init.AudioFreq,
         hasDmaTx ? "yes" : "no",
         hasDmaRx ? "yes" : "no");

    return Status::OK;
}

I2SIO::Status I2SIO::transmitReceive(const uint16_t *txData, uint16_t *rxData,
                                      uint16_t samples, uint32_t timeout)
{
    /*
     * Process in chunks of kBufSize when using internal silence/discard buffers.
     * If both txData and rxData are provided, can do the full size in one call.
     */
    const bool needSilence = (txData == nullptr);
    const bool needDiscard = (rxData == nullptr);

    /* Direct path: both buffers provided, no chunking needed */
    if (!needSilence && !needDiscard) {
        busy_ = true;
        HAL_StatusTypeDef st = HAL_I2SEx_TransmitReceive(
            &hi2s_, txData, rxData, samples, timeout);
        busy_ = false;

        if (st == HAL_BUSY)    return Status::ErrBusy;
        if (st == HAL_TIMEOUT) return Status::ErrTimeout;
        if (st != HAL_OK)      return Status::ErrDMA;
        return Status::OK;
    }

    /* Chunked path: need internal buffer for one direction */
    uint16_t remaining = samples;
    const uint16_t *pTx = txData;
    uint16_t *pRx = rxData;

    busy_ = true;

    while (remaining > 0) {
        uint16_t chunk = (remaining > kBufSize) ? kBufSize : remaining;

        const uint16_t *txPtr = needSilence ? silenceBuf_ : pTx;
        uint16_t *rxPtr       = needDiscard ? discardBuf_ : pRx;

        HAL_StatusTypeDef st = HAL_I2SEx_TransmitReceive(
            &hi2s_, txPtr, rxPtr, chunk, timeout);

        if (st != HAL_OK) {
            busy_ = false;
            if (st == HAL_BUSY)    return Status::ErrBusy;
            if (st == HAL_TIMEOUT) return Status::ErrTimeout;
            return Status::ErrDMA;
        }

        if (!needSilence) pTx += chunk;
        if (!needDiscard) pRx += chunk;
        remaining -= chunk;
    }

    busy_ = false;
    return Status::OK;
}

I2SIO::Status I2SIO::transmit(const uint16_t *data, uint16_t samples, uint32_t timeout)
{
    /* TX data + discard RX */
    return transmitReceive(data, nullptr, samples, timeout);
}

I2SIO::Status I2SIO::receive(uint16_t *data, uint16_t samples, uint32_t timeout)
{
    /* Silence on TX + capture RX */
    return transmitReceive(nullptr, data, samples, timeout);
}

I2SIO::Status I2SIO::stop()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAStop(&hi2s_);
    busy_ = false;
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SIO::Status I2SIO::pause()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAPause(&hi2s_);
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SIO::Status I2SIO::resume()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAResume(&hi2s_);
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SIO::Status I2SIO::setSampleRate(uint32_t freq)
{
    if (busy_) stop();

    hi2s_.Init.AudioFreq = freq;
    HAL_StatusTypeDef st = HAL_I2S_Init(&hi2s_);
    if (st != HAL_OK) {
        LOGE(TAG, "setSampleRate(%lu) failed: %d", freq, (int)st);
        return Status::ErrInit;
    }

    LOGI(TAG, "Sample rate changed to %lu", freq);
    return Status::OK;
}

bool I2SIO::isBusy() const
{
    return busy_;
}

void I2SIO::txRxCompleteHandler()
{
    busy_ = false;
    if (cb_) cb_();
}

void I2SIO::txRxHalfCompleteHandler()
{
    /* Reserved for double-buffering */
}

/* ---------- HAL Callbacks (weak overrides) ------------------------------- */

extern "C" {

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_instance) g_instance->txRxCompleteHandler();
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_instance) g_instance->txRxHalfCompleteHandler();
}

/* Keep TX/RX individual callbacks for future single-direction DMA use */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_instance) g_instance->txRxCompleteHandler();
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_instance) g_instance->txRxCompleteHandler();
}

} /* extern "C" */
