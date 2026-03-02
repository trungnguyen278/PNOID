/**
 * @file    i2s_out.cpp
 * @brief   Low-level I2S transmit driver implementation
 */

#include "i2s_out.hpp"
#include "debug_log.h"

static const char *TAG = "I2S";

/* ---------- Singleton pointer for HAL callbacks -------------------------- */

static I2SOut *g_instance = nullptr;

/* ---------- Constructor -------------------------------------------------- */

I2SOut::I2SOut(I2S_HandleTypeDef &hi2s) : hi2s_(hi2s) {}

/* ---------- Public API --------------------------------------------------- */

I2SOut::Status I2SOut::init()
{
    g_instance = this;

    if (hi2s_.State == HAL_I2S_STATE_RESET) {
        LOGE(TAG, "Handle not initialized (STATE_RESET)");
        return Status::ErrInit;
    }

    LOGI(TAG, "Init OK (Instance=0x%08lX, AudioFreq=%lu)",
         (uint32_t)hi2s_.Instance, hi2s_.Init.AudioFreq);
    return Status::OK;
}

I2SOut::Status I2SOut::transmit(const uint16_t *data, uint16_t samples, uint32_t timeout)
{
    playing_ = true;
    HAL_StatusTypeDef st = HAL_I2S_Transmit(&hi2s_,
                                             const_cast<uint16_t *>(data),
                                             samples, timeout);
    playing_ = false;

    if (st == HAL_BUSY)    return Status::ErrBusy;
    if (st == HAL_TIMEOUT) return Status::ErrTimeout;
    if (st != HAL_OK)      return Status::ErrDMA;

    return Status::OK;
}

I2SOut::Status I2SOut::transmitDMA(const uint16_t *data, uint16_t samples)
{
    HAL_StatusTypeDef st = HAL_I2S_Transmit_DMA(&hi2s_,
                                                 const_cast<uint16_t *>(data),
                                                 samples);
    if (st == HAL_BUSY) return Status::ErrBusy;
    if (st != HAL_OK)   return Status::ErrDMA;

    playing_ = true;
    return Status::OK;
}

I2SOut::Status I2SOut::stop()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAStop(&hi2s_);
    playing_ = false;
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SOut::Status I2SOut::pause()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAPause(&hi2s_);
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SOut::Status I2SOut::resume()
{
    HAL_StatusTypeDef st = HAL_I2S_DMAResume(&hi2s_);
    return (st == HAL_OK) ? Status::OK : Status::ErrDMA;
}

I2SOut::Status I2SOut::setSampleRate(uint32_t freq)
{
    /* Stop any ongoing transfer first */
    if (playing_) stop();

    hi2s_.Init.AudioFreq = freq;
    HAL_StatusTypeDef st = HAL_I2S_Init(&hi2s_);
    if (st != HAL_OK) {
        LOGE(TAG, "setSampleRate(%lu) failed: %d", freq, (int)st);
        return Status::ErrInit;
    }

    LOGI(TAG, "Sample rate changed to %lu", freq);
    return Status::OK;
}

bool I2SOut::isPlaying() const
{
    return playing_;
}

void I2SOut::txCompleteHandler()
{
    playing_ = false;
    if (cb_) cb_();
}

void I2SOut::txHalfCompleteHandler()
{
    /* Reserved for double-buffering — user can override via callback */
}

/* ---------- HAL Callbacks (weak overrides) ------------------------------- */

extern "C" {

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_instance) g_instance->txCompleteHandler();
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (g_instance) g_instance->txHalfCompleteHandler();
}

} /* extern "C" */
