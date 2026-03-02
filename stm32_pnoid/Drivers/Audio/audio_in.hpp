/**
 * @file    audio_in.hpp
 * @brief   High-level audio input (record, peak level, gain) for I2S mic
 * @note    Wraps I2SIn for easy microphone capture via I2S1 SDI (PB4)
 */

#pragma once

#include "i2s_in.hpp"
#include <cstdint>

class AudioIn {
public:
    enum class Status {
        OK = 0,
        ErrInit,
        ErrNoDriver,
        ErrParam,
        ErrBusy,
    };

    explicit AudioIn(I2SIn &i2s);

    Status init();

    /**
     * @brief  Record mono PCM into buffer (blocking)
     * @param  buf      Output buffer for 16-bit signed PCM samples
     * @param  samples  Number of mono samples to capture
     * @param  timeout  HAL timeout in ms
     * @return Status
     * @note   I2S delivers stereo frames (L+R). This extracts the left channel.
     */
    Status record(int16_t *buf, uint32_t samples, uint32_t timeout = HAL_MAX_DELAY);

    /**
     * @brief  Start continuous DMA recording into a stereo buffer
     * @param  buf      User-provided stereo buffer (L+R interleaved, uint16_t)
     * @param  samples  Total number of uint16_t values (frames * 2)
     * @return Status
     * @note   Use registerCallback on I2SIn for half/complete notifications.
     */
    Status recordDMA(uint16_t *buf, uint16_t samples);

    Status stop();

    /**
     * @brief  Compute peak absolute amplitude from signed 16-bit PCM buffer
     */
    static uint16_t peakLevel(const int16_t *buf, uint32_t samples);

    /**
     * @brief  Apply software gain to a PCM buffer in-place
     * @param  gainPercent  100 = unity, 200 = 2x, 50 = 0.5x
     */
    static void applyGain(int16_t *buf, uint32_t samples, uint16_t gainPercent);

    bool isRecording() const;

private:
    I2SIn &i2s_;

    /* Internal stereo capture buffer for blocking record() */
    static constexpr uint32_t kBufFrames = 512;
    uint16_t stereoBuf_[kBufFrames * 2]; /* L+R interleaved */
};
