/**
 * @file    audio_in.cpp
 * @brief   High-level audio input implementation
 */

#include "audio_in.hpp"
#include "debug_log.h"
#include <cstring>
#include <cstdlib>

static const char *TAG = "AUDIO_IN";

/* ---------- Constructor -------------------------------------------------- */

AudioIn::AudioIn(I2SIn &i2s) : i2s_(i2s) {}

/* ---------- Public API --------------------------------------------------- */

AudioIn::Status AudioIn::init()
{
    if (i2s_.init() != I2SIn::Status::OK) {
        LOGE(TAG, "I2S_IN driver init failed");
        return Status::ErrInit;
    }

    LOGI(TAG, "Init OK");
    return Status::OK;
}

AudioIn::Status AudioIn::record(int16_t *buf, uint32_t samples, uint32_t timeout)
{
    if (buf == nullptr || samples == 0) return Status::ErrParam;

    uint32_t remaining = samples;
    uint32_t offset = 0;

    while (remaining > 0) {
        uint32_t chunk = (remaining > kBufFrames) ? kBufFrames : remaining;

        /* Receive stereo data: chunk frames = chunk*2 uint16_t values */
        I2SIn::Status st = i2s_.receive(stereoBuf_, chunk * 2, timeout);
        if (st != I2SIn::Status::OK) {
            LOGE(TAG, "record: receive failed at offset %lu", offset);
            return Status::ErrInit;
        }

        /* Extract left channel (even indices) into mono output */
        for (uint32_t i = 0; i < chunk; i++) {
            buf[offset + i] = static_cast<int16_t>(stereoBuf_[i * 2]);
        }

        offset += chunk;
        remaining -= chunk;
    }

    return Status::OK;
}

AudioIn::Status AudioIn::recordDMA(uint16_t *buf, uint16_t samples)
{
    if (buf == nullptr || samples == 0) return Status::ErrParam;

    I2SIn::Status st = i2s_.receiveDMA(buf, samples);
    if (st == I2SIn::Status::ErrBusy) return Status::ErrBusy;
    if (st != I2SIn::Status::OK) {
        LOGE(TAG, "recordDMA failed");
        return Status::ErrInit;
    }

    return Status::OK;
}

AudioIn::Status AudioIn::stop()
{
    I2SIn::Status st = i2s_.stop();
    return (st == I2SIn::Status::OK) ? Status::OK : Status::ErrInit;
}

uint16_t AudioIn::peakLevel(const int16_t *buf, uint32_t samples)
{
    int16_t peak = 0;
    for (uint32_t i = 0; i < samples; i++) {
        int16_t val = (buf[i] < 0) ? -buf[i] : buf[i];
        if (val > peak) peak = val;
    }
    return static_cast<uint16_t>(peak);
}

void AudioIn::applyGain(int16_t *buf, uint32_t samples, uint16_t gainPercent)
{
    for (uint32_t i = 0; i < samples; i++) {
        int32_t s = buf[i];
        s = s * gainPercent / 100;
        /* Clamp to int16 range */
        if (s > 32767)  s = 32767;
        if (s < -32768) s = -32768;
        buf[i] = static_cast<int16_t>(s);
    }
}

bool AudioIn::isRecording() const
{
    return i2s_.isRecording();
}
