/**
 * @file    audio_out.cpp
 * @brief   High-level audio output implementation
 */

#include "audio_out.hpp"
#include "debug_log.h"
#include <cstring>

static const char *TAG = "AUDIO_OUT";

/* Quarter-wave sine table, 64 entries, amplitude 0..32767 */
const uint16_t AudioOut::kSineTable[64] = {
        0,   804,  1608,  2410,  3212,  4011,  4808,  5602,
     6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
};

/* ---------- Constructor -------------------------------------------------- */

AudioOut::AudioOut(I2SOut &i2s) : i2s_(i2s) {}

/* ---------- Private helpers ---------------------------------------------- */

int16_t AudioOut::sineAt(uint32_t phase) const
{
    /* phase: 0..255 maps to one full period using quarter-wave symmetry */
    uint32_t idx = phase & 0xFF;
    uint32_t quarter = idx >> 6;        /* 0..3 */
    uint32_t offset  = idx & 0x3F;      /* 0..63 */

    uint16_t val;
    switch (quarter) {
    case 0: val = kSineTable[offset];        break;
    case 1: val = kSineTable[63 - offset];   break;
    case 2: val = kSineTable[offset];        break;
    default: val = kSineTable[63 - offset];  break;
    }

    /* Quarters 2,3 are negative */
    return (quarter < 2) ? static_cast<int16_t>(val)
                         : static_cast<int16_t>(-static_cast<int32_t>(val));
}

void AudioOut::applyVolume(int16_t *sample) const
{
    int32_t s = *sample;
    s = s * volume_ / 100;
    *sample = static_cast<int16_t>(s);
}

/* ---------- Public API --------------------------------------------------- */

AudioOut::Status AudioOut::init()
{
    LOGI(TAG, "Init OK (volume=%u%%)", volume_);
    return Status::OK;
}

AudioOut::Status AudioOut::playBuffer(const uint16_t *data, uint16_t samples, bool loop)
{
    (void)loop; /* TODO: implement looping with double-buffer callback */

    I2SOut::Status st = i2s_.transmitDMA(data, samples);
    if (st != I2SOut::Status::OK) {
        LOGE(TAG, "playBuffer DMA failed");
        return Status::ErrInit;
    }

    return Status::OK;
}

AudioOut::Status AudioOut::playTone(uint32_t freqHz, uint32_t durationMs, uint8_t volume)
{
    if (freqHz == 0 || durationMs == 0) return Status::ErrParam;

    const uint32_t sampleRate = 16000; /* matches I2S_AUDIOFREQ_16K */
    const uint32_t totalSamples = sampleRate * durationMs / 1000;

    /* Phase increment per sample: (freqHz * 256) / sampleRate
       256 = full sine period in our lookup */
    const uint32_t phaseInc = (freqHz * 256) / sampleRate;

    uint8_t savedVol = volume_;
    volume_ = volume;

    uint32_t remaining = totalSamples;
    uint32_t phase = 0;

    while (remaining > 0) {
        uint32_t chunk = (remaining > kBufSamples) ? kBufSamples : remaining;

        /* Fill stereo interleaved buffer */
        for (uint32_t i = 0; i < chunk; i++) {
            int16_t s = sineAt(phase);
            applyVolume(&s);
            uint16_t us = static_cast<uint16_t>(s);
            buf_[i * 2]     = us; /* Left */
            buf_[i * 2 + 1] = us; /* Right */
            phase += phaseInc;
        }

        /* Blocking transmit: chunk * 2 because stereo (L+R per sample) */
        I2SOut::Status st = i2s_.transmit(buf_, chunk * 2, HAL_MAX_DELAY);
        if (st != I2SOut::Status::OK) {
            LOGE(TAG, "playTone transmit failed");
            volume_ = savedVol;
            return Status::ErrInit;
        }

        remaining -= chunk;
    }

    volume_ = savedVol;
    return Status::OK;
}

AudioOut::Status AudioOut::silence(uint32_t durationMs)
{
    if (durationMs == 0) return Status::ErrParam;

    const uint32_t sampleRate = 16000;
    uint32_t remaining = sampleRate * durationMs / 1000;

    std::memset(buf_, 0, sizeof(buf_));

    while (remaining > 0) {
        uint32_t chunk = (remaining > kBufSamples) ? kBufSamples : remaining;

        I2SOut::Status st = i2s_.transmit(buf_, chunk * 2, HAL_MAX_DELAY);
        if (st != I2SOut::Status::OK) {
            LOGE(TAG, "silence transmit failed");
            return Status::ErrInit;
        }

        remaining -= chunk;
    }

    return Status::OK;
}

AudioOut::Status AudioOut::stop()
{
    I2SOut::Status st = i2s_.stop();
    return (st == I2SOut::Status::OK) ? Status::OK : Status::ErrInit;
}

void AudioOut::setVolume(uint8_t vol)
{
    volume_ = (vol > 100) ? 100 : vol;
}

bool AudioOut::isPlaying() const
{
    return i2s_.isPlaying();
}
