/**
 * @file    audio_out.hpp
 * @brief   High-level audio output (tone generation, PCM playback, volume)
 * @note    Wraps I2SOut for easy audio output via PCM5102 DAC
 */

#pragma once

#include "i2s_out.hpp"
#include <cstdint>

class AudioOut {
public:
    enum class Status {
        OK = 0,
        ErrInit,
        ErrNoDriver,
        ErrParam,
    };

    explicit AudioOut(I2SOut &i2s);

    Status init();

    Status playBuffer(const uint16_t *data, uint16_t samples, bool loop = false);
    Status playTone(uint32_t freqHz, uint32_t durationMs, uint8_t volume = 80);
    Status silence(uint32_t durationMs);
    Status stop();

    void   setVolume(uint8_t vol);
    uint8_t getVolume() const { return volume_; }

    bool isPlaying() const;

private:
    I2SOut &i2s_;
    uint8_t volume_ = 80;

    /* Sine lookup table (quarter-wave, 64 entries) */
    static const uint16_t kSineTable[64];

    /* DMA buffer for tone/silence generation */
    static constexpr uint16_t kBufSamples = 1024;
    uint16_t buf_[kBufSamples * 2]; /* stereo: L+R interleaved */

    int16_t sineAt(uint32_t phase) const;
    void applyVolume(int16_t *sample) const;
};
