/**
 * @file    camera.hpp
 * @brief   OV2640 Camera Driver via DCMI + I2C2 SCCB (C++ OOP)
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include <functional>

class Camera {
public:
    static constexpr uint8_t OV2640_ADDR = 0x60; // 7-bit 0x30, shifted

    enum class Resolution { QQVGA, QVGA, VGA, SVGA, XGA, SXGA, UXGA };
    enum class Format { Rgb565, Jpeg };

    enum class Status {
        OK = 0,
        ErrInit,
        ErrI2C,
        ErrID,
        ErrDCMI,
        ErrTimeout,
        ErrParam,
    };

    using FrameCallback = std::function<void()>;

    /**
     * @brief  Construct with DCMI and I2C handles
     */
    Camera(DCMI_HandleTypeDef &hdcmi, I2C_HandleTypeDef &hi2c);

    /**
     * @brief  Initialize: power on, verify ID, configure sensor
     */
    Status init(Format format = Format::Rgb565, Resolution res = Resolution::QVGA);

    Status readID(uint16_t &id);
    Status setResolution(Resolution res);
    Status setFormat(Format format);

    /**
     * @brief  Capture a single frame via DMA
     * @param  buf   Buffer for frame data
     * @param  size  Buffer size in 32-bit words
     */
    Status captureSnapshot(uint32_t *buf, uint32_t size);

    /**
     * @brief  Start continuous DMA capture
     */
    Status startContinuous(uint32_t *buf, uint32_t size);
    Status stopContinuous();

    void registerCallback(FrameCallback cb) { callback_ = cb; }
    bool isFrameReady() const { return frameReady_; }
    void clearFrameReady() { frameReady_ = false; }

    /**
     * @brief  Call from HAL_DCMI_FrameEventCallback
     */
    void frameEventHandler();

private:
    DCMI_HandleTypeDef &hdcmi_;
    I2C_HandleTypeDef  &hi2c_;

    volatile bool frameReady_ = false;
    FrameCallback callback_;
    Format currentFormat_ = Format::Rgb565;

    /* SCCB (I2C) helpers */
    Status sccbWrite(uint8_t reg, uint8_t val);
    Status sccbRead(uint8_t reg, uint8_t &val);

    struct RegPair { uint8_t reg; uint8_t val; };
    Status sccbWriteRegs(const RegPair *regs);
    const RegPair* getResRegs(Resolution res);
};
