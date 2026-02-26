/**
 * @file    camera.cpp
 * @brief   OV2640 Camera Driver implementation (DCMI + I2C2 SCCB)
 */

#include "camera.hpp"
#include "ov2640_regs.h"
#include "main.h"
#include "debug_log.h"

static const char *TAG = "CAM";

/* ---------- Constructor -------------------------------------------------- */

Camera::Camera(DCMI_HandleTypeDef &hdcmi, I2C_HandleTypeDef &hi2c)
    : hdcmi_(hdcmi), hi2c_(hi2c)
{
}

/* ---------- SCCB helpers ------------------------------------------------- */

Camera::Status Camera::sccbWrite(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    if (HAL_I2C_Master_Transmit(&hi2c_, OV2640_ADDR, buf, 2, 100) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

Camera::Status Camera::sccbRead(uint8_t reg, uint8_t &val)
{
    if (HAL_I2C_Master_Transmit(&hi2c_, OV2640_ADDR, &reg, 1, 100) != HAL_OK)
        return Status::ErrI2C;
    if (HAL_I2C_Master_Receive(&hi2c_, OV2640_ADDR | 0x01, &val, 1, 100) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

Camera::Status Camera::sccbWriteRegs(const RegPair *regs)
{
    while (!(regs->reg == OV2640_REG_END && regs->val == OV2640_VAL_END)) {
        auto st = sccbWrite(regs->reg, regs->val);
        if (st != Status::OK) return st;
        regs++;
    }
    return Status::OK;
}

const Camera::RegPair* Camera::getResRegs(Resolution res)
{
    switch (res) {
        case Resolution::QQVGA: return reinterpret_cast<const RegPair*>(ov2640_qqvga);
        case Resolution::QVGA:  return reinterpret_cast<const RegPair*>(ov2640_qvga);
        case Resolution::VGA:   return reinterpret_cast<const RegPair*>(ov2640_vga);
        case Resolution::SVGA:  return reinterpret_cast<const RegPair*>(ov2640_svga);
        case Resolution::XGA:   return reinterpret_cast<const RegPair*>(ov2640_xga);
        case Resolution::SXGA:  return reinterpret_cast<const RegPair*>(ov2640_sxga);
        case Resolution::UXGA:  return reinterpret_cast<const RegPair*>(ov2640_uxga);
        default:                 return reinterpret_cast<const RegPair*>(ov2640_qvga);
    }
}

/* ---------- Public API --------------------------------------------------- */

Camera::Status Camera::init(Format format, Resolution res)
{
    /* Power on sequence */
    HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(20);

    /* Software reset */
    auto st = sccbWriteRegs(reinterpret_cast<const RegPair*>(ov2640_reset_regs));
    if (st != Status::OK) {
        LOGE(TAG, "Software reset failed");
        return st;
    }
    HAL_Delay(100);

    /* Verify product ID */
    uint16_t id = 0;
    st = readID(id);
    if (st != Status::OK) return st;
    LOGI(TAG, "OV2640 ID: 0x%04X", id);

    /* Sensor init */
    st = sccbWriteRegs(reinterpret_cast<const RegPair*>(ov2640_sensor_init));
    if (st != Status::OK) return st;

    /* DSP init */
    st = sccbWriteRegs(reinterpret_cast<const RegPair*>(ov2640_dsp_init));
    if (st != Status::OK) return st;

    /* Set format & resolution */
    st = setFormat(format);
    if (st != Status::OK) return st;

    st = setResolution(res);
    if (st != Status::OK) return st;

    LOGI(TAG, "Init OK");
    return Status::OK;
}

Camera::Status Camera::readID(uint16_t &id)
{
    auto st = sccbWrite(OV2640_DSP_RA_DLMT, OV2640_SENSOR_BANK);
    if (st != Status::OK) return st;

    uint8_t pidh = 0, pidl = 0;
    st = sccbRead(OV2640_SENSOR_PIDH, pidh);
    if (st != Status::OK) return st;
    st = sccbRead(OV2640_SENSOR_PIDL, pidl);
    if (st != Status::OK) return st;

    id = (static_cast<uint16_t>(pidh) << 8) | pidl;
    return Status::OK;
}

Camera::Status Camera::setResolution(Resolution res)
{
    return sccbWriteRegs(getResRegs(res));
}

Camera::Status Camera::setFormat(Format format)
{
    currentFormat_ = format;
    if (format == Format::Jpeg)
        return sccbWriteRegs(reinterpret_cast<const RegPair*>(ov2640_jpeg_init));
    else
        return sccbWriteRegs(reinterpret_cast<const RegPair*>(ov2640_yuv422_init));
}

Camera::Status Camera::captureSnapshot(uint32_t *buf, uint32_t size)
{
    if (buf == nullptr || size == 0) return Status::ErrParam;

    frameReady_ = false;

    if (HAL_DCMI_Start_DMA(&hdcmi_, DCMI_MODE_SNAPSHOT, reinterpret_cast<uint32_t>(buf), size) != HAL_OK)
        return Status::ErrDCMI;

    uint32_t start = HAL_GetTick();
    while (!frameReady_) {
        if ((HAL_GetTick() - start) > 5000) {
            HAL_DCMI_Stop(&hdcmi_);
            LOGE(TAG, "Snapshot timeout");
            return Status::ErrTimeout;
        }
    }

    return Status::OK;
}

Camera::Status Camera::startContinuous(uint32_t *buf, uint32_t size)
{
    if (buf == nullptr || size == 0) return Status::ErrParam;

    frameReady_ = false;

    if (HAL_DCMI_Start_DMA(&hdcmi_, DCMI_MODE_CONTINUOUS, reinterpret_cast<uint32_t>(buf), size) != HAL_OK)
        return Status::ErrDCMI;

    return Status::OK;
}

Camera::Status Camera::stopContinuous()
{
    if (HAL_DCMI_Stop(&hdcmi_) != HAL_OK)
        return Status::ErrDCMI;
    return Status::OK;
}

void Camera::frameEventHandler()
{
    frameReady_ = true;
    if (callback_) callback_();
}
