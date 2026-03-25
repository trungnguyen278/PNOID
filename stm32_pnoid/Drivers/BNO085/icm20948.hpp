/**
 * @file    icm20948.hpp
 * @brief   ICM-20948 9-DOF IMU Driver (I2C) — Accel + Gyro + Magnetometer (AK09916)
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include <cmath>

class ICM20948 {
public:
    static constexpr uint8_t WHO_AM_I_VAL = 0xEA;

    enum class Status {
        OK = 0,
        ErrI2C,
        ErrInit,
        ErrID,
    };

    struct Vec3 {
        float x = 0.0f, y = 0.0f, z = 0.0f;
    };

    struct Euler {
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // degrees
    };

    /**
     * @param hi2c  HAL I2C handle
     * @param addr  7-bit address (0x68 default, 0x69 if AD0=HIGH)
     */
    explicit ICM20948(I2C_HandleTypeDef &hi2c, uint8_t addr = 0x68);

    /** Reset, verify WHO_AM_I, configure accel/gyro/mag */
    Status init();

    /** Read all 9 axes (call from main loop) */
    Status read();

    /** Accel in g */
    const Vec3& getAccel() const { return accel_; }
    /** Gyro in degrees/s */
    const Vec3& getGyro() const { return gyro_; }
    /** Mag in uT */
    const Vec3& getMag() const { return mag_; }
    /** Temperature in °C */
    float getTemp() const { return temp_; }

    /** Compute roll/pitch from accel (no yaw without mag fusion) */
    Euler getEuler() const;

private:
    I2C_HandleTypeDef &hi2c_;
    uint8_t addr_;

    Vec3 accel_, gyro_, mag_;
    float temp_ = 0.0f;

    /* Sensitivities (based on full-scale config) */
    float accelSens_ = 16384.0f;  // +/-2g
    float gyroSens_  = 131.0f;    // +/-250 dps

    static constexpr uint32_t I2C_TIMEOUT = 100;

    /* Bank switching */
    static constexpr uint8_t REG_BANK_SEL = 0x7F;

    /* Bank 0 registers */
    static constexpr uint8_t B0_WHO_AM_I     = 0x00;
    static constexpr uint8_t B0_USER_CTRL    = 0x03;
    static constexpr uint8_t B0_PWR_MGMT_1   = 0x06;
    static constexpr uint8_t B0_PWR_MGMT_2   = 0x07;
    static constexpr uint8_t B0_INT_PIN_CFG  = 0x0F;
    static constexpr uint8_t B0_ACCEL_XOUT_H = 0x2D;

    /* Bank 2 registers */
    static constexpr uint8_t B2_GYRO_SMPLRT_DIV   = 0x00;
    static constexpr uint8_t B2_GYRO_CONFIG_1     = 0x01;
    static constexpr uint8_t B2_ACCEL_SMPLRT_DIV1 = 0x10;
    static constexpr uint8_t B2_ACCEL_SMPLRT_DIV2 = 0x11;
    static constexpr uint8_t B2_ACCEL_CONFIG      = 0x14;

    /* Bank 3 registers (I2C master for AK09916) */
    static constexpr uint8_t B3_I2C_MST_CTRL  = 0x01;
    static constexpr uint8_t B3_I2C_SLV0_ADDR = 0x03;
    static constexpr uint8_t B3_I2C_SLV0_REG  = 0x04;
    static constexpr uint8_t B3_I2C_SLV0_CTRL = 0x05;
    static constexpr uint8_t B3_I2C_SLV0_DO   = 0x06;

    /* AK09916 */
    static constexpr uint8_t AK_ADDR  = 0x0C;
    static constexpr uint8_t AK_WIA2  = 0x01;
    static constexpr uint8_t AK_ST1   = 0x10;
    static constexpr uint8_t AK_CNTL2 = 0x31;
    static constexpr uint8_t AK_CNTL3 = 0x32;

    Status setBank(uint8_t bank);
    Status writeReg(uint8_t reg, uint8_t val);
    Status readReg(uint8_t reg, uint8_t &val);
    Status readRegs(uint8_t reg, uint8_t *buf, uint16_t len);
    Status magWrite(uint8_t reg, uint8_t val);
    Status initMag();
};
