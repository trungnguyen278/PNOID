/**
 * @file    bno085.hpp
 * @brief   BNO085 9-DOF IMU Driver (SHTP over I2C)
 * @note    Supports rotation vector, accelerometer, gyroscope reports
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include <cmath>

class BNO085 {
public:
    static constexpr uint16_t MAX_PACKET_SIZE = 128;
    static constexpr uint8_t  NUM_CHANNELS    = 6;

    enum class Status {
        OK = 0,
        ErrI2C,
        ErrInit,
        ErrTimeout,
        ErrPacket,
        ErrNoData,
    };

    /* Sensor report IDs */
    static constexpr uint8_t REPORT_ACCELEROMETER       = 0x01;
    static constexpr uint8_t REPORT_GYROSCOPE           = 0x02;
    static constexpr uint8_t REPORT_MAGNETIC_FIELD       = 0x03;
    static constexpr uint8_t REPORT_LINEAR_ACCELERATION  = 0x04;
    static constexpr uint8_t REPORT_ROTATION_VECTOR      = 0x05;
    static constexpr uint8_t REPORT_GRAVITY              = 0x06;
    static constexpr uint8_t REPORT_GAME_ROTATION_VECTOR = 0x08;

    struct Quaternion {
        float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
        uint8_t accuracy = 0;  // 0=unreliable, 3=high
    };

    struct Euler {
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;  // degrees
    };

    struct Vec3 {
        float x = 0.0f, y = 0.0f, z = 0.0f;
    };

    /**
     * @brief  Constructor
     * @param  hi2c  HAL I2C handle
     * @param  addr  7-bit I2C address (0x4A default)
     */
    explicit BNO085(I2C_HandleTypeDef &hi2c, uint8_t addr = 0x4A);

    /** Initialize: soft reset, wait for boot, read product ID */
    Status init();

    /** Enable a sensor report at the given interval */
    Status enableReport(uint8_t reportId, uint32_t intervalUs);

    /** Poll for new data (call from main loop). Returns OK if new data parsed. */
    Status poll();

    /** Get latest rotation vector quaternion */
    const Quaternion& getQuaternion() const { return quat_; }

    /** Get euler angles (degrees) converted from quaternion */
    Euler getEuler() const;

    /** Get latest accelerometer reading (m/s^2) */
    const Vec3& getAccel() const { return accel_; }

    /** Get latest gyroscope reading (rad/s) */
    const Vec3& getGyro() const { return gyro_; }

    /** Soft reset the sensor */
    Status softReset();

private:
    I2C_HandleTypeDef &hi2c_;
    uint8_t addr_;

    static constexpr uint32_t I2C_TIMEOUT = 150;

    /* SHTP channels */
    static constexpr uint8_t CHANNEL_COMMAND    = 0;
    static constexpr uint8_t CHANNEL_EXECUTABLE = 1;
    static constexpr uint8_t CHANNEL_CONTROL    = 2;
    static constexpr uint8_t CHANNEL_REPORTS    = 3;

    /* SH-2 report/command IDs */
    static constexpr uint8_t SHTP_REPORT_PRODUCT_ID_REQ  = 0xF9;
    static constexpr uint8_t SHTP_REPORT_PRODUCT_ID_RESP = 0xF8;
    static constexpr uint8_t SHTP_REPORT_SET_FEATURE     = 0xFD;
    static constexpr uint8_t SHTP_REPORT_COMMAND_REQ     = 0xF2;
    static constexpr uint8_t SHTP_REPORT_COMMAND_RESP    = 0xF1;
    static constexpr uint8_t SHTP_REPORT_BASE_TIMESTAMP  = 0xFB;

    static constexpr uint8_t COMMAND_INITIALIZE = 0x04;

    /* Buffers */
    uint8_t rxBuf_[MAX_PACKET_SIZE];
    uint8_t txBuf_[MAX_PACKET_SIZE];
    uint8_t seqNum_[NUM_CHANNELS] = {};

    /* Latest sensor data */
    Quaternion quat_;
    Vec3 accel_;
    Vec3 gyro_;

    /* SHTP helpers */
    Status sendPacket(uint8_t channel, const uint8_t *data, uint16_t dataLen);
    Status receivePacket(uint16_t &packetLen, uint8_t &channel);
    Status waitForPacket(uint8_t channel, uint8_t reportId, uint32_t timeoutMs);

    /* Parsers */
    void parseInputReport(const uint8_t *payload, uint16_t len);
    void parseRotationVector(const uint8_t *report);
    void parseAccelerometer(const uint8_t *report);
    void parseGyroscope(const uint8_t *report);

    /* Q-point conversion */
    static float qToFloat(int16_t raw, uint8_t qPoint) {
        return (float)raw / (float)(1 << qPoint);
    }
};
