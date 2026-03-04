/**
 * @file    bno085.cpp
 * @brief   BNO085 IMU Driver (SHTP over I2C) implementation
 */

#include "bno085.hpp"
#include "debug_log.h"
#include <cstring>

static const char *TAG = "BNO085";

/* ============== Constructor ============== */

BNO085::BNO085(I2C_HandleTypeDef &hi2c, uint8_t addr)
    : hi2c_(hi2c), addr_(addr)
{
}

/* ============== SHTP Transport ============== */

BNO085::Status BNO085::sendPacket(uint8_t channel, const uint8_t *data, uint16_t dataLen)
{
    uint16_t packetLen = dataLen + 4;
    if (packetLen > MAX_PACKET_SIZE) return Status::ErrPacket;

    txBuf_[0] = packetLen & 0xFF;
    txBuf_[1] = (packetLen >> 8) & 0x7F;  // bit15 = 0 (no continuation)
    txBuf_[2] = channel;
    txBuf_[3] = seqNum_[channel]++;
    memcpy(&txBuf_[4], data, dataLen);

    if (HAL_I2C_Master_Transmit(&hi2c_, addr_ << 1,
                                 txBuf_, packetLen, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

BNO085::Status BNO085::receivePacket(uint16_t &packetLen, uint8_t &channel)
{
    /* Read header first */
    if (HAL_I2C_Master_Receive(&hi2c_, addr_ << 1,
                                rxBuf_, 4, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;

    packetLen = (uint16_t)rxBuf_[0] | ((uint16_t)(rxBuf_[1] & 0x7F) << 8);
    channel = rxBuf_[2];

    if (packetLen == 0 || packetLen == 0x7FFF)
        return Status::ErrNoData;

    if (packetLen > MAX_PACKET_SIZE)
        packetLen = MAX_PACKET_SIZE;

    /* Read full packet (header + payload) */
    if (packetLen > 4) {
        if (HAL_I2C_Master_Receive(&hi2c_, addr_ << 1,
                                    rxBuf_, packetLen, I2C_TIMEOUT) != HAL_OK)
            return Status::ErrI2C;
    }

    return Status::OK;
}

BNO085::Status BNO085::waitForPacket(uint8_t channel, uint8_t reportId, uint32_t timeoutMs)
{
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeoutMs) {
        uint16_t len;
        uint8_t ch;
        Status st = receivePacket(len, ch);
        if (st == Status::ErrNoData) {
            HAL_Delay(2);
            continue;
        }
        if (st != Status::OK) return st;

        if (ch == channel && len > 4 && rxBuf_[4] == reportId)
            return Status::OK;

        HAL_Delay(2);
    }
    return Status::ErrTimeout;
}

/* ============== Init ============== */

BNO085::Status BNO085::init()
{
    Status st;

    /* Soft reset */
    st = softReset();
    if (st != Status::OK) {
        LOGE(TAG, "0x%02X: soft reset failed", addr_);
        return Status::ErrInit;
    }

    /* Wait for boot — drain advertisement (ch0) and reset-complete (ch1) packets */
    HAL_Delay(100);
    for (int i = 0; i < 10; i++) {
        uint16_t len;
        uint8_t ch;
        st = receivePacket(len, ch);
        if (st == Status::ErrNoData) break;
        if (st != Status::OK) {
            HAL_Delay(10);
            continue;
        }
        /* Check for executable reset complete (channel 1, payload byte = 0x01) */
        if (ch == CHANNEL_EXECUTABLE && len > 4 && rxBuf_[4] == 0x01) {
            LOGI(TAG, "0x%02X: reset complete", addr_);
        }
        HAL_Delay(5);
    }

    /* Request Product ID */
    uint8_t pidReq[2] = { SHTP_REPORT_PRODUCT_ID_REQ, 0x00 };
    st = sendPacket(CHANNEL_CONTROL, pidReq, 2);
    if (st != Status::OK) {
        LOGE(TAG, "0x%02X: product ID request failed", addr_);
        return Status::ErrInit;
    }

    /* Wait for Product ID Response */
    st = waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_PRODUCT_ID_RESP, 500);
    if (st == Status::OK && rxBuf_[4] == SHTP_REPORT_PRODUCT_ID_RESP) {
        uint8_t swMajor = rxBuf_[6];
        uint8_t swMinor = rxBuf_[7];
        uint32_t swPartNo = (uint32_t)rxBuf_[12] | ((uint32_t)rxBuf_[13] << 8)
                          | ((uint32_t)rxBuf_[14] << 16) | ((uint32_t)rxBuf_[15] << 24);
        LOGI(TAG, "0x%02X: FW v%u.%u, part=%lu", addr_, swMajor, swMinor, swPartNo);
    } else {
        LOGW(TAG, "0x%02X: no product ID response (may still work)", addr_);
    }

    /* Send Initialize command */
    uint8_t initCmd[12] = {};
    initCmd[0] = SHTP_REPORT_COMMAND_REQ;
    initCmd[1] = 0;  // command sequence
    initCmd[2] = COMMAND_INITIALIZE;
    st = sendPacket(CHANNEL_CONTROL, initCmd, 12);
    if (st != Status::OK) {
        LOGE(TAG, "0x%02X: initialize command failed", addr_);
        return Status::ErrInit;
    }
    HAL_Delay(50);

    /* Drain any response */
    for (int i = 0; i < 5; i++) {
        uint16_t len;
        uint8_t ch;
        if (receivePacket(len, ch) == Status::ErrNoData) break;
        HAL_Delay(2);
    }

    LOGI(TAG, "0x%02X: init OK", addr_);
    return Status::OK;
}

BNO085::Status BNO085::softReset()
{
    /* Send reset command on executable channel (channel 1) */
    uint8_t resetCmd[1] = { 0x01 };  // reset
    return sendPacket(CHANNEL_EXECUTABLE, resetCmd, 1);
}

/* ============== Enable Report ============== */

BNO085::Status BNO085::enableReport(uint8_t reportId, uint32_t intervalUs)
{
    uint8_t data[17] = {};
    data[0]  = SHTP_REPORT_SET_FEATURE;
    data[1]  = reportId;
    data[2]  = 0;  // feature flags
    data[3]  = 0;  // change sensitivity LSB
    data[4]  = 0;  // change sensitivity MSB
    data[5]  = (intervalUs >> 0)  & 0xFF;
    data[6]  = (intervalUs >> 8)  & 0xFF;
    data[7]  = (intervalUs >> 16) & 0xFF;
    data[8]  = (intervalUs >> 24) & 0xFF;
    // bytes 9-16: batch interval + sensor config = 0

    Status st = sendPacket(CHANNEL_CONTROL, data, 17);
    if (st != Status::OK) {
        LOGE(TAG, "0x%02X: enableReport 0x%02X failed", addr_, reportId);
        return st;
    }

    LOGI(TAG, "0x%02X: report 0x%02X enabled (%lu us)", addr_, reportId, intervalUs);
    return Status::OK;
}

/* ============== Poll ============== */

BNO085::Status BNO085::poll()
{
    uint16_t len;
    uint8_t ch;

    Status st = receivePacket(len, ch);
    if (st != Status::OK) return st;

    /* Sensor input reports arrive on channel 3 */
    if (ch == CHANNEL_REPORTS && len > 4) {
        parseInputReport(&rxBuf_[4], len - 4);
        return Status::OK;
    }

    return Status::ErrNoData;
}

/* ============== Parsers ============== */

void BNO085::parseInputReport(const uint8_t *payload, uint16_t len)
{
    uint16_t offset = 0;

    while (offset < len) {
        uint8_t reportId = payload[offset];

        /* Base Timestamp (5 bytes: 1 ID + 4 timestamp) */
        if (reportId == SHTP_REPORT_BASE_TIMESTAMP) {
            offset += 5;
            continue;
        }

        /* Determine report length based on ID */
        uint16_t reportLen = 0;
        switch (reportId) {
            case REPORT_ROTATION_VECTOR:
                reportLen = 14;  // 10 data + 4 overhead
                if (offset + reportLen <= len)
                    parseRotationVector(&payload[offset]);
                break;
            case REPORT_GAME_ROTATION_VECTOR:
                reportLen = 12;
                if (offset + reportLen <= len)
                    parseRotationVector(&payload[offset]);
                break;
            case REPORT_ACCELEROMETER:
            case REPORT_LINEAR_ACCELERATION:
            case REPORT_GRAVITY:
                reportLen = 10;
                if (offset + reportLen <= len)
                    parseAccelerometer(&payload[offset]);
                break;
            case REPORT_GYROSCOPE:
                reportLen = 10;
                if (offset + reportLen <= len)
                    parseGyroscope(&payload[offset]);
                break;
            default:
                return;  // unknown report, stop parsing
        }

        if (reportLen == 0) return;
        offset += reportLen;
    }
}

void BNO085::parseRotationVector(const uint8_t *r)
{
    // r[0] = report ID, r[1] = seq, r[2] = status, r[3..4] = delay
    // r[5..6] = i(Q14), r[7..8] = j, r[9..10] = k, r[11..12] = real(w)
    quat_.accuracy = r[2] & 0x03;
    quat_.x = qToFloat((int16_t)(r[5]  | (r[6]  << 8)), 14);
    quat_.y = qToFloat((int16_t)(r[7]  | (r[8]  << 8)), 14);
    quat_.z = qToFloat((int16_t)(r[9]  | (r[10] << 8)), 14);
    quat_.w = qToFloat((int16_t)(r[11] | (r[12] << 8)), 14);
}

void BNO085::parseAccelerometer(const uint8_t *r)
{
    // Q8 for accelerometer
    accel_.x = qToFloat((int16_t)(r[5] | (r[6] << 8)), 8);
    accel_.y = qToFloat((int16_t)(r[7] | (r[8] << 8)), 8);
    accel_.z = qToFloat((int16_t)(r[9] | (r[10] << 8)), 8);  // fixed: was r[10] << 8
}

void BNO085::parseGyroscope(const uint8_t *r)
{
    // Q9 for gyroscope
    gyro_.x = qToFloat((int16_t)(r[5] | (r[6] << 8)), 9);
    gyro_.y = qToFloat((int16_t)(r[7] | (r[8] << 8)), 9);
    gyro_.z = qToFloat((int16_t)(r[9] | (r[10] << 8)), 9);  // fixed: was r[10] << 8
}

/* ============== Euler ============== */

BNO085::Euler BNO085::getEuler() const
{
    Euler e;
    float w = quat_.w, x = quat_.x, y = quat_.y, z = quat_.z;

    // Roll (x-axis)
    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    e.roll = atan2f(sinr, cosr) * (180.0f / 3.14159265f);

    // Pitch (y-axis)
    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    e.pitch = asinf(sinp) * (180.0f / 3.14159265f);

    // Yaw (z-axis)
    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    e.yaw = atan2f(siny, cosy) * (180.0f / 3.14159265f);

    return e;
}
