/**
 * @file    icm20948.cpp
 * @brief   ICM-20948 9-DOF IMU Driver implementation
 */

#include "icm20948.hpp"
#include "debug_log.h"

static const char *TAG = "ICM20948";

/* ============== Constructor ============== */

ICM20948::ICM20948(I2C_HandleTypeDef &hi2c, uint8_t addr)
    : hi2c_(hi2c), addr_(addr)
{
}

/* ============== I2C helpers ============== */

ICM20948::Status ICM20948::writeReg(uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(&hi2c_, addr_ << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

ICM20948::Status ICM20948::readReg(uint8_t reg, uint8_t &val)
{
    if (HAL_I2C_Mem_Read(&hi2c_, addr_ << 1, reg,
                         I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

ICM20948::Status ICM20948::readRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c_, addr_ << 1, reg,
                         I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

ICM20948::Status ICM20948::setBank(uint8_t bank)
{
    return writeReg(REG_BANK_SEL, (bank & 0x03) << 4);
}

ICM20948::Status ICM20948::magWrite(uint8_t reg, uint8_t val)
{
    Status st;
    st = setBank(3);                              if (st != Status::OK) return st;
    st = writeReg(B3_I2C_SLV0_ADDR, AK_ADDR);    if (st != Status::OK) return st;  // write
    st = writeReg(B3_I2C_SLV0_REG, reg);          if (st != Status::OK) return st;
    st = writeReg(B3_I2C_SLV0_DO, val);           if (st != Status::OK) return st;
    st = writeReg(B3_I2C_SLV0_CTRL, 0x81);        if (st != Status::OK) return st;  // enable, 1 byte
    st = setBank(0);
    HAL_Delay(10);
    return st;
}

/* ============== Init ============== */

ICM20948::Status ICM20948::init()
{
    Status st;

    /* Bank 0 */
    st = setBank(0);
    if (st != Status::OK) {
        LOGE(TAG, "0x%02X: I2C failed", addr_);
        return Status::ErrI2C;
    }

    /* Device reset */
    st = writeReg(B0_PWR_MGMT_1, 0x80);
    if (st != Status::OK) return Status::ErrInit;
    HAL_Delay(100);

    /* Wake up, auto-select clock */
    st = writeReg(B0_PWR_MGMT_1, 0x01);
    if (st != Status::OK) return Status::ErrInit;
    HAL_Delay(50);

    /* Verify WHO_AM_I */
    uint8_t id;
    st = readReg(B0_WHO_AM_I, id);
    if (st != Status::OK) return Status::ErrInit;
    if (id != WHO_AM_I_VAL) {
        LOGE(TAG, "0x%02X: WHO_AM_I=0x%02X, expected 0xEA", addr_, id);
        return Status::ErrID;
    }

    /* Enable all accel + gyro axes */
    st = writeReg(B0_PWR_MGMT_2, 0x00);
    if (st != Status::OK) return Status::ErrInit;

    /* Bank 2: Gyro config — +/-250 dps, DLPF enabled */
    st = setBank(2);                                    if (st != Status::OK) return Status::ErrInit;
    st = writeReg(B2_GYRO_SMPLRT_DIV, 0x00);           if (st != Status::OK) return Status::ErrInit;
    st = writeReg(B2_GYRO_CONFIG_1, 0x01);              if (st != Status::OK) return Status::ErrInit;
    gyroSens_ = 131.0f;

    /* Bank 2: Accel config — +/-2g, DLPF enabled */
    st = writeReg(B2_ACCEL_SMPLRT_DIV1, 0x00);         if (st != Status::OK) return Status::ErrInit;
    st = writeReg(B2_ACCEL_SMPLRT_DIV2, 0x00);         if (st != Status::OK) return Status::ErrInit;
    st = writeReg(B2_ACCEL_CONFIG, 0x01);               if (st != Status::OK) return Status::ErrInit;
    accelSens_ = 16384.0f;

    /* Bank 0: Enable I2C master for AK09916 */
    st = setBank(0);                                    if (st != Status::OK) return Status::ErrInit;
    st = writeReg(B0_USER_CTRL, 0x20);                  if (st != Status::OK) return Status::ErrInit;

    /* Init magnetometer */
    st = initMag();
    if (st != Status::OK) {
        LOGW(TAG, "0x%02X: Mag init failed (may work without)", addr_);
    }

    /* Enable data-ready interrupt on INT1 pin (active-low, push-pull, pulse 50us) */
    st = setBank(0);                                    if (st != Status::OK) return Status::ErrInit;
    st = writeReg(B0_INT_PIN_CFG, 0x90);               if (st != Status::OK) return Status::ErrInit;
    // 0x90 = bit7: active-LOW, bit4: clear on any read, push-pull, pulse 50us
    st = writeReg(B0_INT_ENABLE_1, 0x01);              if (st != Status::OK) return Status::ErrInit;
    // 0x01 = RAW_DATA_0_RDY_EN

    st = setBank(0);
    LOGI(TAG, "0x%02X: init OK (ICM-20948, INT enabled)", addr_);
    return Status::OK;
}

ICM20948::Status ICM20948::initMag()
{
    Status st;

    /* I2C master clock 345.6 kHz */
    st = setBank(3);                                    if (st != Status::OK) return st;
    st = writeReg(B3_I2C_MST_CTRL, 0x07);              if (st != Status::OK) return st;

    /* Reset AK09916 */
    st = magWrite(AK_CNTL3, 0x01);
    if (st != Status::OK) return st;
    HAL_Delay(100);

    /* Set continuous mode 4 (100 Hz) */
    st = magWrite(AK_CNTL2, 0x08);
    if (st != Status::OK) return st;

    /* Configure SLV0 for continuous mag reads: 8 bytes from ST1 */
    st = setBank(3);                                    if (st != Status::OK) return st;
    st = writeReg(B3_I2C_SLV0_ADDR, AK_ADDR | 0x80);   if (st != Status::OK) return st;  // read
    st = writeReg(B3_I2C_SLV0_REG, AK_ST1);            if (st != Status::OK) return st;
    st = writeReg(B3_I2C_SLV0_CTRL, 0x88);             if (st != Status::OK) return st;  // enable, 8 bytes

    st = setBank(0);
    LOGI(TAG, "0x%02X: AK09916 mag enabled (100Hz)", addr_);
    return st;
}

/* ============== Read ============== */

ICM20948::Status ICM20948::read()
{
    Status st = setBank(0);
    if (st != Status::OK) return st;

    /* Burst read 22 bytes: accel(6) + gyro(6) + temp(2) + mag_status+data(8) */
    uint8_t buf[22];
    st = readRegs(B0_ACCEL_XOUT_H, buf, 22);
    if (st != Status::OK) return st;

    /* Accel — big-endian, flip X and Z for upside-down mounting */
    accel_.x = -(int16_t)(buf[0] << 8 | buf[1]) / accelSens_;
    accel_.y =  (int16_t)(buf[2] << 8 | buf[3]) / accelSens_;
    accel_.z = -(int16_t)(buf[4] << 8 | buf[5]) / accelSens_;

    /* Gyro — big-endian, flip X and Z */
    gyro_.x = -(int16_t)(buf[6]  << 8 | buf[7])  / gyroSens_;
    gyro_.y =  (int16_t)(buf[8]  << 8 | buf[9])  / gyroSens_;
    gyro_.z = -(int16_t)(buf[10] << 8 | buf[11]) / gyroSens_;

    /* Temperature */
    int16_t rawTemp = (int16_t)(buf[12] << 8 | buf[13]);
    temp_ = ((float)rawTemp - 21.0f) / 333.87f + 21.0f;

    /* Mag — little-endian, buf[14]=ST1, buf[15..20]=XL,XH,YL,YH,ZL,ZH, buf[21]=ST2 */
    mag_.x = (int16_t)(buf[15] | buf[16] << 8) * 0.15f;
    mag_.y = (int16_t)(buf[17] | buf[18] << 8) * 0.15f;
    mag_.z = (int16_t)(buf[19] | buf[20] << 8) * 0.15f;

    return Status::OK;
}

/* ============== Euler ============== */

ICM20948::Euler ICM20948::getEuler() const
{
    Euler e;
    /* Roll/Pitch from accelerometer */
    e.roll  = atan2f(accel_.y, accel_.z) * (180.0f / 3.14159265f);
    e.pitch = atan2f(-accel_.x, sqrtf(accel_.y * accel_.y + accel_.z * accel_.z))
              * (180.0f / 3.14159265f);
    /* Yaw from magnetometer (tilt-compensated) */
    float cr = cosf(e.roll * 3.14159265f / 180.0f);
    float sr = sinf(e.roll * 3.14159265f / 180.0f);
    float cp = cosf(e.pitch * 3.14159265f / 180.0f);
    float sp = sinf(e.pitch * 3.14159265f / 180.0f);
    float mx = mag_.x * cp + mag_.y * sp * sr + mag_.z * sp * cr;
    float my = mag_.y * cr - mag_.z * sr;
    e.yaw = atan2f(-my, mx) * (180.0f / 3.14159265f);
    return e;
}
