/**
 * @file    pca9685.cpp
 * @brief   PCA9685 16-Channel PWM Driver implementation
 */

#include "pca9685.hpp"
#include "debug_log.h"

static const char *TAG = "PCA9685";

/* ============== Constructor ============== */

PCA9685::PCA9685(I2C_HandleTypeDef &hi2c, uint8_t addr)
    : hi2c_(hi2c), addr_(addr), freqHz_(SERVO_FREQ)
{
}

/* ============== I2C helpers ============== */

PCA9685::Status PCA9685::writeReg(uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(&hi2c_, addr_ << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

PCA9685::Status PCA9685::readReg(uint8_t reg, uint8_t &val)
{
    if (HAL_I2C_Mem_Read(&hi2c_, addr_ << 1, reg,
                         I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

/* ============== Public API ============== */

PCA9685::Status PCA9685::init()
{
    Status st;

    /* 1. Sleep (required before setting prescaler) */
    st = writeReg(REG_MODE1, MODE1_SLEEP | MODE1_ALLCALL);
    if (st != Status::OK) {
        LOGE(TAG, "0x%02X: I2C write failed", addr_);
        return Status::ErrInit;
    }

    /* 2. Set prescaler for 50Hz: round(25MHz / (4096 * 50)) - 1 = 121 */
    uint8_t prescale = (uint8_t)((OSC_CLOCK / ((uint32_t)PWM_RESOLUTION * SERVO_FREQ)) - 1);
    st = writeReg(REG_PRESCALE, prescale);
    if (st != Status::OK) return Status::ErrInit;

    /* 3. Wake up with auto-increment enabled */
    st = writeReg(REG_MODE1, MODE1_AI | MODE1_ALLCALL);
    if (st != Status::OK) return Status::ErrInit;
    HAL_Delay(1);  // wait for oscillator (500us min)

    /* 4. Restart */
    st = writeReg(REG_MODE1, MODE1_RESTART | MODE1_AI | MODE1_ALLCALL);
    if (st != Status::OK) return Status::ErrInit;

    /* 5. Totem-pole output */
    st = writeReg(REG_MODE2, MODE2_OUTDRV);
    if (st != Status::OK) return Status::ErrInit;

    /* 6. All channels off initially */
    allOff();

    freqHz_ = SERVO_FREQ;
    LOGI(TAG, "0x%02X: init OK (%uHz, prescale=%u)", addr_, freqHz_, prescale);
    return Status::OK;
}

PCA9685::Status PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel >= NUM_CHANNELS) return Status::ErrInit;

    uint8_t reg = REG_LED0_ON_L + 4 * channel;
    uint8_t data[4] = {
        (uint8_t)(on & 0xFF),
        (uint8_t)((on >> 8) & 0x0F),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0x0F),
    };

    if (HAL_I2C_Mem_Write(&hi2c_, addr_ << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, data, 4, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

PCA9685::Status PCA9685::setPulse(uint8_t channel, uint16_t pulseUs)
{
    // period = 1000000 / freqHz_ (in us), maps to 4096 counts
    uint32_t periodUs = 1000000 / freqHz_;
    uint16_t off = (uint16_t)((uint32_t)pulseUs * PWM_RESOLUTION / periodUs);
    if (off > 4095) off = 4095;
    return setPWM(channel, 0, off);
}

PCA9685::Status PCA9685::setAngle(uint8_t channel, uint16_t angle)
{
    if (angle > 180) angle = 180;
    uint16_t pulseUs = SERVO_MIN_US
                     + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
    return setPulse(channel, pulseUs);
}

PCA9685::Status PCA9685::allOff()
{
    uint8_t data[4] = { 0x00, 0x00, 0x00, 0x10 };  // OFF_H bit4 = full off
    if (HAL_I2C_Mem_Write(&hi2c_, addr_ << 1, REG_ALL_LED_ON_L,
                          I2C_MEMADD_SIZE_8BIT, data, 4, I2C_TIMEOUT) != HAL_OK)
        return Status::ErrI2C;
    return Status::OK;
}

PCA9685::Status PCA9685::sleep()
{
    uint8_t mode1;
    Status st = readReg(REG_MODE1, mode1);
    if (st != Status::OK) return st;
    return writeReg(REG_MODE1, mode1 | MODE1_SLEEP);
}

PCA9685::Status PCA9685::wake()
{
    uint8_t mode1;
    Status st = readReg(REG_MODE1, mode1);
    if (st != Status::OK) return st;

    st = writeReg(REG_MODE1, mode1 & ~MODE1_SLEEP);
    if (st != Status::OK) return st;
    HAL_Delay(1);

    return writeReg(REG_MODE1, (mode1 & ~MODE1_SLEEP) | MODE1_RESTART);
}

PCA9685::Status PCA9685::setFrequency(uint16_t freqHz)
{
    if (freqHz < 24 || freqHz > 1526) return Status::ErrInit;

    uint8_t prescale = (uint8_t)((OSC_CLOCK / ((uint32_t)PWM_RESOLUTION * freqHz)) - 1);

    Status st = sleep();
    if (st != Status::OK) return st;

    st = writeReg(REG_PRESCALE, prescale);
    if (st != Status::OK) return st;

    st = wake();
    if (st != Status::OK) return st;

    freqHz_ = freqHz;
    return Status::OK;
}
