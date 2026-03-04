/**
 * @file    pca9685.hpp
 * @brief   PCA9685 16-Channel 12-bit PWM Driver (I2C)
 * @note    Default 50Hz for servo control (SG92R etc.)
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class PCA9685 {
public:
    static constexpr uint8_t  NUM_CHANNELS   = 16;
    static constexpr uint32_t OSC_CLOCK      = 25000000;  // 25 MHz internal
    static constexpr uint16_t PWM_RESOLUTION = 4096;      // 12-bit

    /* Servo defaults (50Hz) */
    static constexpr uint16_t SERVO_FREQ     = 50;
    static constexpr uint16_t SERVO_MIN_US   = 500;   // 0°
    static constexpr uint16_t SERVO_MAX_US   = 2500;  // 180°

    enum class Status {
        OK = 0,
        ErrI2C,
        ErrInit,
    };

    /**
     * @brief  Constructor
     * @param  hi2c  HAL I2C handle (e.g. hi2c1)
     * @param  addr  7-bit I2C address (0x40 default, 0x41 if A0 soldered)
     */
    explicit PCA9685(I2C_HandleTypeDef &hi2c, uint8_t addr = 0x40);

    /** Initialize: set 50Hz, totem-pole output, wake up */
    Status init();

    /** Set raw 12-bit PWM on/off values for a channel */
    Status setPWM(uint8_t channel, uint16_t on, uint16_t off);

    /** Set pulse width in microseconds (at current frequency) */
    Status setPulse(uint8_t channel, uint16_t pulseUs);

    /** Set servo angle 0-180° (maps to SERVO_MIN_US..SERVO_MAX_US) */
    Status setAngle(uint8_t channel, uint16_t angle);

    /** Set all channels off */
    Status allOff();

    /** Enter low-power sleep (oscillator off, outputs off) */
    Status sleep();

    /** Wake from sleep and restart PWM */
    Status wake();

    /** Set PWM frequency (re-configures prescaler, requires brief sleep) */
    Status setFrequency(uint16_t freqHz);

private:
    I2C_HandleTypeDef &hi2c_;
    uint8_t addr_;          // 7-bit address
    uint16_t freqHz_;       // current PWM frequency

    static constexpr uint32_t I2C_TIMEOUT = 100;

    /* Register addresses */
    static constexpr uint8_t REG_MODE1       = 0x00;
    static constexpr uint8_t REG_MODE2       = 0x01;
    static constexpr uint8_t REG_LED0_ON_L   = 0x06;
    static constexpr uint8_t REG_ALL_LED_ON_L  = 0xFA;
    static constexpr uint8_t REG_ALL_LED_OFF_L = 0xFC;
    static constexpr uint8_t REG_PRESCALE    = 0xFE;

    /* MODE1 bits */
    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_AI      = 0x20;  // auto-increment
    static constexpr uint8_t MODE1_SLEEP   = 0x10;
    static constexpr uint8_t MODE1_ALLCALL = 0x01;

    /* MODE2 bits */
    static constexpr uint8_t MODE2_OUTDRV  = 0x04;  // totem-pole

    Status writeReg(uint8_t reg, uint8_t val);
    Status readReg(uint8_t reg, uint8_t &val);
};
