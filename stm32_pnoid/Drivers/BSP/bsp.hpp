/**
 * @file    bsp.hpp
 * @brief   Board Support Package for DevEBox STM32H743VIT6 (C++ OOP)
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class BSP {
public:
    enum class Button { K1 = 0, K2 };

    /**
     * @brief  Initialize BSP (DWT cycle counter, default GPIO states)
     */
    static void init();

    /* LED control (active low) */
    static void ledOn();
    static void ledOff();
    static void ledToggle();

    /* Button reading */
    static bool buttonRead(Button btn);
    static bool buttonPressed(Button btn);  // with ~20ms debounce

    /* Microsecond delay via DWT */
    static void delayUs(uint32_t us);

    /* Camera GPIO helpers */
    static void camPowerDown(bool enable);
    static void camReset(bool active);
    static void camPowerOn();

    /* System info print via USART1 */
    static void printSystemInfo();
};
