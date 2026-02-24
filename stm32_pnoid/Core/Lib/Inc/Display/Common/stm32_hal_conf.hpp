/**
 * @file stm32_hal_conf.hpp
 * @brief STM32 HAL configuration for Display library
 * 
 * This file provides a portable way to include the correct STM32 HAL header.
 * Modify this file when porting to different STM32 series.
 */

#ifndef STM32_HAL_CONF_HPP
#define STM32_HAL_CONF_HPP

/* ============================================================================
 * STM32 Family Selection
 * Uncomment ONE of the following lines based on your MCU series:
 * ============================================================================ */

// #define STM32F4_SERIES
// #define STM32F7_SERIES
#define STM32H7_SERIES

/* ============================================================================
 * Include appropriate HAL header
 * ============================================================================ */

#if defined(STM32F4_SERIES)
    #include "stm32f4xx_hal.h"
#elif defined(STM32F7_SERIES)
    #include "stm32f7xx_hal.h"
#elif defined(STM32H7_SERIES)
    #include "stm32h7xx_hal.h"
#elif defined(STM32L4_SERIES)
    #include "stm32l4xx_hal.h"
#elif defined(STM32G4_SERIES)
    #include "stm32g4xx_hal.h"
#else
    /* Fallback: use main.h which includes the correct HAL */
    #include "main.h"
#endif

#endif /* STM32_HAL_CONF_HPP */
