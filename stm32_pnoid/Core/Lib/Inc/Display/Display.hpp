#ifndef DISPLAY_HPP
#define DISPLAY_HPP

/**
 * @file Display.hpp
 * @brief Main include file for Display Library
 * 
 * Include this file to access all display components.
 * 
 * Folder Structure:
 * ├── Common/           - Shared utilities
 * │   ├── Color.hpp     - RGB565 color definitions
 * │   ├── GPIOPin.hpp   - GPIO abstraction
 * │   └── Font5x7.hpp   - Basic 5x7 font
 * │
 * ├── Interface/        - Abstract interfaces
 * │   ├── IDisplay.hpp  - Display interface
 * │   ├── ISPI.hpp      - SPI interface
 * │   └── IQSPI.hpp     - QSPI interface
 * │
 * ├── HAL/              - Hardware implementations
 * │   ├── HAL_SPI.hpp   - STM32 HAL SPI implementation
 * │   └── SoftQSPI.hpp  - Software bit-banging QSPI
 * │
 * └── Driver/           - Display drivers
 *     ├── ST7789.hpp    - ST7789 SPI TFT driver (240x240)
 *     └── ST77916.hpp   - ST77916 QSPI TFT driver (360x360)
 * 
 * Usage Example (ST7789):
 * @code
 * #include "Display/Display.hpp"
 * 
 * HAL_SPI spi(&hspi1);
 * ST7789_Config cfg;
 * cfg.setSize(240, 240)
 *    .setDCPin(GPIOA, GPIO_PIN_1)
 *    .setRSTPin(GPIOA, GPIO_PIN_2);
 * 
 * ST7789 display(&spi, cfg);
 * display.init();
 * display.fillScreen(Color::BLUE);
 * @endcode
 * 
 * Usage Example (ST77916 with Software QSPI):
 * @code
 * #include "Display/Display.hpp"
 * 
 * SoftQSPI_Pins pins;
 * pins.clk = GPIOPin(GPIOA, GPIO_PIN_0);
 * pins.cs = GPIOPin(GPIOA, GPIO_PIN_1);
 * pins.d0 = GPIOPin(GPIOA, GPIO_PIN_2);
 * pins.d1 = GPIOPin(GPIOA, GPIO_PIN_3);
 * pins.d2 = GPIOPin(GPIOA, GPIO_PIN_4);
 * pins.d3 = GPIOPin(GPIOA, GPIO_PIN_5);
 * 
 * SoftQSPI qspi(pins);
 * qspi.init();
 * 
 * ST77916_Config cfg;
 * cfg.setSize(360, 360)
 *    .setRSTPin(GPIOB, GPIO_PIN_0);
 * 
 * ST77916 display(&qspi, cfg);
 * display.init();
 * display.fillScreen(Color::RED);
 * @endcode
 */

// ============================================================================
// Common Utilities
// ============================================================================
#include "Common/Color.hpp"
#include "Common/GPIOPin.hpp"
#include "Common/Font5x7.hpp"

// ============================================================================
// Interfaces
// ============================================================================
#include "Interface/IDisplay.hpp"
#include "Interface/ISPI.hpp"
#include "Interface/IQSPI.hpp"

// ============================================================================
// HAL Implementations
// ============================================================================
#include "HAL/HAL_SPI.hpp"
#include "HAL/SoftQSPI.hpp"

// ============================================================================
// Display Drivers
// ============================================================================
#include "Driver/ST7789.hpp"
#include "Driver/ST77916.hpp"

#endif // DISPLAY_HPP
