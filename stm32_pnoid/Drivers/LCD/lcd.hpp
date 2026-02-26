/**
 * @file    lcd.hpp
 * @brief   LCD Driver for GC9A01 / ST7789 via SPI (C++ OOP)
 * @note    Select IC with: #define LCD_IC_GC9A01 or #define LCD_IC_ST7789
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class LCD {
public:
    /* IC Selection — define ONE before including this header */
    #if !defined(LCD_IC_GC9A01) && !defined(LCD_IC_ST7789)
    #define LCD_IC_ST7789
    #endif

    #ifdef LCD_IC_GC9A01
      static constexpr uint16_t DEFAULT_WIDTH  = 240;
      static constexpr uint16_t DEFAULT_HEIGHT = 240;
    #else
      static constexpr uint16_t DEFAULT_WIDTH  = 240;
      static constexpr uint16_t DEFAULT_HEIGHT = 240; // Change to 320 for 240x320 panels
    #endif

    /* Common RGB565 colors */
    static constexpr uint16_t BLACK   = 0x0000;
    static constexpr uint16_t WHITE   = 0xFFFF;
    static constexpr uint16_t RED     = 0xF800;
    static constexpr uint16_t GREEN   = 0x07E0;
    static constexpr uint16_t BLUE    = 0x001F;
    static constexpr uint16_t YELLOW  = 0xFFE0;
    static constexpr uint16_t CYAN    = 0x07FF;
    static constexpr uint16_t MAGENTA = 0xF81F;
    static constexpr uint16_t ORANGE  = 0xFD20;
    static constexpr uint16_t GRAY    = 0x8410;

    enum class Rotation { R0 = 0, R90, R180, R270 };

    enum class Status { OK = 0, ErrInit, ErrSPI, ErrParam };

    /**
     * @brief  Construct with SPI handle and GPIO pins
     * @param  hspi  SPI handle (SPI2 from CubeMX)
     * @param  csPort/csPin    Chip select GPIO
     * @param  dcPort/dcPin    Data/Command GPIO
     * @param  blkPort/blkPin  Backlight GPIO
     */
    LCD(SPI_HandleTypeDef &hspi,
        GPIO_TypeDef *csPort,  uint16_t csPin,
        GPIO_TypeDef *dcPort,  uint16_t dcPin,
        GPIO_TypeDef *blkPort, uint16_t blkPin);

    Status init();
    void setRotation(Rotation rot);

    void fillScreen(uint16_t color);
    void drawPixel(uint16_t x, uint16_t y, uint16_t color);
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

    void drawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg);
    void drawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg);
    void drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

    void backlightOn();
    void backlightOff();

    uint16_t width()  const { return width_; }
    uint16_t height() const { return height_; }

    static constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }

private:
    SPI_HandleTypeDef &hspi_;
    GPIO_TypeDef *csPort_;   uint16_t csPin_;
    GPIO_TypeDef *dcPort_;   uint16_t dcPin_;
    GPIO_TypeDef *blkPort_;  uint16_t blkPin_;

    uint16_t width_  = DEFAULT_WIDTH;
    uint16_t height_ = DEFAULT_HEIGHT;

    void csLow();
    void csHigh();
    void dcCmd();
    void dcData();

    void writeCommand(uint8_t cmd);
    void writeData8(uint8_t data);
    void writeData16(uint16_t data);
    void writeDataBulk(const uint8_t *data, uint32_t len);
    void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void initSequence();
};
