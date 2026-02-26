/**
 * @file    lcd.cpp
 * @brief   LCD Driver implementation for GC9A01 / ST7789 via SPI
 */

#include "lcd.hpp"
#include "lcd_font.h"
#include "debug_log.h"

static const char *TAG = "LCD";

/* ---------- Constructor -------------------------------------------------- */

LCD::LCD(SPI_HandleTypeDef &hspi,
         GPIO_TypeDef *csPort,  uint16_t csPin,
         GPIO_TypeDef *dcPort,  uint16_t dcPin,
         GPIO_TypeDef *blkPort, uint16_t blkPin)
    : hspi_(hspi)
    , csPort_(csPort),  csPin_(csPin)
    , dcPort_(dcPort),  dcPin_(dcPin)
    , blkPort_(blkPort), blkPin_(blkPin)
{
}

/* ---------- GPIO helpers ------------------------------------------------- */

void LCD::csLow()  { HAL_GPIO_WritePin(csPort_,  csPin_,  GPIO_PIN_RESET); }
void LCD::csHigh() { HAL_GPIO_WritePin(csPort_,  csPin_,  GPIO_PIN_SET);   }
void LCD::dcCmd()  { HAL_GPIO_WritePin(dcPort_,  dcPin_,  GPIO_PIN_RESET); }
void LCD::dcData() { HAL_GPIO_WritePin(dcPort_,  dcPin_,  GPIO_PIN_SET);   }

/* ---------- SPI helpers -------------------------------------------------- */

void LCD::writeCommand(uint8_t cmd)
{
    dcCmd();
    csLow();
    HAL_SPI_Transmit(&hspi_, &cmd, 1, 100);
    csHigh();
}

void LCD::writeData8(uint8_t data)
{
    dcData();
    csLow();
    HAL_SPI_Transmit(&hspi_, &data, 1, 100);
    csHigh();
}

void LCD::writeData16(uint16_t data)
{
    uint8_t buf[2] = { static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF) };
    dcData();
    csLow();
    HAL_SPI_Transmit(&hspi_, buf, 2, 100);
    csHigh();
}

void LCD::writeDataBulk(const uint8_t *data, uint32_t len)
{
    dcData();
    csLow();
    while (len > 0) {
        uint16_t chunk = (len > 65535) ? 65535 : static_cast<uint16_t>(len);
        HAL_SPI_Transmit(&hspi_, const_cast<uint8_t*>(data), chunk, 1000);
        data += chunk;
        len  -= chunk;
    }
    csHigh();
}

void LCD::setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    writeCommand(0x2A);
    writeData16(x0);
    writeData16(x1);

    writeCommand(0x2B);
    writeData16(y0);
    writeData16(y1);

    writeCommand(0x2C);
}

/* ---------- Init sequences ----------------------------------------------- */

#ifdef LCD_IC_GC9A01

void LCD::initSequence()
{
    writeCommand(0xEF);
    writeCommand(0xEB); writeData8(0x14);
    writeCommand(0xFE);
    writeCommand(0xEF);

    writeCommand(0xEB); writeData8(0x14);
    writeCommand(0x84); writeData8(0x40);
    writeCommand(0x85); writeData8(0xFF);
    writeCommand(0x86); writeData8(0xFF);
    writeCommand(0x87); writeData8(0xFF);
    writeCommand(0x88); writeData8(0x0A);
    writeCommand(0x89); writeData8(0x21);
    writeCommand(0x8A); writeData8(0x00);
    writeCommand(0x8B); writeData8(0x80);
    writeCommand(0x8C); writeData8(0x01);
    writeCommand(0x8D); writeData8(0x01);
    writeCommand(0x8E); writeData8(0xFF);
    writeCommand(0x8F); writeData8(0xFF);

    writeCommand(0xB6); writeData8(0x00); writeData8(0x00);
    writeCommand(0x3A); writeData8(0x55);

    writeCommand(0x90);
    writeData8(0x08); writeData8(0x08); writeData8(0x08); writeData8(0x08);

    writeCommand(0xBD); writeData8(0x06);
    writeCommand(0xBC); writeData8(0x00);

    writeCommand(0xFF);
    writeData8(0x60); writeData8(0x01); writeData8(0x04);

    writeCommand(0xC3); writeData8(0x13);
    writeCommand(0xC4); writeData8(0x13);
    writeCommand(0xC9); writeData8(0x22);

    writeCommand(0xBE); writeData8(0x11);
    writeCommand(0xE1); writeData8(0x10); writeData8(0x0E);
    writeCommand(0xDF); writeData8(0x21); writeData8(0x0C); writeData8(0x02);

    writeCommand(0xF0);
    writeData8(0x45); writeData8(0x09); writeData8(0x08);
    writeData8(0x08); writeData8(0x26); writeData8(0x2A);

    writeCommand(0xF1);
    writeData8(0x43); writeData8(0x70); writeData8(0x72);
    writeData8(0x36); writeData8(0x37); writeData8(0x6F);

    writeCommand(0xF2);
    writeData8(0x45); writeData8(0x09); writeData8(0x08);
    writeData8(0x08); writeData8(0x26); writeData8(0x2A);

    writeCommand(0xF3);
    writeData8(0x43); writeData8(0x70); writeData8(0x72);
    writeData8(0x36); writeData8(0x37); writeData8(0x6F);

    writeCommand(0xED); writeData8(0x1B); writeData8(0x0B);
    writeCommand(0xAE); writeData8(0x77);
    writeCommand(0xCD); writeData8(0x63);

    writeCommand(0x70);
    writeData8(0x07); writeData8(0x07); writeData8(0x04);
    writeData8(0x0E); writeData8(0x0F); writeData8(0x09);
    writeData8(0x07); writeData8(0x08); writeData8(0x03);

    writeCommand(0xE8); writeData8(0x34);

    writeCommand(0x62);
    writeData8(0x18); writeData8(0x0D); writeData8(0x71);
    writeData8(0xED); writeData8(0x70); writeData8(0x70);
    writeData8(0x18); writeData8(0x0F); writeData8(0x71);
    writeData8(0xEF); writeData8(0x70); writeData8(0x70);

    writeCommand(0x63);
    writeData8(0x18); writeData8(0x11); writeData8(0x71);
    writeData8(0xF1); writeData8(0x70); writeData8(0x70);
    writeData8(0x18); writeData8(0x13); writeData8(0x71);
    writeData8(0xF3); writeData8(0x70); writeData8(0x70);

    writeCommand(0x64);
    writeData8(0x28); writeData8(0x29); writeData8(0xF1);
    writeData8(0x01); writeData8(0xF1); writeData8(0x00); writeData8(0x07);

    writeCommand(0x66);
    writeData8(0x3C); writeData8(0x00); writeData8(0xCD);
    writeData8(0x67); writeData8(0x45); writeData8(0x45);
    writeData8(0x10); writeData8(0x00); writeData8(0x00); writeData8(0x00);

    writeCommand(0x67);
    writeData8(0x00); writeData8(0x3C); writeData8(0x00);
    writeData8(0x00); writeData8(0x00); writeData8(0x01);
    writeData8(0x54); writeData8(0x10); writeData8(0x32); writeData8(0x98);

    writeCommand(0x74);
    writeData8(0x10); writeData8(0x85); writeData8(0x80);
    writeData8(0x00); writeData8(0x00); writeData8(0x4E); writeData8(0x00);

    writeCommand(0x98); writeData8(0x3E); writeData8(0x07);
    writeCommand(0x35);
    writeCommand(0x21);
    writeCommand(0x36); writeData8(0x48);

    writeCommand(0x11);
    HAL_Delay(120);
    writeCommand(0x29);
    HAL_Delay(20);
}

#else /* LCD_IC_ST7789 */

void LCD::initSequence()
{
    writeCommand(0x01);
    HAL_Delay(150);

    writeCommand(0x11);
    HAL_Delay(120);

    writeCommand(0x36); writeData8(0x00);
    writeCommand(0x3A); writeData8(0x55);

    writeCommand(0xB2);
    writeData8(0x0C); writeData8(0x0C); writeData8(0x00);
    writeData8(0x33); writeData8(0x33);

    writeCommand(0xB7); writeData8(0x35);
    writeCommand(0xBB); writeData8(0x19);
    writeCommand(0xC0); writeData8(0x2C);
    writeCommand(0xC2); writeData8(0x01);
    writeCommand(0xC3); writeData8(0x12);
    writeCommand(0xC4); writeData8(0x20);
    writeCommand(0xC6); writeData8(0x0F);
    writeCommand(0xD0); writeData8(0xA4); writeData8(0xA1);

    writeCommand(0xE0);
    writeData8(0xD0); writeData8(0x04); writeData8(0x0D);
    writeData8(0x11); writeData8(0x13); writeData8(0x2B);
    writeData8(0x3F); writeData8(0x54); writeData8(0x4C);
    writeData8(0x18); writeData8(0x0D); writeData8(0x0B);
    writeData8(0x1F); writeData8(0x23);

    writeCommand(0xE1);
    writeData8(0xD0); writeData8(0x04); writeData8(0x0C);
    writeData8(0x11); writeData8(0x13); writeData8(0x2C);
    writeData8(0x3F); writeData8(0x44); writeData8(0x51);
    writeData8(0x2F); writeData8(0x1F); writeData8(0x1F);
    writeData8(0x20); writeData8(0x23);

    writeCommand(0x21);
    writeCommand(0x29);
    HAL_Delay(20);
}

#endif

/* ---------- Public API --------------------------------------------------- */

LCD::Status LCD::init()
{
    csHigh();
    HAL_Delay(5);
    csLow();
    HAL_Delay(20);
    csHigh();
    HAL_Delay(150);

    initSequence();
    backlightOn();

    LOGI(TAG, "Init OK (%dx%d)", width_, height_);
    return Status::OK;
}

void LCD::setRotation(Rotation rot)
{
    uint8_t madctl;

#ifdef LCD_IC_GC9A01
    switch (rot) {
        case Rotation::R0:   madctl = 0x48; break;
        case Rotation::R90:  madctl = 0x28; break;
        case Rotation::R180: madctl = 0x88; break;
        case Rotation::R270: madctl = 0xE8; break;
        default:             madctl = 0x48; break;
    }
    width_  = DEFAULT_WIDTH;
    height_ = DEFAULT_HEIGHT;
#else
    switch (rot) {
        case Rotation::R0:
            madctl = 0x00; width_ = DEFAULT_WIDTH; height_ = DEFAULT_HEIGHT; break;
        case Rotation::R90:
            madctl = 0x60; width_ = DEFAULT_HEIGHT; height_ = DEFAULT_WIDTH; break;
        case Rotation::R180:
            madctl = 0xC0; width_ = DEFAULT_WIDTH; height_ = DEFAULT_HEIGHT; break;
        case Rotation::R270:
            madctl = 0xA0; width_ = DEFAULT_HEIGHT; height_ = DEFAULT_WIDTH; break;
        default:
            madctl = 0x00; width_ = DEFAULT_WIDTH; height_ = DEFAULT_HEIGHT; break;
    }
#endif

    writeCommand(0x36);
    writeData8(madctl);
}

void LCD::fillScreen(uint16_t color)
{
    fillRect(0, 0, width_, height_, color);
}

void LCD::drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= width_ || y >= height_) return;
    setWindow(x, y, x, y);
    writeData16(color);
}

void LCD::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= width_ || y >= height_) return;
    if (x + w > width_)  w = width_  - x;
    if (y + h > height_) h = height_ - y;

    setWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = static_cast<uint8_t>(color >> 8);
    uint8_t lo = static_cast<uint8_t>(color & 0xFF);

    constexpr uint16_t LINE_BUF_PX = 240;
    uint8_t buf[LINE_BUF_PX * 2];

    uint16_t fill_w = (w > LINE_BUF_PX) ? LINE_BUF_PX : w;
    for (uint16_t i = 0; i < fill_w; i++) {
        buf[i * 2]     = hi;
        buf[i * 2 + 1] = lo;
    }

    dcData();
    csLow();
    uint32_t total = static_cast<uint32_t>(w) * h;
    while (total > 0) {
        uint16_t chunk = (total > fill_w) ? fill_w : static_cast<uint16_t>(total);
        HAL_SPI_Transmit(&hspi_, buf, chunk * 2, 1000);
        total -= chunk;
    }
    csHigh();
}

void LCD::drawChar(uint16_t x, uint16_t y, char ch, uint16_t fg, uint16_t bg)
{
    if (ch < FONT_FIRST_CHAR || ch > FONT_LAST_CHAR) ch = ' ';

    const uint8_t *glyph = Font8x16[ch - FONT_FIRST_CHAR];
    setWindow(x, y, x + FONT_WIDTH - 1, y + FONT_HEIGHT - 1);

    uint8_t buf[FONT_WIDTH * FONT_HEIGHT * 2];
    uint16_t idx = 0;

    uint8_t fg_hi = static_cast<uint8_t>(fg >> 8), fg_lo = static_cast<uint8_t>(fg & 0xFF);
    uint8_t bg_hi = static_cast<uint8_t>(bg >> 8), bg_lo = static_cast<uint8_t>(bg & 0xFF);

    for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
        uint8_t line = glyph[row];
        for (uint8_t col = 0; col < FONT_WIDTH; col++) {
            if (line & (0x80 >> col)) {
                buf[idx++] = fg_hi; buf[idx++] = fg_lo;
            } else {
                buf[idx++] = bg_hi; buf[idx++] = bg_lo;
            }
        }
    }

    writeDataBulk(buf, sizeof(buf));
}

void LCD::drawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg)
{
    while (*str) {
        if (x + FONT_WIDTH > width_) { x = 0; y += FONT_HEIGHT; }
        if (y + FONT_HEIGHT > height_) break;
        drawChar(x, y, *str, fg, bg);
        x += FONT_WIDTH;
        str++;
    }
}

void LCD::drawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
    if (data == nullptr || x >= width_ || y >= height_) return;
    setWindow(x, y, x + w - 1, y + h - 1);
    writeDataBulk(reinterpret_cast<const uint8_t*>(data), static_cast<uint32_t>(w) * h * 2);
}

void LCD::backlightOn()  { HAL_GPIO_WritePin(blkPort_, blkPin_, GPIO_PIN_SET);   }
void LCD::backlightOff() { HAL_GPIO_WritePin(blkPort_, blkPin_, GPIO_PIN_RESET); }
