#include "../../Inc/Display/Driver/ST7789.hpp"

// ============================================================================
// ST7789 Constructor
// ============================================================================
ST7789::ST7789(ISPI* spi, const ST7789_Config& config)
    : spi_(spi)
    , config_(config)
    , width_(config.width)
    , height_(config.height)
    , xOffset_(config.xOffset)
    , yOffset_(config.yOffset)
    , rotation_(0)
    , cursorX_(0)
    , cursorY_(0)
    , textColor_(Color::WHITE)
    , textBgColor_(Color::BLACK)
    , textSize_(1)
{
}

// ============================================================================
// Private Helper Methods
// ============================================================================
void ST7789::selectCS() {
    if (config_.useCS) {
        config_.csPin.low();
    }
}

void ST7789::deselectCS() {
    if (config_.useCS) {
        config_.csPin.high();
    }
}

void ST7789::writeCommand(uint8_t cmd) {
    config_.dcPin.low();  // Command mode
    selectCS();
    spi_->transmit(&cmd, 1);
    deselectCS();
}

void ST7789::writeData(uint8_t data) {
    config_.dcPin.high();  // Data mode
    selectCS();
    spi_->transmit(&data, 1);
    deselectCS();
}

void ST7789::writeData16(uint16_t data) {
    uint8_t buf[2] = {
        static_cast<uint8_t>(data >> 8),
        static_cast<uint8_t>(data & 0xFF)
    };
    config_.dcPin.high();
    selectCS();
    spi_->transmit(buf, 2);
    deselectCS();
}

void ST7789::writeDataBuffer(const uint8_t* data, uint16_t size) {
    config_.dcPin.high();
    selectCS();
    spi_->transmit(data, size);
    deselectCS();
}

void ST7789::setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    x0 += xOffset_;
    x1 += xOffset_;
    y0 += yOffset_;
    y1 += yOffset_;
    
    // Column address set
    writeCommand(ST7789_CMD::CASET);
    uint8_t colData[4] = {
        static_cast<uint8_t>(x0 >> 8),
        static_cast<uint8_t>(x0 & 0xFF),
        static_cast<uint8_t>(x1 >> 8),
        static_cast<uint8_t>(x1 & 0xFF)
    };
    writeDataBuffer(colData, 4);
    
    // Row address set
    writeCommand(ST7789_CMD::RASET);
    uint8_t rowData[4] = {
        static_cast<uint8_t>(y0 >> 8),
        static_cast<uint8_t>(y0 & 0xFF),
        static_cast<uint8_t>(y1 >> 8),
        static_cast<uint8_t>(y1 & 0xFF)
    };
    writeDataBuffer(rowData, 4);
    
    // Write to RAM
    writeCommand(ST7789_CMD::RAMWR);
}

void ST7789::hardwareReset() {
    config_.rstPin.high();
    HAL_Delay(10);
    config_.rstPin.low();
    HAL_Delay(10);
    config_.rstPin.high();
    HAL_Delay(120);
}

void ST7789::softwareReset() {
    writeCommand(ST7789_CMD::SWRESET);
    HAL_Delay(150);
}

void ST7789::initSequence() {
    // Sleep out
    writeCommand(ST7789_CMD::SLPOUT);
    HAL_Delay(120);
    
    // Color mode - 16bit/pixel
    writeCommand(ST7789_CMD::COLMOD);
    writeData(0x55);  // 16-bit color
    HAL_Delay(10);
    
    // Memory data access control
    writeCommand(ST7789_CMD::MADCTL);
    writeData(0x00);
    
    // Porch setting
    writeCommand(ST7789_CMD::FRMCTR2);
    uint8_t porchData[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    writeDataBuffer(porchData, 5);
    
    // Gate control
    writeCommand(ST7789_CMD::GCTRL);
    writeData(0x35);
    
    // VCOM setting
    writeCommand(ST7789_CMD::VMCTR1);
    writeData(0x1F);
    
    // LCM Control
    writeCommand(ST7789_CMD::PWCTR1);
    writeData(0x2C);
    
    // VDV and VRH Command Enable
    writeCommand(ST7789_CMD::PWCTR3);
    writeData(0x01);
    
    // VRH Set
    writeCommand(ST7789_CMD::PWCTR4);
    writeData(0x12);
    
    // VDV Set
    writeCommand(ST7789_CMD::PWCTR5);
    writeData(0x20);
    
    // Frame Rate Control in Normal Mode
    writeCommand(ST7789_CMD::FRCTRL2);
    writeData(0x0F);  // 60Hz
    
    // Power Control 1
    writeCommand(0xD0);
    uint8_t pwrData[] = {0xA4, 0xA1};
    writeDataBuffer(pwrData, 2);
    
    // Positive Voltage Gamma Control
    writeCommand(ST7789_CMD::PVGAMCTRL);
    uint8_t pvGamma[] = {0xD0, 0x08, 0x11, 0x08, 0x0C, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2D};
    writeDataBuffer(pvGamma, 14);
    
    // Negative Voltage Gamma Control
    writeCommand(ST7789_CMD::NVGAMCTRL);
    uint8_t nvGamma[] = {0xD0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0B, 0x16, 0x14, 0x2F, 0x31};
    writeDataBuffer(nvGamma, 14);
    
    // Inversion ON (for correct colors)
    writeCommand(ST7789_CMD::INVON);
    
    // Normal Display Mode ON
    writeCommand(ST7789_CMD::NORON);
    HAL_Delay(10);
    
    // Display ON
    writeCommand(ST7789_CMD::DISPON);
    HAL_Delay(10);
}

// ============================================================================
// Public Methods - IDisplay Implementation
// ============================================================================
void ST7789::init() {
    // Deselect CS if used
    if (config_.useCS) {
        config_.csPin.high();
    }
    
    // Hardware reset
    hardwareReset();
    
    // Initialization sequence
    initSequence();
    
    // Set default rotation
    setRotation(0);
    
    // Turn on backlight if available
    if (config_.useBL) {
        setBacklight(true);
    }
    
    // Clear screen
    fillScreen(Color::BLACK);
}

void ST7789::reset() {
    hardwareReset();
    initSequence();
}

void ST7789::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= static_cast<int16_t>(width_) || 
        y < 0 || y >= static_cast<int16_t>(height_)) {
        return;
    }
    
    setAddressWindow(x, y, x, y);
    writeData16(color);
}

void ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (x >= static_cast<int16_t>(width_) || y >= static_cast<int16_t>(height_)) return;
    if (x + w - 1 < 0 || y + h - 1 < 0) return;
    
    // Clip to display bounds
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > static_cast<int16_t>(width_)) { w = width_ - x; }
    if (y + h > static_cast<int16_t>(height_)) { h = height_ - y; }
    
    setAddressWindow(x, y, x + w - 1, y + h - 1);
    
    uint8_t colorH = color >> 8;
    uint8_t colorL = color & 0xFF;
    
    config_.dcPin.high();
    selectCS();
    
    // Send color data
    uint8_t colorBuf[2] = {colorH, colorL};
    uint32_t totalPixels = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);
    
    for (uint32_t i = 0; i < totalPixels; i++) {
        spi_->transmit(colorBuf, 2);
    }
    
    deselectCS();
}

void ST7789::fillScreen(uint16_t color) {
    fillRect(0, 0, width_, height_, color);
}

void ST7789::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    fillRect(x, y, w, 1, color);
}

void ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    fillRect(x, y, 1, h, color);
}

void ST7789::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    // Bresenham's line algorithm
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    
    if (steep) {
        int16_t temp = x0; x0 = y0; y0 = temp;
        temp = x1; x1 = y1; y1 = temp;
    }
    
    if (x0 > x1) {
        int16_t temp = x0; x0 = x1; x1 = temp;
        temp = y0; y0 = y1; y1 = temp;
    }
    
    int16_t dx = x1 - x0;
    int16_t dy = abs(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep = (y0 < y1) ? 1 : -1;
    
    for (; x0 <= x1; x0++) {
        if (steep) {
            drawPixel(y0, x0, color);
        } else {
            drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void ST7789::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    drawFastHLine(x, y, w, color);
    drawFastHLine(x, y + h - 1, w, color);
    drawFastVLine(x, y, h, color);
    drawFastVLine(x + w - 1, y, h, color);
}

void ST7789::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    drawPixel(x0, y0 + r, color);
    drawPixel(x0, y0 - r, color);
    drawPixel(x0 + r, y0, color);
    drawPixel(x0 - r, y0, color);
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        drawPixel(x0 + x, y0 + y, color);
        drawPixel(x0 - x, y0 + y, color);
        drawPixel(x0 + x, y0 - y, color);
        drawPixel(x0 - x, y0 - y, color);
        drawPixel(x0 + y, y0 + x, color);
        drawPixel(x0 - y, y0 + x, color);
        drawPixel(x0 + y, y0 - x, color);
        drawPixel(x0 - y, y0 - x, color);
    }
}

void ST7789::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    drawFastVLine(x0, y0 - r, 2 * r + 1, color);
    
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        drawFastVLine(x0 + x, y0 - y, 2 * y + 1, color);
        drawFastVLine(x0 - x, y0 - y, 2 * y + 1, color);
        drawFastVLine(x0 + y, y0 - x, 2 * x + 1, color);
        drawFastVLine(x0 - y, y0 - x, 2 * x + 1, color);
    }
}

void ST7789::setCursor(int16_t x, int16_t y) {
    cursorX_ = x;
    cursorY_ = y;
}

void ST7789::setTextColor(uint16_t color) {
    textColor_ = color;
}

void ST7789::setTextSize(uint8_t size) {
    textSize_ = (size > 0) ? size : 1;
}

void ST7789::drawCharAt(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size) {
    if (c < 32 || c > 127) c = '?';
    
    const uint8_t* charData = Font5x7[c - 32];
    
    for (int8_t i = 0; i < 5; i++) {
        uint8_t line = charData[i];
        for (int8_t j = 0; j < 7; j++) {
            if (line & (1 << j)) {
                if (size == 1) {
                    drawPixel(x + i, y + j, color);
                } else {
                    fillRect(x + i * size, y + j * size, size, size, color);
                }
            } else if (bg != color) {
                if (size == 1) {
                    drawPixel(x + i, y + j, bg);
                } else {
                    fillRect(x + i * size, y + j * size, size, size, bg);
                }
            }
        }
    }
}

void ST7789::printChar(char c) {
    if (c == '\n') {
        cursorX_ = 0;
        cursorY_ += textSize_ * 8;
    } else if (c == '\r') {
        cursorX_ = 0;
    } else {
        if (cursorX_ + textSize_ * 6 > static_cast<int16_t>(width_)) {
            cursorX_ = 0;
            cursorY_ += textSize_ * 8;
        }
        drawCharAt(cursorX_, cursorY_, c, textColor_, textBgColor_, textSize_);
        cursorX_ += textSize_ * 6;
    }
}

void ST7789::print(const char* str) {
    while (*str) {
        printChar(*str++);
    }
}

void ST7789::setRotation(uint8_t rotation) {
    rotation_ = rotation % 4;
    
    writeCommand(ST7789_CMD::MADCTL);
    
    switch (rotation_) {
        case 0:
            writeData(MADCTL::MX | MADCTL::MY | MADCTL::RGB);
            width_ = config_.width;
            height_ = config_.height;
            xOffset_ = config_.xOffset;
            yOffset_ = config_.yOffset;
            break;
        case 1:
            writeData(MADCTL::MY | MADCTL::MV | MADCTL::RGB);
            width_ = config_.height;
            height_ = config_.width;
            xOffset_ = config_.yOffset;
            yOffset_ = config_.xOffset;
            break;
        case 2:
            writeData(MADCTL::RGB);
            width_ = config_.width;
            height_ = config_.height;
            xOffset_ = config_.xOffset;
            yOffset_ = config_.yOffset;
            break;
        case 3:
            writeData(MADCTL::MX | MADCTL::MV | MADCTL::RGB);
            width_ = config_.height;
            height_ = config_.width;
            xOffset_ = config_.yOffset;
            yOffset_ = config_.xOffset;
            break;
    }
}

void ST7789::setBacklight(bool on) {
    if (config_.useBL) {
        config_.blPin.write(on);
    }
}

void ST7789::invertDisplay(bool invert) {
    writeCommand(invert ? ST7789_CMD::INVON : ST7789_CMD::INVOFF);
}

void ST7789::drawImage(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* data) {
    if (x >= static_cast<int16_t>(width_) || y >= static_cast<int16_t>(height_)) return;
    if (x + w <= 0 || y + h <= 0) return;
    
    setAddressWindow(x, y, x + w - 1, y + h - 1);
    
    config_.dcPin.high();
    selectCS();
    
    uint32_t totalPixels = static_cast<uint32_t>(w) * static_cast<uint32_t>(h);
    for (uint32_t i = 0; i < totalPixels; i++) {
        uint16_t color = data[i];
        uint8_t buf[2] = {
            static_cast<uint8_t>(color >> 8),
            static_cast<uint8_t>(color & 0xFF)
        };
        spi_->transmit(buf, 2);
    }
    
    deselectCS();
}

void ST7789::setScrollArea(uint16_t top, uint16_t scrollArea, uint16_t bottom) {
    writeCommand(ST7789_CMD::VSCRDEF);
    uint8_t data[6] = {
        static_cast<uint8_t>(top >> 8),
        static_cast<uint8_t>(top & 0xFF),
        static_cast<uint8_t>(scrollArea >> 8),
        static_cast<uint8_t>(scrollArea & 0xFF),
        static_cast<uint8_t>(bottom >> 8),
        static_cast<uint8_t>(bottom & 0xFF)
    };
    writeDataBuffer(data, 6);
}

void ST7789::scrollTo(uint16_t y) {
    writeCommand(ST7789_CMD::VSCRSADD);
    uint8_t data[2] = {
        static_cast<uint8_t>(y >> 8),
        static_cast<uint8_t>(y & 0xFF)
    };
    writeDataBuffer(data, 2);
}

void ST7789::sleep() {
    writeCommand(ST7789_CMD::SLPIN);
    HAL_Delay(120);
}

void ST7789::wakeup() {
    writeCommand(ST7789_CMD::SLPOUT);
    HAL_Delay(120);
}

void ST7789::displayOn() {
    writeCommand(ST7789_CMD::DISPON);
}

void ST7789::displayOff() {
    writeCommand(ST7789_CMD::DISPOFF);
}

void ST7789::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
                          int16_t x2, int16_t y2, uint16_t color) {
    drawLine(x0, y0, x1, y1, color);
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x0, y0, color);
}

void ST7789::fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
                          int16_t x2, int16_t y2, uint16_t color) {
    // Sort vertices by y-coordinate
    if (y0 > y1) { 
        int16_t t = y0; y0 = y1; y1 = t; 
        t = x0; x0 = x1; x1 = t; 
    }
    if (y1 > y2) { 
        int16_t t = y1; y1 = y2; y2 = t; 
        t = x1; x1 = x2; x2 = t; 
    }
    if (y0 > y1) { 
        int16_t t = y0; y0 = y1; y1 = t; 
        t = x0; x0 = x1; x1 = t; 
    }
    
    if (y0 == y2) {
        int16_t minX = x0, maxX = x0;
        if (x1 < minX) minX = x1;
        if (x2 < minX) minX = x2;
        if (x1 > maxX) maxX = x1;
        if (x2 > maxX) maxX = x2;
        drawFastHLine(minX, y0, maxX - minX + 1, color);
        return;
    }
    
    int16_t dx01 = x1 - x0, dy01 = y1 - y0;
    int16_t dx02 = x2 - x0, dy02 = y2 - y0;
    int16_t dx12 = x2 - x1, dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;
    int16_t last = (y1 == y2) ? y1 : y1 - 1;
    
    for (int16_t y = y0; y <= last; y++) {
        int16_t a = x0 + sa / dy01;
        int16_t b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if (a > b) { int16_t t = a; a = b; b = t; }
        drawFastHLine(a, y, b - a + 1, color);
    }
    
    sa = dx12 * (last + 1 - y1);
    sb = dx02 * (last + 1 - y0);
    
    for (int16_t y = last + 1; y <= y2; y++) {
        int16_t a = x1 + sa / dy12;
        int16_t b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b) { int16_t t = a; a = b; b = t; }
        drawFastHLine(a, y, b - a + 1, color);
    }
}

void ST7789::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, 
                           int16_t r, uint16_t color) {
    drawFastHLine(x + r, y, w - 2 * r, color);
    drawFastHLine(x + r, y + h - 1, w - 2 * r, color);
    drawFastVLine(x, y + r, h - 2 * r, color);
    drawFastVLine(x + w - 1, y + r, h - 2 * r, color);
    
    // Draw corners
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t cx = 0;
    int16_t cy = r;
    
    while (cx < cy) {
        if (f >= 0) {
            cy--;
            ddF_y += 2;
            f += ddF_y;
        }
        cx++;
        ddF_x += 2;
        f += ddF_x;
        
        drawPixel(x + w - 1 - r + cx, y + r - cy, color);
        drawPixel(x + w - 1 - r + cy, y + r - cx, color);
        drawPixel(x + w - 1 - r + cx, y + h - 1 - r + cy, color);
        drawPixel(x + w - 1 - r + cy, y + h - 1 - r + cx, color);
        drawPixel(x + r - cx, y + h - 1 - r + cy, color);
        drawPixel(x + r - cy, y + h - 1 - r + cx, color);
        drawPixel(x + r - cx, y + r - cy, color);
        drawPixel(x + r - cy, y + r - cx, color);
    }
}

void ST7789::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, 
                           int16_t r, uint16_t color) {
    fillRect(x + r, y, w - 2 * r, h, color);
    
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t cx = 0;
    int16_t cy = r;
    
    while (cx < cy) {
        if (f >= 0) {
            cy--;
            ddF_y += 2;
            f += ddF_y;
        }
        cx++;
        ddF_x += 2;
        f += ddF_x;
        
        drawFastVLine(x + r - cx, y + r - cy, h - 2 * r + 2 * cy, color);
        drawFastVLine(x + r - cy, y + r - cx, h - 2 * r + 2 * cx, color);
        drawFastVLine(x + w - 1 - r + cx, y + r - cy, h - 2 * r + 2 * cy, color);
        drawFastVLine(x + w - 1 - r + cy, y + r - cx, h - 2 * r + 2 * cx, color);
    }
}
