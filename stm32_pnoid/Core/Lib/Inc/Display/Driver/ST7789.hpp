#ifndef ST7789_HPP
#define ST7789_HPP

#include "../Interface/IDisplay.hpp"
#include "../Interface/ISPI.hpp"
#include "../Common/GPIOPin.hpp"
#include "../Common/Color.hpp"
#include "../Common/Font5x7.hpp"
#include "../Common/stm32_hal_conf.hpp"

// ============================================================================
// ST7789 Commands
// ============================================================================
namespace ST7789_CMD {
    constexpr uint8_t NOP       = 0x00;
    constexpr uint8_t SWRESET   = 0x01;
    constexpr uint8_t RDDID     = 0x04;
    constexpr uint8_t RDDST     = 0x09;
    constexpr uint8_t SLPIN     = 0x10;
    constexpr uint8_t SLPOUT    = 0x11;
    constexpr uint8_t PTLON     = 0x12;
    constexpr uint8_t NORON     = 0x13;
    constexpr uint8_t INVOFF    = 0x20;
    constexpr uint8_t INVON     = 0x21;
    constexpr uint8_t DISPOFF   = 0x28;
    constexpr uint8_t DISPON    = 0x29;
    constexpr uint8_t CASET     = 0x2A;
    constexpr uint8_t RASET     = 0x2B;
    constexpr uint8_t RAMWR     = 0x2C;
    constexpr uint8_t RAMRD     = 0x2E;
    constexpr uint8_t PTLAR     = 0x30;
    constexpr uint8_t VSCRDEF   = 0x33;
    constexpr uint8_t COLMOD    = 0x3A;
    constexpr uint8_t MADCTL    = 0x36;
    constexpr uint8_t VSCRSADD  = 0x37;
    constexpr uint8_t FRMCTR1   = 0xB1;
    constexpr uint8_t FRMCTR2   = 0xB2;
    constexpr uint8_t FRMCTR3   = 0xB3;
    constexpr uint8_t INVCTR    = 0xB4;
    constexpr uint8_t DISSET5   = 0xB6;
    constexpr uint8_t GCTRL     = 0xB7;
    constexpr uint8_t PWCTR1    = 0xC0;
    constexpr uint8_t PWCTR2    = 0xC1;
    constexpr uint8_t PWCTR3    = 0xC2;
    constexpr uint8_t PWCTR4    = 0xC3;
    constexpr uint8_t PWCTR5    = 0xC4;
    constexpr uint8_t VMCTR1    = 0xC5;
    constexpr uint8_t FRCTRL2   = 0xC6;
    constexpr uint8_t RDID1     = 0xDA;
    constexpr uint8_t RDID2     = 0xDB;
    constexpr uint8_t RDID3     = 0xDC;
    constexpr uint8_t RDID4     = 0xDD;
    constexpr uint8_t PVGAMCTRL = 0xE0;
    constexpr uint8_t NVGAMCTRL = 0xE1;
}

// MADCTL Flags
namespace MADCTL {
    constexpr uint8_t MY  = 0x80;  // Row Address Order
    constexpr uint8_t MX  = 0x40;  // Column Address Order
    constexpr uint8_t MV  = 0x20;  // Row/Column Exchange
    constexpr uint8_t ML  = 0x10;  // Vertical Refresh Order
    constexpr uint8_t RGB = 0x00;  // RGB color order
    constexpr uint8_t BGR = 0x08;  // BGR color order
    constexpr uint8_t MH  = 0x04;  // Horizontal Refresh Order
}

// ============================================================================
// ST7789 Configuration
// ============================================================================
struct ST7789_Config {
    uint16_t width;
    uint16_t height;
    uint16_t xOffset;
    uint16_t yOffset;
    GPIOPin dcPin;      // Data/Command pin
    GPIOPin rstPin;     // Reset pin
    GPIOPin csPin;      // Chip Select pin (optional)
    GPIOPin blPin;      // Backlight pin (optional)
    bool useCS;         // Use CS pin
    bool useBL;         // Use Backlight pin
    
    // Default constructor
    ST7789_Config() 
        : width(240), height(240), xOffset(0), yOffset(0)
        , useCS(false), useBL(false) {}
    
    // Builder pattern methods
    ST7789_Config& setSize(uint16_t w, uint16_t h) { width = w; height = h; return *this; }
    ST7789_Config& setOffset(uint16_t x, uint16_t y) { xOffset = x; yOffset = y; return *this; }
    ST7789_Config& setDCPin(GPIO_TypeDef* port, uint16_t pin) { dcPin = GPIOPin(port, pin); return *this; }
    ST7789_Config& setRSTPin(GPIO_TypeDef* port, uint16_t pin) { rstPin = GPIOPin(port, pin); return *this; }
    ST7789_Config& setCSPin(GPIO_TypeDef* port, uint16_t pin) { csPin = GPIOPin(port, pin); useCS = true; return *this; }
    ST7789_Config& setBLPin(GPIO_TypeDef* port, uint16_t pin) { blPin = GPIOPin(port, pin); useBL = true; return *this; }
};

// ============================================================================
// ST7789 Display Class Implementation
// ============================================================================
class ST7789 : public IDisplay {
private:
    ISPI* spi_;
    ST7789_Config config_;
    
    uint16_t width_;
    uint16_t height_;
    uint16_t xOffset_;
    uint16_t yOffset_;
    uint8_t rotation_;
    
    // Text properties
    int16_t cursorX_;
    int16_t cursorY_;
    uint16_t textColor_;
    uint16_t textBgColor_;
    uint8_t textSize_;
    
    // Private helper methods
    void writeCommand(uint8_t cmd);
    void writeData(uint8_t data);
    void writeData16(uint16_t data);
    void writeDataBuffer(const uint8_t* data, uint16_t size);
    void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void selectCS();
    void deselectCS();
    void hardwareReset();
    void softwareReset();
    void initSequence();
    
    // Drawing helpers
    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    void drawCharAt(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
    
public:
    ST7789(ISPI* spi, const ST7789_Config& config);
    virtual ~ST7789() = default;
    
    // ========================================================================
    // IDisplay Implementation
    // ========================================================================
    void init() override;
    void reset() override;
    
    void drawPixel(int16_t x, int16_t y, uint16_t color) override;
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;
    void fillScreen(uint16_t color) override;
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) override;
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override;
    void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) override;
    void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) override;
    
    void setCursor(int16_t x, int16_t y) override;
    void setTextColor(uint16_t color) override;
    void setTextSize(uint8_t size) override;
    void print(const char* str) override;
    void printChar(char c) override;
    
    uint16_t getWidth() const override { return width_; }
    uint16_t getHeight() const override { return height_; }
    void setRotation(uint8_t rotation) override;
    
    void setBacklight(bool on) override;
    void invertDisplay(bool invert) override;
    
    // ========================================================================
    // Additional ST7789-specific Methods
    // ========================================================================
    void setTextBgColor(uint16_t color) { textBgColor_ = color; }
    uint16_t getTextColor() const { return textColor_; }
    uint16_t getTextBgColor() const { return textBgColor_; }
    uint8_t getTextSize() const { return textSize_; }
    int16_t getCursorX() const { return cursorX_; }
    int16_t getCursorY() const { return cursorY_; }
    uint8_t getRotation() const { return rotation_; }
    
    void drawImage(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* data);
    void scrollTo(uint16_t y);
    void setScrollArea(uint16_t top, uint16_t scrollArea, uint16_t bottom);
    
    void sleep();
    void wakeup();
    void displayOn();
    void displayOff();
    
    // Draw triangle
    void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
                      int16_t x2, int16_t y2, uint16_t color);
    void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
                      int16_t x2, int16_t y2, uint16_t color);
    
    // Draw rounded rectangle
    void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, 
                       int16_t r, uint16_t color);
    void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, 
                       int16_t r, uint16_t color);
};

#endif // ST7789_HPP
