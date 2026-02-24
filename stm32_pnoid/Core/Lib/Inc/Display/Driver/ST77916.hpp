#ifndef ST77916_HPP
#define ST77916_HPP

#include "../Interface/IDisplay.hpp"
#include "../Interface/IQSPI.hpp"
#include "../Common/GPIOPin.hpp"
#include "../Common/Color.hpp"
#include "../Common/Font5x7.hpp"
#include "../Common/stm32_hal_conf.hpp"

// ============================================================================
// ST77916 Commands
// ============================================================================
namespace ST77916_CMD {
    // System Commands
    constexpr uint8_t NOP       = 0x00;
    constexpr uint8_t SWRESET   = 0x01;
    constexpr uint8_t RDDID     = 0x04;
    constexpr uint8_t RDDST     = 0x09;
    constexpr uint8_t RDDPM     = 0x0A;
    constexpr uint8_t RDDMADCTL = 0x0B;
    constexpr uint8_t RDDCOLMOD = 0x0C;
    constexpr uint8_t RDDIM     = 0x0D;
    constexpr uint8_t RDDSM     = 0x0E;
    constexpr uint8_t RDDSDR    = 0x0F;
    
    // Sleep Commands
    constexpr uint8_t SLPIN     = 0x10;
    constexpr uint8_t SLPOUT    = 0x11;
    
    // Display Commands
    constexpr uint8_t PTLON     = 0x12;
    constexpr uint8_t NORON     = 0x13;
    constexpr uint8_t INVOFF    = 0x20;
    constexpr uint8_t INVON     = 0x21;
    constexpr uint8_t DISPOFF   = 0x28;
    constexpr uint8_t DISPON    = 0x29;
    
    // Column/Row Address
    constexpr uint8_t CASET     = 0x2A;
    constexpr uint8_t RASET     = 0x2B;
    constexpr uint8_t RAMWR     = 0x2C;
    constexpr uint8_t RAMRD     = 0x2E;
    
    // Partial Display
    constexpr uint8_t PTLAR     = 0x30;
    constexpr uint8_t VSCRDEF   = 0x33;
    constexpr uint8_t TEOFF     = 0x34;
    constexpr uint8_t TEON      = 0x35;
    
    // Memory Access
    constexpr uint8_t MADCTL    = 0x36;
    constexpr uint8_t VSCRSADD  = 0x37;
    constexpr uint8_t IDMOFF    = 0x38;
    constexpr uint8_t IDMON     = 0x39;
    constexpr uint8_t COLMOD    = 0x3A;
    
    // Write Memory Continue
    constexpr uint8_t RAMWRC    = 0x3C;
    constexpr uint8_t RAMRDC    = 0x3E;
    
    // Tearing Effect
    constexpr uint8_t TESCAN    = 0x44;
    constexpr uint8_t RDTESCAN  = 0x45;
    
    // Brightness
    constexpr uint8_t WRDISBV   = 0x51;
    constexpr uint8_t RDDISBV   = 0x52;
    constexpr uint8_t WRCTRLD   = 0x53;
    constexpr uint8_t RDCTRLD   = 0x54;
    
    // QSPI Specific Commands
    constexpr uint8_t SPI_MODE  = 0xC4;  // SPI Mode setting
    constexpr uint8_t QSPI_2A   = 0x02;  // QSPI 2-wire address
    constexpr uint8_t QSPI_2D   = 0x32;  // QSPI 2-wire data
    constexpr uint8_t QSPI_4A   = 0x12;  // QSPI 4-wire address
    constexpr uint8_t QSPI_4D   = 0x32;  // QSPI 4-wire data
    
    // Command2 Enable
    constexpr uint8_t CMD2EN    = 0xDF;
    
    // Frame Rate Control
    constexpr uint8_t FRMCTR1   = 0xB1;
    constexpr uint8_t FRMCTR2   = 0xB2;
    constexpr uint8_t FRMCTR3   = 0xB3;
    
    // Power Control
    constexpr uint8_t PWCTR1    = 0xC0;
    constexpr uint8_t PWCTR2    = 0xC1;
    constexpr uint8_t PWCTR3    = 0xC2;
    constexpr uint8_t PWCTR4    = 0xC3;
    constexpr uint8_t PWCTR5    = 0xC4;
    constexpr uint8_t VMCTR1    = 0xC5;
    
    // Gamma Control
    constexpr uint8_t PVGAMCTRL = 0xE0;
    constexpr uint8_t NVGAMCTRL = 0xE1;
}

// MADCTL Flags
namespace ST77916_MADCTL {
    constexpr uint8_t MY  = 0x80;  // Row Address Order
    constexpr uint8_t MX  = 0x40;  // Column Address Order
    constexpr uint8_t MV  = 0x20;  // Row/Column Exchange
    constexpr uint8_t ML  = 0x10;  // Vertical Refresh Order
    constexpr uint8_t RGB = 0x00;  // RGB color order
    constexpr uint8_t BGR = 0x08;  // BGR color order
    constexpr uint8_t MH  = 0x04;  // Horizontal Refresh Order
}

// ============================================================================
// ST77916 Configuration
// ============================================================================
struct ST77916_Config {
    uint16_t width;
    uint16_t height;
    uint16_t xOffset;
    uint16_t yOffset;
    GPIOPin rstPin;     // Reset pin
    GPIOPin tePin;      // Tearing Effect pin (optional)
    bool useTE;         // Use Tearing Effect pin
    bool useQuadMode;   // Use Quad SPI mode (4 data lines)
    
    // Default constructor - 360x360 round display
    ST77916_Config() 
        : width(360), height(360), xOffset(0), yOffset(0)
        , useTE(false), useQuadMode(true) {}
    
    // Builder pattern methods
    ST77916_Config& setSize(uint16_t w, uint16_t h) { width = w; height = h; return *this; }
    ST77916_Config& setOffset(uint16_t x, uint16_t y) { xOffset = x; yOffset = y; return *this; }
    ST77916_Config& setRSTPin(GPIO_TypeDef* port, uint16_t pin) { rstPin = GPIOPin(port, pin); return *this; }
    ST77916_Config& setTEPin(GPIO_TypeDef* port, uint16_t pin) { tePin = GPIOPin(port, pin); useTE = true; return *this; }
    ST77916_Config& setQuadMode(bool enable) { useQuadMode = enable; return *this; }
};

// ============================================================================
// ST77916 Display Class Implementation
// ============================================================================
class ST77916 : public IDisplay {
private:
    IQSPI* qspi_;
    ST77916_Config config_;
    
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
    void writeDataBuffer(const uint8_t* data, uint32_t size);
    void writeDataBuffer16(const uint16_t* data, uint32_t size);
    void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void hardwareReset();
    void initSequence();
    
    // Drawing helpers
    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
    void drawCharAt(int16_t x, int16_t y, char c, uint16_t color, uint16_t bg, uint8_t size);
    
    // QSPI mode helper
    QSPIMode getDataMode() const { 
        return config_.useQuadMode ? QSPIMode::Mode_4Lines : QSPIMode::Mode_1Line; 
    }
    
public:
    ST77916(IQSPI* qspi, const ST77916_Config& config);
    virtual ~ST77916() = default;
    
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
    // Additional ST77916-specific Methods
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
    
    // Tearing effect control
    void setTearingEffect(bool enable, bool mode = false);
    
    // Brightness control
    void setBrightness(uint8_t brightness);
    
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
    
    // Enable/disable quad mode
    void setQuadMode(bool enable);
};

#endif // ST77916_HPP
