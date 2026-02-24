#ifndef IDISPLAY_HPP
#define IDISPLAY_HPP

#include <cstdint>

// ============================================================================
// Display Interface - Abstract interface for display devices
// ============================================================================
class IDisplay {
public:
    virtual ~IDisplay() = default;
    
    // ========================================================================
    // Initialization
    // ========================================================================
    virtual void init() = 0;
    virtual void reset() = 0;
    
    // ========================================================================
    // Basic Drawing
    // ========================================================================
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color) = 0;
    virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) = 0;
    virtual void fillScreen(uint16_t color) = 0;
    virtual void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) = 0;
    virtual void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) = 0;
    virtual void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) = 0;
    virtual void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) = 0;
    
    // ========================================================================
    // Text Rendering
    // ========================================================================
    virtual void setCursor(int16_t x, int16_t y) = 0;
    virtual void setTextColor(uint16_t color) = 0;
    virtual void setTextSize(uint8_t size) = 0;
    virtual void print(const char* str) = 0;
    virtual void printChar(char c) = 0;
    
    // ========================================================================
    // Display Properties
    // ========================================================================
    virtual uint16_t getWidth() const = 0;
    virtual uint16_t getHeight() const = 0;
    virtual void setRotation(uint8_t rotation) = 0;
    
    // ========================================================================
    // Display Control
    // ========================================================================
    virtual void setBacklight(bool on) = 0;
    virtual void invertDisplay(bool invert) = 0;
};

#endif // IDISPLAY_HPP
