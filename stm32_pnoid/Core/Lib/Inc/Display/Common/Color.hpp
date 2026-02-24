#ifndef COLOR_HPP
#define COLOR_HPP

#include <cstdint>

// ============================================================================
// Color Definitions (RGB565)
// ============================================================================
namespace Color {
    constexpr uint16_t BLACK   = 0x0000;
    constexpr uint16_t WHITE   = 0xFFFF;
    constexpr uint16_t RED     = 0xF800;
    constexpr uint16_t GREEN   = 0x07E0;
    constexpr uint16_t BLUE    = 0x001F;
    constexpr uint16_t CYAN    = 0x07FF;
    constexpr uint16_t MAGENTA = 0xF81F;
    constexpr uint16_t YELLOW  = 0xFFE0;
    constexpr uint16_t ORANGE  = 0xFD20;
    constexpr uint16_t PURPLE  = 0x8010;
    constexpr uint16_t GRAY    = 0x8410;
    constexpr uint16_t DARK_GRAY   = 0x4208;
    constexpr uint16_t LIGHT_GRAY  = 0xC618;
    constexpr uint16_t NAVY    = 0x000F;
    constexpr uint16_t DARK_GREEN  = 0x03E0;
    constexpr uint16_t DARK_CYAN   = 0x03EF;
    constexpr uint16_t MAROON  = 0x7800;
    constexpr uint16_t OLIVE   = 0x7BE0;
    constexpr uint16_t PINK    = 0xFE19;
    constexpr uint16_t BROWN   = 0xA145;
    
    // Convert RGB888 to RGB565
    constexpr uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
    
    // Convert RGB565 to RGB888 components
    constexpr void toRGB888(uint16_t color, uint8_t& r, uint8_t& g, uint8_t& b) {
        r = (color >> 8) & 0xF8;
        g = (color >> 3) & 0xFC;
        b = (color << 3) & 0xF8;
    }
    
    // Blend two colors (alpha: 0-255, 0=color1, 255=color2)
    inline uint16_t blend(uint16_t color1, uint16_t color2, uint8_t alpha) {
        uint8_t r1, g1, b1, r2, g2, b2;
        toRGB888(color1, r1, g1, b1);
        toRGB888(color2, r2, g2, b2);
        
        uint8_t r = r1 + ((alpha * (r2 - r1)) >> 8);
        uint8_t g = g1 + ((alpha * (g2 - g1)) >> 8);
        uint8_t b = b1 + ((alpha * (b2 - b1)) >> 8);
        
        return RGB(r, g, b);
    }
}

#endif // COLOR_HPP
