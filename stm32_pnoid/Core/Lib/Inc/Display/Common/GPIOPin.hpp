#ifndef GPIO_PIN_HPP
#define GPIO_PIN_HPP

#include "stm32_hal_conf.hpp"

// ============================================================================
// GPIO Pin Configuration Structure
// ============================================================================
struct GPIOPin {
    GPIO_TypeDef* port;
    uint16_t pin;
    
    GPIOPin() : port(nullptr), pin(0) {}
    GPIOPin(GPIO_TypeDef* p, uint16_t pn) : port(p), pin(pn) {}
    
    void high() const { 
        if (port) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); 
    }
    
    void low() const { 
        if (port) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); 
    }
    
    void toggle() const { 
        if (port) HAL_GPIO_TogglePin(port, pin); 
    }
    
    void write(bool state) const {
        if (port) HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    
    bool read() const {
        if (port) return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
        return false;
    }
    
    bool isValid() const {
        return port != nullptr;
    }
};

#endif // GPIO_PIN_HPP
