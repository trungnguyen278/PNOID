#ifndef IQSPI_HPP
#define IQSPI_HPP

#include <cstdint>

// ============================================================================
// QSPI Interface - Abstract interface for QSPI communication
// ============================================================================

// QSPI Mode
enum class QSPIMode : uint8_t {
    Mode_1Line = 1,     // Single line (standard SPI)
    Mode_2Lines = 2,    // Dual line
    Mode_4Lines = 4     // Quad line
};

// QSPI Command structure
struct QSPICommand {
    uint8_t instruction;
    uint32_t address;
    uint8_t addressSize;    // 0, 1, 2, 3, or 4 bytes
    uint8_t dummyCycles;    // Number of dummy cycles
    QSPIMode instructionMode;
    QSPIMode addressMode;
    QSPIMode dataMode;
    bool hasAddress;
    bool hasData;
    
    QSPICommand() 
        : instruction(0), address(0), addressSize(0), dummyCycles(0)
        , instructionMode(QSPIMode::Mode_1Line)
        , addressMode(QSPIMode::Mode_1Line)
        , dataMode(QSPIMode::Mode_1Line)
        , hasAddress(false), hasData(false) {}
    
    // Builder pattern
    QSPICommand& setInstruction(uint8_t inst, QSPIMode mode = QSPIMode::Mode_1Line) {
        instruction = inst;
        instructionMode = mode;
        return *this;
    }
    
    QSPICommand& setAddress(uint32_t addr, uint8_t size, QSPIMode mode = QSPIMode::Mode_1Line) {
        address = addr;
        addressSize = size;
        addressMode = mode;
        hasAddress = true;
        return *this;
    }
    
    QSPICommand& setDummyCycles(uint8_t cycles) {
        dummyCycles = cycles;
        return *this;
    }
    
    QSPICommand& setDataMode(QSPIMode mode) {
        dataMode = mode;
        hasData = true;
        return *this;
    }
};

class IQSPI {
public:
    virtual ~IQSPI() = default;
    
    // Send command only (no data)
    virtual void sendCommand(uint8_t cmd) = 0;
    
    // Send command with data
    virtual void sendCommand(const QSPICommand& cmd, const uint8_t* data, uint32_t size) = 0;
    
    // Send command and receive data
    virtual void receiveCommand(const QSPICommand& cmd, uint8_t* data, uint32_t size) = 0;
    
    // Transmit data (after command)
    virtual void transmit(const uint8_t* data, uint32_t size, QSPIMode mode = QSPIMode::Mode_4Lines) = 0;
    
    // Transmit 16-bit data
    virtual void transmit16(const uint16_t* data, uint32_t size, QSPIMode mode = QSPIMode::Mode_4Lines) = 0;
    
    // Check if busy
    virtual bool isBusy() const = 0;
    
    // Wait for completion
    virtual void waitComplete() const {
        while (isBusy()) {}
    }
};

#endif // IQSPI_HPP
