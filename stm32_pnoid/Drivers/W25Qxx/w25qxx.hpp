/**
 * @file    w25qxx.hpp
 * @brief   W25Q64JV QSPI Flash Driver (C++ OOP)
 * @note    Uses QUADSPI peripheral via HAL. Flash size: 8MB (64Mbit)
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class W25Qxx {
public:
    /* Flash specifications */
    static constexpr uint32_t CHIP_SIZE     = 8 * 1024 * 1024;  // 8 MB
    static constexpr uint32_t PAGE_SIZE      = 256;
    static constexpr uint32_t SECTOR_SIZE    = 4 * 1024;         // 4 KB
    static constexpr uint32_t BLOCK_SIZE_32K = 32 * 1024;
    static constexpr uint32_t BLOCK_SIZE_64K = 64 * 1024;
    static constexpr uint32_t JEDEC_ID       = 0xEF4017;
    static constexpr uint32_t MMAP_BASE      = 0x90000000;

    enum class Status {
        OK = 0,
        ErrInit,
        ErrID,
        ErrWriteEnable,
        ErrAutoPolling,
        ErrErase,
        ErrRead,
        ErrWrite,
        ErrMemoryMapped,
        ErrTimeout,
    };

    /**
     * @brief  Construct with QSPI handle from CubeMX
     */
    explicit W25Qxx(QSPI_HandleTypeDef &hqspi);

    /**
     * @brief  Initialize: reset device, verify JEDEC ID, enable QE bit
     */
    Status init();

    /**
     * @brief  Read JEDEC ID (expect 0xEF4017 for W25Q64)
     */
    Status readJEDECID(uint32_t &id);

    /**
     * @brief  Erase a 4KB sector
     * @param  address  Any address within the sector
     */
    Status eraseSector(uint32_t address);

    /**
     * @brief  Erase a 64KB block
     */
    Status eraseBlock64K(uint32_t address);

    /**
     * @brief  Erase entire chip (takes several seconds)
     */
    Status eraseChip();

    /**
     * @brief  Read data using Quad I/O mode (cmd 0xEB)
     */
    Status readQuad(uint32_t address, uint8_t *data, uint32_t size);

    /**
     * @brief  Program a single page (max 256 bytes) in Quad mode (cmd 0x32)
     */
    Status programPageQuad(uint32_t address, const uint8_t *data, uint32_t size);

    /**
     * @brief  Write arbitrary data with automatic page splitting
     */
    Status writeData(uint32_t address, const uint8_t *data, uint32_t size);

    /**
     * @brief  Enable memory-mapped mode. Flash readable at 0x90000000
     * @note   After calling this, no other QSPI commands can be issued
     */
    Status enableMemoryMapped();

private:
    QSPI_HandleTypeDef &hqspi_;

    static constexpr uint32_t TIMEOUT_DEFAULT = 5000;
    static constexpr uint32_t TIMEOUT_ERASE   = 30000;

    /* Commands */
    static constexpr uint8_t CMD_WRITE_ENABLE      = 0x06;
    static constexpr uint8_t CMD_READ_STATUS_REG1   = 0x05;
    static constexpr uint8_t CMD_READ_STATUS_REG2   = 0x35;
    static constexpr uint8_t CMD_WRITE_STATUS_REG2  = 0x31;
    static constexpr uint8_t CMD_READ_JEDEC_ID      = 0x9F;
    static constexpr uint8_t CMD_SECTOR_ERASE       = 0x20;
    static constexpr uint8_t CMD_BLOCK_ERASE_64K    = 0xD8;
    static constexpr uint8_t CMD_CHIP_ERASE         = 0xC7;
    static constexpr uint8_t CMD_QUAD_READ          = 0xEB;
    static constexpr uint8_t CMD_QUAD_PAGE_PROGRAM  = 0x32;
    static constexpr uint8_t CMD_ENABLE_RESET       = 0x66;
    static constexpr uint8_t CMD_RESET_DEVICE       = 0x99;

    static constexpr uint8_t SR1_BUSY = 0x01;
    static constexpr uint8_t SR1_WEL  = 0x02;
    static constexpr uint8_t SR2_QE   = 0x02;

    Status sendCommand(uint32_t instruction, uint32_t address,
                       uint32_t addressMode, uint32_t addressSize,
                       uint32_t dataMode, uint32_t dummyCycles, uint32_t nbData);
    Status writeEnable();
    Status waitBusy(uint32_t timeout);
    Status resetDevice();
    Status enableQE();
};
