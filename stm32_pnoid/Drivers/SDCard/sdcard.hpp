/**
 * @file    sdcard.hpp
 * @brief   SD Card driver with FATFS for SDMMC1 (C++ OOP)
 * @note    Requires FATFS middleware enabled in CubeMX
 */

#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>

class SDCard {
public:
    enum class Status {
        OK = 0,
        ErrInit,
        ErrMount,
        ErrUnmount,
        ErrFile,
        ErrRead,
        ErrWrite,
        ErrNotMounted,
    };

    struct CardInfo {
        uint32_t capacityMB;
        uint32_t blockSize;
        uint32_t blockCount;
        uint8_t  cardType;
    };

    /**
     * @brief  Construct with SD handle from CubeMX
     */
    explicit SDCard(SD_HandleTypeDef &hsd);

    /**
     * @brief  Initialize SD card and mount filesystem
     */
    Status init();

    Status mount();
    Status unmount();
    bool   isMounted() const { return mounted_; }

    Status getInfo(CardInfo &info);

    Status readFile(const char *path, uint8_t *buf, uint32_t bufSize, uint32_t *bytesRead = nullptr);
    Status writeFile(const char *path, const uint8_t *data, uint32_t size);
    Status appendFile(const char *path, const uint8_t *data, uint32_t size);

    Status mkdir(const char *path);
    Status listDir(const char *path);
    Status getFreeSpace(uint32_t &freeKB);

private:
    SD_HandleTypeDef &hsd_;
    bool mounted_ = false;
};
