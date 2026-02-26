/**
 * @file    sdcard.cpp
 * @brief   SD Card driver implementation with FATFS
 * @note    Requires FATFS middleware enabled in CubeMX.
 */

#include "sdcard.hpp"
#include "fatfs.h"
#include "debug_log.h"
#include <cstring>

static const char *TAG = "SD";

static FATFS sd_fs;

/* ---------- Constructor -------------------------------------------------- */

SDCard::SDCard(SD_HandleTypeDef &hsd) : hsd_(hsd) {}

/* ---------- Public API --------------------------------------------------- */

SDCard::Status SDCard::init()
{
    auto state = HAL_SD_GetCardState(&hsd_);
    if (state == HAL_SD_CARD_ERROR) {
        LOGE(TAG, "Card not detected");
        return Status::ErrInit;
    }

    return mount();
}

SDCard::Status SDCard::mount()
{
    if (mounted_) return Status::OK;

    FRESULT res = f_mount(&sd_fs, SDPath, 1);
    if (res != FR_OK) {
        LOGE(TAG, "Mount failed: %d", res);
        return Status::ErrMount;
    }

    mounted_ = true;
    LOGI(TAG, "Mounted OK");
    return Status::OK;
}

SDCard::Status SDCard::unmount()
{
    if (!mounted_) return Status::OK;

    FRESULT res = f_mount(nullptr, SDPath, 0);
    if (res != FR_OK) return Status::ErrUnmount;

    mounted_ = false;
    return Status::OK;
}

SDCard::Status SDCard::getInfo(CardInfo &info)
{
    HAL_SD_CardInfoTypeDef ci;
    if (HAL_SD_GetCardInfo(&hsd_, &ci) != HAL_OK)
        return Status::ErrInit;

    info.blockSize  = ci.BlockSize;
    info.blockCount = ci.BlockNbr;
    info.capacityMB = static_cast<uint32_t>(
        static_cast<uint64_t>(ci.BlockNbr) * ci.BlockSize / (1024 * 1024));
    info.cardType   = ci.CardType;

    return Status::OK;
}

SDCard::Status SDCard::readFile(const char *path, uint8_t *buf,
                                 uint32_t bufSize, uint32_t *bytesRead)
{
    if (!mounted_) return Status::ErrNotMounted;

    FIL file;
    FRESULT res = f_open(&file, path, FA_READ);
    if (res != FR_OK) return Status::ErrFile;

    UINT br = 0;
    res = f_read(&file, buf, bufSize, &br);
    f_close(&file);

    if (res != FR_OK) return Status::ErrRead;
    if (bytesRead) *bytesRead = br;

    return Status::OK;
}

SDCard::Status SDCard::writeFile(const char *path, const uint8_t *data, uint32_t size)
{
    if (!mounted_) return Status::ErrNotMounted;

    FIL file;
    FRESULT res = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return Status::ErrFile;

    UINT bw = 0;
    res = f_write(&file, data, size, &bw);
    f_close(&file);

    if (res != FR_OK || bw != size) return Status::ErrWrite;
    return Status::OK;
}

SDCard::Status SDCard::appendFile(const char *path, const uint8_t *data, uint32_t size)
{
    if (!mounted_) return Status::ErrNotMounted;

    FIL file;
    FRESULT res = f_open(&file, path, FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK) return Status::ErrFile;

    UINT bw = 0;
    res = f_write(&file, data, size, &bw);
    f_close(&file);

    if (res != FR_OK || bw != size) return Status::ErrWrite;
    return Status::OK;
}

SDCard::Status SDCard::mkdir(const char *path)
{
    if (!mounted_) return Status::ErrNotMounted;

    FRESULT res = f_mkdir(path);
    if (res != FR_OK && res != FR_EXIST) return Status::ErrFile;
    return Status::OK;
}

SDCard::Status SDCard::listDir(const char *path)
{
    if (!mounted_) return Status::ErrNotMounted;

    DIR dir;
    FILINFO fno;

    FRESULT res = f_opendir(&dir, path);
    if (res != FR_OK) return Status::ErrFile;

    LOGI(TAG, "Directory: %s", path);
    LOGI(TAG, "  %-12s  %10s  %s", "Date", "Size", "Name");
    LOGI(TAG, "  ------------------------------------");

    while (true) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        if (fno.fattrib & AM_DIR) {
            LOGI(TAG, "  %04d-%02d-%02d       <DIR>  %s",
                 (fno.fdate >> 9) + 1980, (fno.fdate >> 5) & 0x0F,
                 fno.fdate & 0x1F, fno.fname);
        } else {
            LOGI(TAG, "  %04d-%02d-%02d  %10lu  %s",
                 (fno.fdate >> 9) + 1980, (fno.fdate >> 5) & 0x0F,
                 fno.fdate & 0x1F, static_cast<unsigned long>(fno.fsize), fno.fname);
        }
    }

    f_closedir(&dir);
    return Status::OK;
}

SDCard::Status SDCard::getFreeSpace(uint32_t &freeKB)
{
    if (!mounted_) return Status::ErrNotMounted;

    DWORD freClust;
    FATFS *fs;

    FRESULT res = f_getfree(SDPath, &freClust, &fs);
    if (res != FR_OK) return Status::ErrFile;

    freeKB = static_cast<uint32_t>(freClust * fs->csize) / 2;
    return Status::OK;
}
