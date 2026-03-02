/**
 * @file    debug_log_sd.cpp
 * @brief   SD card logging backend for debug_log.h
 * @note    Only compiled when LOG_SD_ENABLE == 1
 */

#include "debug_log.h"

#if LOG_SD_ENABLE

#include "fatfs.h"
#include <cstring>

#ifndef LOG_SD_FILENAME
#define LOG_SD_FILENAME  "log.txt"
#endif

#ifndef LOG_SD_SYNC_INTERVAL
#define LOG_SD_SYNC_INTERVAL  10   /* f_sync every N writes */
#endif

static FIL   logFile;
static bool  logFileOpen = false;
static int   writeCount  = 0;

extern "C" int LOG_SD_Init(void)
{
    if (logFileOpen) return 0;

    FRESULT res = f_open(&logFile, LOG_SD_FILENAME,
                         FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK) return -1;

    logFileOpen = true;
    writeCount  = 0;

    /* Write a separator on each boot */
    const char *sep = "\r\n===== BOOT =====\r\n";
    UINT bw;
    f_write(&logFile, sep, strlen(sep), &bw);
    f_sync(&logFile);

    return 0;
}

extern "C" void LOG_SD_DeInit(void)
{
    if (!logFileOpen) return;
    f_sync(&logFile);
    f_close(&logFile);
    logFileOpen = false;
}

extern "C" void LOG_SD_Write(const char *buf, int len)
{
    if (!logFileOpen || buf == nullptr || len <= 0) return;

    UINT bw;
    f_write(&logFile, buf, (UINT)len, &bw);

    /* Periodic sync to avoid data loss on power cut */
    if (++writeCount >= LOG_SD_SYNC_INTERVAL) {
        f_sync(&logFile);
        writeCount = 0;
    }
}

#endif /* LOG_SD_ENABLE */
