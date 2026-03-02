/**
 * @file debug_log.h
 * @brief Debug logging via USART with log levels, ANSI color, and optional SD card logging
 *
 * Usage:
 *   static const char* TAG = "APP";
 *   LOGI(TAG, "System started, clock = %lu MHz", sysclk);
 *   LOGW(TAG, "Buffer almost full: %d%%", usage);
 *   LOGE(TAG, "Failed to init SD card");
 *   LOGD(TAG, "Raw data: 0x%08X", val);
 *
 * Output example (with color on terminal):
 *   I (1234) APP: System started, clock = 480 MHz
 *   W (1235) APP: Buffer almost full: 90%
 *   E (1236) APP: Failed to init SD card
 *   D (1237) APP: Raw data: 0xDEADBEEF
 *
 * Config:
 *   #define LOG_COLOR_ENABLE  1   -- ANSI color codes (default: enabled)
 *   #define LOG_LEVEL  LOG_LEVEL_DEBUG  -- minimum log level (default: DEBUG)
 *   #define LOG_SD_ENABLE     1   -- also write logs to SD card (default: disabled)
 *   #define LOG_SD_LEVEL LOG_LEVEL_INFO -- min level for SD (default: INFO)
 *
 * SD card logging:
 *   Call LOG_SD_Init() after SD card + FATFS are mounted.
 *   Call LOG_SD_DeInit() before unmounting.
 *   Logs are written to "log.txt" (appended). No ANSI colors on SD.
 */

#ifndef DEBUG_LOG_H_
#define DEBUG_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

/* UART handle for log output -- defined in main.c */
extern UART_HandleTypeDef huart1;

/* ---------- Configuration ------------------------------------------------ */

#define LOG_LEVEL_NONE    0
#define LOG_LEVEL_ERROR   1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_INFO    3
#define LOG_LEVEL_DEBUG   4

#ifndef LOG_LEVEL
#define LOG_LEVEL         LOG_LEVEL_DEBUG
#endif

#ifndef LOG_COLOR_ENABLE
#define LOG_COLOR_ENABLE  1
#endif

#ifndef LOG_BUF_SIZE
#define LOG_BUF_SIZE      256
#endif

#ifndef LOG_SD_ENABLE
#define LOG_SD_ENABLE     1
#endif

#ifndef LOG_SD_LEVEL
#define LOG_SD_LEVEL      LOG_LEVEL_INFO
#endif

/* ---------- ANSI Color codes --------------------------------------------- */

#if LOG_COLOR_ENABLE
  #define LOG_COLOR_RED     "\033[31m"
  #define LOG_COLOR_YELLOW  "\033[33m"
  #define LOG_COLOR_GREEN   "\033[32m"
  #define LOG_COLOR_CYAN    "\033[36m"
  #define LOG_COLOR_RESET   "\033[0m"
#else
  #define LOG_COLOR_RED     ""
  #define LOG_COLOR_YELLOW  ""
  #define LOG_COLOR_GREEN   ""
  #define LOG_COLOR_CYAN    ""
  #define LOG_COLOR_RESET   ""
#endif

/* ---------- SD card logging API ----------------------------------------- */

#if LOG_SD_ENABLE
  /**
   * @brief  Open log file on SD card (appends to LOG_SD_FILENAME)
   * @retval 0 on success, -1 on failure
   * @note   Call after FATFS mount. Defined in debug_log_sd.cpp
   */
  int  LOG_SD_Init(void);

  /**
   * @brief  Close log file on SD card
   * @note   Call before unmount or power off
   */
  void LOG_SD_DeInit(void);

  /**
   * @brief  Write a pre-formatted string to SD log file (used internally by macros)
   */
  void LOG_SD_Write(const char *buf, int len);
#endif

/* ---------- Internal log macro ------------------------------------------- */

#if LOG_SD_ENABLE

#define _LOG_IMPL(color, level_char, level_num, tag, fmt, ...) do { \
    char _log_buf[LOG_BUF_SIZE]; \
    /* UART: with color */ \
    int _log_len = snprintf(_log_buf, LOG_BUF_SIZE, \
        color level_char " (%lu) %s: " fmt LOG_COLOR_RESET "\r\n", \
        HAL_GetTick(), (tag), ##__VA_ARGS__); \
    if (_log_len > 0) { \
        if (_log_len > LOG_BUF_SIZE - 1) _log_len = LOG_BUF_SIZE - 1; \
        HAL_UART_Transmit(&huart1, (uint8_t*)_log_buf, (uint16_t)_log_len, HAL_MAX_DELAY); \
    } \
    /* SD: without color */ \
    if ((level_num) <= LOG_SD_LEVEL) { \
        int _sd_len = snprintf(_log_buf, LOG_BUF_SIZE, \
            level_char " (%lu) %s: " fmt "\r\n", \
            HAL_GetTick(), (tag), ##__VA_ARGS__); \
        if (_sd_len > 0) { \
            if (_sd_len > LOG_BUF_SIZE - 1) _sd_len = LOG_BUF_SIZE - 1; \
            LOG_SD_Write(_log_buf, _sd_len); \
        } \
    } \
} while(0)

#else /* LOG_SD_ENABLE == 0 */

#define _LOG_IMPL(color, level_char, level_num, tag, fmt, ...) do { \
    char _log_buf[LOG_BUF_SIZE]; \
    int _log_len = snprintf(_log_buf, LOG_BUF_SIZE, \
        color level_char " (%lu) %s: " fmt LOG_COLOR_RESET "\r\n", \
        HAL_GetTick(), (tag), ##__VA_ARGS__); \
    if (_log_len > 0) { \
        if (_log_len > LOG_BUF_SIZE - 1) _log_len = LOG_BUF_SIZE - 1; \
        HAL_UART_Transmit(&huart1, (uint8_t*)_log_buf, (uint16_t)_log_len, HAL_MAX_DELAY); \
    } \
} while(0)

#endif /* LOG_SD_ENABLE */

/* ---------- Public log macros -------------------------------------------- */

#if LOG_LEVEL >= LOG_LEVEL_ERROR
  #define LOGE(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_RED,    "E", LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)
#else
  #define LOGE(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
  #define LOGW(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_YELLOW, "W", LOG_LEVEL_WARN, tag, fmt, ##__VA_ARGS__)
#else
  #define LOGW(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
  #define LOGI(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_GREEN,  "I", LOG_LEVEL_INFO, tag, fmt, ##__VA_ARGS__)
#else
  #define LOGI(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
  #define LOGD(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_CYAN,   "D", LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)
#else
  #define LOGD(tag, fmt, ...) ((void)0)
#endif

/* ---------- Legacy compatibility ----------------------------------------- */
#define LOG(tag, fmt, ...) LOGI(tag, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOG_H_ */
