/**
 * @file debug_log.h
 * @brief Debug logging via USART with log levels and ANSI color support
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
 *   #define LOG_COLOR_ENABLE  1   — ANSI color codes (default: enabled)
 *   #define LOG_LEVEL  LOG_LEVEL_DEBUG  — minimum log level (default: DEBUG)
 */

#ifndef DEBUG_LOG_H_
#define DEBUG_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

/* UART handle for log output — defined in main.c */
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

/* ---------- Internal log macro ------------------------------------------- */

#define _LOG_IMPL(color, level_char, tag, fmt, ...) do { \
    char _log_buf[LOG_BUF_SIZE]; \
    int _log_len = snprintf(_log_buf, LOG_BUF_SIZE, \
        color level_char " (%lu) %s: " fmt LOG_COLOR_RESET "\r\n", \
        HAL_GetTick(), (tag), ##__VA_ARGS__); \
    if (_log_len > 0) { \
        if (_log_len > LOG_BUF_SIZE - 1) _log_len = LOG_BUF_SIZE - 1; \
        HAL_UART_Transmit(&huart1, (uint8_t*)_log_buf, (uint16_t)_log_len, HAL_MAX_DELAY); \
    } \
} while(0)

/* ---------- Public log macros -------------------------------------------- */

#if LOG_LEVEL >= LOG_LEVEL_ERROR
  #define LOGE(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_RED,    "E", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGE(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
  #define LOGW(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_YELLOW, "W", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGW(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
  #define LOGI(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_GREEN,  "I", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGI(tag, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
  #define LOGD(tag, fmt, ...) _LOG_IMPL(LOG_COLOR_CYAN,   "D", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGD(tag, fmt, ...) ((void)0)
#endif

/* ---------- Legacy compatibility ----------------------------------------- */
#define LOG(tag, fmt, ...) LOGI(tag, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOG_H_ */
