/**
 * @file debug_log.h
 * @brief Debug logging via USART1
 */

#ifndef DEBUG_LOG_H_
#define DEBUG_LOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

#define LOG_BUF_SIZE 256

/**
 * @brief Printf-style log over USART1
 * @usage LOG("Hello %d\r\n", 42);
 */
#define LOG(fmt, ...) do { \
    char _log_buf[LOG_BUF_SIZE]; \
    int _log_len = snprintf(_log_buf, LOG_BUF_SIZE, fmt, ##__VA_ARGS__); \
    if (_log_len > 0) { \
        HAL_UART_Transmit(&huart1, (uint8_t*)_log_buf, (uint16_t)_log_len, HAL_MAX_DELAY); \
    } \
} while(0)

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOG_H_ */
