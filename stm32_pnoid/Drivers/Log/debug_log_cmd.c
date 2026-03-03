/**
 * @file    debug_log_cmd.c
 * @brief   UART RX command receiver — register-level RXNE interrupt
 * @note    Uses huart1 (USART1) RX.
 *          Bypasses HAL RX API to avoid lock conflict with HAL_UART_Transmit.
 *          Commands are delimited by \r, \n, or idle timeout (300ms).
 *          Callback is invoked from LOG_CMD_Poll() (main loop context, not ISR).
 */

#include "debug_log.h"
#include <string.h>

/* ---------- Configuration ------------------------------------------------ */

#ifndef LOG_CMD_TIMEOUT_MS
#define LOG_CMD_TIMEOUT_MS  300   /* idle timeout: no newline needed */
#endif

/* ---------- Internal state ----------------------------------------------- */

static char             cmdBuf[LOG_CMD_BUF_SIZE];   /* line accumulator   */
static volatile uint8_t cmdLen   = 0;               /* current length     */
static volatile uint8_t cmdReady = 0;               /* flag: line complete */
static volatile uint32_t lastRxTick = 0;            /* tick of last byte  */
static LOG_CMD_Callback_t cmdCallback = NULL;

/* ---------- Public API --------------------------------------------------- */

void LOG_CMD_Init(void)
{
    /* Enable USART1 NVIC IRQ */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* Enable RXNE (RX Not Empty) interrupt directly via register
       — no HAL_UART_Receive_IT, so no HAL lock conflict */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void LOG_CMD_RegisterCallback(LOG_CMD_Callback_t cb)
{
    cmdCallback = cb;
}

void LOG_CMD_Poll(void)
{
    /* Idle timeout: if bytes received but no \r\n, trigger after timeout */
    if (!cmdReady && cmdLen > 0) {
        if ((HAL_GetTick() - lastRxTick) >= LOG_CMD_TIMEOUT_MS) {
            cmdReady = 1;
        }
    }

    if (!cmdReady) return;

    /* Null-terminate and invoke callback */
    cmdBuf[cmdLen] = '\0';

    if (cmdCallback && cmdLen > 0) {
        cmdCallback(cmdBuf);
    }

    /* Reset for next command */
    cmdLen   = 0;
    cmdReady = 0;
}

/* ---------- IRQ Handler -------------------------------------------------- */

void USART1_IRQHandler(void)
{
    /* Check RXNE flag (RX data register not empty) */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        /* Read byte — this also clears RXNE flag */
        uint8_t ch = (uint8_t)(huart1.Instance->RDR & 0xFF);

        lastRxTick = HAL_GetTick();

        if (ch == '\r' || ch == '\n') {
            /* Explicit end-of-line */
            if (cmdLen > 0) {
                cmdReady = 1;
            }
        } else if (!cmdReady) {
            /* Accumulate printable characters */
            if (cmdLen < LOG_CMD_BUF_SIZE - 1) {
                cmdBuf[cmdLen++] = (char)ch;
            }
        }
    }

    /* Clear overrun error if any — prevents stuck interrupt */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
    }
}
