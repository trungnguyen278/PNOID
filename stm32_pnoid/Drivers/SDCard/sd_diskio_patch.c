/**
 * @file    sd_diskio_patch.c
 * @brief   Override __weak BSP_SD_ReadBlocks/WriteBlocks for STM32H7
 * @note    Uses 32-byte aligned scratch buffer for SDMMC IDMA.
 *          Overrides __weak functions in bsp_driver_sd.c
 */

#include "bsp_driver_sd.h"
#include "debug_log.h"
#include <string.h>

static const char *TAG = "SDIO";

extern SD_HandleTypeDef hsd1;

#define SD_BLOCK_SIZE  512

/* 32-byte aligned scratch buffer for SDMMC IDMA */
__ALIGNED(32) static uint8_t sd_scratch[SD_BLOCK_SIZE];

uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
    uint8_t *dst = (uint8_t *)pData;
    uint32_t i;

    for (i = 0; i < NumOfBlocks; i++)
    {
        if (HAL_SD_ReadBlocks(&hsd1, sd_scratch, ReadAddr + i, 1, Timeout) != HAL_OK)
        {
            LOGE(TAG, "Read fail blk %lu: err=0x%08lX", ReadAddr + i, hsd1.ErrorCode);
            return MSD_ERROR;
        }
        memcpy(dst + (i * SD_BLOCK_SIZE), sd_scratch, SD_BLOCK_SIZE);
    }

    return MSD_OK;
}

uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
    const uint8_t *src = (const uint8_t *)pData;ư
    uint32_t i;

    for (i = 0; i < NumOfBlocks; i++)
    {
        memcpy(sd_scratch, src + (i * SD_BLOCK_SIZE), SD_BLOCK_SIZE);

        if (HAL_SD_WriteBlocks(&hsd1, sd_scratch, WriteAddr + i, 1, Timeout) != HAL_OK)
        {
            LOGE(TAG, "Write fail blk %lu: err=0x%08lX", WriteAddr + i, hsd1.ErrorCode);
            return MSD_ERROR;
        }

        /* Wait for card to finish programming */
        while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {}
    }

    return MSD_OK;
}
