/**
 * @file    w25qxx.cpp
 * @brief   W25Q64JV QSPI Flash Driver implementation
 */

#include "w25qxx.hpp"
#include "debug_log.h"

static const char *TAG = "W25Q";

W25Qxx::W25Qxx(QSPI_HandleTypeDef &hqspi) : hqspi_(hqspi) {}

/* ---------- Private helpers ---------------------------------------------- */

W25Qxx::Status W25Qxx::sendCommand(uint32_t instruction, uint32_t address,
                                    uint32_t addressMode, uint32_t addressSize,
                                    uint32_t dataMode, uint32_t dummyCycles,
                                    uint32_t nbData)
{
    QSPI_CommandTypeDef cmd{};
    cmd.Instruction       = instruction;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Address           = address;
    cmd.AddressMode       = addressMode;
    cmd.AddressSize       = addressSize;
    cmd.DataMode          = dataMode;
    cmd.DummyCycles       = dummyCycles;
    cmd.NbData            = nbData;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    HAL_StatusTypeDef hal_st = HAL_QSPI_Command(&hqspi_, &cmd, TIMEOUT_DEFAULT);
    if (hal_st != HAL_OK)
    {
        LOGE(TAG, "QSPI cmd 0x%02lX failed: HAL=%d, State=%lu, Err=0x%08lX",
             instruction, (int)hal_st, (uint32_t)hqspi_.State, hqspi_.ErrorCode);
        return Status::ErrInit;
    }
    return Status::OK;
}

W25Qxx::Status W25Qxx::writeEnable()
{
    auto st = sendCommand(CMD_WRITE_ENABLE, 0,
                          QSPI_ADDRESS_NONE, QSPI_ADDRESS_24_BITS,
                          QSPI_DATA_NONE, 0, 0);
    if (st != Status::OK) return Status::ErrWriteEnable;

    QSPI_CommandTypeDef cmd{};
    cmd.Instruction     = CMD_READ_STATUS_REG1;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 1;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DummyCycles     = 0;
    cmd.DdrMode         = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode        = QSPI_SIOO_INST_EVERY_CMD;

    QSPI_AutoPollingTypeDef cfg{};
    cfg.Match           = SR1_WEL;
    cfg.Mask            = SR1_WEL;
    cfg.MatchMode       = QSPI_MATCH_MODE_AND;
    cfg.StatusBytesSize = 1;
    cfg.Interval        = 0x10;
    cfg.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_QSPI_AutoPolling(&hqspi_, &cmd, &cfg, TIMEOUT_DEFAULT) != HAL_OK)
        return Status::ErrWriteEnable;

    return Status::OK;
}

W25Qxx::Status W25Qxx::waitBusy(uint32_t timeout)
{
    QSPI_CommandTypeDef cmd{};
    cmd.Instruction     = CMD_READ_STATUS_REG1;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 1;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DummyCycles     = 0;
    cmd.DdrMode         = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode        = QSPI_SIOO_INST_EVERY_CMD;

    QSPI_AutoPollingTypeDef cfg{};
    cfg.Match           = 0x00;
    cfg.Mask            = SR1_BUSY;
    cfg.MatchMode       = QSPI_MATCH_MODE_AND;
    cfg.StatusBytesSize = 1;
    cfg.Interval        = 0x10;
    cfg.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    if (HAL_QSPI_AutoPolling(&hqspi_, &cmd, &cfg, timeout) != HAL_OK)
        return Status::ErrAutoPolling;

    return Status::OK;
}

W25Qxx::Status W25Qxx::resetDevice()
{
    //LOGD(TAG, "resetDevice: starting...");

    /* If flash is stuck in QPI mode from a previous run, send Exit QPI (0xFF)
       on 4 lines first. We ignore errors here since the chip might not be
       in QPI mode at all. */
    {
        QSPI_CommandTypeDef cmd{};
        cmd.Instruction     = 0xFF; /* Exit QPI */
        cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
        cmd.AddressMode     = QSPI_ADDRESS_NONE;
        cmd.DataMode        = QSPI_DATA_NONE;
        cmd.DummyCycles     = 0;
        cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        cmd.DdrMode         = QSPI_DDR_MODE_DISABLE;
        cmd.SIOOMode        = QSPI_SIOO_INST_EVERY_CMD;
        // HAL_StatusTypeDef r = HAL_QSPI_Command(&hqspi_, &cmd, 100);
        //LOGD(TAG, "Exit QPI (4-line): %d", (int)r);
    }

    /* Also try reset on 4 lines (in case flash was in QPI mode) */
    {
        QSPI_CommandTypeDef cmd{};
        cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
        cmd.AddressMode     = QSPI_ADDRESS_NONE;
        cmd.DataMode        = QSPI_DATA_NONE;
        cmd.DummyCycles     = 0;
        cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        cmd.DdrMode         = QSPI_DDR_MODE_DISABLE;
        cmd.SIOOMode        = QSPI_SIOO_INST_EVERY_CMD;

        cmd.Instruction = CMD_ENABLE_RESET;
        // HAL_StatusTypeDef r1 = HAL_QSPI_Command(&hqspi_, &cmd, 100);
        cmd.Instruction = CMD_RESET_DEVICE;
        // HAL_StatusTypeDef r2 = HAL_QSPI_Command(&hqspi_, &cmd, 100);
        //LOGD(TAG, "Reset 4-line: enable=%d, reset=%d", (int)r1, (int)r2);
    }
    HAL_Delay(5);

    /* Now reset on 1 line (normal SPI mode) */
    auto st = sendCommand(CMD_ENABLE_RESET, 0,
                          QSPI_ADDRESS_NONE, QSPI_ADDRESS_24_BITS,
                          QSPI_DATA_NONE, 0, 0);
    //LOGD(TAG, "Reset 1-line enable: %d", (int)st);
    if (st != Status::OK) return st;

    st = sendCommand(CMD_RESET_DEVICE, 0,
                     QSPI_ADDRESS_NONE, QSPI_ADDRESS_24_BITS,
                     QSPI_DATA_NONE, 0, 0);
    //LOGD(TAG, "Reset 1-line device: %d", (int)st);
    if (st != Status::OK) return st;

    HAL_Delay(30); /* tRST max = 30us, use 30ms for safety */
    //LOGD(TAG, "resetDevice: done");
    return Status::OK;
}

W25Qxx::Status W25Qxx::enableQE()
{
    uint8_t sr2 = 0;

    auto st = sendCommand(CMD_READ_STATUS_REG2, 0,
                          QSPI_ADDRESS_NONE, QSPI_ADDRESS_24_BITS,
                          QSPI_DATA_1_LINE, 0, 1);
    if (st != Status::OK) return st;

    if (HAL_QSPI_Receive(&hqspi_, &sr2, TIMEOUT_DEFAULT) != HAL_OK)
        return Status::ErrRead;

    if (sr2 & SR2_QE) return Status::OK; // Already enabled

    st = writeEnable();
    if (st != Status::OK) return st;

    sr2 |= SR2_QE;

    st = sendCommand(CMD_WRITE_STATUS_REG2, 0,
                     QSPI_ADDRESS_NONE, QSPI_ADDRESS_24_BITS,
                     QSPI_DATA_1_LINE, 0, 1);
    if (st != Status::OK) return st;

    if (HAL_QSPI_Transmit(&hqspi_, &sr2, TIMEOUT_DEFAULT) != HAL_OK)
        return Status::ErrWrite;

    return waitBusy(TIMEOUT_DEFAULT);
}

/* ---------- Public API --------------------------------------------------- */

W25Qxx::Status W25Qxx::init()
{
    LOGI(TAG, "Starting init...");
    //LOGD(TAG, "QSPI Instance: 0x%08lX", (uint32_t)hqspi_.Instance);
    //LOGD(TAG, "QSPI State: %lu, ErrorCode: 0x%08lX",
        //  (uint32_t)hqspi_.State, hqspi_.ErrorCode);
    //LOGD(TAG, "QUADSPI->CR=0x%08lX, SR=0x%08lX",
        //  QUADSPI->CR, QUADSPI->SR);

    /* Abort any ongoing QSPI operation (e.g. memory-mapped mode from previous run) */
    // HAL_StatusTypeDef abortSt = HAL_QSPI_Abort(&hqspi_);
    //LOGD(TAG, "Abort result: %d, State after: %lu",
        //  (int)abortSt, (uint32_t)hqspi_.State);

    auto st = resetDevice();
    if (st != Status::OK) {
        LOGE(TAG, "Reset failed");
        return st;
    }

    uint32_t id = 0;
    st = readJEDECID(id);
    if (st != Status::OK) {
        LOGE(TAG, "Read JEDEC ID failed");
        return st;
    }
    LOGI(TAG, "JEDEC ID: 0x%06lX", id);

    if (id != JEDEC_ID) {
        LOGE(TAG, "ID mismatch, expected 0x%06lX", JEDEC_ID);
        return Status::ErrID;
    }

    st = enableQE();
    if (st != Status::OK) {
        LOGE(TAG, "Enable QE failed");
        return st;
    }

    LOGI(TAG, "Init OK (8MB Quad Flash)");
    return Status::OK;
}

W25Qxx::Status W25Qxx::readJEDECID(uint32_t &id)
{
    uint8_t buf[3]{};

    //LOGD(TAG, "QSPI State before cmd: %lu, ErrorCode: 0x%08lX",
        //  (uint32_t)hqspi_.State, hqspi_.ErrorCode);

    auto st = sendCommand(CMD_READ_JEDEC_ID, 0,
                          QSPI_ADDRESS_NONE, QSPI_ADDRESS_24_BITS,
                          QSPI_DATA_1_LINE, 0, 3);
    if (st != Status::OK) {
        LOGE(TAG, "Failed to send JEDEC ID command");
        return st;
    }

    //LOGD(TAG, "QSPI State after cmd: %lu, ErrorCode: 0x%08lX",(uint32_t)hqspi_.State, hqspi_.ErrorCode);

    HAL_StatusTypeDef hal_st = HAL_QSPI_Receive(&hqspi_, buf, TIMEOUT_DEFAULT);
    if (hal_st != HAL_OK)
    {
        LOGE(TAG, "HAL_QSPI_Receive failed: HAL_Status=%d", (int)hal_st);
        LOGE(TAG, "  QSPI State: %lu, ErrorCode: 0x%08lX",
             (uint32_t)hqspi_.State, hqspi_.ErrorCode);
        LOGE(TAG, "  QUADSPI->SR=0x%08lX, CR=0x%08lX, CCR=0x%08lX",
             QUADSPI->SR, QUADSPI->CR, QUADSPI->CCR);
        LOGE(TAG, "  QUADSPI->DLR=0x%08lX, AR=0x%08lX",
             QUADSPI->DLR, QUADSPI->AR);
        return Status::ErrRead;
    }

    id = (static_cast<uint32_t>(buf[0]) << 16) |
         (static_cast<uint32_t>(buf[1]) << 8)  | buf[2];
    LOGI(TAG, "Raw bytes: 0x%02X 0x%02X 0x%02X", buf[0], buf[1], buf[2]);
    return Status::OK;
}

W25Qxx::Status W25Qxx::eraseSector(uint32_t address)
{
    auto st = writeEnable();
    if (st != Status::OK) return st;

    st = sendCommand(CMD_SECTOR_ERASE, address,
                     QSPI_ADDRESS_1_LINE, QSPI_ADDRESS_24_BITS,
                     QSPI_DATA_NONE, 0, 0);
    if (st != Status::OK) return Status::ErrErase;

    return waitBusy(TIMEOUT_DEFAULT);
}

W25Qxx::Status W25Qxx::eraseBlock64K(uint32_t address)
{
    auto st = writeEnable();
    if (st != Status::OK) return st;

    st = sendCommand(CMD_BLOCK_ERASE_64K, address,
                     QSPI_ADDRESS_1_LINE, QSPI_ADDRESS_24_BITS,
                     QSPI_DATA_NONE, 0, 0);
    if (st != Status::OK) return Status::ErrErase;

    return waitBusy(TIMEOUT_DEFAULT);
}

W25Qxx::Status W25Qxx::eraseChip()
{
    auto st = writeEnable();
    if (st != Status::OK) return st;

    st = sendCommand(CMD_CHIP_ERASE, 0,
                     QSPI_ADDRESS_NONE, QSPI_ADDRESS_24_BITS,
                     QSPI_DATA_NONE, 0, 0);
    if (st != Status::OK) return Status::ErrErase;

    return waitBusy(TIMEOUT_ERASE);
}

W25Qxx::Status W25Qxx::readQuad(uint32_t address, uint8_t *data, uint32_t size)
{
    if (data == nullptr || size == 0) return Status::ErrRead;

    QSPI_CommandTypeDef cmd{};
    cmd.Instruction       = CMD_QUAD_READ;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Address           = address;
    cmd.AddressMode       = QSPI_ADDRESS_4_LINES;
    cmd.AddressSize       = QSPI_ADDRESS_24_BITS;
    cmd.DataMode          = QSPI_DATA_4_LINES;
    cmd.DummyCycles       = 6;
    cmd.NbData            = size;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi_, &cmd, TIMEOUT_DEFAULT) != HAL_OK)
        return Status::ErrRead;

    if (HAL_QSPI_Receive(&hqspi_, data, TIMEOUT_DEFAULT) != HAL_OK)
        return Status::ErrRead;

    return Status::OK;
}

W25Qxx::Status W25Qxx::programPageQuad(uint32_t address, const uint8_t *data, uint32_t size)
{
    if (data == nullptr || size == 0 || size > PAGE_SIZE) return Status::ErrWrite;

    auto st = writeEnable();
    if (st != Status::OK) return st;

    QSPI_CommandTypeDef cmd{};
    cmd.Instruction       = CMD_QUAD_PAGE_PROGRAM;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.Address           = address;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize       = QSPI_ADDRESS_24_BITS;
    cmd.DataMode          = QSPI_DATA_4_LINES;
    cmd.DummyCycles       = 0;
    cmd.NbData            = size;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi_, &cmd, TIMEOUT_DEFAULT) != HAL_OK)
        return Status::ErrWrite;

    if (HAL_QSPI_Transmit(&hqspi_, const_cast<uint8_t*>(data), TIMEOUT_DEFAULT) != HAL_OK)
        return Status::ErrWrite;

    return waitBusy(TIMEOUT_DEFAULT);
}

W25Qxx::Status W25Qxx::writeData(uint32_t address, const uint8_t *data, uint32_t size)
{
    if (data == nullptr || size == 0) return Status::ErrWrite;

    uint32_t offset = 0;
    while (offset < size) {
        uint32_t pageOffset   = (address + offset) % PAGE_SIZE;
        uint32_t pageRemain   = PAGE_SIZE - pageOffset;
        uint32_t bytesToWrite = size - offset;
        if (bytesToWrite > pageRemain) bytesToWrite = pageRemain;

        auto st = programPageQuad(address + offset, data + offset, bytesToWrite);
        if (st != Status::OK) return st;

        offset += bytesToWrite;
    }

    return Status::OK;
}

W25Qxx::Status W25Qxx::enableMemoryMapped()
{
    QSPI_CommandTypeDef cmd{};
    cmd.Instruction       = CMD_QUAD_READ;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_4_LINES;
    cmd.AddressSize       = QSPI_ADDRESS_24_BITS;
    cmd.DataMode          = QSPI_DATA_4_LINES;
    cmd.DummyCycles       = 6;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    QSPI_MemoryMappedTypeDef mmap{};
    mmap.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

    if (HAL_QSPI_MemoryMapped(&hqspi_, &cmd, &mmap) != HAL_OK)
        return Status::ErrMemoryMapped;

    LOGI(TAG, "Memory-mapped mode enabled at 0x%08lX", MMAP_BASE);
    return Status::OK;
}
