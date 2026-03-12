/**
 * @file app.cpp
 * @brief Main application C++ code
 */

extern "C" {
#include "main.h"
}

#include "app.hpp"
#include "debug_log.h"
#include "bsp.hpp"
#include "w25qxx.hpp"
#include "lcd.hpp"
#include "sdcard.hpp"
#include "i2s_io.hpp"
#include "audio_out.hpp"
#include "pca9685.hpp"
#include "bno085.hpp"
// #include "camera.hpp"   // Uncomment when camera is connected

/* ============== External HAL handles from main.c ============== */

extern QSPI_HandleTypeDef hqspi;
extern SPI_HandleTypeDef  hspi2;
extern SD_HandleTypeDef   hsd1;
extern DCMI_HandleTypeDef hdcmi;
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c2;
extern I2S_HandleTypeDef  hi2s1;

/* ============== Driver instances ============== */

static W25Qxx  flash(hqspi);
static LCD     lcd(hspi2,
                   LCD_CS_GPIO_Port,  LCD_CS_Pin,
                   LCD_D_C_GPIO_Port, LCD_D_C_Pin,
                   LCD_BLK_GPIO_Port, LCD_BLK_Pin);
static SDCard   sd(hsd1);
static I2SIO    i2s(hi2s1);
static AudioOut audioOut(i2s);
static PCA9685  servo1(hi2c1, 0x40);   // PCA9685 #1 (default addr)
static PCA9685  servo2(hi2c1, 0x42);   // PCA9685 #2 (A1 soldered)
static BNO085   imu(hi2c1, 0x4A);      // BNO085 IMU
// static Camera  cam(hdcmi, hi2c2);   // Uncomment when camera connected

/* ============== Application ============== */

static const char *TAG = "APP";

namespace App {

void init() {
    BSP::init();
    BSP::printSystemInfo();

    /* Init SD Card — nếu có thì mount và bật SD logging */
    if (sd.init() == SDCard::Status::OK) {
        LOGI(TAG, "SD Card mounted");
#if LOG_SD_ENABLE
        if (LOG_SD_Init() == 0) {
            LOGI(TAG, "SD logging enabled");
        }
#endif
    } else {
        LOGW(TAG, "SD Card not available, UART log only");
    }

    /* Init QSPI Flash */
    if (flash.init() != W25Qxx::Status::OK) {
        LOGE(TAG, "W25Qxx init failed!");
    }

    /* Init Audio (I2S + PCM5102 DAC) */
    if (audioOut.init() != AudioOut::Status::OK) {
        LOGE(TAG, "Audio init failed!");
    }

    /* Init LCD */
    if (lcd.init() != LCD::Status::OK) {
        LOGE(TAG, "LCD init failed!");
    } else {
        lcd.fillScreen(LCD::BLACK);
        lcd.drawString(20, 100, "PNOID Ready!", LCD::GREEN, LCD::BLACK);
    }

    /* Test PCA9685 #2 (0x42) */
    if (servo2.init() != PCA9685::Status::OK) {
        LOGE(TAG, "PCA9685 #2 (0x42) init failed!");
    }

    LOGI(TAG, "All peripherals initialized");
}

void run() {
    /* Mỗi kênh hold 1 góc: 0→45→90→135→180 lặp lại */
    static const uint16_t angles[] = { 0, 45, 90, 135, 180 };
    for (uint8_t ch = 0; ch < 16; ch++) {
        uint16_t angle = angles[ch % 5];
        servo2.setAngle(ch, angle);
        LOGI(TAG, "CH%u = %u deg", ch, angle);
    }

    while (1) {
        BSP::ledToggle();
        HAL_Delay(500);
    }
}

} // namespace App

/* ============== C Wrapper Functions ============== */

extern "C" {

void App_Init(void) {
    App::init();
}

void App_Main(void) {
    App::run();
}

} // extern "C"
