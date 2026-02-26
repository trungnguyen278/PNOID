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
// #include "sdcard.hpp"   // Uncomment after enabling FATFS in CubeMX
// #include "camera.hpp"   // Uncomment when camera is connected

/* ============== External HAL handles from main.c ============== */

extern QSPI_HandleTypeDef hqspi;
extern SPI_HandleTypeDef  hspi2;
extern SD_HandleTypeDef   hsd1;
extern DCMI_HandleTypeDef hdcmi;
extern I2C_HandleTypeDef  hi2c2;

/* ============== Driver instances ============== */

static W25Qxx flash(hqspi);
static LCD    lcd(hspi2,
                  LCD_CS_GPIO_Port,  LCD_CS_Pin,
                  LCD_D_C_GPIO_Port, LCD_D_C_Pin,
                  LCD_BLK_GPIO_Port, LCD_BLK_Pin);

// static SDCard  sd(hsd1);            // Uncomment after FATFS
// static Camera  cam(hdcmi, hi2c2);   // Uncomment when camera connected

/* ============== Application ============== */

static const char *TAG = "APP";

namespace App {

void init() {
    BSP::init();
    BSP::printSystemInfo();

    /* Init QSPI Flash */
    if (flash.init() != W25Qxx::Status::OK) {
        LOGE(TAG, "W25Qxx init failed!");
    }

    /* Init LCD */
    if (lcd.init() != LCD::Status::OK) {
        LOGE(TAG, "LCD init failed!");
    } else {
        lcd.fillScreen(LCD::BLACK);
        lcd.drawString(20, 100, "PNOID Ready!", LCD::GREEN, LCD::BLACK);
    }

    LOGI(TAG, "All peripherals initialized");
}

void run() {
    while (1) {
        BSP::ledToggle();

        if (BSP::buttonPressed(BSP::Button::K1)) {
            LOGI(TAG, "K1 pressed");
        }

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
