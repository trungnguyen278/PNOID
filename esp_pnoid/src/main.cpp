#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "AppController.hpp"
#include "config/DeviceProfile.hpp"

static const char* TAG = "MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "PNoid V2 - ESP32-C5 starting...");

    auto& app = AppController::instance();

    if (!DeviceProfile::setup(app)) {
        ESP_LOGE(TAG, "DeviceProfile setup failed");
        return;
    }

    if (!app.init()) {
        ESP_LOGE(TAG, "AppController init failed");
        return;
    }

    app.start();
}
