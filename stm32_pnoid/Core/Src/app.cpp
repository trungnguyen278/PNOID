/**
 * @file app.cpp
 * @brief Main application C++ code
 */

extern "C" {
#include "main.h"
}

#include "app.hpp"
#include "debug_log.h"

/* ============== C++ Application Code ============== */

namespace App {

void init() {
    // TODO: Add your C++ initialization code here
    
}

void run() {
    // TODO: Add your C++ main loop code here
    while (1) {
        LOG("Test log");
        HAL_Delay(500);
    }
}

} // namespace App

/* ============== C Wrapper Functions ============== */

extern "C" {

/**
 * @brief Initialize application (call from main.c before while loop)
 */
void App_Init(void) {
    App::init();
}

/**
 * @brief Main application loop (call from main.c inside while loop)
 */
void App_Main(void) {
    App::run();
}

} // extern "C"
