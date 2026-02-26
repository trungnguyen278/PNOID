/**
 * @file    bsp.cpp
 * @brief   Board Support Package implementation
 */

#include "bsp.hpp"
#include "main.h"
#include "debug_log.h"

static const char *TAG = "BSP";

/* ---------- Init --------------------------------------------------------- */

void BSP::init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    ledOff();
    LOGI(TAG, "Init OK");
}

/* ---------- LED ---------------------------------------------------------- */

void BSP::ledOn()     { HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET); }
void BSP::ledOff()    { HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_SET);   }
void BSP::ledToggle() { HAL_GPIO_TogglePin(LED_D2_GPIO_Port, LED_D2_Pin); }

/* ---------- Buttons ------------------------------------------------------ */

bool BSP::buttonRead(Button btn)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    switch (btn) {
        case Button::K1: port = K1_GPIO_Port; pin = K1_Pin; break;
        case Button::K2: port = K2_GPIO_Port; pin = K2_Pin; break;
        default: return false;
    }

    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}

bool BSP::buttonPressed(Button btn)
{
    if (!buttonRead(btn)) return false;
    HAL_Delay(20);
    return buttonRead(btn);
}

/* ---------- Delay -------------------------------------------------------- */

void BSP::delayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles) {}
}

/* ---------- Camera GPIO -------------------------------------------------- */

void BSP::camPowerDown(bool enable)
{
    HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP::camReset(bool active)
{
    HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin,
                      active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BSP::camPowerOn()
{
    camPowerDown(false);
    HAL_Delay(10);
    camReset(true);
    HAL_Delay(10);
    camReset(false);
    HAL_Delay(20);
}

/* ---------- System info -------------------------------------------------- */

void BSP::printSystemInfo()
{
    LOGI(TAG, "=== STM32H743VIT6 DevEBox Board ===");
    LOGI(TAG, "SYSCLK : %lu MHz", HAL_RCC_GetSysClockFreq() / 1000000);
    LOGI(TAG, "HCLK   : %lu MHz", HAL_RCC_GetHCLKFreq() / 1000000);
    LOGI(TAG, "APB1   : %lu MHz", HAL_RCC_GetPCLK1Freq() / 1000000);
    LOGI(TAG, "APB2   : %lu MHz", HAL_RCC_GetPCLK2Freq() / 1000000);
    LOGI(TAG, "===================================");
}
