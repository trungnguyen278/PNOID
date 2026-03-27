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
#include "icm20948.hpp"
#include "humanoid.hpp"
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
static PCA9685  servo1(hi2c1, 0x41);   // PCA9685 #1 (A0 soldered)
static PCA9685  servo2(hi2c1, 0x42);   // PCA9685 #2 (A1 soldered)
static ICM20948 imu(hi2c1, 0x68);      // ICM-20948 IMU
static Humanoid robot(servo1, servo2); // Humanoid: left=PCA#1, right=PCA#2
// static Camera  cam(hdcmi, hi2c2);   // Uncomment when camera connected

/* ============== IMU Data-Ready Flag (set by EXTI) ============== */

static volatile bool imuDataReady = false;

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

    /* Scan I2C1 */
    LOGI(TAG, "Scanning I2C1...");
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 1, 10) == HAL_OK)
            LOGI(TAG, "  0x%02X found", a);
    }

    /* Init Humanoid (PCA9685 x2 + joint config) */
    if (robot.init() != Humanoid::Status::OK) {
        LOGE(TAG, "Humanoid init failed!");
    }

    /* Init ICM-20948 IMU */
    if (imu.init() != ICM20948::Status::OK) {
        LOGE(TAG, "ICM-20948 init failed!");
    }

    LOGI(TAG, "All peripherals initialized");
}

void run() {
    /*
     * Bent-knee stance + IMU stabilizer
     *
     * Tư thế cơ sở (base pose):
     *   KneePitch  = 25°   gập gối
     *   HipPitch   = 12°   nghiêng đùi ra trước
     *   AnklePitch = 13°   bù mũi chân
     *
     * Stabilizer:
     *   Complementary filter (gyro + accel) → estimated roll, pitch
     *   Bù vào ankle (nhanh) + hip (chậm) để giữ roll≈0, pitch≈0
     */

    /* ── Base pose ── */
    const int16_t BASE_KNEE  = 25;
    const int16_t BASE_HIP_P = 12;
    const int16_t BASE_ANK_P = 13;

    /* ── Stabilizer gains ── */
    const float Kp_pitch = 0.8f;   // deg correction / deg error
    const float Kd_pitch = 0.05f;  // deg correction / (deg/s)
    const float Kp_roll  = 1.0f;
    const float Kd_roll  = 0.06f;

    /* ankle nhận 60% correction, hip 40% */
    const float ANKLE_SHARE = 0.6f;
    const float HIP_SHARE   = 0.4f;

    /* Complementary filter */
    const float ALPHA = 0.98f;     // gyro trust (0.98 = tau ≈ 1s)
    float est_roll  = 0.0f;
    float est_pitch = 0.0f;

    /* Correction limits */
    const float CORR_MAX = 12.0f;  // deg

    uint32_t lastTick = HAL_GetTick();
    uint32_t lastLog  = 0;

    /* Set base pose */
    LOGI(TAG, "Setting bent-knee stance...");
    robot.torso.setJoint(Torso::Yaw,  0);
    robot.torso.setJoint(Torso::Roll, 0);

    auto setBasePose = [&](Leg &leg) {
        leg.setJoint(Leg::HipYaw,     0);
        leg.setJoint(Leg::HipRoll,    0);
        leg.setJoint(Leg::HipPitch,   BASE_HIP_P);
        leg.setJoint(Leg::KneePitch,  BASE_KNEE);
        leg.setJoint(Leg::AnklePitch, BASE_ANK_P);
        leg.setJoint(Leg::AnkleRoll,  0);
    };
    setBasePose(robot.leftLeg);
    setBasePose(robot.rightLeg);

    HAL_Delay(500);  // đợi servo về vị trí

    /* Init filter từ accel hiện tại */
    if (imu.read() == ICM20948::Status::OK) {
        auto e = imu.getEuler();
        est_roll  = e.roll;
        est_pitch = e.pitch;
    }

    LOGI(TAG, "Stabilizer running (Kp_p=%.1f Kp_r=%.1f)",
         (double)Kp_pitch, (double)Kp_roll);

    /* ── Main control loop ── */
    while (1) {
        /* Wait for IMU data-ready interrupt (PE7 falling edge) */
        if (!imuDataReady) {
            HAL_Delay(1);
            continue;
        }
        imuDataReady = false;

        uint32_t now = HAL_GetTick();
        float dt = (now - lastTick) * 0.001f;
        if (dt < 0.001f) dt = 0.001f;  // safety clamp
        lastTick = now;

        BSP::ledToggle();

        /* 1. Đọc IMU (data guaranteed ready by INT pin) */
        if (imu.read() != ICM20948::Status::OK) continue;

        auto accel = imu.getAccel();
        auto gyro  = imu.getGyro();

        /* 2. Complementary filter */
        float accel_roll  = atan2f(accel.y, accel.z) * 57.2958f;
        float accel_pitch = atan2f(-accel.x,
                            sqrtf(accel.y * accel.y + accel.z * accel.z)) * 57.2958f;

        est_roll  = ALPHA * (est_roll  + gyro.x * dt) + (1.0f - ALPHA) * accel_roll;
        est_pitch = ALPHA * (est_pitch + gyro.y * dt) + (1.0f - ALPHA) * accel_pitch;

        /* 3. Tính correction (target = 0° cho cả roll và pitch) */
        float corr_pitch = Kp_pitch * (-est_pitch) + Kd_pitch * (-gyro.y);
        float corr_roll  = Kp_roll  * (-est_roll)  + Kd_roll  * (-gyro.x);

        /* Clamp */
        if (corr_pitch >  CORR_MAX) corr_pitch =  CORR_MAX;
        if (corr_pitch < -CORR_MAX) corr_pitch = -CORR_MAX;
        if (corr_roll  >  CORR_MAX) corr_roll  =  CORR_MAX;
        if (corr_roll  < -CORR_MAX) corr_roll  = -CORR_MAX;

        /* 4. Phân bổ vào khớp */
        int16_t ankle_pitch_corr = (int16_t)(corr_pitch * ANKLE_SHARE);
        int16_t hip_pitch_corr   = (int16_t)(corr_pitch * HIP_SHARE);
        int16_t ankle_roll_corr  = (int16_t)(corr_roll  * ANKLE_SHARE);
        int16_t hip_roll_corr    = (int16_t)(corr_roll  * HIP_SHARE);

        /* 5. Gửi servo = base + correction */
        /* Chân trái */
        robot.leftLeg.setJoint(Leg::AnklePitch, BASE_ANK_P + ankle_pitch_corr);
        robot.leftLeg.setJoint(Leg::HipPitch,   BASE_HIP_P - hip_pitch_corr);
        robot.leftLeg.setJoint(Leg::AnkleRoll,  ankle_roll_corr);
        robot.leftLeg.setJoint(Leg::HipRoll,    hip_roll_corr);

        /* Chân phải — cùng correction (direction trong config lo mirror) */
        robot.rightLeg.setJoint(Leg::AnklePitch, BASE_ANK_P + ankle_pitch_corr);
        robot.rightLeg.setJoint(Leg::HipPitch,   BASE_HIP_P - hip_pitch_corr);
        robot.rightLeg.setJoint(Leg::AnkleRoll,  ankle_roll_corr);
        robot.rightLeg.setJoint(Leg::HipRoll,    hip_roll_corr);

        /* Torso bù ngược roll */
        robot.torso.setJoint(Torso::Roll, (int16_t)(-corr_roll * 0.3f));

        /* 6. Log mỗi 500ms */
        if ((now - lastLog) >= 500) {
            LOGI(TAG, "R=%d.%d P=%d.%d  cr=%d cp=%d",
                 (int)est_roll, abs((int)(est_roll * 10) % 10),
                 (int)est_pitch, abs((int)(est_pitch * 10) % 10),
                 (int)corr_roll, (int)corr_pitch);
            lastLog = now;
        }
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BNO_INT_Pin) {
        imuDataReady = true;
    }
}

} // extern "C"
