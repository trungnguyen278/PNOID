/**
 * @file    humanoid.cpp
 * @brief   Humanoid robot joint abstraction implementation
 */

#include "humanoid.hpp"
#include "debug_log.h"
#include <cstring>

static const char *TAG = "HUMANOID";

/* ============== Leg ============== */

void Leg::configure(const JointConfig configs[NUM_JOINTS])
{
    memcpy(cfg_, configs, sizeof(JointConfig) * NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++)
        currentAngle_[i] = configs[i].homeAngle;
}

Leg::Status Leg::setJoint(Joint joint, int16_t angle)
{
    if (joint >= NUM_JOINTS) return Status::ErrRange;

    auto &c = cfg_[joint];

    /* Clamp to mechanical limits */
    if (angle < c.minAngle) angle = c.minAngle;
    if (angle > c.maxAngle) angle = c.maxAngle;

    /* Convert to servo angle: apply direction and offset */
    int16_t servoAngle = 90 + (angle - 0) * c.direction + c.offset;

    /* Clamp servo angle to 0-180 */
    if (servoAngle < 0) servoAngle = 0;
    if (servoAngle > 180) servoAngle = 180;

    if (c.pca->setAngle(c.channel, (uint16_t)servoAngle) != PCA9685::Status::OK)
        return Status::ErrPCA;

    currentAngle_[joint] = angle;
    return Status::OK;
}

Leg::Status Leg::home()
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        Status st = setJoint((Joint)i, cfg_[i].homeAngle);
        if (st != Status::OK) return st;
    }
    return Status::OK;
}

void Leg::setOffset(Joint joint, int16_t offset)
{
    if (joint < NUM_JOINTS)
        cfg_[joint].offset = offset;
}

const char* Leg::jointName(Joint joint)
{
    static const char *names[] = {
        "HipYaw", "HipRoll", "HipPitch",
        "KneePitch", "AnklePitch", "AnkleRoll"
    };
    if (joint < NUM_JOINTS) return names[joint];
    return "?";
}

/* ============== Torso ============== */

void Torso::configure(const JointConfig configs[NUM_JOINTS])
{
    memcpy(cfg_, configs, sizeof(JointConfig) * NUM_JOINTS);
    for (int i = 0; i < NUM_JOINTS; i++)
        currentAngle_[i] = configs[i].homeAngle;
}

Torso::Status Torso::setJoint(Joint joint, int16_t angle)
{
    if (joint >= NUM_JOINTS) return Status::ErrRange;

    auto &c = cfg_[joint];

    if (angle < c.minAngle) angle = c.minAngle;
    if (angle > c.maxAngle) angle = c.maxAngle;

    int16_t servoAngle = 90 + (angle - 0) * c.direction + c.offset;
    if (servoAngle < 0) servoAngle = 0;
    if (servoAngle > 180) servoAngle = 180;

    if (c.pca->setAngle(c.channel, (uint16_t)servoAngle) != PCA9685::Status::OK)
        return Status::ErrPCA;

    currentAngle_[joint] = angle;
    return Status::OK;
}

Torso::Status Torso::home()
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        Status st = setJoint((Joint)i, cfg_[i].homeAngle);
        if (st != Status::OK) return st;
    }
    return Status::OK;
}

void Torso::setOffset(Joint joint, int16_t offset)
{
    if (joint < NUM_JOINTS)
        cfg_[joint].offset = offset;
}

const char* Torso::jointName(Joint joint)
{
    static const char *names[] = { "TorsoYaw", "TorsoRoll" };
    if (joint < NUM_JOINTS) return names[joint];
    return "?";
}

/* ============== Humanoid ============== */

Humanoid::Humanoid(PCA9685 &pcaLeft, PCA9685 &pcaRight)
    : pcaLeft_(pcaLeft), pcaRight_(pcaRight)
{
}

Humanoid::Status Humanoid::init()
{
    /* Init PCA modules */
    if (pcaLeft_.init() != PCA9685::Status::OK) {
        LOGE(TAG, "PCA Left (0x41) init failed!");
        return Status::ErrInit;
    }
    if (pcaRight_.init() != PCA9685::Status::OK) {
        LOGE(TAG, "PCA Right (0x42) init failed!");
        return Status::ErrInit;
    }

    /*
     * PCA#1 (0x41) — Left leg CH0-5, Torso CH6-7
     * PCA#2 (0x42) — Right leg CH0-5
     *
     * Joint angle convention:
     *   0 = neutral/standing position
     *   Positive = forward/outward
     *   Negative = backward/inward
     *
     * direction: +1 = normal, -1 = mirrored (right leg mirrors left)
     *
     * Servo 90° = robot joint 0° (neutral)
     * Limits can be tuned per joint after calibration
     */

    /*
     * Servo mapping (xác nhận thực tế):
     *
     * Robot angle convention (tất cả joint):
     *   0° = tư thế đứng thẳng (home)
     *   Dương (+) = hướng "mở" / "lên" / "ra ngoài":
     *     HipYaw   + = xoay chân ra ngoài
     *     HipRoll  + = dạng chân ra ngoài
     *     HipPitch + = nhấc đùi lên trước
     *     KneePitch + = gập gối
     *     AnklePitch + = gập mũi chân lên (dorsiflexion)
     *     AnkleRoll + = nghiêng lòng bàn chân ra ngoài
     *
     * servo = 90 + robotAngle * direction + offset
     *
     * LEFT (servo1):
     *   CH0 HipYaw:    90=giữa, 0=ra ngoài, 180=vào trong    → dir=-1
     *   CH1 HipRoll:   90=giữa, 180=dạng ra ngoài(trái)      → dir=+1
     *   CH2 HipPitch:  90=giữa, 180=co về sau                 → dir=-1
     *   CH3 KneePitch: 170=thẳng, giảm=co gối                 → dir=-1, offset=+80
     *   CH4 AnklePitch:90=thẳng, 180=mũi chân lên             → dir=+1
     *   CH5 AnkleRoll: 90=thẳng, 180=lòng bàn chân ra ngoài  → dir=+1
     *
     * RIGHT (servo2):
     *   CH0 HipYaw:    90=giữa, 180=ra ngoài(phải)            → dir=+1
     *   CH1 HipRoll:   90=giữa, 180=nghiêng vào trong(trái)   → dir=-1
     *   CH2 HipPitch:  90=giữa, 180=nhấc chân lên             → dir=+1
     *   CH3 KneePitch: 0=thẳng, tăng=co gối                   → dir=+1, offset=-90
     *   CH4 AnklePitch:90=giữa, 0=mũi chân lên                → dir=-1
     *   CH5 AnkleRoll: 90=giữa, 180=lòng bàn chân vào trong   → dir=-1
     *
     * TORSO (servo2):
     *   CH8 TorsoYaw:  90=giữa, 0=phải, 180=trái              → dir=+1
     *   CH9 TorsoRoll: 90=giữa, 0=nghiêng trái, 180=phải, ±20° → dir=+1
     */

    /* Left leg — PCA#1 (0x41), CH0-5 */
    JointConfig leftCfg[Leg::NUM_JOINTS] = {
    /*  pca          ch  min   max  home  dir  offset */
        {&pcaLeft_,  0,  -45,  45,   0,  -1,  -5},  // HipYaw   (0=out,180=in)
        {&pcaLeft_,  1,  -30,  30,   0,  +1, -10},  // HipRoll  (180=out left, -10° bù rạng)
        {&pcaLeft_,  2,  -45,  90,   0,  -1,   0},  // HipPitch (180=back → -1)
        {&pcaLeft_,  3,    0,  80,   0,  -1, +80},  // KneePitch(170=straight)
        {&pcaLeft_,  4,  -45,  45,   0,  +1,   0},  // AnklePitch(180=toe up)
        {&pcaLeft_,  5,  -30,  30,   0,  +1, -10},  // AnkleRoll(180=sole out, -10° nghiêng vào)
    };
    leftLeg.configure(leftCfg);

    /* Right leg — PCA#2 (0x42), CH0-5 */
    JointConfig rightCfg[Leg::NUM_JOINTS] = {
    /*  pca           ch  min   max  home  dir  offset */
        {&pcaRight_,  0,  -45,  45,   0,  +1,   0},  // HipYaw   (180=out right)
        {&pcaRight_,  1,  -30,  30,   0,  -1,   0},  // HipRoll  (180=in → -1)
        {&pcaRight_,  2,  -45,  90,   0,  +1,   0},  // HipPitch (180=up → +1)
        {&pcaRight_,  3,    0,  80,   0,  +1, -85},  // KneePitch(0=straight, giảm co)
        {&pcaRight_,  4,  -45,  45,   0,  -1, -25},  // AnklePitch(bù nghiêng sau)
        {&pcaRight_,  5,  -30,  30,   0,  -1, -10},  // AnkleRoll(180=sole in, -10° nghiêng vào)
    };
    rightLeg.configure(rightCfg);

    /* Torso — PCA#2 (0x42), CH8-9 */
    JointConfig torsoCfg[Torso::NUM_JOINTS] = {
    /*  pca           ch  min   max  home  dir  offset */
        {&pcaRight_,  8,  -45,  45,   0,  +1,   0},  // TorsoYaw  (180=left)
        {&pcaRight_,  9,  -20,  20,   0,  +1,   0},  // TorsoRoll (180=right, ±20°)
    };
    torso.configure(torsoCfg);

    /* Home position */
    home();

    LOGI(TAG, "Init OK (14 joints: 2 legs + torso)");
    return Status::OK;
}

Humanoid::Status Humanoid::home()
{
    LOGI(TAG, "Moving to home position...");

    if (leftLeg.home() != Leg::Status::OK) return Status::ErrPCA;
    if (rightLeg.home() != Leg::Status::OK) return Status::ErrPCA;
    if (torso.home() != Torso::Status::OK) return Status::ErrPCA;

    LOGI(TAG, "Home position OK");
    return Status::OK;
}
