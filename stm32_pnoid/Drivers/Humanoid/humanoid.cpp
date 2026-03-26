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

    /* Left leg — PCA#1, CH0-5 */
    JointConfig leftCfg[Leg::NUM_JOINTS] = {
    /*  pca          ch  min   max  home  dir  offset */
        {&pcaLeft_,  0,  -45,  45,   0,  +1,   0},  // HipYaw
        {&pcaLeft_,  1,  -30,  30,   0,  +1,   0},  // HipRoll
        {&pcaLeft_,  2,  -90,  45,   0,  +1,   0},  // HipPitch
        {&pcaLeft_,  3,    0,  135,  0,  +1,   0},  // KneePitch
        {&pcaLeft_,  4,  -45,  45,   0,  +1,   0},  // AnklePitch
        {&pcaLeft_,  5,  -30,  30,   0,  +1,   0},  // AnkleRoll
    };
    leftLeg.configure(leftCfg);

    /* Right leg — PCA#2, CH0-5 (mirrored: Roll and Yaw reversed) */
    JointConfig rightCfg[Leg::NUM_JOINTS] = {
    /*  pca           ch  min   max  home  dir  offset */
        {&pcaRight_,  0,  -45,  45,   0,  -1,   0},  // HipYaw (mirrored)
        {&pcaRight_,  1,  -30,  30,   0,  -1,   0},  // HipRoll (mirrored)
        {&pcaRight_,  2,  -90,  45,   0,  +1,   0},  // HipPitch
        {&pcaRight_,  3,    0,  135,  0,  +1,   0},  // KneePitch
        {&pcaRight_,  4,  -45,  45,   0,  +1,   0},  // AnklePitch
        {&pcaRight_,  5,  -30,  30,   0,  -1,   0},  // AnkleRoll (mirrored)
    };
    rightLeg.configure(rightCfg);

    /* Torso — PCA#1, CH6-7 */
    JointConfig torsoCfg[Torso::NUM_JOINTS] = {
    /*  pca          ch  min   max  home  dir  offset */
        {&pcaLeft_,  6,  -45,  45,   0,  +1,   0},  // TorsoYaw
        {&pcaLeft_,  7,  -30,  30,   0,  +1,   0},  // TorsoPitch
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
