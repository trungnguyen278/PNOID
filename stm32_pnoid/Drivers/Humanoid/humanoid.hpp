/**
 * @file    humanoid.hpp
 * @brief   Humanoid robot joint abstraction over PCA9685
 *
 * Layout:  Torso (2) + Left leg (6) + Right leg (6) = 14 servos
 * PCA#1 (0x41): Left leg CH0-5, Torso CH6-7
 * PCA#2 (0x42): Right leg CH0-5
 */

#pragma once

#include "pca9685.hpp"
#include <cstdint>

/* ============== Joint Config ============== */

struct JointConfig {
    PCA9685  *pca;        // which PCA9685 module
    uint8_t   channel;    // PCA channel 0-15
    int16_t   minAngle;   // mechanical limit (degrees)
    int16_t   maxAngle;   // mechanical limit (degrees)
    int16_t   homeAngle;  // default/standing position
    int8_t    direction;  // +1 = normal, -1 = reversed (left/right mirror)
    int16_t   offset;     // trim offset (degrees)
};

/* ============== Leg ============== */

class Leg {
public:
    enum Joint : uint8_t {
        HipYaw = 0,     // xoay ngang
        HipRoll,        // dạng chân
        HipPitch,       // gập đùi
        KneePitch,      // gập gối
        AnklePitch,     // gập cổ chân
        AnkleRoll,      // nghiêng bàn chân
        NUM_JOINTS
    };

    enum class Status {
        OK = 0,
        ErrRange,
        ErrPCA,
    };

    Leg() = default;

    /** Configure all 6 joints */
    void configure(const JointConfig configs[NUM_JOINTS]);

    /** Set a single joint angle (degrees, in robot frame) */
    Status setJoint(Joint joint, int16_t angle);

    /** Move all joints to home position */
    Status home();

    /** Get current commanded angle */
    int16_t getAngle(Joint joint) const { return currentAngle_[joint]; }

    /** Set trim offset for a joint */
    void setOffset(Joint joint, int16_t offset);

    /** Get joint name string */
    static const char* jointName(Joint joint);

private:
    JointConfig cfg_[NUM_JOINTS] = {};
    int16_t currentAngle_[NUM_JOINTS] = {};
};

/* ============== Torso ============== */

class Torso {
public:
    enum Joint : uint8_t {
        Yaw = 0,     // xoay trái/phải
        Roll,        // nghiêng trái/phải
        NUM_JOINTS
    };

    enum class Status {
        OK = 0,
        ErrRange,
        ErrPCA,
    };

    Torso() = default;

    void configure(const JointConfig configs[NUM_JOINTS]);
    Status setJoint(Joint joint, int16_t angle);
    Status home();
    int16_t getAngle(Joint joint) const { return currentAngle_[joint]; }
    void setOffset(Joint joint, int16_t offset);
    static const char* jointName(Joint joint);

private:
    JointConfig cfg_[NUM_JOINTS] = {};
    int16_t currentAngle_[NUM_JOINTS] = {};
};

/* ============== Humanoid ============== */

class Humanoid {
public:
    enum class Status {
        OK = 0,
        ErrInit,
        ErrRange,
        ErrPCA,
    };

    /**
     * @param pcaLeft   PCA9685 for left leg + torso (0x41)
     * @param pcaRight  PCA9685 for right leg (0x42)
     */
    Humanoid(PCA9685 &pcaLeft, PCA9685 &pcaRight);

    /** Init both PCA9685 modules and configure all joints */
    Status init();

    /** Move all joints to home (standing) position */
    Status home();

    Leg   leftLeg;
    Leg   rightLeg;
    Torso torso;

private:
    PCA9685 &pcaLeft_;
    PCA9685 &pcaRight_;
};
