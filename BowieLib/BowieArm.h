/*
 * Robot Missions Bowie Arm
 * ------------------------
 *
 * Servo routines for Bowie the robot's arm (excluding
 * the end effector).
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * Using Teensy 3.6, works with Teensy 3.2
 * 
 * Erin RobotGrrl
 * Dec. 29th, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 */

#include "Arduino.h"
#include <Streaming.h>
#include <Servo.h>

#ifndef _BOWIEARM_H_
#define _BOWIEARM_H_

// Servo positions
#define ARM_MIN 500 // down 700
#define ARM_HOME 1400 // middle 1200 orig
#define ARM_MAX 2300 // up, 90 deg perpenducular to base
#define ARM_PARK 2400 // leaning against hopper a bit

#define SERVO_MAX_US 2500
#define SERVO_MIN_US 500

// Keys
#define SERVO_ARM_KEY 1
#define SERVO_ARM_AND_END_KEY 6

class BowieArm {

  public:
    BowieArm();
    void setArm1ServoPin(uint8_t p);
    void setArm2ServoPin(uint8_t p);
    void initServos();

    // Callback
    void setServoInterruptCallback( void (*servoInterruptCallback)(int key, int val) );

    // Servos
    Servo arm;
    Servo arm2;

    // Other
    void servoInterruption(int key, int val);

    // Arm
    void moveArm(int armPos);
    void moveArm(int armPos, int step, int del);
    void parkArm();
    void unparkArm();
    bool getArmParked();
    int getArmPos();

  private:

    // Callback
    void (*_servoInterruptCallback)(int key, int val);

    // Pins
    uint8_t SERVO_ARM1;
    uint8_t SERVO_ARM2;

    // Positions
    int arm_position;
    
    // Park positions
    bool arm_parked;

};

#endif