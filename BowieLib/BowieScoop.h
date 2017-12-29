/*
 * Robot Missions Bowie Scoop
 * ---------------------------
 *
 * Servo routines for Bowie the robot's scoop end effector.
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

#ifndef _BOWIESCOOP_H_
#define _BOWIESCOOP_H_

// Servo positions
#define END_MIN 700 // up
#define END_PARALLEL_TOP 2300 // parallel to ground when arm is raised, raised a bit to keep debris in
#define END_PARALLEL_BOTTOM 1100 // parallel to ground when arm is lowered 1100
#define END_HOME 1400 // parallel to arm, good for digging
#define END_MAX 1800 // down

#define SERVO_MAX_US 2500
#define SERVO_MIN_US 500

// Keys
#define SERVO_END_KEY 2

class BowieScoop {

  public:
    BowieScoop();
    void setServoScoopPin(uint8_t p);
    void setProbeLPin(uint8_t p);
    void setProbeRPin(uint8_t p);
    void initServos();

    // Callback
    void setServoInterruptCallback( void (*servoInterruptCallback)(int key, int val) );

    // Servos
    Servo scoop;

    // Other
    void servoInterruption(int key, int val);

    // Scoop
    void moveEnd(int endPos);
    void moveEnd(int endPos, int step, int del);
    void parkEnd();
    void unparkEnd();
    bool getEndParked();
    int getEndPos();

    // Probes
    uint16_t END_TOUCHDOWN;
    void updateScoopProbes();
    bool getScoopProbeL();
    bool getScoopProbeR();

  private:

    // Callback
    void (*_servoInterruptCallback)(int key, int val);

    // Pins
    uint8_t SERVO_END_EFFECTOR;
    uint8_t SCOOP_PROBE_LEFT;
    uint8_t SCOOP_PROBE_RIGHT;

    // Positions
    int end_position;
    
    // Park positions
    bool end_parked;

    // Probes
    uint8_t scoop_probe_left_val;
    uint8_t scoop_probe_right_val;
    
};

#endif