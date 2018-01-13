/*
 * Robot Missions Bowie Hopper
 * ---------------------------
 *
 * Servo routines for Bowie the robot's hopper payload
 * to tilt it to empty, and opening and closing the lid.
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * Using Teensy 3.6, works with Teensy 3.2
 * 
 * Erin RobotGrrl
 * Dec. 28th, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 */

#include "Arduino.h"
#include <Streaming.h>
#ifndef Servo_h
#include <Servo.h>
#endif

#ifndef _BOWIEHOPPER_H_
#define _BOWIEHOPPER_H_

// Servo positions
#define TILT_MIN 700 // tilted (emptying)
#define TILT_MAX 1600 // flush

#define LID_MIN 850 //1000 // open
#define LID_MAX 1850 //2000 // closed

#define SERVO_MAX_US 2500
#define SERVO_MIN_US 500

// Keys
#define SERVO_HOPPER_KEY 3
#define SERVO_LID_KEY 4

class BowieHopper {

  public:

    BowieHopper();
    void begin();
    void setServoHopperPivotPin(uint8_t p);
    void setServoHopperLidPin(uint8_t p);
    void initServos();

    // Callback
    void setServoInterruptCallback( void (*servoInterruptCallback)(int key, int val) );

    // Servos
    Servo tilt;
    Servo lid;

    // Other
    void servoInterruption(int key, int val);

    // Hopper
    void moveHopper(int hopperPos);
    void moveHopper(int hopperPos, int step, int del);
    void parkHopper();
    void unparkHopper();
    bool getHopperParked();
    int getHopperPos();

    // Lid
    void moveLid(int lidPos);
    void moveLid(int lidPos, int step, int del);
    void parkLid();
    void unparkLid();
    bool getLidParked();
    int getLidPos();

  private:

    // Callback
    void (*_servoInterruptCallback)(int key, int val);

    // Pins
    uint8_t SERVO_HOPPER_PIVOT;
    uint8_t SERVO_HOPPER_LID;

    // Positions
    int hopper_position;
    int lid_position;
    
    // Park positions
    bool hopper_parked;
    bool lid_parked;

};

#endif