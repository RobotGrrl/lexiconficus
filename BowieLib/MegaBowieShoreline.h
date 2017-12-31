/*
 * Robot Missions Mega Bowie Shoreline (Mission #1)
 * ------------------------------------------------
 *
 * Library for the control and logic of the Robot
 * Missions robot platform (Bowie) that is designed
 * for Mission #1 - Shoreline Cleanup. Mega version,
 * using Teensy 3.6. (For Teensy 3.2, please see
 * Mini version).
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 * 
 * Erin RobotGrrl
 * Dec. 30th, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 */

#include "Arduino.h"
#include <Streaming.h>

#include "BowieArm.h"
#include "BowieCurrentSensor.h"

#ifndef _MEGABOWIESHORELINE_H_
#define _MEGABOWIESHORELINE_H_

// motor pins
#define MOTORA_SPEED 23
#define MOTORB_SPEED 22
#define MOTORA_CTRL1 17 // ?BR
#define MOTORA_CTRL2 16 // ?FR
#define MOTORB_CTRL1 11 // ?FL
#define MOTORB_CTRL2 12 // ?BL

// servo pins
#define SERVO_ARM1 6
#define SERVO_ARM2 5
#define SERVO_END_EFFECTOR 4
#define SERVO_HOPPER_PIVOT 3
#define SERVO_HOPPER_LID 25
#define SERVO_EXTRA 24

// sensor pins
#define CURRENT_SERVO_SENS 15
#define CURRENT_MOTOR_SENS 14
  
// led pins
#define BOARD_LED 13
#define BRIGHT_LED_FRONT_LEFT 21
#define BRIGHT_LED_BACK_LEFT 20
#define BRIGHT_LED_FRONT_RIGHT 36
#define BRIGHT_LED_BACK_RIGHT 35
#define COMM_LED 2
#define SPEAKER 30

// touch probe pins
#define SCOOP_PROBE_LEFT A18
#define SCOOP_PROBE_RIGHT A17

struct Packet {
  char cmd;
  uint8_t key;
  uint16_t val;
};

struct Msg {
  uint8_t priority;
  char action;
  Packet pck1;
  Packet pck2;
  char delim;
};


#define REMOTE_OP_TIMEOUT 300



class MegaBowieShoreline {

  public:
    MegaBowieShoreline();
    void begin();

    // Components
    BowieArm bowiearm;
    BowieCurrentSensor servoCurrent = BowieCurrentSensor();
    BowieCurrentSensor motorCurrent = BowieCurrentSensor();


    // Control
    void update(bool force_no_sleep);
    void control(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    static void servoInterrupt(int key, int val);
    
    // States
    void enableRemoteOp();
    void disableRemoteOp();
    void enableLogging();
    void disableLogging();

    // Combined movements
    void moveArmAndEnd(int armPos, int step, int del, int armMin, int armMax, int endMin, int endMax);
    int clawParallelVal(int arm_Val);
    int clawParallelValBounds(int arm_Val, int armMin, int armMax, int endMin, int endMax);


  private:

    // States
    bool REMOTE_OP_ENABLED; // true by default
    bool PREVENT_OVER_CURRENT; // false by default (advanced functionality)
    bool LOGGING_ENABLED; // true by default

    uint8_t unlikely_count;
    long current_time;
    long last_ctrl;

    // Current
    static void waitingToCoolDown_ServosCallback(bool first);
    static void reactivateAfterCoolDown_ServosCallback();
    static void overCurrentThreshold_ServosCallback(bool first);
    static void waitingToCoolDown_MotorsCallback(bool first);
    static void reactivateAfterCoolDown_MotorsCallback();
    static void overCurrentThreshold_MotorsCallback(bool first);

    void waitingToCoolDown_Servos(bool first);
    void reactivateAfterCoolDown_Servos();
    void overCurrentThreshold_Servos(bool first);
    void waitingToCoolDown_Motors(bool first);
    void reactivateAfterCoolDown_Motors();
    void overCurrentThreshold_Motors(bool first);

};

#endif