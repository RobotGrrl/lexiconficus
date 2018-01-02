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
#include "BowieDrive.h"
#include "BowieHopper.h"
#include "BowieLights.h"
#include "BowieLogger.h"
#include "BowieScoop.h"
#include "BowieComms.h"

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

#define REMOTE_OP_TIMEOUT 300

#ifndef SERVO_ARM_KEY
#define SERVO_ARM_KEY 1
#endif

#ifndef SERVO_END_KEY
#define SERVO_END_KEY 2
#endif

#ifndef SERVO_HOPPER_KEY
#define SERVO_HOPPER_KEY 3
#endif

#ifndef SERVO_LID_KEY
#define SERVO_LID_KEY 4
#endif

#ifndef SERVO_EXTRA_KEY
#define SERVO_EXTRA_KEY 5
#endif


class MegaBowieShoreline {

  static MegaBowieShoreline *bowieInstance;

  public:
    MegaBowieShoreline();
    void begin();

    // Components
    BowieArm bowiearm;
    BowieCurrentSensor servoCurrent;
    BowieCurrentSensor motorCurrent;
    BowieDrive bowiedrive;
    BowieHopper bowiehopper;
    BowieLights bowielights;
    BowieLogger bowielogger;
    BowieScoop bowiescoop;
    BowieComms bowiecomms_xbee;
    BowieComms bowiecomms_arduino;

    // Control
    void update(bool force_no_sleep);
    void control(Msg m);
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

    // Comms Callbacks - Arduino
    static void receivedAction_Arduino(Msg m);
    static void commsTimeout_Arduino();
    static void controllerAdded_Arduino();
    static void controllerRemoved_Arduino();

    // Comms Callbacks - Xbee
    static void receivedAction_Xbee(Msg m);
    static void commsTimeout_Xbee();
    static void controllerAdded_Xbee();
    static void controllerRemoved_Xbee();

    // Current Callbacks
    static void waitingToCoolDown_ServosCallback(bool first);
    static void reactivateAfterCoolDown_ServosCallback();
    static void overCurrentThreshold_ServosCallback(bool first);
    static void waitingToCoolDown_MotorsCallback(bool first);
    static void reactivateAfterCoolDown_MotorsCallback();
    static void overCurrentThreshold_MotorsCallback(bool first);

    // Current
    void waitingToCoolDown_Servos(bool first);
    void reactivateAfterCoolDown_Servos();
    void overCurrentThreshold_Servos(bool first);
    void waitingToCoolDown_Motors(bool first);
    void reactivateAfterCoolDown_Motors();
    void overCurrentThreshold_Motors(bool first);

    // Update logging sensors
    void updateLogSensorData();

    // Specific
    void beep();

};

#endif