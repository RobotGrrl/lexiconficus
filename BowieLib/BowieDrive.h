/*
 * Robot Missions Bowie Drive
 * --------------------------
 *
 * Drive routines, for use with the default Bowie
 * action wheels. 
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

#ifndef _BOWIEDRIVE_H_
#define _BOWIEDRIVE_H_

// motor pins
/*
#define MOTORA_SPEED 23
#define MOTORB_SPEED 22
#define MOTORA_CTRL1 17 // ?BR
#define MOTORA_CTRL2 16 // ?FR
#define MOTORB_CTRL1 11 // ?FL
#define MOTORB_CTRL2 12 // ?BL
*/

// motor directions
#define MOTOR_DIR_FWD false
#define MOTOR_DIR_REV true

class BowieDrive {

  public:
    BowieDrive();

    // pins
    void setMotorASpeedPin(uint8_t p);
    void setMotorBSpeedPin(uint8_t p);
    void setMotorACtrl1Pin(uint8_t p);
    void setMotorACtrl2Pin(uint8_t p);
    void setMotorBCtrl1Pin(uint8_t p);
    void setMotorBCtrl2Pin(uint8_t p);

    // motor control
    void motor_setDir(uint8_t motorNum, bool dir);
    void motor_setSpeed(uint8_t motorNum, uint8_t speed);
    void motor_setBrake(uint8_t motorNum);
    void motor_setCoast(uint8_t motorNum);
    void leftBork();

    // driving algorithms
    void rampSpeed(bool dir, int start, int end, int step, int del);
    void goSpeed(bool dir, int speed, int del);
    void turnSequence(bool dir);
    void resetTurnSequence();

  private:

    // pins
    uint8_t MOTORA_SPEED;
    uint8_t MOTORB_SPEED;
    uint8_t MOTORA_CTRL1;
    uint8_t MOTORA_CTRL2;
    uint8_t MOTORB_CTRL1;
    uint8_t MOTORB_CTRL2;

    // states
    bool TURN_SEQUENCE_MODE;

    // other
    long current_time;

    // driving algorithms
    uint8_t turn_sequence_step;
    bool restart_step_timer;
    long step_start;

    // init methods
    void initMotors();

};

#endif 