/*
 * Bowie Drive Example
 * -------------------
 * 
 * Test the drive system with the dc gear motors, on the robot.
 * 
 * Erin RobotGrrl for RobotMissions
 * Dec. 28th, 2017
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */


#include "BowieDrive.h"

#define MOTORA_SPEED 23
#define MOTORB_SPEED 22
#define MOTORA_CTRL1 17
#define MOTORA_CTRL2 16
#define MOTORB_CTRL1 11
#define MOTORB_CTRL2 12

BowieDrive bowiedrive = BowieDrive();

long current_time = 0;
long start_turn_time = 0;

void setup() {
  Serial.begin(9600);

  bowiedrive.begin();
  bowiedrive.setMotorASpeedPin(MOTORA_SPEED);
  bowiedrive.setMotorBSpeedPin(MOTORB_SPEED);
  bowiedrive.setMotorACtrl1Pin(MOTORA_CTRL1);
  bowiedrive.setMotorACtrl2Pin(MOTORA_CTRL2);
  bowiedrive.setMotorBCtrl1Pin(MOTORB_CTRL1);
  bowiedrive.setMotorBCtrl2Pin(MOTORB_CTRL2);

}

void loop() {

  current_time = millis();

  // ramp to go forward, then slow down
  bowiedrive.rampSpeed(true, 0, 255, 20, 10);
  bowiedrive.goSpeed(true, 255, 100);
  bowiedrive.rampSpeed(true, 255, 0, 20, 10);
  bowiedrive.goSpeed(true, 0, 10);

  // turning example
  start_turn_time = current_time;
  while(current_time-start_turn_time <= 5000) {
    current_time = millis();
    // turning to the right
    bowiedrive.turnSequence(true);
  }
  // reset it when done
  bowiedrive.resetTurnSequence();
  bowiedrive.goSpeed(true, 0, 10);

}



