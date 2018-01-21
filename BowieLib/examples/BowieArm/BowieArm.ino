/*
 * Bowie Arm Example
 * ------------------
 * 
 * Test the movement of the robot's arm.
 * 
 * Erin RobotGrrl for RobotMissions
 * Dec. 29th, 2017
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "BowieArm.h"

#define SERVO_ARM1 6
#define SERVO_ARM2 5

BowieArm bowiearm = BowieArm();

void armInterrupt(int key, int val);

void setup() {
  Serial.begin(9600);

  bowiearm.begin();
  bowiearm.setArm1ServoPin(SERVO_ARM1);
  bowiearm.setArm2ServoPin(SERVO_ARM2);

  bowiearm.setServoInterruptCallback(armInterrupt);

  bowiearm.initServos();

}

void loop() {

  // arm up
  bowiearm.moveArm(ARM_MAX);
  delay(1000);
  
  // arm middle
  bowiearm.moveArm(ARM_HOME);
  delay(1000);
  
  // arm down
  bowiearm.moveArm(ARM_MIN);
  delay(1000);
  
  // arm middle
  bowiearm.moveArm(ARM_HOME);
  delay(1000);
  
}

void armInterrupt(int key, int val) {
  
}

