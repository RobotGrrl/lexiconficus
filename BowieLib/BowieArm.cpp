#include "BowieArm.h"

BowieArm::BowieArm() {

}

void BowieArm::begin() {
  arm_position = ARM_PARK;
  arm_parked = false;
}

void BowieArm::setArm1ServoPin(uint8_t p) {
  SERVO_ARM1 = p;
}

void BowieArm::setArm2ServoPin(uint8_t p) {
  SERVO_ARM2 = p;
}

void BowieArm::initServos() {
  pinMode(SERVO_ARM1, OUTPUT);
  pinMode(SERVO_ARM2, OUTPUT);
  
  arm.attach(SERVO_ARM1);
  arm2.attach(SERVO_ARM2);
}

void BowieArm::setServoInterruptCallback( void (*servoInterruptCallback)(int key, int val) ) {
  _servoInterruptCallback = servoInterruptCallback;
}

void BowieArm::servoInterruption(int key, int val) {
  _servoInterruptCallback(key, val);
}

// - Arm
void BowieArm::moveArm(int armPos) {
  moveArm(armPos, 1, 3);
}

void BowieArm::moveArm(int armPos, int step, int del) {

  unparkArm();
  
  int prev_pos = getArmPos();
  if(prev_pos > armPos) { // headed towards ARM_MIN
    for(int i=prev_pos; i>armPos; i-=step) {
      //Serial << i << endl;
      arm.writeMicroseconds(i);
      arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      arm_position = i;      
      delay(del);
      servoInterruption(SERVO_ARM_KEY, i);
    }
  } else if(prev_pos <= armPos) { // headed towards ARM_MAX
    for(int i=prev_pos; i<armPos; i+=step) {
      //Serial << i << endl;
      arm.writeMicroseconds(i);
      arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      arm_position = i;
      delay(del); 
      servoInterruption(SERVO_ARM_KEY, i);
    }
  }
  arm.writeMicroseconds(armPos);
  arm2.writeMicroseconds(SERVO_MAX_US - armPos + SERVO_MIN_US);
  arm_position = armPos;
  delay(del);
  servoInterruption(SERVO_ARM_KEY, armPos);

}

void BowieArm::parkArm() {
  if(arm_parked) return;
  moveArm(ARM_PARK);
  arm.detach();
  arm2.detach();
  arm_parked = true;
}

void BowieArm::unparkArm() {
  if(!arm_parked) return;
  arm.attach(SERVO_ARM1);
  arm2.attach(SERVO_ARM2);
  arm_parked = false;
  moveArm(getArmPos());
}

bool BowieArm::getArmParked() {
  return arm_parked;
}

int BowieArm::getArmPos() {
  return arm_position;
}
