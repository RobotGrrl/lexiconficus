#include "BowieCurrentSensor.h"

#define CURRENT_SERVO_SENS 15
#define CURRENT_MOTOR_SENS 14

BowieCurrentSensor servoCurrent = BowieCurrentSensor();
BowieCurrentSensor motorCurrent = BowieCurrentSensor();

void setup() {
  Serial.begin(9600);

  servoCurrent.setCurrentSensePin(CURRENT_SERVO_SENS);
  servoCurrent.setCurrentSenseName("Servo");
  servoCurrent.initCurrentSensor();
  servoCurrent.set_waitingToCoolDown_callback(waitingToCoolDown_Servos);
  servoCurrent.set_reactivateAfterCoolDown_callback(reactivateAfterCoolDown_Servos);
  servoCurrent.set_overCurrentThreshold_callback(overCurrentThreshold_Servos);
  
  motorCurrent.setCurrentSensePin(CURRENT_MOTOR_SENS);
  motorCurrent.setCurrentSenseName("Motor");
  motorCurrent.initCurrentSensor();
  motorCurrent.set_waitingToCoolDown_callback(waitingToCoolDown_Motors);
  motorCurrent.set_reactivateAfterCoolDown_callback(reactivateAfterCoolDown_Motors);
  motorCurrent.set_overCurrentThreshold_callback(overCurrentThreshold_Motors);

}

void loop() {

  servoCurrent.updateCurrentSensor();
  motorCurrent.updateCurrentSensor();

}

void waitingToCoolDown_Servos(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
}

void reactivateAfterCoolDown_Servos() {
 // you might want to re-attach the servos here
 // and send them back to a position
 // (or re-activate the dc motors)
}

void overCurrentThreshold_Servos(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
}

void waitingToCoolDown_Motors(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
}

void reactivateAfterCoolDown_Motors() {
 // you might want to re-attach the servos here
 // and send them back to a position
 // (or re-activate the dc motors)
}

void overCurrentThreshold_Motors(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
}


