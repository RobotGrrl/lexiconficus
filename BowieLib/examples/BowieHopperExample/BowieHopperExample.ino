#include "BowieHopper.h"

#define SERVO_HOPPER_PIVOT 3
#define SERVO_HOPPER_LID 25

BowieHopper bowiehopper = BowieHopper();

void hopperAndLidInterrupt(int key, int val);

void setup() {
  Serial.begin(9600);

  bowiehopper.setServoHopperPivotPin(SERVO_HOPPER_PIVOT);
  bowiehopper.setServoHopperLidPin(SERVO_HOPPER_LID);

  bowiehopper.setServoInterruptCallback(hopperAndLidInterrupt);

  bowiehopper.initServos();

}

void loop() {

  // lid open
  bowiehopper.moveLid(LID_MIN);
  delay(1000);

  // hopper tilt open
  bowiehopper.moveHopper(TILT_MIN);
  delay(3000);

  // hopper tilt closed
  bowiehopper.moveHopper(TILT_MAX);
  delay(1000);

  // lid closed
  bowiehopper.moveLid(LID_MAX);
  delay(1000);

}

void hopperAndLidInterrupt(int key, int val) {
  // This will be called any time there is a long
  // duration movement - in between the delays.
  // This is a good place to check for serial
  // messages. Val = the position.
  
  if(key == SERVO_HOPPER_KEY) {
    // was the hopper
  } else if(key == SERVO_LID_KEY) {
    // was the lid
  }
}

