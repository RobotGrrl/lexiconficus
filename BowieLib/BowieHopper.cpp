#include "BowieHopper.h"

BowieHopper::BowieHopper() {

}

void BowieHopper::begin() {
  hopper_position = TILT_MAX;
  lid_position = LID_MAX;

  hopper_parked = false;
  lid_parked = false;

}

void BowieHopper::setServoHopperPivotPin(uint8_t p) {
  SERVO_HOPPER_PIVOT = p;
}

void BowieHopper::setServoHopperLidPin(uint8_t p) {
  SERVO_HOPPER_LID = p;
}

void BowieHopper::initServos() {
  pinMode(SERVO_HOPPER_PIVOT, OUTPUT);
  pinMode(SERVO_HOPPER_LID, OUTPUT);
  
  tilt.attach(SERVO_HOPPER_PIVOT);
  lid.attach(SERVO_HOPPER_LID);
}

void BowieHopper::setServoInterruptCallback( void (*servoInterruptCallback)(int key, int val) ) {
  _servoInterruptCallback = servoInterruptCallback;
}

void BowieHopper::servoInterruption(int key, int val) {
  _servoInterruptCallback(key, val);
}

// - Hopper
void BowieHopper::moveHopper(int hopperPos) {
  moveHopper(hopperPos, 1, 3);
}

void BowieHopper::moveHopper(int hopperPos, int step, int del) {
  bool did_move_lid = false;
  bool was_lid_parked = lid_parked;
  int lid_original_pos = getLidPos();
  
  unparkHopper();

  // if(hopperPos != TILT_MAX) { // if it's going anywhere other than flush
  //   if(getLidPos() != LID_MIN) { // check if the lid is not open
  //     moveLid(LID_MIN); // move the lid open
  //     did_move_lid = true;
  //   }
  // }

  Serial << "hopper" << endl;

  int prev_pos = getHopperPos();
  if(prev_pos > hopperPos) { // towards TILT_MIN
    for(int i=prev_pos; i>hopperPos; i-=step) {
      Serial << i << endl;
      tilt.writeMicroseconds(i);
      hopper_position = i;
      delay(del);
      servoInterruption(SERVO_HOPPER_KEY, i);
    }
  } else if(prev_pos <= hopperPos) { // towards TILT_MAX
    for(int i=prev_pos; i<hopperPos; i+=step) {
      Serial << i << endl;
      tilt.writeMicroseconds(i);
      hopper_position = i;
      delay(del);
      servoInterruption(SERVO_HOPPER_KEY, i);
    }
  }
  tilt.writeMicroseconds(hopperPos);
  hopper_position = hopperPos;
  delay(del);
  servoInterruption(SERVO_HOPPER_KEY, hopperPos);
  
  if(did_move_lid) {
    moveLid(lid_original_pos);
    if(was_lid_parked) parkLid();
  }

}

void BowieHopper::parkHopper() {
  if(hopper_parked) return;
  moveHopper(TILT_MAX);
  tilt.detach();
  hopper_parked = true;
}

void BowieHopper::unparkHopper() {
  if(!hopper_parked) return;
  tilt.attach(SERVO_HOPPER_PIVOT);
  hopper_parked = false;
  moveHopper(TILT_MAX);
}

bool BowieHopper::getHopperParked() {
  return hopper_parked;
}

int BowieHopper::getHopperPos() {
  return hopper_position;
}

// - Lid
void BowieHopper::moveLid(int lidPos) {
  moveLid(lidPos, 1, 3);
}

void BowieHopper::moveLid(int lidPos, int step, int del) {
  
  // if(getHopperPos() == TILT_MIN) { // check if the hopper is up
  //   Serial << "!!! Cannot move lid when hopper is up" << endl;
  //   return;
  // }

  unparkLid();

  int prev_pos = getLidPos();
  if(prev_pos > lidPos) { // going to LID_MIN
    for(int i=prev_pos; i>lidPos; i-=step) {
      Serial << "H" << i << endl;
      //lid.write(90);
      lid.writeMicroseconds(i);
      lid_position = i;
      delay(del);
      servoInterruption(SERVO_LID_KEY, i);
    }
  } else if(prev_pos <= lidPos) {
    for(int i=prev_pos; i<lidPos; i+=step) {
      Serial << "H" << i << endl;
      //lid.write(180);
      lid.writeMicroseconds(i);
      lid_position = i;
      delay(del);
      servoInterruption(SERVO_LID_KEY, i);
    }
  }
  lid.writeMicroseconds(lidPos);
  lid_position = lidPos;
  delay(del);
  servoInterruption(SERVO_LID_KEY, lidPos);

}

void BowieHopper::parkLid() {
  if(lid_parked) return;
  moveLid(LID_MAX);
  lid.detach();
  lid_parked = true;
}

void BowieHopper::unparkLid() {
  if(!lid_parked) return;
  lid.attach(SERVO_HOPPER_LID);
  lid_parked = false;
  moveLid(LID_MAX);
}

bool BowieHopper::getLidParked() {
  return lid_parked;
}

int BowieHopper::getLidPos() {
  return lid_position;
}

