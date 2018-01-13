#include "BowieScoop.h"

BowieScoop::BowieScoop() {

}

void BowieScoop::begin() {
  end_position = END_PARALLEL_TOP;

  end_parked = false;

  END_TOUCHDOWN = END_PARALLEL_BOTTOM;

  scoop_probe_left_val = 0;

  scoop_probe_right_val = 0;

}

void BowieScoop::setServoScoopPin(uint8_t p) {
  SERVO_END_EFFECTOR = p;
}

void BowieScoop::setProbeLPin(uint8_t p) {
  SCOOP_PROBE_LEFT = p;
}

void BowieScoop::setProbeRPin(uint8_t p) {
  SCOOP_PROBE_RIGHT = p;
}

void BowieScoop::initServos() {
  pinMode(SERVO_END_EFFECTOR, OUTPUT);
  pinMode(SCOOP_PROBE_LEFT, INPUT);
  pinMode(SCOOP_PROBE_RIGHT, INPUT);
  
  scoop.attach(SERVO_END_EFFECTOR);
}

void BowieScoop::setServoInterruptCallback( void (*servoInterruptCallback)(int key, int val) ) {
  _servoInterruptCallback = servoInterruptCallback;
}

void BowieScoop::servoInterruption(int key, int val) {
  _servoInterruptCallback(key, val);
}


// ---- Scoop Probes

void BowieScoop::updateScoopProbes() {
  scoop_probe_left_val = digitalRead(SCOOP_PROBE_LEFT);
  scoop_probe_right_val = digitalRead(SCOOP_PROBE_RIGHT);
  //Serial << "Scoop probes- L: " << scoop_probe_left_val;
  //Serial << "Scoop probes- R: " << scoop_probe_right_val << endl;
}

bool BowieScoop::getScoopProbeL() {
  updateScoopProbes();
  return scoop_probe_left_val;
}

bool BowieScoop::getScoopProbeR() {
  updateScoopProbes();
  return scoop_probe_right_val;
}

// - End
void BowieScoop::moveEnd(int endPos) {
  moveEnd(endPos, 1, 3);
}

void BowieScoop::moveEnd(int endPos, int step, int del) {

  end_parked = false;

  //unparkEnd();

  int prev_pos = getEndPos();
  if(prev_pos > endPos) { // going towards END_MIN
    for(int i=prev_pos; i>endPos; i-=step) {
      Serial << "S" << i << endl;
      scoop.writeMicroseconds(i);
      end_position = i;
      delay(del);
      servoInterruption(SERVO_END_KEY, i);
    }
  } else if(prev_pos <= endPos) { // going towards END_MAX
    for(int i=prev_pos; i<endPos; i+=step) {
      Serial << "S" << i << endl;
      scoop.writeMicroseconds(i);
      end_position = i;
      delay(del);
      servoInterruption(SERVO_END_KEY, i);
    }
  }
  end_position = endPos;
  scoop.writeMicroseconds(endPos);
  delay(del);
  servoInterruption(SERVO_END_KEY, endPos);

}

void BowieScoop::parkEnd() {
  if(end_parked) return;
  moveEnd(END_PARALLEL_TOP);
  scoop.detach();
  end_parked = true;
}

void BowieScoop::unparkEnd() {
  if(!end_parked) return;
  scoop.attach(SERVO_END_EFFECTOR);
  end_parked = false;
  moveEnd(END_PARALLEL_TOP);
}

bool BowieScoop::getEndParked() {
  return end_parked;
}

int BowieScoop::getEndPos() {
  return end_position;
}

