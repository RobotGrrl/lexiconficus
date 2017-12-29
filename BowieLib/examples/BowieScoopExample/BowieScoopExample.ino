#include "BowieScoop.h"

#define SERVO_END_EFFECTOR 4
#define SCOOP_PROBE_LEFT A18
#define SCOOP_PROBE_RIGHT A17

BowieScoop bowiescoop = BowieScoop();

void scoopInterrupt(int key, int val);

void setup() {
  Serial.begin(9600);

  bowiescoop.setServoScoopPin(SERVO_END_EFFECTOR);
  bowiescoop.setProbeLPin(SCOOP_PROBE_LEFT);
  bowiescoop.setProbeRPin(SCOOP_PROBE_RIGHT);

  bowiescoop.setServoInterruptCallback(scoopInterrupt);

  bowiescoop.initServos();

}

void loop() {

  // scoop up
  bowiescoop.moveEnd(END_MIN);
  delay(1000);
  
  // scoop middle
  bowiescoop.moveEnd(END_PARALLEL_TOP);
  delay(1000);
  
  // scoop down
  bowiescoop.moveEnd(END_MAX);
  delay(1000);
  
  // scoop middle
  bowiescoop.moveEnd(END_PARALLEL_TOP);
  delay(1000);
  
}

void scoopInterrupt(int key, int val) {
  // This will be called any time there is a long
  // duration movement - in between the delays.
  // This is a good place to check for serial
  // messages. Val = the position.
  
  if(key == SERVO_END_KEY) {
    // It's the scoop!
  }
}

