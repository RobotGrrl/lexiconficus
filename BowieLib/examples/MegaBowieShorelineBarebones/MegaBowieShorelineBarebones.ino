#include "MegaBowieShoreline.h"

#define ROBOT_ID 3

MegaBowieShoreline bowie;

void setup() {
  Serial.begin(9600);
  bowie = MegaBowieShoreline();
  bowie.setRobotID(ROBOT_ID);
  bowie.begin();
}

void loop() {
  bowie.update(false);
}


