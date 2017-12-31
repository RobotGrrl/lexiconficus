#include "MegaBowieShoreline.h"

MegaBowieShoreline bowie = MegaBowieShoreline();

void setup() {
  Serial.begin(9600);
  bowie.begin();
}

void loop() {
  bowie.update(false);
}


