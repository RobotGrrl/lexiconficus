#include "BowieLights.h"

#define BRIGHT_LED_FRONT_LEFT 21
#define BRIGHT_LED_BACK_LEFT 20
#define BRIGHT_LED_FRONT_RIGHT 36
#define BRIGHT_LED_BACK_RIGHT 35

BowieLights bowielights = BowieLights();

void setup() {
  Serial.begin(9600);

  bowielights.setFrontLeftPin(BRIGHT_LED_FRONT_LEFT);
  bowielights.setFrontRightPin(BRIGHT_LED_FRONT_RIGHT);
  bowielights.setBackLeftPin(BRIGHT_LED_BACK_LEFT);
  bowielights.setBackRightPin(BRIGHT_LED_BACK_RIGHT);

  bowielights.initLeds();

}

void loop() {

  bowielights.setLight(0, 60); // front left
  delay(1000);

  bowielights.setLight(1, 60); // front right
  delay(1000);

  bowielights.setLight(2, 60); // back left
  delay(1000);

  bowielights.setLight(3, 60); // back right
  delay(1000);

  delay(4000);

  bowielights.turnOffLights();
  delay(5000);

}
