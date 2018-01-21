/*
 * Bowie Lights Example
 * --------------------
 * 
 * Test that the super bright leds are working on
 * the robot.
 * 
 * Erin RobotGrrl for RobotMissions
 * Jan. 5th, 2018
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "BowieLights.h"

#define BRIGHT_LED_FRONT_LEFT 21
#define BRIGHT_LED_BACK_LEFT 20
#define BRIGHT_LED_FRONT_RIGHT 36
#define BRIGHT_LED_BACK_RIGHT 35

BowieLights bowielights = BowieLights();

void setup() {
  Serial.begin(9600);

  bowielights.begin();
  bowielights.setFrontLeftPin(BRIGHT_LED_FRONT_LEFT);
  bowielights.setFrontRightPin(BRIGHT_LED_FRONT_RIGHT);
  bowielights.setBackLeftPin(BRIGHT_LED_BACK_LEFT);
  bowielights.setBackRightPin(BRIGHT_LED_BACK_RIGHT);

  bowielights.initLeds();

}

void loop() {

  for(int i=0; i<256; i++) {
    for(int j=0; j<4; j++) {
      bowielights.setLight(j, i);
    }
    delay(2);
  }
  delay(100);

  for(int i=255; i>0; i--) {
    for(int j=0; j<4; j++) {
      bowielights.setLight(j, i);
    }
    delay(2);
  }
  delay(100);

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
