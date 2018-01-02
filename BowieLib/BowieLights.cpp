#include "BowieLights.h"

BowieLights::BowieLights() {
  for(int i=0; i<4; i++) {
    led_val[i] = 0;
  }
}

void BowieLights::setFrontLeftPin(uint8_t p) {
  BRIGHT_LED_FRONT_LEFT = p;
}

void BowieLights::setFrontRightPin(uint8_t p) {
  BRIGHT_LED_FRONT_RIGHT = p;
}

void BowieLights::setBackLeftPin(uint8_t p) {
  BRIGHT_LED_BACK_LEFT = p;
}

void BowieLights::setBackRightPin(uint8_t p) {
  BRIGHT_LED_BACK_RIGHT = p;
}

void BowieLights::initLeds() {
  pinMode(BRIGHT_LED_FRONT_LEFT, OUTPUT);
  pinMode(BRIGHT_LED_FRONT_RIGHT, OUTPUT);
  pinMode(BRIGHT_LED_BACK_LEFT, OUTPUT);
  pinMode(BRIGHT_LED_BACK_RIGHT, OUTPUT);
}

void BowieLights::turnOnLights() {
  analogWrite(BRIGHT_LED_FRONT_LEFT, MAX_BRIGHTNESS);
  analogWrite(BRIGHT_LED_FRONT_RIGHT, MAX_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_LEFT, MAX_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_RIGHT, MAX_BRIGHTNESS);
  for(int i=0; i<4; i++) {
    led_val[i] = MAX_BRIGHTNESS;
  }
}

void BowieLights::turnOffLights() {
  digitalWrite(BRIGHT_LED_FRONT_LEFT, LOW);
  digitalWrite(BRIGHT_LED_FRONT_RIGHT, LOW);
  digitalWrite(BRIGHT_LED_BACK_LEFT, LOW);
  digitalWrite(BRIGHT_LED_BACK_RIGHT, LOW);
  for(int i=0; i<4; i++) {
    led_val[i] = 0;
  }
}

void BowieLights::dimLights() {
  analogWrite(BRIGHT_LED_FRONT_LEFT, MIN_BRIGHTNESS);
  analogWrite(BRIGHT_LED_FRONT_RIGHT, MIN_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_LEFT, MIN_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_RIGHT, MIN_BRIGHTNESS);
  for(int i=0; i<4; i++) {
    led_val[i] = MIN_BRIGHTNESS;
  }
}

void BowieLights::setLight(uint8_t led_num, uint8_t val) {
  if(led_num == 0) {
    analogWrite(BRIGHT_LED_FRONT_LEFT, val);
    led_val[0] = val;
  } else if(led_num == 1) {
    analogWrite(BRIGHT_LED_FRONT_RIGHT, val);
    led_val[1] = val;
  } else if(led_num == 2) {
    analogWrite(BRIGHT_LED_BACK_LEFT, val);
    led_val[2] = val;
  } else if(led_num == 3) {
    analogWrite(BRIGHT_LED_BACK_RIGHT, val);
    led_val[3] = val;
  } else if(led_num == 99) {
    analogWrite(BRIGHT_LED_FRONT_LEFT, val);
    analogWrite(BRIGHT_LED_BACK_LEFT, val);
    analogWrite(BRIGHT_LED_FRONT_RIGHT, val);
    analogWrite(BRIGHT_LED_BACK_RIGHT, val);
    for(int i=0; i<4; i++) {
      led_val[i] = val;
    }
  }
}

int BowieLights::getLight(uint8_t p) {
  return led_val[p];
}
