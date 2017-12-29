#include "BowieLights.h"

BowieLights::BowieLights() {

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
}

void BowieLights::turnOffLights() {
  digitalWrite(BRIGHT_LED_FRONT_LEFT, LOW);
  digitalWrite(BRIGHT_LED_FRONT_RIGHT, LOW);
  digitalWrite(BRIGHT_LED_BACK_LEFT, LOW);
  digitalWrite(BRIGHT_LED_BACK_RIGHT, LOW);
}

void BowieLights::dimLights() {
  analogWrite(BRIGHT_LED_FRONT_LEFT, MIN_BRIGHTNESS);
  analogWrite(BRIGHT_LED_FRONT_RIGHT, MIN_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_LEFT, MIN_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_RIGHT, MIN_BRIGHTNESS);
}

void BowieLights::setLight(uint8_t led_num, uint8_t led_val) {
  if(led_num == 0) {
    analogWrite(BRIGHT_LED_FRONT_LEFT, led_val);
  } else if(led_num == 1) {
    analogWrite(BRIGHT_LED_FRONT_RIGHT, led_val);
  } else if(led_num == 2) {
    analogWrite(BRIGHT_LED_BACK_LEFT, led_val);
  } else if(led_num == 3) {
    analogWrite(BRIGHT_LED_BACK_RIGHT, led_val);
  } else if(led_num == 99) {
    analogWrite(BRIGHT_LED_FRONT_LEFT, led_val);
    analogWrite(BRIGHT_LED_BACK_LEFT, led_val);
    analogWrite(BRIGHT_LED_FRONT_RIGHT, led_val);
    analogWrite(BRIGHT_LED_BACK_RIGHT, led_val);
  }
}
