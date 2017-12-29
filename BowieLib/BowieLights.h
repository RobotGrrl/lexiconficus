/*
 * Robot Missions Bowie Lights
 * ----------------------------
 *
 * Functions for Bowie's super bright leds
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * Using Teensy 3.6, works with Teensy 3.2
 * 
 * Erin RobotGrrl
 * Dec. 29th, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 */

#include "Arduino.h"
#include <Streaming.h>

#ifndef _BOWIELIGHTS_H_
#define _BOWIELIGHTS_H_

// super bright led values
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0

class BowieLights {

  public:
    BowieLights();

    void setFrontLeftPin(uint8_t p);
    void setFrontRightPin(uint8_t p);
    void setBackLeftPin(uint8_t p);
    void setBackRightPin(uint8_t p);
    void initLeds();

    // leds and speaker
    void turnOnLights();
    void turnOffLights();
    void dimLights();
    void setLight(uint8_t led_num, uint8_t led_val);

  private:
    uint8_t BRIGHT_LED_FRONT_LEFT;
    uint8_t BRIGHT_LED_BACK_LEFT;
    uint8_t BRIGHT_LED_FRONT_RIGHT;
    uint8_t BRIGHT_LED_BACK_RIGHT;

};

#endif