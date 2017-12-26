/*
 * Robot Missions Mega Bowie Shoreline (Mission #1)
 * ------------------------------------------------
 *
 * Library for the control and logic of the Robot
 * Missions robot platform (Bowie) that is designed
 * for Mission #1 - Shoreline Cleanup. Mega version,
 * using Teensy 3.6. (For Teensy 3.2, please see
 * Mini version).
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 * 
 * Erin RobotGrrl
 * Dec. 24th, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 *
 */

#include "Arduino.h"
#include <Streaming.h>

#ifndef _MEGABOWIESHORELINE_H_
#define _MEGABOWIESHORELINE_H_

/*
Serial1 = Raspberry Pi
Serial2 = Xbee
Serial3 = GPS
Serial4 = Pixy
Serial5 = BT
Serial6 = External Arduino (obstacle avoidance)
*/

struct Cmd {
  char cmd;
  uint8_t key;
  uint16_t val;
};

class MegaBowieShoreline {

  public:
    MegaBowieShoreline();
    void control(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);

  private:


};

#endif