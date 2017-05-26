/*
 * Robot Missions Operator Control Unit
 * -------------------------------------
 *
 *  Arduino Duiemilinove
 *  
 *  xbee rx to 3
 *  xbee tx to 2
 *  nunchuck purple (d) to a4
 *  nunchuck orange (c) to a5
 *  
 *  0 - gnd
 *  1 - 5v
 *  2 - A4
 *  3 - A5
 *  4 - pin 3
 *  5 - pin 2
 *  
 *  buttons
 *  red: brown 8
 *  green: red 7
 *  yellow: orange 6
 *  blue: yellow 5
 *  white: green 4
 *  
 *  leds
 *  1: white A3
 *  2: grey A2
 *  3: purple A1
 *  4: blue A0
 *
 * March 20, 2017
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 */

#include "Arduino.h"
#include "Streaming.h"
#include <ArduinoNunchuk.h>
#include <Bounce2.h>

#ifndef _OPERATORSIMPLE_H_
#define _OPERATORSIMPLE_H_

//#define DEBUG false
#define COMM_DEBUG false
#define OP_DEBUG true

// button pins
#define REDBUTTON 8
#define GREENBUTTON 7
#define YELLOWBUTTON 6
#define BLUEBUTTON 5
#define WHITEBUTTON 4
#define DEBOUNCE 10

// led pins
#define BOARD_LED 13
#define LED1 A3
#define LED2 A2
#define LED3 A1
#define LED4 A0

// messages
#define MSG_QUEUE_SIZE 3

// nunchuk
#define ACTIVITY_TIMEOUT 1000
#define IDLE_UPDATE_FREQ 250
#define MAX_X 228
#define MIN_X 34
#define MAX_Y 223
#define MIN_Y 34
#define HOME_X 125
#define HOME_Y 135

struct Msg {
  uint8_t priority;
  char action;
  char cmd;
  uint8_t key;
  uint16_t val;
  char cmd2;
  uint8_t key2;
  uint16_t val2;
  char delim;
};

class Operator {

  public:
    Operator();
    void init();
    void update();

    // messages
    Msg msg_none = { 9, '^', '0', 0, 0, '0', 0, 0, '!' };
    uint8_t getMsgQueueLength();
    Msg popNextMsg();
    void addNextMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void addNextMsg(Msg m);
    void insertNextMsg(Msg m);

    // buttons
    Bounce redbouncer;
    Bounce greenbouncer;
    Bounce yellowbouncer;
    Bounce bluebouncer;
    Bounce whitebouncer;
    bool getRedButton();
    bool getGreenButton();
    bool getYellowButton();
    bool getBlueButton();
    bool getWhiteButton();

  private:

    // update
    unsigned long current_time;

    // messages
    uint8_t msgs_in_queue;
    Msg msg_queue[MSG_QUEUE_SIZE];

    // modes
    bool turn_on_spot;
    bool slower_speed;
    uint8_t slow_speed;

    // init
    void initLeds();
    void initButtons();

    // nunchuck
    bool nunchuck_idle;
    unsigned long last_activity;
    unsigned long last_idle_update;
    bool first_idle;
    ArduinoNunchuk nunchuk;
    void nunchuckControl();

    // buttons
    void updateButtons();
    bool red_on;
    bool green_on;
    bool yellow_on;
    bool blue_on;
    bool white_on;
    bool auton_blink;
    unsigned long last_auton_blink;

};

#endif