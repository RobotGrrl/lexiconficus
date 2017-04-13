/*
 * Robot Missions Operator Control Unit NEW
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
#define OP_DEBUG false
#define BUTTON_DEBUG true

// button pins
#define REDBUTTON 14
#define GREENBUTTON 2
#define YELLOWBUTTON 3
#define BLUEBUTTON 4
#define WHITEBUTTON 7
#define BLACKBUTTON 8
#define DEBOUNCE 10

// led pins
#define BOARD_LED 13
#define LED1 23
#define LED2 22
#define LED3 21
#define LED4 20
#define LED5 6
#define LED6 5

// messages
#define MSG_QUEUE_SIZE 3

// joystick
#define JOYSTICK_X 17
#define JOYSTICK_Y 16
#define JOYSTICK_SW 15
#define ACTIVITY_TIMEOUT 1000
#define IDLE_UPDATE_FREQ 250
#define MAX_X 1015 // left
#define MIN_X 1 // right
#define MAX_Y 1023 // up
#define MIN_Y 1 // down
#define HOME_X 347
#define HOME_Y 356

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
    void breatheLeds();

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
    Bounce blackbouncer;
    bool getRedButton();
    bool getGreenButton();
    bool getYellowButton();
    bool getBlueButton();
    bool getWhiteButton();
    bool getBlackButton();

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
    void initJoystick();

    // joystick
    uint16_t joy_x;
    uint16_t joy_y;
    uint16_t joy_sw;
    bool joystick_idle;
    unsigned long last_activity;
    unsigned long last_idle_update;
    bool first_idle;
    void joystickControl();

    // buttons
    void updateButtons();
    bool red_on;
    bool green_on;
    bool yellow_on;
    bool blue_on;
    bool white_on;
    bool black_on;
    bool auton_blink;
    unsigned long last_auton_blink;

};

#endif