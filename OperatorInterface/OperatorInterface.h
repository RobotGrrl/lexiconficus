/*
 * Robot Missions Operator Interface
 * -------------------------------------
 *
 * Teensy 3.2
 * Board by RGlenn

Xbee is on Serial1

 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 */

#include "Arduino.h"
#include <Streaming.h>
#include <XBee.h>
#include "PromulgateBig.h"
#include <Bounce2.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifndef _OPINTERFACE_H_
#define _OPINTERFACE_H_

//#define DEBUG false
#define COMM_DEBUG false
#define OP_DEBUG false
#define BUTTON_DEBUG true

// speeds
#define MAX_SPEED 255
#define MIN_SPEED 120
#define TURN_SPEED_FWD 180
#define TURN_SPEED_REV 180

// button pins
#define BUTTON1 7
#define BUTTON2 8
#define BUTTON3 14
#define BUTTON4 2
#define BUTTON5 3
#define BUTTON6 4

// buttons
#define DEBOUNCE 20

// mode switch
#define MODE_SW A10
#define MODE1_THRESH 1000 // 1023
#define MODE2_THRESH 500 // 512
#define MODE3_THRESH 0 // 1
#define MODE1 1
#define MODE2 2
#define MODE3 3

// led pins
#define BOARD_LED 13
#define LED1 5
#define LED2 6
#define LED3 20
#define LED4 23
#define LED5 22
#define LED6 21

// speaker pin
#define SPEAKER 10

// display pins
#define OLED_RESET 12

// messages
#define MSG_QUEUE_SIZE 3

// joystick
#define JOYSTICK_X 16
#define JOYSTICK_Y 17
#define JOYSTICK_SW 15
#define ACTIVITY_TIMEOUT 1000
#define IDLE_UPDATE_FREQ 250
#define MAX_X 1015 // left
#define MIN_X 1 // right
#define MAX_Y 1023 // up
#define MIN_Y 1 // down
#define ZERO_ZONE 30

// states
#define IDLE_STATE 0
#define DRIVE_STATE 1
#define ARM_STATE 2
#define DUMP_STATE 3
#define SCOOP_S_STATE 4
#define SCOOP_F_STATE 5
#define EMPTY_STATE 6

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

struct Cmd {
  char cmd;
  uint8_t key;
  uint16_t val;
};

// width: 128 height: 64
PROGMEM const unsigned char robot_missions_logo[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x0f, 0xff, 0xbf, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0xff, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x3f, 0xfe, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x7f, 0xfc, 0x07, 0xff, 0xc0, 0x00, 0x00, 0x01, 0x80, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 
0x00, 0xff, 0xf8, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x01, 0x80, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 
0x00, 0xff, 0xf0, 0x01, 0xff, 0xe0, 0x06, 0xe3, 0xe1, 0xbc, 0x1f, 0x0f, 0x80, 0x00, 0x00, 0x00, 
0x01, 0xff, 0xe0, 0x00, 0xff, 0xf0, 0x07, 0xe7, 0xf1, 0xfe, 0x3f, 0x8f, 0x80, 0x00, 0x00, 0x00, 
0x03, 0xff, 0xc0, 0xe0, 0x7f, 0xf8, 0x07, 0x0e, 0x39, 0xc7, 0x71, 0xc6, 0x00, 0x00, 0x00, 0x00, 
0x03, 0xff, 0x81, 0xf0, 0x3f, 0xf8, 0x06, 0x0c, 0x19, 0x83, 0x60, 0xc6, 0x00, 0x00, 0x00, 0x00, 
0x03, 0xff, 0x03, 0x98, 0x1f, 0xf8, 0x06, 0x0c, 0x19, 0x83, 0x60, 0xc6, 0x00, 0x00, 0x00, 0x00, 
0x07, 0xfe, 0x07, 0x0c, 0x0f, 0xfc, 0x06, 0x0c, 0x19, 0x83, 0x60, 0xc6, 0x00, 0x00, 0x00, 0x00, 
0x07, 0xfc, 0x0f, 0x06, 0x07, 0xfc, 0x06, 0x0e, 0x39, 0xc7, 0x71, 0xc6, 0x00, 0x00, 0x00, 0x00, 
0x07, 0xf8, 0x0f, 0x02, 0x03, 0xfc, 0x06, 0x07, 0xf1, 0xfe, 0x3f, 0x87, 0xc0, 0x00, 0x00, 0x00, 
0x0f, 0xf0, 0x1f, 0xc3, 0x01, 0xfe, 0x06, 0x03, 0xe1, 0xbc, 0x1f, 0x03, 0x80, 0x00, 0x00, 0x00, 
0x0f, 0xe0, 0x1f, 0x81, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x0f, 0xc0, 0x3f, 0x21, 0x80, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x0f, 0x80, 0x3f, 0xc0, 0x80, 0x3e, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 
0x0f, 0x00, 0x3f, 0x98, 0x80, 0x1e, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 
0x0f, 0x80, 0x3f, 0xf0, 0x80, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x0f, 0xc0, 0x3f, 0xc1, 0x80, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x0f, 0xe0, 0x1f, 0xc1, 0x00, 0xfe, 0x06, 0xf3, 0xc6, 0x3e, 0x0f, 0x8c, 0x3e, 0x1b, 0xe3, 0xe0, 
0x0f, 0xf0, 0x1f, 0xe3, 0x01, 0xfe, 0x07, 0xff, 0xe6, 0x7f, 0x1f, 0xcc, 0x7f, 0x1f, 0xf7, 0xf0, 
0x07, 0xf8, 0x0f, 0xe2, 0x03, 0xfc, 0x07, 0x1c, 0x66, 0x61, 0x98, 0x6c, 0xe3, 0x9c, 0x36, 0x18, 
0x07, 0xfc, 0x0f, 0xe6, 0x07, 0xfc, 0x06, 0x18, 0x66, 0x70, 0x1c, 0x0c, 0xc1, 0x98, 0x37, 0x00, 
0x07, 0xfe, 0x07, 0xec, 0x0f, 0xfc, 0x06, 0x18, 0x66, 0x1e, 0x07, 0x8c, 0xc1, 0x98, 0x31, 0xe0, 
0x03, 0xff, 0x03, 0xf8, 0x1f, 0xf8, 0x06, 0x18, 0x66, 0x03, 0x00, 0xcc, 0xc1, 0x98, 0x30, 0x30, 
0x03, 0xff, 0x81, 0xf0, 0x3f, 0xf8, 0x06, 0x18, 0x66, 0x61, 0x98, 0x6c, 0xe3, 0x98, 0x36, 0x18, 
0x03, 0xff, 0xc0, 0xe0, 0x7f, 0xf8, 0x06, 0x18, 0x66, 0x7f, 0x9f, 0xec, 0x7f, 0x18, 0x37, 0xf8, 
0x01, 0xff, 0xe0, 0x40, 0xff, 0xf0, 0x06, 0x18, 0x66, 0x3e, 0x0f, 0x8c, 0x3e, 0x18, 0x33, 0xe0, 
0x00, 0xff, 0xf0, 0x01, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0xff, 0xf8, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x7f, 0xfc, 0x07, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x3f, 0xfe, 0x0f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x1f, 0xff, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x0f, 0xff, 0xbf, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x03, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x01, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x07, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};


class Operator {

  public:
    Operator();

    void init();
    void update();
    void calibrateHome();
    void breatheLeds();

    // display
    Adafruit_SSD1306 display;

    // state
    int CURRENT_STATE;
    bool GO_TIME;
    int LAST_STATE;

    // mode (operator / semi-autonomous / full autonomous)
    int CURRENT_MODE;
    void updateModeSwitch();

    // can't believe this joystick drifts
    int HOME_X;
    int HOME_Y;

    // messages
    Msg msg_none = { 9, '^', '0', 0, 0, '0', 0, 0, '!' };
    uint8_t getMsgQueueLength();
    Msg popNextMsg();
    void addNextMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void addNextMsg(Msg m);
    void insertNextMsg(Msg m);

    // joystick
    int getJoyX();
    int getJoyY();
    bool joystick_on;

    // buttons
    bool getRedButton();
    bool getGreenButton();
    bool getYellowButton();
    bool getBlueButton();
    bool getWhiteButton();
    bool getBlackButton();
    bool getJoystickButton();

    // leds
    void ledsOff();
    void buttonsOff();
    void introLedSequence();

    // display
    void displayLogo();
    void scrollLogo();
    void displayTitleBar();
    void mainMenu();

    // speaker
    void buzz(int targetPin, long frequency, long length);


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
    void initSpeaker();

    // joystick
    uint16_t joy_x;
    uint16_t joy_y;
    uint16_t joy_sw;
    uint16_t joy_x_prev;
    uint16_t joy_y_prev;
    bool joystick_idle;
    unsigned long last_activity;
    unsigned long last_idle_update;
    bool first_idle;
    int motor_speed;
    int incr_speed;
    int arm_pos;
    void joystickDriveControl();
    void joystickArmControl();

    // buttons
    void updateButtons();
    bool red_on;
    bool green_on;
    bool yellow_on;
    bool blue_on;
    bool white_on;
    bool black_on;
    
    // joystick
    long last_increment;
    bool scrolling_up;
    int turn_speed;

};

#endif