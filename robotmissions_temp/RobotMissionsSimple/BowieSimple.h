/*
 * Robot Missions Robot Platform Library
 * -------------------------------------
 *
 * The robot's nickname is Bowie
 * - Serial = serial port
 * - Serial1 = bluetooth (optional)
 * - Serial2 = xbee
 * - Serial3 = pixy or gps
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
#include <Servo.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>

#ifndef _BOWIESIMPLE_H_
#define _BOWIESIMPLE_H_

//#define DEBUG false
#define COMM_DEBUG false
#define OP_DEBUG false
#define SENS_DEBUG false

// motor pins
#define MOTORA_SPEED 22
#define MOTORB_SPEED 23
#define MOTORA_CTRL1 12
#define MOTORA_CTRL2 11
#define MOTORB_CTRL1 16
#define MOTORB_CTRL2 17

// servo pins
#define SERVO_ARM1 4
#define SERVO_ARM2 6
#define SERVO_END 5
#define SERVO_OTHER 3

// sensor pins
#define FORCE_SENSOR_LEFT A0
#define FORCE_SENSOR_RIGHT A1
#define SONAR_LEFT A10
#define SONAR_RIGHT A11
  
// led pins
#define BOARD_LED 13
#define BRIGHT_LED_LEFT 20//21
#define BRIGHT_LED_RIGHT 3//20
#define COMM_LED 21//2

// gpio pins
#define GPIO_PIN1 A18
#define GPIO_PIN2 A19
#define GPIO_PIN3 A20
#define GPIO_PIN4 A17
#define GPIO_PIN5 A16

// motor directions
#define MOTOR_DIR_FWD false
#define MOTOR_DIR_REV true

// servo values
#define ARM_MAX 130
#define ARM_HOME 100
#define ARM_MIN 30

#define CLAW_MIN 1300
#define CLAW_HOME 1100
#define CLAW_MAX 500

// super bright led values
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 100

// messages
#define MSG_QUEUE_SIZE 3
#define REMOTE_OP_TIMEOUT 300

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

class Bowie {
  
  public:

    Bowie();

    // inits and updates
    void init();
    void update();

    // api
    void control(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);

    // states
    void enableRemoteOp();
    void disableRemoteOp();

    // leds
    void turnOnLights();

    // motor control
    void motor_setDir(uint8_t motorNum, bool dir);
    void motor_setSpeed(uint8_t motorNum, uint8_t speed);
    void motor_setBrake(uint8_t motorNum);
    void motor_setCoast(uint8_t motorNum);
    void leftBork();

    // messages
    Msg msg_none = { 9, '^', '0', 0, 0, '0', 0, 0, '!' };
    uint8_t getMsgQueueLength();
    Msg popNextMsg();
    void addNextMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);
    void addNextMsg(Msg m);
    void insertNextMsg(Msg m);
    void chooseNextMessage();

  private:

    // states
    bool REMOTE_OP_ENABLED;
    bool GYRO_ENABLED;
    bool MAG_ENABLED;
    bool ACCEL_ENABLED;
    bool BMP_ENABLED;

    // comms
    uint8_t msgs_in_queue;
    uint8_t msg_send_index;
    Msg msg_queue[MSG_QUEUE_SIZE];
    unsigned long last_rx;
    uint8_t unlikely_count = 0;

    // servos
    Servo arm;
    Servo claw;
    Servo arm2;

    // gpio
    bool gpio_pin1_input;
    bool gpio_pin2_input;
    bool gpio_pin3_input;
    bool gpio_pin4_input;
    bool gpio_pin5_input;

    // sensors
    uint16_t gyro_msg_x;
    uint16_t gyro_msg_y;
    uint16_t gyro_msg_z;
    uint16_t mag_msg_x;
    uint16_t mag_msg_y;
    uint16_t mag_msg_z;
    uint16_t accel_msg_x;
    uint16_t accel_msg_y;
    uint16_t accel_msg_z;
    uint16_t alt_msg;
    uint16_t temp_msg;
    uint16_t sonar_val_left;
    uint16_t sonar_val_right;
    uint16_t force_sensor_val_left;
    uint16_t force_sensor_val_right;
    uint16_t gpio_pin1_val;
    uint16_t gpio_pin2_val;
    uint16_t gpio_pin3_val;
    uint16_t gpio_pin4_val;
    uint16_t gpio_pin5_val;

    // adafruit 10dof board
    Adafruit_L3GD20_Unified       gyro;//gyro(20);
    Adafruit_LSM303_Accel_Unified accel;//accel(30301);
    Adafruit_LSM303_Mag_Unified   mag;//mag(30302);
    Adafruit_BMP085_Unified       bmp;//bmp(18001);

    // init methods
    void initMotors();
    void initServos();
    void initSensors();
    void initLeds();
    void initGPIO(uint8_t p, uint8_t state);

};

#endif