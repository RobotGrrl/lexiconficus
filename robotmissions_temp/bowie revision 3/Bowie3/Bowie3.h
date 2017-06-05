/*
 * Robot Missions Bowie Revision 3 Library
 * ----------------------------------------
 *
 * May 26, 2017
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

#ifndef _BOWIE3_H_
#define _BOWIE3_H_

//#define DEBUG false
#define COMM_DEBUG false
#define OP_DEBUG false
#define SENS_DEBUG false

// motor pins
#define MOTORA_SPEED 23
#define MOTORB_SPEED 22
#define MOTORA_CTRL1 16 // BR
#define MOTORA_CTRL2 17 // FR
#define MOTORB_CTRL1 12 // FL
#define MOTORB_CTRL2 11 // BL

// servo pins
#define SERVO_ARM1 3
#define SERVO_ARM2 4
#define SERVO_END_EFFECTOR 5
#define SERVO_HOPPER_PIVOT 24
#define SERVO_HOPPER_LID 25
#define SERVO_EXTRA 6

// sensor pins
#define CURRENT_SERVO_SENS 15
#define CURRENT_MOTOR_SENS 14

#define FORCE_SENSOR_LEFT A0
#define FORCE_SENSOR_RIGHT A1
#define SONAR_LEFT A10
#define SONAR_RIGHT A11
  
// led pins
#define BOARD_LED 13
#define BRIGHT_LED_FRONT_LEFT 20
#define BRIGHT_LED_FRONT_RIGHT 21
#define BRIGHT_LED_BACK_LEFT 35
#define BRIGHT_LED_BACK_RIGHT 36
#define COMM_LED 13 //2

// gpio pins
#define GPIO_PIN1 A18
#define GPIO_PIN2 A19
#define GPIO_PIN3 A20
#define GPIO_PIN4 A17
#define GPIO_PIN5 A16

// motor directions
#define MOTOR_DIR_FWD true
#define MOTOR_DIR_REV false

// servo values
#define ARM_MIN 700 // down
#define ARM_HOME 1200 // middle
#define ARM_MAX 2300 // up

#define END_MIN 700 // up
#define END_PARALLEL_TOP 2300 // parallel to ground when arm is raised, raised a bit to keep debris in
#define END_PARALLEL_BOTTOM 1100 // parallel to ground when arm is lowered
#define END_HOME 1400 // parallel to arm, good for digging
#define END_MAX 1800 // down

#define TILT_MIN 700 // tilted (emptying)
#define TILT_MAX 1600 // flush

#define LID_MIN 1000 // open
#define LID_MAX 2000 // closed

#define SERVO_MAX_US 2500
#define SERVO_MIN_US 500

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

    // servos
    Servo arm;
    Servo arm2;
    Servo end;
    Servo tilt;
    Servo lid;
    Servo extra;
    void moveArm(int armPos);
    int getArmPos();
    int clawParallelVal(int arm_Val);
    void moveScoop(int targetArmuS, int targetClawuS);

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

    // gpio
    bool gpio_pin1_input;
    bool gpio_pin2_input;
    bool gpio_pin3_input;
    bool gpio_pin4_input;
    bool gpio_pin5_input;

    // sensors
    uint16_t current_motor;
    uint16_t current_servo;
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

    // servos
    int arm_position;

    // init methods
    void initMotors();
    void initServos();
    void initSensors();
    void initLeds();
    void initGPIO(uint8_t p, uint8_t state);

};

#endif