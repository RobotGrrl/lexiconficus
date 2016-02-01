/*

Teleoperational Headband (v0.2)
for controlling RDAS XL
--------------------------------

Tracks motion with gyro to see if it exceeds a threshold, runs through behaviours

Libraries:
Adafruit_L3GD20
Adafruit_NeoPixel
Adafruit_TiCoServo
Streaming
Promulgate
SoftwareSerial
Wire

Pro Trinket: pins 2 & 7 unavailable
Gyro: x controls left or right, z controls fwd bwd

Neopixel pin: 6

Based on the Headband v0.2 cleaned code

Erin RobotGrrl
Jan. 31, 2016

*/

#include <Wire.h> 
#include <Adafruit_L3GD20.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TiCoServo.h>
#include <Streaming.h>
#include <SoftwareSerial.h>
#include "Promulgate.h"

#define DEBUG true
#define MOTOR_SPEED 100

// pins
#define SERVO_R_PIN 10
#define SERVO_L_PIN 9
#define XBEE_RX 4
#define XBEE_TX 3
#define N_LEDS 7
#define X_PIN A1
#define Y_PIN A2
#define Z_PIN A3
#define GYRO_PIN A5
#define LED_PIN 6
#define PULSE_PIN A0
#define BLINK_PIN 13
#define BUTTON_A_PIN 12
#define BUTTON_B_PIN 11

// servos
#define SERVO_L_MIN 1000 // 1 ms pulse
#define SERVO_L_MAX 2000 // 2 ms pulse
#define SERVO_L_HOME 90
#define SERVO_R_MIN 1000 // 1 ms pulse
#define SERVO_R_MAX 2000 // 2 ms pulse
#define SERVO_R_HOME 70

// cool
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, LED_PIN);
Adafruit_TiCoServo servo_L;
Adafruit_TiCoServo servo_R;
SoftwareSerial xbee(XBEE_RX, XBEE_TX);
Promulgate serial_promulgate = Promulgate(&Serial, &Serial);
Promulgate xbee_promulgate = Promulgate(&xbee, &xbee);
Adafruit_L3GD20 gyro;

// pulse sensor interrupt variables
volatile int BPM;
volatile int Signal;
volatile int IBI = 600;
volatile boolean Pulse = false;
volatile boolean QS = false;

// pusle sensor
int pulsePin = A0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat

// timing
long current_time = 0;
long last_servo_update = 0;
long last_led_update = 0;
long last_gyro_trig = 0;

// motion
float x_home = 0;
float y_home = 0;
float z_home = 0;

float x_acc = 0;
float y_acc = 0;
float z_acc = 0;

int z_val = 0;
int x_val = 0;

// gyro
float gyro_home = 0;
float gyro_zero_voltage = 0;
float max_voltage = 5;

// states
int NONE = 99;
int state = NONE;
int LEFT = 0;
int RIGHT = 1;
int FRONT = 2;
int BACK = 3;
long last_refresh = 0;


void setup() {
  
  pinMode(BLINK_PIN, OUTPUT);
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);
  pinMode(GYRO_PIN, INPUT);
  pinMode(BUTTON_A_PIN, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);
  
  Serial.begin(9600);
  xbee.begin(9600);
  
  initComm();
  
  interruptSetup();

  initWings();
  
  strip.begin();
  
  initMotion();
  
}


void loop() {

  current_time = millis();
    
  updateGyro();
  
  checkGyroBounds();
  
  detectMotionZ();
  
  detectMotionX();

  refreshStates(false);
  
  wingWiggleBehaviour();
    
  heartbeatBehaviour();
    
  checkCommunication(); 

}


void refreshStates(boolean force) {

  if(current_time-last_refresh >= 250 || force == true) {

    if(state == NONE) {
      Serial << "NONE" << endl;
      xbee_promulgate.transmit_action('#', 'L', 1, 0, '!');
      xbee_promulgate.transmit_action('#', 'R', 1, 0, '!');
    } else if(state == LEFT) {
      Serial << "LEFT" << endl;
      xbee_promulgate.transmit_action('#', 'L', 0, 128, '!');
      xbee_promulgate.transmit_action('#', 'R', 1, 255, '!');
    } else if(state == RIGHT) {
      Serial << "RIGHT" << endl;
      xbee_promulgate.transmit_action('#', 'L', 1, 255, '!');
      xbee_promulgate.transmit_action('#', 'R', 0, 128, '!');
    } else if(state == BACK) {
      Serial << "BACK" << endl;
      xbee_promulgate.transmit_action('#', 'L', 0, 200, '!');
      xbee_promulgate.transmit_action('#', 'R', 0, 200, '!');
    } else if(state == FRONT) {
      Serial << "FRONT" << endl;
      xbee_promulgate.transmit_action('#', 'L', 1, 200, '!');
      xbee_promulgate.transmit_action('#', 'R', 1, 200, '!');
    }

    last_refresh = current_time;

  }
  
}


