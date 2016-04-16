/*
 *  RDAS Sensor Core
 *  ----------------
 *  Adafruit Pro Trinket
 * 
 *  xbee to hw uart (has wire jumpers)
 *  teensy-lc tx to 4
 *  teensy-lc rx to 8
 *  sonar sensor A0
 * 
 *  Erin RobotGrrl
 *  Jan. 25, 2016
 * 
 */

#include <Streaming.h>
#include <SoftwareSerial.h>
#include "Promulgate.h"

boolean DEBUG = false;
boolean MYO_MODE = false;

SoftwareSerial mySerial(4, 8); // RX, TX
Promulgate promulgate = Promulgate(&mySerial, &mySerial);
Promulgate promulgate_hw = Promulgate(&Serial, &Serial);

int led = 13;
int big_led = 3;
long current_time = 0;
long last_time = 0;
boolean meepmeep = true;
int soil = A0;
int soil_dry = 0;
int soil_hand = 250;
int soil_wet = 500;
long last_sensor_send = 0;
long last_print = 0;
long last_fwd_msg = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(big_led, OUTPUT);
  pinMode(soil, INPUT);

  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);

  promulgate_hw.LOG_LEVEL = Promulgate::ERROR_;
  promulgate_hw.set_rx_callback(received_action_hw);
  promulgate_hw.set_tx_callback(transmit_complete_hw);

  digitalWrite(big_led, HIGH);
  delay(1000);
  digitalWrite(big_led, LOW);
  
}

void loop() {

  current_time = millis();

  if(mySerial.available()) {
    char c = mySerial.read();
    promulgate.organize_message(c);
  }

  if(Serial.available()) {
    char c = Serial.read();

    if(!MYO_MODE) {
      promulgate_hw.organize_message(c);  
    } else {
      if(c == 'B') { // wave left
        promulgate.transmit_action('#', 'L', 1, 255, '!');
        promulgate.transmit_action('#', 'R', 1, 255, '!');
      } else if(c == 'C') { // wave right
        promulgate.transmit_action('#', 'L', 0, 255, '!');
        promulgate.transmit_action('#', 'R', 0, 255, '!');
      } else if(c == 'A') { // fist
        promulgate.transmit_action('#', 'L', 1, 0, '!');
        promulgate.transmit_action('#', 'R', 1, 0, '!');
      }
    }
    
  }

  int soil_val = analogRead(soil);
  
  if(current_time-last_print >= 100) {
    //promulgate_hw.transmit_action('#', 'G', 0, soil_val, '!');
    Serial << "Soil, " << soil_val << ", !" << endl;
    last_print = current_time;
  }

  if(current_time-last_sensor_send >= 1000) {
      
    if(soil_val < soil_hand) { // dry
      promulgate.transmit_action('#', 'Z', 0, soil_val, '!');
    } else if(soil_val < soil_wet) { // medium
      promulgate.transmit_action('#', 'Z', 1, soil_val, '!');
    } else { // wet
      promulgate.transmit_action('#', 'Z', 2, soil_val, '!');
    }
    last_sensor_send = current_time;

  }

  if(current_time-last_fwd_msg >= 1000) {
    digitalWrite(big_led, LOW);    
  } else {
    digitalWrite(big_led, (current_time%1000) < 100 || ((current_time%1000) > 200 && (current_time%1000) < 300));
  }
  
}

void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim) {
  
  if(DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "delim: " << delim << endl;
  }
  
}

void transmit_complete() {
  if(DEBUG) Serial << "transmit complete!" << endl;
}


void received_action_hw(char action, char cmd, uint8_t key, uint16_t val, char delim) {
  
  if(DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "delim: " << delim << endl;
  }

  // forwarding the messages
  if(cmd == 'T' || cmd == 'S' || cmd == 'L' || cmd == 'R' || cmd == 'C') {
    promulgate.transmit_action(action, cmd, key, val, delim);
    last_fwd_msg = current_time;
  }
  
}

void transmit_complete_hw() {
  if(DEBUG) Serial << "transmit complete!" << endl;
}


