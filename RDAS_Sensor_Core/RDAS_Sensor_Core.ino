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

SoftwareSerial mySerial(4, 8); // RX, TX
Promulgate promulgate = Promulgate(&mySerial, &mySerial);
Promulgate promulgate_hw = Promulgate(&Serial, &Serial);

int led = 13;
long current_time = 0;
long last_time = 0;
boolean meepmeep = true;
int soil = A0;
int soil_dry = 0;
int soil_hand = 250;
int soil_wet = 500;
long last_sensor_send = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(soil, INPUT);

  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);

  promulgate_hw.LOG_LEVEL = Promulgate::ERROR_;
  promulgate_hw.set_rx_callback(received_action_hw);
  promulgate_hw.set_tx_callback(transmit_complete_hw);
  
}

void loop() {

  current_time = millis();

  if(mySerial.available()) {
    char c = mySerial.read();
    promulgate.organize_message(c);
  }

  if(Serial.available()) {
    char c = Serial.read();
    promulgate_hw.organize_message(c);
  }

  int soil_val = analogRead(soil);
  //Serial << "soil: " << soil_val << endl;

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

  if(cmd == 'T') {
    digitalWrite(led, HIGH);
  }

  if(cmd == 'S') {
    digitalWrite(led, LOW);
  }

  // forwarding the messages
  if(cmd == 'T' || cmd == 'S' || cmd == 'L' || cmd == 'R') {
    promulgate.transmit_action(action, cmd, key, val, delim);
  }
  
}

void transmit_complete_hw() {
  if(DEBUG) Serial << "transmit complete!" << endl;
}


