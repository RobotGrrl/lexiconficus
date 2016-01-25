/*
 *  RDAS Sensor Core
 *  ----------------
 *  Adafruit Pro Trinket
 * 
 *  xbee to hw uart (has wire jumpers)
 *  teensy-lc tx to 4
 *  teensy-lc rx to 8
 * 
 *  Erin RobotGrrl
 *  Jan. 25, 2016
 * 
 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 8); // RX, TX

int led = 13;
long current_time = 0;
long last_time = 0;
boolean meepmeep = true;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(led, OUTPUT);
}

void loop() {

  current_time = millis();
  
  if (mySerial.available()) {
    char c = mySerial.read();
    if(c == 'a') {
      digitalWrite(led, HIGH);
    } else if(c == 'b') {
      digitalWrite(led, LOW);
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    if(c == 'a') {
      digitalWrite(13, HIGH);
    } else if(c == 'b') {
      digitalWrite(13, LOW);
    } else if(c == 'z') {
      mySerial.println("m");
    } else if(c == 'y') {
      mySerial.println("n");
    } else if(c == 'c') {
      mySerial.println("s");
    } else if(c == 'd') {
      mySerial.println("t");
    }
  }

  if(current_time-last_time >= 1000) {
    if(meepmeep) {
      mySerial.println("a");
      Serial.println("a");
    } else {
      mySerial.println("b");
      Serial.println("b");
    }
    meepmeep = !meepmeep;
    last_time = current_time;
  }
  
}



