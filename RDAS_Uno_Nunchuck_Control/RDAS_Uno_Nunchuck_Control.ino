/*
 *  RDAS Nunchuck Control
 *  ---------------------
 *  Arduino UNO
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
 *  Erin RobotGrrl
 *  Jan. 25, 2016
 * 
 */

#include <Wire.h>
#include <ArduinoNunchuk.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 2); // RX, TX

int led = 13;
long current_time = 0;
long last_time = 0;
boolean meepmeep = true;

ArduinoNunchuk nunchuk = ArduinoNunchuk();

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("RDAS Nunchuck Control");
  
  pinMode(led, OUTPUT);
  nunchuk.init();
}

void loop() {

  nunchuk.update();

  Serial.print(nunchuk.analogX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.analogY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelZ, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.zButton, DEC);
  Serial.print(' ');
  Serial.println(nunchuk.cButton, DEC);

  delay(100);

  /*
  current_time = millis();
  nunchuk.update();

  if (mySerial.available()) {
    char c = mySerial.read();
    if(c == 'a') {
      digitalWrite(led, HIGH);
    } else if(c == 'b') {
      digitalWrite(led, LOW);
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

  if(nunchuk.zButton == 1) {
      mySerial.println("z");
      Serial.println("z");
  } else if(nunchuk.zButton == 0) {
    mySerial.println("y");
    Serial.println("y");
  }

  if(nunchuk.cButton == 1) {
      mySerial.println("c");
      Serial.println("c");
  } else if(nunchuk.cButton == 0) {
      mySerial.println("d");
      Serial.println("d");
  }
  */
  
}

