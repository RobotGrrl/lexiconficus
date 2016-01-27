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
#include <Streaming.h>

SoftwareSerial mySerial(3, 2); // RX, TX

int led = 13;
long current_time = 0;
long last_time = 0;
boolean meepmeep = true;

ArduinoNunchuk nunchuk = ArduinoNunchuk();

int max_x = 228;
int min_x = 34;
int max_y = 223;
int min_y = 34;
int home_x = 125;
int home_y = 135;

int tilt_mode = 0;
int max_tilt_modes = 5;
long last_c = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println("RDAS Nunchuck Control");
  
  pinMode(led, OUTPUT);
  nunchuk.init();
}

void loop() {

  nunchuk.update();
  current_time = millis();

  /*  
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
  */

  if(nunchuk.zButton == 0 && nunchuk.cButton == 0) { // drive

    int motor_speed = 0;
    boolean motor_dir = true;

    if(nunchuk.analogY > max_y) nunchuk.analogY = max_y;
    if(nunchuk.analogY < min_y) nunchuk.analogY = min_y;
    if(nunchuk.analogX > max_x) nunchuk.analogX = max_x;
    if(nunchuk.analogX < min_x) nunchuk.analogX = min_x;

    if(nunchuk.analogY >= home_y) { // fwd
      motor_speed = map(nunchuk.analogY, home_y, max_y, 0, 255);
      motor_dir = true;
    } else { // bwd
      motor_speed = map(nunchuk.analogY, min_y, home_y, 255, 0);
      motor_dir = false;
    }

    float percent_r = (float)map(nunchuk.analogX, min_x, max_x, 0, 100);
    percent_r /= 100.0;
    float percent_l = 1.0-percent_r;

    int speed_r = (int)((float)motor_speed * percent_r);
    int speed_l = (int)((float)motor_speed * percent_l);

    // TODO: send speeds here

    if(motor_dir) {
      Serial << "FWD- ";
    } else {
      Serial << "BWD- ";
    }
    //Serial << "speed L: " << speed_l << " R: " << speed_r << endl;
    Serial << "motor_speed: " << motor_speed << endl;
    
  } else if(nunchuk.zButton == 1 && nunchuk.cButton == 0) { // arm

    int servo_pos = map(nunchuk.analogY, min_y, max_y, 0, 180);
    // TODO: send the servo pos here

    Serial << "arm pos: " << servo_pos << endl;
    
  } else if(nunchuk.zButton == 0 && nunchuk.cButton == 1) { // tilt

    if(current_time-last_c >= 5000) { // cycle through tilt modes
      tilt_mode++;
      if(tilt_mode > max_tilt_modes) tilt_mode = 0;
      // TODO: send the tilt mode here
      Serial << "tilt mode: " << tilt_mode << endl;
      last_c = current_time;
    }
    
  }

  delay(20);

  

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

