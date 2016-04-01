/*
 * RDAS XL Motor Core
 * ------------------
 * Teensy-LC
 * 
 * connects to sensor core on Serial3
 * 
 * Erin RobotGrrl
 * Jan. 25, 2016
 * 
 */

#include <Streaming.h>
#include <Servo.h>
#include "FastLED.h"
#include "Promulgate.h"

boolean DEBUG = false;

Promulgate promulgate = Promulgate(&Serial3, &Serial3);

#define NUM_LEDS 8
#define DATA_PIN 17
CRGB leds[NUM_LEDS];

int pwm_b = 3;
int bin_1 = 5;
int bin_2 = 6;
int servo_b_pin = 23;

int pwm_a = 4;
int ain_1 = 11;
int ain_2 = 12;
int servo_a_pin = 22;

int servo_linka_pin = 16;
int servo_linkb_pin = 17;
int servo_claw_pin = 20;

Servo servo_a;
Servo servo_b;
Servo link_a;
Servo link_b;
Servo claw;

// the side nearest to the power plug is A
// the side farthest away from power plug is B

int led = 13;
long current_time = 0;
long last_blink = 0;
boolean blink_on = true;

int motor_speed = 255;

int mid_a = 1500;
int cast_a = 1200;
int drive_a = 1800;
int prev_tilt_pos_a = mid_a;

int mid_b = 1500;
int cast_b = 1800;
int drive_b = 1200;
int prev_tilt_pos_b = mid_b;

boolean FWD = true;
boolean BWD = false;

float brightness = 0.1;
CRGB wat  = CRGB( brightness*200, brightness*50, brightness*100);

int claw_open = 180;
int claw_closed = 0;
boolean claw_state = true; // open by default

int link_b_max = 120;
int link_b_min = 180;
int link_b_level = 140;


void motor_a(boolean dir, int speedy) {
  if(dir == BWD) {
   digitalWrite(ain_1, HIGH);
   digitalWrite(ain_2, LOW); 
  } else if(dir == FWD) {
   digitalWrite(ain_1, LOW);
   digitalWrite(ain_2, HIGH);  
  }
  analogWrite(pwm_a, speedy);
}

void motor_b(boolean dir, int speedy) {
  if(dir == BWD) {
    digitalWrite(bin_1, HIGH);
    digitalWrite(bin_2, LOW); 
  } else if(dir == FWD) {
    digitalWrite(bin_1, LOW);
    digitalWrite(bin_2, HIGH); 
  }
  analogWrite(pwm_b, speedy);
}

void transmit_complete();
void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim);

void openClaw();
void closeClaw();

void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);
  Serial.println("hi");
    
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(pwm_a, OUTPUT);
  pinMode(ain_1, OUTPUT);
  pinMode(ain_2, OUTPUT);

  pinMode(pwm_b, OUTPUT);
  pinMode(bin_1, OUTPUT);
  pinMode(bin_2, OUTPUT);

  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);
  
  // mid: 1500, casting @ 90deg: 1200, driving @ 90deg: 1800
  servo_a.attach(servo_a_pin);
  // mid: 1500, casting @ 90deg: 1800, driving @ 90deg: 1200
  servo_b.attach(servo_b_pin);

  link_a.attach(servo_linka_pin);
  link_b.attach(servo_linkb_pin);
  claw.attach(servo_claw_pin);

  claw.write(claw_open);
  delay(100);

  for(int i=0; i<3; i++) {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
  }

  /*
  while(1<3) {
    motor_a(FWD, 255);
    motor_b(FWD, 255);
  }
  */

  for(int i=NUM_LEDS-1; i>=0; i--) {
        leds[i] = CRGB::Yellow; 
    }
    FastLED.show();

  delay(100);

    for(int i=NUM_LEDS-1; i>=0; i--) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
 
}


void loop() { 

  openClaw();
  delay(100);
  closeClaw();
  delay(100);

  link_b.write(link_b_min);
  delay(1000);  
  link_b.write(link_b_level);
  delay(1000);
  link_b.write(link_b_max);
  delay(1000);
  
  current_time = millis();
  //digitalWrite(led, (current_time%1000) < 100 || ((current_time%1000) > 200 && (current_time%1000) < 300));// || ((current_time%1000) > 400 && (current_time%1000) < 500));

  if(Serial3.available()) {
    char c = Serial3.read();
    promulgate.organize_message(c);
  }

}

void openClaw() {
  if(!claw_state) {
    for(int i=claw_closed; i<claw_open; i++) {
      claw.write(i);
      delay(10);
    }
    claw_state = !claw_state;
  }
}

void closeClaw() {
  if(claw_state) {
    for(int i=claw_open; i>claw_closed; i--) {
      claw.write(i);
      delay(10);
    }
    claw_state = !claw_state;
  }
}


void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim) {

  /* API
   * ----
   * L = left motor
   * R = right motor
   * S = arm
   * T = tilt / center of gravity
   * Z = soil sample leds
   * C = claw
   */
  
  if(DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "delim: " << delim << endl;
  }

  if(cmd == 'L') { // left motor
    if(key == 1) { // fwd
      motor_a(FWD, val);
    } else if(key == 0) { // bwd
      motor_a(BWD, val);
    }
  }

  if(cmd == 'R') { // right motor
    if(key == 1) { // fwd
      motor_b(FWD, val);
    } else if(key == 0) { // bwd
      motor_b(BWD, val);
    }
  }

  if(cmd == 'S') { // arm (data from 0-45)

    // for the soil arm
    int servo_pos = val;
    link_a.write(servo_pos);

    // for the claw arm
    int claw_arm_pos = (int)map(val, 0, 45, link_b_min, link_b_max);
    link_b.write(claw_arm_pos);
    
  }

  if(cmd == 'C') { // claw
    digitalWrite(led, HIGH);
    if(claw_state) {
      closeClaw();
    } else {
      openClaw();
    }
    digitalWrite(led, LOW);
  }

  if(cmd == 'T') { // tilt mode

    digitalWrite(led, HIGH);

    int tilt_pos_a = (int)map(val, 0, 45, drive_a, cast_a);
    int tilt_pos_b = (int)map(val, 0, 45, drive_b, cast_b);

    if(val == 999) {
      tilt_pos_a = mid_a;
      tilt_pos_b = mid_b;
    }

    if(tilt_pos_a > prev_tilt_pos_a) {
      for(int i=prev_tilt_pos_a; i<tilt_pos_a; i++) {
        servo_a.write(i);
        delay(20);
      }
    } else if(tilt_pos_a < prev_tilt_pos_a) {
      for(int i=prev_tilt_pos_a; i>tilt_pos_a; i--) {
        servo_a.write(i);
        delay(20);
      }
    }

    if(tilt_pos_b > prev_tilt_pos_b) {
      for(int i=prev_tilt_pos_b; i<tilt_pos_b; i++) {
        servo_b.write(i);
        delay(20);
      }
    } else if(tilt_pos_b < prev_tilt_pos_b) {
      for(int i=prev_tilt_pos_b; i>tilt_pos_b; i--) {
        servo_b.write(i);
        delay(20);
      }
    }

    prev_tilt_pos_a = tilt_pos_a;
    prev_tilt_pos_b = tilt_pos_b;

    /*
    // no longer doing it this way
    int tilt_mode = val;

    if(tilt_mode%3 == 0) {

      servo_a.write(mid_a);
      servo_b.write(mid_b); 
    
      for(int i=0; i<300; i++) {
        servo_a.write(mid_a-i);
        servo_b.write(mid_b+i);
        delay(10);
      }
    
      servo_a.write(cast_a);
      servo_b.write(cast_b); 
      
    } else if(tilt_mode%3 == 1) {

      servo_a.write(cast_a);
      servo_b.write(cast_b);
    
      for(int i=0; i<300; i++) {
        servo_a.write(cast_a+i);
        servo_b.write(cast_b-i);
        delay(10);
      }

      servo_a.write(mid_a);
      servo_b.write(mid_b);
      
    } else if(tilt_mode%3 == 2) {

      servo_a.write(mid_a);
      servo_b.write(mid_b);
    
      for(int i=0; i<300; i++) {
        servo_a.write(mid_a+i); // drive
        servo_b.write(mid_b-i); // drive
        delay(10);
      }
    
      servo_a.write(drive_a);
      servo_b.write(drive_b);
      
    }
    */

    digitalWrite(led, LOW);
    
  }

  if(cmd == 'Z') { // soil sample

    if(key == 0) {
      for(int i=NUM_LEDS-1; i>=0; i--) {
        CRGB wat  = CRGB( brightness*255, brightness*0, brightness*0);
        leds[i] = wat; 
      }
    } else if(key == 1) {
      for(int i=NUM_LEDS-1; i>=0; i--) {
        CRGB wat  = CRGB( brightness*0, brightness*255, brightness*0);
        leds[i] = wat; 
      }
    } else if(key == 2) {
      for(int i=NUM_LEDS-1; i>=0; i--) {
        CRGB wat  = CRGB( brightness*0, brightness*0, brightness*255);
        leds[i] = wat; 
      }
    }
    FastLED.show();
    
  }
  
}

void transmit_complete() {
  
}

























void allDemos() {

  // a bit of a delay so that i can press the switch to start the motors
  for(int j=0; j<25; j++) {
  
    for(int i=NUM_LEDS-1; i>=0; i--) {
      if(i%2 == 0) {
       leds[i] = CRGB::Black; 
      } else {
       leds[i] = wat;
      }
    }
    FastLED.show();
    delay(100);

    for(int i=NUM_LEDS-1; i>=0; i--) {
      if(i%2 == 1) {
       leds[i] = CRGB::Black; 
      } else {
       leds[i] = wat;
      }
    }
    FastLED.show();
    delay(100);

  }


  // -- demo 1
  // go from middle to cast, then drive forwards and backwards

  for(int i=NUM_LEDS-1; i>=0; i--) {
    leds[i] = CRGB::Black;
  }
  leds[3] = wat;
  leds[4] = wat;
  FastLED.show();
  
  servo_a.write(mid_a);
  servo_b.write(mid_b); 

  for(int i=0; i<300; i++) {
    servo_a.write(mid_a-i);
    servo_b.write(mid_b+i);
    delay(10);
  }

  servo_a.write(cast_a);
  servo_b.write(cast_b); 

  motor_a(FWD, 100);
  motor_b(FWD, 100);
  delay(1000);

  motor_a(BWD, 100);
  motor_b(BWD, 100);
  delay(1000);

  motor_a(FWD, 0);
  motor_b(FWD, 0);
  

  // -- demo 2
  // go from cast to middle, then perform a turning maneouver

  for(int i=NUM_LEDS-1; i>=0; i--) {
    leds[i] = CRGB::Black;
  }
  leds[2] = wat;
  leds[5] = wat;
  FastLED.show();

  servo_a.write(cast_a);
  servo_b.write(cast_b);

  for(int i=0; i<300; i++) {
    servo_a.write(cast_a+i);
    servo_b.write(cast_b-i);
    delay(10);
  }

  servo_a.write(mid_a);
  servo_b.write(mid_b);

  motor_a(FWD, 100);
  motor_b(BWD, 180);
  delay(1000);

  motor_a(FWD, 0);
  motor_b(BWD, 0);
  delay(500);

  motor_a(BWD, 180);
  motor_b(FWD, 100);
  delay(1000);

  motor_a(FWD, 0);
  motor_b(FWD, 0);


  // -- demo 3
  // go from middle to drive, then accelerate forwards

  for(int i=NUM_LEDS-1; i>=0; i--) {
    leds[i] = CRGB::Black;
  }
  leds[1] = wat;
  leds[6] = wat;
  FastLED.show();

  servo_a.write(mid_a);
  servo_b.write(mid_b);

  for(int i=0; i<300; i++) {
    servo_a.write(mid_a+i); // drive
    servo_b.write(mid_b-i); // drive
    delay(10);
  }

  servo_a.write(drive_a);
  servo_b.write(drive_b);

  for(int i=50; i<180; i++) {
    motor_a(FWD, i);
    motor_b(FWD, i);
    delay(15);
  }

  delay(1000);

  for(int i=180; i>50; i--) {
    motor_a(FWD, i);
    motor_b(FWD, i);
    delay(15);
  }

  motor_a(FWD, 0);
  motor_b(FWD, 0);


  // -- demo 4
  // go back to middle, then alternate both positions

  for(int i=NUM_LEDS-1; i>=0; i--) {
    leds[i] = CRGB::Black;
  }
  leds[0] = wat;
  leds[7] = wat;
  FastLED.show();
  
  servo_a.write(drive_a);
  servo_b.write(drive_b);

  for(int i=0; i<300; i++) {
    servo_a.write(mid_a-i); // cast
    servo_b.write(mid_b-i); // drive
    delay(10);
  }

  for(int i=0; i<300; i++) {
    servo_a.write(cast_a+i); // mid
    servo_b.write(drive_b+i); // mid
    delay(10);
  }

  for(int i=0; i<300; i++) {
    servo_a.write(mid_a+i); // drive
    servo_b.write(mid_b+i); // cast
    delay(10);
  }

  motor_a(FWD, 100);
  motor_b(FWD, 0);
  delay(1500);

  motor_a(FWD, 0);
  motor_b(FWD, 0);

  for(int i=0; i<300; i++) {
    servo_a.write(cast_a-i); // mid
    servo_b.write(drive_b-i); // mid
    delay(10);
  }

  for(int i=0; i<300; i++) {
    servo_a.write(mid_a-i); // cast
    servo_b.write(mid_b-i); // drive
    delay(10);
  }

  motor_a(FWD, 0);
  motor_b(FWD, 100);
  delay(1500);

  motor_a(FWD, 0);
  motor_b(FWD, 0);

  for(int i=0; i<300; i++) {
    servo_a.write(cast_a+i); // mid
    servo_b.write(drive_b+i); // mid
    delay(10);
  }

  servo_a.write(mid_a);
  servo_b.write(mid_b);
  

  // -- demo 5
  // raise one of the wheels up and move

  for(int i=NUM_LEDS-1; i>=0; i--) {
    leds[i] = CRGB::Black;
  }
  leds[0] = wat;
  leds[2] = wat;
  leds[4] = wat;
  leds[6] = wat;
  FastLED.show();

  servo_a.write(mid_a);
  servo_b.write(mid_b);
  
  for(int i=0; i<300; i++) {
    servo_a.write(mid_a+i); // drive
    delay(10);
  }

  motor_a(FWD, 100);
  motor_b(FWD, 100);
  delay(1500);

  motor_a(FWD, 0);
  motor_b(FWD, 0);

  for(int i=0; i<300; i++) {
    servo_a.write(drive_a-i); // mid
    delay(10);
  }

  servo_a.write(mid_a);
  servo_b.write(mid_b);


  
}


