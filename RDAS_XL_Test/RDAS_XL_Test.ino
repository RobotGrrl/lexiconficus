#include <Servo.h>
#include "FastLED.h"

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

Servo servo_a;
Servo servo_b;

// the side nearest to the power plug is A
// the side farthest away from power plug is B

int led = 13;
long current_time = 0;
long last_blink = 0;
boolean blink_on = true;

int motor_speed = 100;

int mid_a = 1500;
int cast_a = 1200;
int drive_a = 1800;

int mid_b = 1500;
int cast_b = 1800;
int drive_b = 1200;


void motor_a(int speedy) {
  digitalWrite(ain_1, HIGH);
  digitalWrite(ain_2, LOW);
  analogWrite(pwm_a, speedy);
}

void motor_b(int speedy) {
  digitalWrite(bin_1, LOW);
  digitalWrite(bin_2, HIGH);
  analogWrite(pwm_b, speedy);
}


void setup() {

  Serial.begin(9600);
  Serial.println("hi");
    
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  pinMode(led, OUTPUT);

  pinMode(pwm_a, OUTPUT);
  pinMode(ain_1, OUTPUT);
  pinMode(ain_2, OUTPUT);

  pinMode(pwm_b, OUTPUT);
  pinMode(bin_1, OUTPUT);
  pinMode(bin_2, OUTPUT);

  motor_a(0);

  // mid: 1500, casting @ 90deg: 1200, driving @ 90deg: 1800
  servo_a.attach(servo_a_pin);

  // mid: 1500, casting @ 90deg: 1800, driving @ 90deg: 1200
  servo_b.attach(servo_b_pin);

  motor_b(0);
  
  


  int del = 5;

  // -- demo 1
  // go from middle to cast, then drive forwards and backwards
  
  servo_a.write(mid_a);
  servo_b.write(mid_b); 

  for(int i=0; i<300; i++) {
    servo_a.write(1500-i);
    servo_b.write(1500+i);
    delay(10);
  }

  

  
 
  // from mid to casting
  for(int i=0; i<300; i++) {
    servo_a.write(1500-i);
    servo_b.write(1500+i);
    delay(del);
  }
  
  // from casting to mid
  for(int i=0; i<300; i++) {
    servo_a.write(1200+i);
    servo_b.write(1800-i);
    delay(del);
  }

  // from mid to driving
  for(int i=0; i<300; i++) {
    servo_a.write(1500+i);
    servo_b.write(1500-i);
    delay(del);
  }

  // from driving to mid
  for(int i=0; i<300; i++) {
    servo_a.write(1800-i);
    servo_b.write(1200+i);
    delay(del);
  }
  


// ----------




  digitalWrite(ain_1, HIGH);
  digitalWrite(ain_2, LOW);
  analogWrite(pwm_a, motor_speed);

  digitalWrite(bin_1, LOW);
  digitalWrite(bin_2, HIGH);
  analogWrite(pwm_b, motor_speed);


}


// This will always work on Teensy, does not depend on buffer size
boolean getNumbersFromSerial() {
  
  int count=0;
  const int cap = 1;
  char buf[cap];
  
  while (count < cap) {
    if (Serial.available()) {  // receive all 11 bytes into "buf"
      buf[count++] = Serial.read();
    }
  }
  
  if (buf[0] == '@') {
    Serial.print("@");
    return true;   // return true if time message received
  }
  
  return false;  //if no message return false
}


void loop() { 
  

  /*
  int maxp = 1800;
  int minp = 1200;

  for(int i=minp; i<maxp; i++) {
    servo_a.write(i);
    servo_b.write(i);
    delay(1);
  }

  for(int i=maxp; i>minp; i--) {
    servo_a.write(i);
    servo_b.write(i);
    delay(1);
  }
  */

  //getNumbersFromSerial();

  current_time = millis();
  digitalWrite(led, (current_time%1000) < 100 || ((current_time%1000) > 200 && (current_time%1000) < 300));// || ((current_time%1000) > 400 && (current_time%1000) < 500));

  float brightness = 0.1;
  CRGB wat  = CRGB( brightness*200, brightness*50, brightness*100);

  if(current_time-last_blink >= 500) {
    Serial.println("sup");
    if(blink_on) {
      for(int i=0; i<NUM_LEDS; i++) {
        leds[i] = wat;
        FastLED.show();
      }    
    } else {
      for(int i=NUM_LEDS-1; i>=0; i--) {
        leds[i] = CRGB::Black;
        FastLED.show();
      }
    }
    blink_on = !blink_on;
    last_blink = current_time;
  }

}



void mleep() {


  Serial.println("hi");

  CRGB wat  = CRGB( 10, 10, 10);

  //servo_a.write(800);
  
  // Turn the LED on, then pause
  for(int i=0; i<NUM_LEDS; i++) {
  leds[i] = wat;
  FastLED.show();
  }
  delay(1000);


  //servo_a.write(2200);
  
  // Now turn the LED off, then pause
  for(int i=NUM_LEDS-1; i>=0; i--) {
  leds[i] = CRGB::Black;
  FastLED.show();
  }
  delay(1000);
  
}

