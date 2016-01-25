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

int motor_speed = 255;

int mid_a = 1500;
int cast_a = 1200;
int drive_a = 1800;

int mid_b = 1500;
int cast_b = 1800;
int drive_b = 1200;

boolean FWD = true;
boolean BWD = false;

float brightness = 0.1;
CRGB wat  = CRGB( brightness*200, brightness*50, brightness*100);




void motor_a(boolean dir, int speedy) {
  if(dir == FWD) {
   digitalWrite(ain_1, HIGH);
   digitalWrite(ain_2, LOW); 
  } else if(dir == BWD) {
   digitalWrite(ain_1, LOW);
   digitalWrite(ain_2, HIGH);  
  }
  analogWrite(pwm_a, speedy);
}

void motor_b(boolean dir, int speedy) {
  if(dir == FWD) {
    digitalWrite(bin_1, HIGH);
    digitalWrite(bin_2, LOW); 
  } else if(dir == BWD) {
    digitalWrite(bin_1, LOW);
    digitalWrite(bin_2, HIGH); 
  }
  analogWrite(pwm_b, speedy);
}



void setup() {

  Serial.begin(9600);
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


  // mid: 1500, casting @ 90deg: 1200, driving @ 90deg: 1800
  servo_a.attach(servo_a_pin);
  // mid: 1500, casting @ 90deg: 1800, driving @ 90deg: 1200
  servo_b.attach(servo_b_pin);

  while(1<2) {
    motor_a(BWD, 255);
    motor_b(FWD, 255);
  }

  /*
  int del = 5;
 
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
  */


}


void loop() { 
  
  current_time = millis();
  //digitalWrite(led, (current_time%1000) < 100 || ((current_time%1000) > 200 && (current_time%1000) < 300));// || ((current_time%1000) > 400 && (current_time%1000) < 500));

  /*
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
  */


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



