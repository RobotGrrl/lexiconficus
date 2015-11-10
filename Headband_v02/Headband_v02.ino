

/*

pro trinket pins 2 & 7 unavailable



*/

#include <Wire.h> 
#include <Adafruit_L3GD20.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TiCoServo.h>
#include <Streaming.h>
#include <SoftwareSerial.h>
#include "Promulgate.h"

#define DEBUG true

long last_transmit = 0;



int pulsePin = A0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat

// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, must be seeded! 
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

// NeoPixel parameters. These are configurable, but the pin number must
// be different than the servo(s).
#define N_LEDS       10
#define LED_PIN       6

Adafruit_L3GD20 gyro;

#define SERVO_L_PIN    9
#define SERVO_L_MIN 1000 // 1 ms pulse
#define SERVO_L_MAX 2000 // 2 ms pulse
#define SERVO_L_HOME 90

#define SERVO_R_PIN    10
#define SERVO_R_MIN 1000 // 1 ms pulse
#define SERVO_R_MAX 2000 // 2 ms pulse
#define SERVO_R_HOME 70

Adafruit_NeoPixel  strip = Adafruit_NeoPixel(N_LEDS, LED_PIN);
Adafruit_TiCoServo servo_L;
Adafruit_TiCoServo servo_R;

#define X_PIN A1
#define Y_PIN A2
#define Z_PIN A3

#define GYRO_PIN A5

int button_front = 12;
int button_back = 11;

#define BUTTON_3 8
#define BUTTON_2 7
#define BUTTON_1 5
#define LDR A4

#define XBEE_RX 4
#define XBEE_TX 3

SoftwareSerial xbee(XBEE_RX, XBEE_TX);

Promulgate serial_promulgate = Promulgate(&Serial, &Serial);
Promulgate xbee_promulgate = Promulgate(&xbee, &xbee);

float xval = 0;
float yval = 0;
float zval = 0;

float xcount = 0;
float ycount = 0;
float zcount = 0;

float xavg = 0;
float yavg = 0;
float zavg = 0;

boolean motion_track = false;

uint16_t xthresh = 120;
uint16_t ythresh = 50;
uint16_t zthresh = 30;

long last_trigger_x = 0;
long last_trigger_y = 0;
long last_trigger_z = 0;


uint8_t window = 50;
uint8_t count = 0;

long current_time = 0;
long last_servo_update = 0;
long last_send = 0;

boolean pos_dir = true;
int pos = 0;


//The minimum and maximum values that came from
//the accelerometer while standing still
//You very well may need to change these
int minVal = 265;
int maxVal = 402;

//to hold the caculated values
double x;
double y;
double z;

long last_button_front = 0;


float x_home = 0;
float y_home = 0;
float z_home = 0;

float x_acc = 0;
float y_acc = 0;
float z_acc = 0;

float acc_scale = 1.0;
float acc_pitch = 0;
float acc_roll = 0;
float acc_yaw = 0;

float gyro_x_home = 0;
float gyro_y_home = 0;
float gyro_z_home = 0;

float gyro_home = 0;
float gyro_scale = 0.01;
float gyro_rate = 0;
float gyro_x_rate = 0;
float gyro_y_rate = 0;
float gyro_z_rate = 0;
float gyro_voltage = 0;
float gyro_zero_voltage = 0;
float max_voltage = 5;
float rotation_thresh = 3;
long last_measure_time = 0;

float pitch = 0;
float roll = 0;
float yaw = 0;
float gyro_pitch = 0;
float gyro_roll = 0;
float gyro_yaw = 0;
float alpha = 0.8;

int max_range = 20;
int min_speed = 0;
int max_speed = 255;

int speed_val = 0;
float pitch_start = 0;

long max_control_time = 10000;
long button3_press_time = 0;
long button2_press_time = 0;
long button1_press_time = 0;
boolean headband_speed_control = false;
boolean ears_wiggle_mode = false;
boolean rainbow_mode = false;



int z_val = 0;
int x_val = 0;
int movement_num = 0;
long last_gyro_trig = 0;
int headband_mode = 0;
long last_button_back = 0;
long last_led_update = 0;

int MOTOR_SPEED = 100;


void setup() {
  
  pinMode(blinkPin, OUTPUT);
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);
  pinMode(GYRO_PIN, INPUT);
  pinMode(BUTTON_3, INPUT);
  pinMode(BUTTON_2, INPUT);
  pinMode(BUTTON_1, INPUT);
  pinMode(LDR, INPUT);
  pinMode(button_front, INPUT);
  pinMode(button_back, INPUT);
  
  Serial.begin(9600);
  xbee.begin(9600);
  
  xbee_promulgate.LOG_LEVEL = Promulgate::ERROR_;
  xbee_promulgate.set_rx_callback(received_action);
  xbee_promulgate.set_tx_callback(transmit_complete);

  serial_promulgate.LOG_LEVEL = Promulgate::ERROR_;
  serial_promulgate.set_rx_callback(received_action);
  serial_promulgate.set_tx_callback(transmit_complete);
  
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 

  servo_L.attach(SERVO_L_PIN, SERVO_L_MIN, SERVO_L_MAX);
  servo_R.attach(SERVO_R_PIN, SERVO_R_MIN, SERVO_R_MAX);
  
  servo_L.write(SERVO_L_HOME);
  servo_R.write(SERVO_R_HOME);
  
  strip.begin();
  
  x_home = analogRead(X_PIN);
  y_home = analogRead(Y_PIN);
  z_home = analogRead(Z_PIN);  
  
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  
  gyro_home = analogRead(GYRO_PIN);
  gyro_zero_voltage = ( gyro_home * max_voltage ) / 1023;
  
}

void loop() {

  current_time = millis();
  
  /*
  
  if(current_time < 2000) {
     headband_mode = 0;
  }
  
  if(digitalRead(button_front) == HIGH && current_time-last_button_front > 1000) {
    if(headband_mode == 1) {
      headband_mode = 0;
    } else {
      headband_mode = 1; 
    }
    last_button_front = current_time;
  } else if(digitalRead(button_back) == HIGH && current_time-last_button_back > 1000) {
    if(headband_mode == 2) {
      headband_mode = 0;
    } else {
      headband_mode = 2; 
    }
    last_button_back = current_time;
  }
  
  */
  
    
  gyro.read();
  
  Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print(" ");
  
  // x controls left or right
  // z controls fwd bwd
  
  if(gyro.data.z < -100) z_val = -100;
  if(gyro.data.z > 100) z_val = 100;
  
  if(gyro.data.x < -100) x_val = -100;
  if(gyro.data.x > 100) x_val = 100;
  
  if(gyro.data.z > 40) {
    //if(movement_num == 0 || movement_num == 1 || movement_num == 3 || movement_num == 4) {
    //  movement_num = 1;
      Serial << "forwards!" << endl;
      
      xbee_promulgate.transmit_action('#', 'F', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME-90);
      servo_R.write(SERVO_R_HOME+90);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 100, 255, 10);
      }
      strip.show();
      
      last_gyro_trig = current_time;
      
    //}
  } else if(gyro.data.z < -40) {
    //if(movement_num == 0 || movement_num == 2 || movement_num == 3 || movement_num == 4) {
      Serial << "backwards!" << endl;
      
      xbee_promulgate.transmit_action('#', 'B', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME+60);
      servo_R.write(SERVO_R_HOME-60);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 10, 50, 200);
      }
      strip.show();
      
      last_gyro_trig = current_time;
      
    //  movement_num = 2;
    //}
  }
  
  
  if(gyro.data.x > 40) {
    //if(movement_num == 0 || movement_num == 1 || movement_num == 2 || movement_num == 3) {
      Serial << "left!" << endl;
      
      xbee_promulgate.transmit_action('#', 'L', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME+60);
      servo_R.write(SERVO_R_HOME+60);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 200, 200, 200);
      }
      strip.setPixelColor(1, 10, 255, 100);
      strip.setPixelColor(2, 10, 255, 100);
      strip.setPixelColor(3, 10, 255, 100);
      strip.show();
      
      last_gyro_trig = current_time;
      
    //  movement_num = 3;
    //}
  } else if(gyro.data.x < -40) {
    //if(movement_num == 0 || movement_num == 1 || movement_num == 2 || movement_num == 4) {
      Serial << "right!" << endl;
      
      xbee_promulgate.transmit_action('#', 'R', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME-60);
      servo_R.write(SERVO_R_HOME-60);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 200, 200, 200);
      }
      strip.setPixelColor(3, 10, 255, 100);
      strip.setPixelColor(4, 10, 255, 100);
      strip.setPixelColor(5, 10, 255, 100);
      strip.show();
      
      last_gyro_trig = current_time;
      
    //  movement_num = 4;
    //}
  }
  
   
   
  if(current_time-last_gyro_trig > 3000) {
    
    
    
    
    
    
    
    if(current_time-last_servo_update >= 7000) {
     
    servo_L.attach(SERVO_L_PIN);
    servo_R.attach(SERVO_R_PIN);
    
    int r = (int)random(0, 2);
    
    if(r == 0) {
    
      for(int i=0; i<30; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);
        delay(10);
      }
      
      for(int i=30; i>-30; i--) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      for(int i=-30; i<0; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
    
    } else if(r == 1) {
      
      for(int i=0; i<30; i++) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);
        delay(10);
      }
      
      for(int i=30; i>-30; i--) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      for(int i=-30; i<0; i++) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
    }
    
    servo_L.detach();
    servo_R.detach();
      
    last_servo_update = current_time;  
    
    }
    
    
    
    if(QS == true) {                       // Quantified Self flag is true when arduino finds a heartbeat
      
      Serial << "BPM: " << BPM << " IBI: " << IBI << endl;
    
      //xbee_promulgate.transmit_action('#', 'F', 200, 100, '!');
      
      if(current_time-last_servo_update >= 1000) {
      strip.clear();
        for(int i=0; i<N_LEDS; i++) {
        strip.setPixelColor(i, 10, 100, 10);
      }
      strip.show();
      delay(20);
      }
         
      QS = false;                      // reset the Quantified Self flag for next time    
    
    } else {
      
     if(current_time-last_led_update >= 500) {
       strip.clear();
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, (int)(random(0,255)/10), (int)(random(0,255)/10), (int)(random(0,255)/10));
    }
    strip.show();
    delay(20);
    last_led_update = current_time;
    }
      
    }
    
    
    
    
  }
    
    
    
    
    
    
    /*
  } else if(headband_mode == 2) {
    
    strip.clear();

    if(QS == true) {                       // Quantified Self flag is true when arduino finds a heartbeat
      
      Serial << "BPM: " << BPM << " IBI: " << IBI << endl;
    
      xbee_promulgate.transmit_action('#', 'F', 150, 100, '!');
      
      if(current_time-last_servo_update >= 1000) {
      for(int i=0; i<N_LEDS; i++) {
        strip.setPixelColor(i, 10, 0, 0);
      }
      strip.show();
      delay(20);
      }
         
      QS = false;                      // reset the Quantified Self flag for next time    
    
    } else {//if(headband_speed_control != true) {
    
      for(int i=0; i<N_LEDS; i++) {
        strip.setPixelColor(i, 0, 10, 0);
       }
       strip.show();
      
    }
    
  }
  
  
 */
  
  
  
  
  
  /*
  if(digitalRead(button_front) == HIGH && current_time-last_button_front > 7000) {
    
    // set home positions
    x_home = analogRead(X_PIN);
    y_home = analogRead(Y_PIN);
    z_home = analogRead(Z_PIN);
    
    gyro.read();
    gyro_x_home = gyro.data.x;
    gyro_y_home = gyro.data.y;
    gyro_z_home = gyro.data.z;
    
    motion_track = true;
    last_button_front = current_time;
    
  }
   
   
  if(motion_track) {
    
    // retrieve values
    xval = 5*(analogRead(X_PIN)/1023);
    yval = 5*(analogRead(Y_PIN)/1023);
    zval = 5*(analogRead(Z_PIN)/1023);
  
    x_acc = (xval-x_home)/acc_scale;
    y_acc = (yval-y_home)/acc_scale;
    z_acc = (zval-z_home)/acc_scale;
    
    gyro.read();
    gyro_x_rate = gyro.data.x / gyro_scale;
    gyro_y_rate = gyro.data.y / gyro_scale;
    gyro_z_rate = gyro.data.z / gyro_scale;
    
    acc_pitch = atan(x_acc / sqrt( pow(y_acc,2) + pow(z_acc,2) ) );
    acc_roll = atan(y_acc / sqrt( pow(x_acc,2) + pow(z_acc,2) ) );
    acc_yaw = atan(z_acc / sqrt( pow(x_acc,2) + pow(y_acc,2) ) );
    
    acc_pitch = acc_pitch * (180/PI);
    acc_roll = acc_roll * (180/PI);
    acc_yaw = acc_yaw * (180/PI);
    
    if(gyro_x_rate > rotation_thresh || gyro_x_rate < -rotation_thresh) {
      gyro_pitch = gyro_pitch + gyro_x_rate * (current_time-last_measure_time)/1000.0;
      last_measure_time = current_time;
    }
    
    if(gyro_y_rate > rotation_thresh || gyro_y_rate < -rotation_thresh) {
      gyro_roll = gyro_roll + gyro_y_rate * (current_time-last_measure_time)/1000.0;
      last_measure_time = current_time;
    }
    
    if(gyro_z_rate > rotation_thresh || gyro_z_rate < -rotation_thresh) {
      gyro_yaw = gyro_yaw + gyro_z_rate * (current_time-last_measure_time)/1000.0;
      last_measure_time = current_time;
    }
    
    if(gyro_pitch > 360) gyro_pitch -= 360;
    if(gyro_pitch < -360) gyro_pitch += 360;
    
    if(gyro_roll > 360) gyro_roll -= 360;
    if(gyro_roll < -360) gyro_roll += 360;
    
    if(gyro_yaw > 360) gyro_yaw -= 360;
    if(gyro_yaw < -360) gyro_yaw += 360;
    
    pitch = alpha*gyro_pitch + (1-alpha)*acc_pitch;
    roll = alpha*gyro_roll + (1-alpha)*acc_roll;
    yaw = alpha*gyro_yaw + (1-alpha)*acc_yaw;
    
    Serial << "Pitch: " << pitch << " roll: " << roll << " yaw: " << yaw << endl;
    delay(100);
    
  }
  */
  
  
  
   
   /*
   // ramp up forwards
   for(int i=100; i>50; i-=5) {
     xbee_promulgate.transmit_action('#', 'P', i, i, '!');
     delay(100);
   }
   
   xbee_promulgate.transmit_action('#', 'S', 1, 1, '!');
   
   delay(2000);
   
   // ramp up backwards
   for(int i=45; i>0; i-=5) {
     xbee_promulgate.transmit_action('#', 'P', i, i, '!');
     delay(100);
   }
   
   xbee_promulgate.transmit_action('#', 'S', 1, 1, '!');
   
   delay(2000);
   */
     
  /*
  if(current_time-last_transmit > 1000) {
    xbee_promulgate.transmit_action('#', 'F', 100, 500, '!');
    last_transmit = current_time;
  }
  */
        
  while(xbee.available()) {
    char c = xbee.read();
    Serial << c;
    xbee_promulgate.organize_message(c);
  }
  
  if(Serial.available() > 0) {
    char c = Serial.read();
    serial_promulgate.organize_message(c);    
  }
        
        
        
        
        
        
        
        
        
        /*
        
        
  
  xval = 5*(analogRead(X_PIN)/1023);
  yval = 5*(analogRead(Y_PIN)/1023);
  zval = 5*(analogRead(Z_PIN)/1023);
  xcount += xval;
  ycount += yval;
  zcount += zval;
  count++;

  x_acc = (xval-x_home)/acc_scale;
  y_acc = (yval-y_home)/acc_scale;
  z_acc = (zval-z_home)/acc_scale;
  
  gyro_voltage = ( analogRead(GYRO_PIN) * max_voltage ) / 1023;
  gyro_rate = (gyro_voltage - gyro_zero_voltage) / gyro_scale;
  
  //Serial << "gyro: " << gyro_rate << endl;
  
  //Serial << "Acc X: " << x_acc << "Acc Y: " << y_acc << "Acc Z: " << z_acc << endl;
  
  acc_pitch = atan(x_acc / sqrt( pow(y_acc,2) + pow(z_acc,2) ) );
  acc_roll = atan(y_acc / sqrt( pow(x_acc,2) + pow(z_acc,2) ) );
  //float pitch = atan(xval / sqrt( pow(yval,2) + pow(zval,2) ) );
  //float roll = atan(yval / sqrt( pow(xval,2) + pow(zval,2) ) );
  acc_pitch = acc_pitch * (180/PI);
  acc_roll = acc_roll * (180/PI);
  
  //Serial << "pitch: " << pitch << " roll: " << roll << endl;
  //delay(100);
  
  
  if(gyro_rate > rotation_thresh || gyro_rate < -rotation_thresh) {
    gyro_pitch = gyro_pitch + gyro_rate * (current_time-last_measure_time)/1000.0;
    last_measure_time = current_time;
  }
  
  if(gyro_pitch > 360) gyro_pitch -= 360;
  if(gyro_pitch < -360) gyro_pitch += 360;
  
  //Serial << "pitch: " << current_angle << endl;
  //delay(100);
  
  
  
  pitch = alpha*gyro_pitch + (1-alpha)*acc_pitch;
  //Serial << "pitch: " << pitch << endl;
  //delay(100);
  
  
  
  
  
  
  
  if(digitalRead(BUTTON_3) == HIGH) {
    
    x_home = analogRead(X_PIN);
    y_home = analogRead(Y_PIN);
    z_home = analogRead(Z_PIN);  
    
    gyro_home = analogRead(GYRO_PIN);
    gyro_zero_voltage = ( gyro_home * max_voltage ) / 1023;
    
    
    xval = 5*(analogRead(X_PIN)/1023);
  yval = 5*(analogRead(Y_PIN)/1023);
  zval = 5*(analogRead(Z_PIN)/1023);
    
     x_acc = (xval-x_home)/acc_scale;
  y_acc = (yval-y_home)/acc_scale;
  z_acc = (zval-z_home)/acc_scale;
  
  gyro_voltage = ( analogRead(GYRO_PIN) * max_voltage ) / 1023;
  gyro_rate = (gyro_voltage - gyro_zero_voltage) / gyro_scale;
    
    acc_pitch = atan(x_acc / sqrt( pow(y_acc,2) + pow(z_acc,2) ) );
  acc_roll = atan(y_acc / sqrt( pow(x_acc,2) + pow(z_acc,2) ) );
  //float pitch = atan(xval / sqrt( pow(yval,2) + pow(zval,2) ) );
  //float roll = atan(yval / sqrt( pow(xval,2) + pow(zval,2) ) );
  acc_pitch = acc_pitch * (180/PI);
  acc_roll = acc_roll * (180/PI);
    
    if(gyro_rate > rotation_thresh || gyro_rate < -rotation_thresh) {
    gyro_pitch = gyro_pitch + gyro_rate * (current_time-last_measure_time)/1000.0;
    last_measure_time = current_time;
  }
  
  if(gyro_pitch > 360) gyro_pitch -= 360;
  if(gyro_pitch < -360) gyro_pitch += 360;
  
    pitch_start = alpha*gyro_pitch + (1-alpha)*acc_pitch;
    
    Serial << "yes" << endl;
    
    button3_press_time = current_time;
    headband_speed_control = true;
  }
  
  
  if(current_time-button3_press_time < max_control_time && headband_speed_control == true) {
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 0, 0, 255);
    }
    strip.show();
    delay(20);
    
    speed_val = (int)map(pitch, pitch_start-10, pitch_start+50, min_speed, max_speed);
    if(speed_val < min_speed) speed_val = min_speed;
    if(speed_val > max_speed) speed_val = max_speed;
    
    if(current_time-last_send > 100) {
      Serial << speed_val << endl;
      xbee << speed_val << endl;
      last_send = current_time;
    }
  
    
    //Serial << speed_val << endl;
    //delay(100);
    
  }
  
  
  if(current_time-button3_press_time > max_control_time && headband_speed_control == true) {
    headband_speed_control = false;
  }
  
  
  
  
  if(digitalRead(BUTTON_2) == HIGH) {
    
    xbee << "A" << endl;
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 0, 0, 255);
    }
    strip.show();
    delay(3000);
    button2_press_time = current_time;
    ears_wiggle_mode = true;
  }
  
  if(current_time-button2_press_time < max_control_time && ears_wiggle_mode == true) {
   
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 0, 0, 255);
    }
    strip.show();
    
    
    
    servo_L.attach(SERVO_L_PIN);
    servo_R.attach(SERVO_R_PIN);
    
    for(int i=0; i<30; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);
        delay(10);
      }
      
      for(int i=30; i>-30; i--) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      for(int i=-30; i<0; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      servo_L.detach();
      servo_R.detach();
   
   
   last_servo_update = current_time;
    
  }
  
  if(current_time-button2_press_time > max_control_time && ears_wiggle_mode == true) {
    ears_wiggle_mode = false;
  }
  
  
  
  
  
  if(digitalRead(BUTTON_1) == HIGH) {
    
    xbee << "B" << endl;
    
    button1_press_time = current_time;
    rainbow_mode = true;
  }
  
  rainbow_mode = true;
  if(rainbow_mode == true) {//current_time-button1_press_time < max_control_time && rainbow_mode == true) {
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 255, 0, 0);
    }
    strip.show();
    
    delay(100);
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 128, 128, 0);
    }
    strip.show();
    
    delay(100);
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 0, 255, 0);
    }
    strip.show();
    
    delay(100);
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 0, 128, 128);
    }
    strip.show();
    
    delay(100);
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 0, 0, 255);
    }
    strip.show();
    
    delay(100);
    
    for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 128, 0, 128);
    }
    strip.show();
    
    delay(100);
    
  }
  
  if(current_time-button1_press_time > max_control_time && rainbow_mode == true) {
    rainbow_mode = false;
  }
  
  
  
  
  
  */
  
  
  
  
  
  
  
  
  
  /*
  if(xbee.available() > 0) {
    Serial << (char)xbee.read();
  }
  */











  /*


  if(current_time-last_servo_update >= 3000 && headband_speed_control == false && ears_wiggle_mode == false) {
    
    servo_L.attach(SERVO_L_PIN);
    servo_R.attach(SERVO_R_PIN);
    
    int r = (int)random(0, 2);
    
    if(r == 0) {
    
      for(int i=0; i<30; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);
        delay(10);
      }
      
      for(int i=30; i>-30; i--) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      for(int i=-30; i<0; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
    
    } else if(r == 1) {
      
      for(int i=0; i<30; i++) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);
        delay(10);
      }
      
      for(int i=30; i>-30; i--) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      for(int i=-30; i<0; i++) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
    }
    
    servo_L.detach();
    servo_R.detach();
      
    last_servo_update = current_time;  
      
  }
  
  
  
  delay(20);




  */




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
  
  // #P<s>,<d>!
  // <s> from 0-255, <d> from 0-65536
  // P: goPlaces
  // F: goForwards
  // B: goBackwards
  // L: turnLeft
  // R: turnRight
  // S: stopMotors
  
  // this is sent to the sensor board when the motors are finished moving:
  // ^Z1,1!
  
}

void transmit_complete() {
  if(DEBUG) {
    Serial << "--transmit complete--" << endl;
  }
}








