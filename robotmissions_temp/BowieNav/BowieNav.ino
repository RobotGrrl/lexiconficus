#include "MegaBowieShoreline.h"

// Neopixels for compass
#include <Adafruit_NeoPixel.h>

// GPS
#include <TinyGPS++.h>

// Compass
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

#define ROBOT_ID 3

MegaBowieShoreline bowie;

long current_time = 0;

// ---- Compass Neopixel
/* connected to Sensor Input 1:
 * Gnd      -  Gnd
 * 5V uC    -  Pwr
 * 3.3V ext -  NC
 * A21      -  NC
 * 39 / A20 -  Sig
 */
#define PIN 39
#define NUMPIXELS 24
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ---- GPS
// connects to Serial3
/* Connected to Navigation slot:
 * Gnd
 * 3.3V ext
 * Fix (28)
 * RST (29)
 * TX3 (8)
 * RX3 (7)
 * SDA (18)
 * SCL (19)
 */
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
#define GPS_FIX_PIN 28
uint8_t printCount = 0;
#define MAX_POINTS 1
double POINTS_LAT[MAX_POINTS];
double POINTS_LON[MAX_POINTS];
uint8_t POINTS_ACTION[MAX_POINTS];
uint8_t current_point = 0;
#define DISTANCE_THRESH 3.1 // meters
bool following_course = true;

// ---- Compass & IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
bool COMPASS_ENABLED = true;
#define CALIBRATING_NOW false
#define LED_BRIGHTNESS 30
double compass_heading = 0.0;
double compass_roll = 0.0;
double compass_pitch = 0.0;
long last_imu_print = 0;

// ---- Air quality
String air_quality_data = "";

// ---- DHT22
#include <dht.h>
dht DHT;
#define DHT22_PIN 32
long last_dht22 = 0;

// ---- Compass Navigation
int north_indicator;
int heading_indicator;
int robot_direction_indicator;
bool at_heading = false;
double tracking_heading = 320.94;
long last_diff_print = 0;
enum all_drive_states {
  IDLE,
  FORWARD,
  RIGHT,
  HARD_RIGHT,
  LEFT,
  HARD_LEFT,
  REVERSE
};
int DRIVE_STATE = 0;
int PREV_DRIVE_STATE = 0;
uint32_t pixel_colours[NUMPIXELS];
uint8_t pixel_states[NUMPIXELS]; // 0 = off, 1 = indicators, 2 = turning
long last_change_heading = 0;
double given_headings[] = { 0, 60, 120, 180, 240, 300 };
int change_heading_step = 0;

// ---- PID Control
double kp = 4.0;
double ki = 0.5;
double kd = 0.3;
long last_update_pid = 0;
bool updating_pid = false;
float elapsed_seconds = 0.0;
long time_previous = 0;
double error = 0.0;
double previous_error = 0.0;
int min_speed = 60;
int speed_left = 0;
int speed_right = 0;
float PID = 0.0;
float pid_p = 0.0;
float pid_i = 0.0;
float pid_d = 0.0;

/*
 * notes about calibration:
 * with kp = 1, ki = 0, kd = 0
 * 
 */

void setup() {
  Serial.begin(9600);
  Serial5.begin(9600); // uRADMonitor Air Quality sensor
  Serial5.setTimeout(100);
  
  bowie = MegaBowieShoreline();
  bowie.setRobotID(ROBOT_ID);

  delay(5000);

  // GPS
  gpsInit();

  // Compass
  compassInit();
  
  bowie.begin();

  compassUpdate();
  tracking_heading = compass_heading;

}

void loop() {

  if(current_time-last_update_pid >= 100 && current_time > 0) {
    elapsed_seconds = float ( float(current_time-time_previous) / 1000 );
    Serial << "tp: " << time_previous << " ct: " << current_time << " e: " << elapsed_seconds << endl;    
    updating_pid = true;
  }

  current_time = millis();
  
  // we're not updating bowie in this example
  //bowie.update(false);

  // GPS
  //gpsUpdate();

  // Compass
  compassUpdate(); // update the sensors
  compassIndicatorLeds(); // update the indicator leds
  navigateHeadingDirection(); // set DRIVE_STATE based on tracking_heading
  compassDriveLeds(); // update the drive leds
  
  /*
  robotDriveTrackHeading();

  if(current_time-last_change_heading >= 3000) {
    tracking_heading = given_headings[change_heading_step];
    change_heading_step++;
    if(change_heading_step > 5) change_heading_step = 0;
    last_change_heading = current_time;
  }
  */

  if(updating_pid) {
  
    // determine how far off we are from the current heading
    // to the tracking heading
    previous_error = error;
    error = tracking_heading - compass_heading;//distanceAngles((int)floor(tracking_heading), (int)floor(compass_heading));

  
    // proportional value is the proportional constant multiplied
    // by the error
    pid_p = kp*error;
  
    // integral value will only play a part when we are close to
    // the desired heading. when the error is small, then it will
    // kick in. it adds itself to itself, since it is an integral
    // value
    if(abs(error) < 20) {
      Serial << "!!!!!!!!!!!!!!! YUP" << endl;
      pid_i = pid_i+(ki*error);
    } else {
      pid_i = 0;
    }
  
    // derivative value takes the change in error over the certain
    // amount of time between the updates. it's the rate of change
    // of the error. time is in seconds
    pid_d = kd * ( (error-previous_error)/elapsed_seconds );
  
    Serial << "P: " << pid_p << " I: " << pid_i << " D: " << pid_d << endl;
  
    // these then get added together into the final value
    PID = pid_p + pid_i + pid_d;

    // constrain it so it would oscillate around min_speed
    constrain(PID, min_speed-50, min_speed+50);

    // adding the PID values to the left and right sides
    speed_left = min_speed - PID;
    speed_right = min_speed + PID;
  
    // be sure to constrain the values to the max (and min) speed
    speed_left = constrain(speed_left, min_speed, 255);
    speed_right = constrain(speed_right, min_speed, 255);
  
    Serial << "::::: Left: " << speed_left << " Right: " << speed_right << " PID: " << PID << endl;
  
    bowie.bowiedrive.goSpeed(true, true, speed_left, 1); // fwd, left, speed, del
    bowie.bowiedrive.goSpeed(true, false, speed_right, 1); // fwd, right, speed, del
  
    time_previous = current_time;
    last_update_pid = current_time;
    updating_pid = false;

  }
  

  /*
  // Updating our logging sensors
  bowie.bowielogger.setLogData_f(LOG_IMU_PITCH, compass_pitch);
  bowie.bowielogger.setLogData_f(LOG_IMU_ROLL, compass_roll);
  bowie.bowielogger.setLogData_f(LOG_IMU_YAW, 0.0);
  bowie.bowielogger.setLogData_f(LOG_COMPASS_HEADING, compass_heading);
  bowie.bowielogger.setLogData_u8(LOG_GPS_SATS, gps.satellites.value());
  bowie.bowielogger.setLogData_f(LOG_GPS_HDOP, gps.hdop.value());
  bowie.bowielogger.setLogData_f(LOG_GPS_LATITUDE, gps.location.lat());
  bowie.bowielogger.setLogData_f(LOG_GPS_LONGITUDE, gps.location.lng());
  bowie.bowielogger.setLogData_f(LOG_GPS_ALTITUDE, gps.altitude.meters());

  // Updating our air quality sensor logger
  while(Serial5.available()) {
    air_quality_data = Serial5.readString();
  }
  
  if(air_quality_data.length() > 0) {
    Serial << "AQ--->" << air_quality_data << endl;
    bowie.bowielogger.setLogData_s(LOG_AQ_DATA, air_quality_data);  
    air_quality_data = "";
  }

  // Updating the DHT-22
  if(millis()-last_dht22 >= 1000) {
    // update it
    int chk = DHT.read22(DHT22_PIN);
    bowie.bowielogger.setLogData_f(LOG_HUMIDITY, DHT.humidity);
    bowie.bowielogger.setLogData_f(LOG_TEMPERATURE, DHT.temperature);
    last_dht22 = millis();
  }
  */

}


