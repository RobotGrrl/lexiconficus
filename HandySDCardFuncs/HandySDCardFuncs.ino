/*
 * Handy SD Card Functions
 * ------------------------
 * Using Teensy 3.6 on-board SD card and RTC
 * 
 * Erin RobotGrrl
 * Dec. 9th, 2017
 * 
 * Directory is MM_DD_YY, then each log file is LOG_N.CSV.
 * 
 * The data logged is a 'snapshot', meaning, it's not an average - it is what
 * the sensors reported at that time. Getting the sensor data is done in
 * getLogData() in LogFuncs.
 * 
 * There is a log timeout, when the timer reaches 0 and on setup, this occurs:
 * - Space on the card is checked. If there's no space, it won't log.
 * - The previous log file is closed.
 * - A new log file is started.
 * - Headers are printed at the top of the new log file.
 * 
 * The reason for the log timeout is in the possibility that an error would
 * occur, preventing the data from being saved. By adding in this redundancy,
 * we can hopefully always have some data that is saved, even if one "snippet"
 * of time was not logged. An event that could trigger this would for example
 * be if the battery all of a sudden stopped powering the robot. It might not
 * effect it though, seeing how the file is closed after writing. Maybe I'm
 * just paranoid about losing logging data.
 * 
 * Portions of this code were borrowed from the examples of Time and SD lib,
 * thank you to those authours (credited in the functions).
 * 
 */

#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = BUILTIN_SDCARD; 

// -- logging
bool DEBUG_PRINTS = true; // change this to hide all the serial.prints
bool LOGGING = false;
File logging_dir;
File logging_file;
String logging_path;
time_t log_file_started;
time_t log_file_timeout;
uint8_t timeout_mins = 15; // change this for how often to write a new file (mins)
uint8_t timeout_secs = 0; // change this for how often to write a new file (seconds)
time_t log_interval;
uint8_t interval_secs = 1; // change this for how often to log data (seconds)
long last_time_check = 0;

// --
// if you have more or less data, modify these three variables below

String log_headers[] = { "Time", "Motor A Speed", "Motor A Dir", "Motor B Speed", 
                          "Motor B Dir", "Motor Current Sensor", "Servo Pos - Arm L",
                          "Servo Pos - Arm R", "Servo Pos - End", "Servo Pos - Hopper",
                          "Servo Pos - Lid", "Servo Pos - Extra", "Servo Current Sensor",
                          "LED - Front L", "LED - Front R", "LED - Back L", "LED - Back R",
                          "IMU Pitch", "IMU Roll", "IMU Yaw", "Compass Heading",
                          "GPS Latitude", "GPS Longitude", "GPS Altitude", "Battery"};

struct LogLine { // 25 points
  time_t sample_time;
  uint8_t motor_a_speed;
  uint8_t motor_a_dir;
  uint8_t motor_b_speed;
  uint8_t motor_b_dir;
  uint16_t motor_current_sensor;
  uint16_t servo_pos_arm_l;
  uint16_t servo_pos_arm_r;
  uint16_t servo_pos_end;
  uint16_t servo_pos_hopper;
  uint16_t servo_pos_lid;
  uint16_t servo_pos_extra;
  uint16_t servo_current_sensor;
  uint8_t led_front_l;
  uint8_t led_front_r;
  uint8_t led_back_l;
  uint8_t led_back_r;
  float imu_pitch;
  float imu_roll;
  float imu_yaw;
  float compass_heading;
  float gps_latitude;
  float gps_longitude;
  float gps_altitude;
  uint16_t battery_sensor;
};

LogLine logdata = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// --

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  delay(1000);

  initTime(); // should always be first
  
  if(!initSd()) {
    Serial.println("!!! Unable to start logging !!!");
    LOGGING = false;
  } else {
  
    // only log if we have space!
    if(getPercentAvailable() >= 5.0) {
      LOGGING = true;
      createLogFile();
    } else {
      Serial.println("!!! Not enough space available for logging !!!");
      LOGGING = false;
    }

  }

}

void loop() {

  if(LOGGING) {

    uint32_t difference = (uint32_t)(log_interval - now());
    uint8_t difference_sec = difference % 60;

    if(millis()-last_time_check >= 1000) {
      logFileTimer();
      last_time_check = millis();
    }
  
    if(difference_sec <= 0) {
  
      // get the data
      getLogData();
  
      // open the file
      openLogFile();
  
      // log it to file
      if(DEBUG_PRINTS) Serial.println("Logging data");
      writeLogData();
  
      // close the file
      closeLogFile();
      log_interval = now() + interval_secs;
       
    }

  }
  
}




