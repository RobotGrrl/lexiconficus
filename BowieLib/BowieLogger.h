/*
 * Robot Missions Bowie Logger
 * ---------------------------
 *
 * Datalogging for Bowie the robot
 *
 * Erin RobotGrrl for Robot Missions
 * --> http://RobotMissions.org
 *
 * Using Teensy 3.6 on-board SD card and RTC
 * 
 * Erin RobotGrrl
 * Dec. 23rd, 2017
 *
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
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

#include "Arduino.h"
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

#ifndef _BOWIELOGGER_H_
#define _BOWIELOGGER_H_


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

class BowieLogger {
  
  public:
    BowieLogger();
    
    Sd2Card card;
    SdVolume volume;
    SdFile root;
    //const int chipSelect = BUILTIN_SDCARD; 


  private:
    void initTime();
    void digitalClockDisplay();
    time_t getTeensy3Time();
    void printDigits(int digits);

};

#endif