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

#define DEBUG_PRINTS true

// --
// if you have more or less data, modify these three variables below

#define NUM_DATA_POINTS 27

struct LogLine { // 26 points
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
  uint16_t comm_xbee_latency;
  uint16_t comm_arduino_latency;
};

#define LOG_TIME 0
#define LOG_MOTOR_A_SPEED 1
#define LOG_MOTOR_A_DIR 2
#define LOG_MOTOR_B_SPEED 3
#define LOG_MOTOR_B_DIR 4
#define LOG_MOTOR_CURRENT_SENSOR 5
#define LOG_SERVO_POS_ARM_L 6
#define LOG_SERVO_POS_ARM_R 7
#define LOG_SERVO_POS_END 8
#define LOG_SERVO_POS_HOPPER 9

#define LOG_SERVO_POS_LID 10
#define LOG_SERVO_POS_EXTRA 11
#define LOG_SERVO_CURRENT_SENSOR 12
#define LOG_LED_FRONT_L 13
#define LOG_LED_FRONT_R 14
#define LOG_LED_BACK_L 15
#define LOG_LED_BACK_R 16
#define LOG_IMU_PITCH 17
#define LOG_IMU_ROLL 18
#define LOG_IMU_YAW 19

#define LOG_COMPASS_HEADING 20
#define LOG_GPS_LATITUDE 21
#define LOG_GPS_LONGITUDE 22
#define LOG_GPS_ALTITUDE 23
#define LOG_BATTERY_SENSOR 24
#define LOG_COMM_XBEE_LATENCY 25
#define LOG_COMM_ARDUINO_LATENCY 26

// --

class BowieLogger {
  
  public:
    BowieLogger();
    void begin();

    void setLoggingLed(int pin);
    void initLogging();
    void initTime();
    void updateLogging();

    // Logging
    uint8_t TIMEOUT_MINS;
    uint8_t TIMEOUT_SECS;
    uint8_t INTERVAL_SECS;

    LogLine logdata;

    // TimeFuncs
    void digitalClockDisplay();

    // SdFuncs
    bool initSd();
    void getAllFiles();
    void printDirectory(File dir, int numTabs);
    uint32_t getSizeUsed();
    uint32_t getSizeTotal();
    float getPercentAvailable();
    void sendEntireFile(String filename, Stream *s);

    // LogFuncs
    void setLogData_t(int log_item, time_t val);
    void setLogData_u8(int log_item, uint8_t val);
    void setLogData_u16(int log_item, uint16_t val);
    void setLogData_f(int log_item, float val);

  private:
    // TimeFuncs
    void printDigits(int digits);
    static time_t getTeensy3Time();

    // LogFuncs
    void randomLogData();
    void writeLogData();
    void closeLogFile();
    void openLogFile();
    void logFileTimer();
    void createLogFile();

    // SdFuncs
    uint32_t enterNextDir(File dir);

    uint8_t LOG_LED;

    // SD
    Sd2Card card;
    SdVolume volume;
    SdFile root;
    int chipSelect = BUILTIN_SDCARD; 

    // Logging
    String log_headers[NUM_DATA_POINTS];
    bool LOGGING;
    File logging_dir;
    File logging_file;
    String logging_path;
    time_t log_file_started;
    time_t log_file_timeout;
    time_t log_interval;
    long last_time_check;

};

#endif