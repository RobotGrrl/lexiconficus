#include "BowieLogger.h"

void BowieLogger::setLogData_t(int log_item, time_t val) {
  if(log_item == LOG_TIME) logdata.sample_time = val;
}

void BowieLogger::setLogData_u8(int log_item, uint8_t val) {
  if(log_item == LOG_MOTOR_A_SPEED) logdata.motor_a_speed = val;
  if(log_item == LOG_MOTOR_A_DIR) logdata.motor_a_dir = val;
  if(log_item == LOG_MOTOR_B_SPEED) logdata.motor_b_speed = val;
  if(log_item == LOG_MOTOR_B_DIR) logdata.motor_b_dir = val;
  if(log_item == LOG_LED_FRONT_L) logdata.led_front_l = val;
  if(log_item == LOG_LED_FRONT_R) logdata.led_front_r = val;
  if(log_item == LOG_LED_BACK_L) logdata.led_back_l = val;
  if(log_item == LOG_LED_BACK_R) logdata.led_back_r = val;
  if(log_item == LOG_GPS_SATS) logdata.gps_sats = val;
}

void BowieLogger::setLogData_u16(int log_item, uint16_t val) {
  if(log_item == LOG_MOTOR_CURRENT_SENSOR) logdata.motor_current_sensor = val;
  if(log_item == LOG_SERVO_POS_ARM_L) logdata.servo_pos_arm_l = val;
  if(log_item == LOG_SERVO_POS_ARM_R) logdata.servo_pos_arm_r = val;
  if(log_item == LOG_SERVO_POS_END) logdata.servo_pos_end = val;
  if(log_item == LOG_SERVO_POS_HOPPER) logdata.servo_pos_hopper = val;
  if(log_item == LOG_SERVO_POS_LID) logdata.servo_pos_lid = val;
  if(log_item == LOG_SERVO_POS_EXTRA) logdata.servo_pos_extra = val;
  if(log_item == LOG_SERVO_CURRENT_SENSOR) logdata.servo_current_sensor = val;
  if(log_item == LOG_BATTERY_SENSOR) logdata.battery_sensor = val;
  if(log_item == LOG_COMM_XBEE_LATENCY) logdata.comm_xbee_latency = val;
  if(log_item == LOG_COMM_ARDUINO_LATENCY) logdata.comm_arduino_latency = val;
}

void BowieLogger::setLogData_f(int log_item, float val) {
  if(log_item == LOG_IMU_PITCH) logdata.imu_pitch = val;
  if(log_item == LOG_IMU_ROLL) logdata.imu_roll = val;
  if(log_item == LOG_IMU_YAW) logdata.imu_yaw = val;
  if(log_item == LOG_COMPASS_HEADING) logdata.compass_heading = val;
  if(log_item == LOG_GPS_HDOP) logdata.gps_hdop = val;
  if(log_item == LOG_GPS_LATITUDE) logdata.gps_latitude = val;
  if(log_item == LOG_GPS_LONGITUDE) logdata.gps_longitude = val;
  if(log_item == LOG_GPS_ALTITUDE) logdata.gps_altitude = val;
}


void BowieLogger::randomLogData() {
  logdata.sample_time = now();
  logdata.motor_a_speed = (int)random(0, 256);
  logdata.motor_a_dir = (int)random(0, 2);
  logdata.motor_b_speed = (int)random(0, 256);
  logdata.motor_b_dir = (int)random(0, 2);
  logdata.motor_current_sensor = (int)random(0, 1024);
  logdata.servo_pos_arm_l = (int)random(0, 2500);
  logdata.servo_pos_arm_r = (int)random(0, 2500);
  logdata.servo_pos_end = (int)random(0, 2500);
  logdata.servo_pos_hopper = (int)random(0, 2500);
  logdata.servo_pos_lid = (int)random(0, 2500);
  logdata.servo_pos_extra = (int)random(0, 2500);
  logdata.servo_current_sensor = (int)random(0, 1024);
  logdata.led_front_l = (int)random(0, 256);
  logdata.led_front_r = (int)random(0, 256);
  logdata.led_back_l = (int)random(0, 256);
  logdata.led_back_r = (int)random(0, 256);
  logdata.imu_pitch = (float)random(-90, 90);
  logdata.imu_roll = (float)random(-90, 90);
  logdata.imu_yaw = (float)random(-90, 90);
  logdata.compass_heading = (float)random(0, 360);
  logdata.gps_sats = (float)random(40, 42);
  logdata.gps_hdop = (float)random(40, 42);
  logdata.gps_latitude = (float)random(40, 42);
  logdata.gps_longitude = (float)random(70, 75);
  logdata.gps_altitude = (float)random(0, 10);
  logdata.battery_sensor = (int)random(0, 1024);
  logdata.comm_xbee_latency = (int)random(50, 200);
  logdata.comm_arduino_latency = (int)random(50, 200);
}

void BowieLogger::writeLogData() {
  logging_file.print(hour(logdata.sample_time));
  logging_file.print(":");
  if(minute(logdata.sample_time) < 10) {
    logging_file.print("0");
  }
  logging_file.print(minute(logdata.sample_time));
  logging_file.print(":");
  if(second(logdata.sample_time) < 10) {
    logging_file.print("0");  
  }
  logging_file.print(second(logdata.sample_time));
  logging_file.print(",");

  logging_file.print(logdata.motor_a_speed);
  logging_file.print(",");
  
  logging_file.print(logdata.motor_a_dir);
  logging_file.print(",");

  logging_file.print(logdata.motor_b_speed);
  logging_file.print(",");

  logging_file.print(logdata.motor_b_dir);
  logging_file.print(",");

  logging_file.print(logdata.motor_current_sensor);
  logging_file.print(",");

  logging_file.print(logdata.servo_pos_arm_l);
  logging_file.print(",");

  logging_file.print(logdata.servo_pos_arm_r);
  logging_file.print(",");

  logging_file.print(logdata.servo_pos_end);
  logging_file.print(",");

  logging_file.print(logdata.servo_pos_hopper);
  logging_file.print(",");

  logging_file.print(logdata.servo_pos_lid);
  logging_file.print(",");

  logging_file.print(logdata.servo_pos_extra);
  logging_file.print(",");

  logging_file.print(logdata.servo_current_sensor);
  logging_file.print(",");

  logging_file.print(logdata.led_front_l);
  logging_file.print(",");

  logging_file.print(logdata.led_front_r);
  logging_file.print(",");

  logging_file.print(logdata.led_back_l);
  logging_file.print(",");

  logging_file.print(logdata.led_back_r);
  logging_file.print(",");

  logging_file.print(logdata.imu_pitch, 7);
  logging_file.print(",");

  logging_file.print(logdata.imu_roll, 7);
  logging_file.print(",");

  logging_file.print(logdata.imu_yaw, 7);
  logging_file.print(",");

  logging_file.print(logdata.compass_heading, 7);
  logging_file.print(",");

  logging_file.print(logdata.gps_sats);
  logging_file.print(",");  

  logging_file.print(logdata.gps_hdop, 7);
  logging_file.print(",");

  logging_file.print(logdata.gps_latitude, 7);
  logging_file.print(",");

  logging_file.print(logdata.gps_longitude, 7);
  logging_file.print(",");

  logging_file.print(logdata.gps_altitude, 7);
  logging_file.print(",");

  logging_file.print(logdata.battery_sensor);
  logging_file.print(",");

  logging_file.print(logdata.comm_xbee_latency);
  logging_file.print(",");
  
  logging_file.print(logdata.comm_arduino_latency);

  logging_file.print("\r\n");

  log_interval = now() + INTERVAL_SECS;
}

void BowieLogger::closeLogFile() {
  if(DEBUG_PRINTS) Serial.println("Closing log file");
  logging_file.close();
}

void BowieLogger::openLogFile() {
  if(SD.exists(logging_path.c_str())) { 
    if(DEBUG_PRINTS) Serial.print("Opening ");
    if(DEBUG_PRINTS) Serial.println(logging_path);
    logging_file = SD.open(logging_path.c_str(), FILE_WRITE);
  } else {
    if(DEBUG_PRINTS) Serial.println("Was unable to open, creating new one");
    createLogFile();
  }
}

void BowieLogger::logFileTimer() {
  uint32_t difference = (uint32_t)(log_file_timeout - now());
  uint8_t difference_sec = difference % 60;
  difference /= 60;
  uint8_t difference_min = difference % 60;

  Serial.print(difference_min);
  Serial.print(":");
  if(difference_sec < 10) {
    Serial.print("0");  
  }
  Serial.print(difference_sec);
  Serial.println(" remaining to new log file");

  if(difference_min <= 0 && difference_sec <= 0) {
    if(DEBUG_PRINTS) Serial.println("Saving last log file");
    closeLogFile();
    if(DEBUG_PRINTS) Serial.println("Time up! Creating new log file");
    createLogFile();
  }
}

void BowieLogger::createLogFile() {

  int log_files_count = 0;

  int short_year = year()%100;
  
  String dir_name = "";
  dir_name.concat(month());
  dir_name += "_";
  dir_name.concat(day());
  dir_name += "_";
  dir_name.concat(short_year);

  File root;
  root = SD.open("/");
  bool create_the_dir = false;

  while(true) {
    File entry = root.openNextFile();
    if (!entry) {
       // no more files, so we will create the dir
       create_the_dir = true;
       break;
    }
    if(DEBUG_PRINTS) Serial.println(entry.name());
    String s = "";
    s.concat(entry.name());
    if(s == dir_name) {
      // already exists, we don't need to make it
      if(DEBUG_PRINTS) Serial.println("dir exists");
      break;
    }
  }

  if(create_the_dir) {
    if(DEBUG_PRINTS) Serial.println("dir does NOT exist, creating it"); 
    SD.mkdir(dir_name.c_str());
  }
  
  // open the dir
  logging_dir = SD.open(dir_name.c_str());
  if(DEBUG_PRINTS) Serial.println("Opening dir " + dir_name);

  // counting the log files
  while(true) {
    File entry = logging_dir.openNextFile();
    if (!entry) {
       // no more files
       if(log_files_count == 0) if(DEBUG_PRINTS) Serial.println("No files in the dir yet");
       break;
    }
    if(DEBUG_PRINTS) Serial.println(entry.name());
    if (!entry.isDirectory()) {
      // increment the count of log files
      log_files_count++;
    }
  }

  if(DEBUG_PRINTS) Serial.print("Number of log files: ");
  if(DEBUG_PRINTS) Serial.println(log_files_count);

  // creating the next log file
  while(true) {
    String log_file_name = "LOG_";
    log_file_name.concat(log_files_count);
    log_file_name += ".CSV";

    logging_path = dir_name + "/" + log_file_name;
    
    if(SD.exists(logging_path.c_str())) { 
      if(DEBUG_PRINTS) Serial.println("That file already exists...");
      // why would this happen? let's try another number?
      log_files_count++;
    } else {
      // good. let's create the file.
      if(DEBUG_PRINTS) Serial.print("Creating file ");
      if(DEBUG_PRINTS) Serial.println(log_file_name);
      logging_file = SD.open(logging_path.c_str(), FILE_WRITE);
      log_file_started = now();
      log_file_timeout = log_file_started + (TIMEOUT_MINS * 60) + TIMEOUT_SECS;
      break;
    }
  }

  // only log if we have space!
  if(getPercentAvailable() >= 1.0) {
    LOGGING = true;
  } else {
    LOGGING = false;
    return;
  }

  // log the first line
  if(logging_file) {
    int headers_size = sizeof(log_headers) / sizeof(String);
    for(int i=0; i<headers_size; i++) {
      logging_file.print(log_headers[i]);
      if(i < (headers_size-1)) logging_file.print(", ");
    }
    logging_file.print("\r\n");
    // save and re-open
    closeLogFile();
    openLogFile();
    log_interval = now() + INTERVAL_SECS;
    if(DEBUG_PRINTS) Serial.println("Logged the first line");
  } else {
    if(DEBUG_PRINTS) Serial.println("Error opening the log file");
  }

}
