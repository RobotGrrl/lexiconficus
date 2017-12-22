void getLogData() {
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
  logdata.gps_latitude = (float)random(40, 42);
  logdata.gps_longitude = (float)random(70, 75);
  logdata.gps_altitude = (float)random(0, 10);
  logdata.battery_sensor = (int)random(0, 1024);
}

void writeLogData() {
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

  logging_file.print(logdata.imu_pitch);
  logging_file.print(",");

  logging_file.print(logdata.imu_roll);
  logging_file.print(",");

  logging_file.print(logdata.imu_yaw);
  logging_file.print(",");

  logging_file.print(logdata.compass_heading);
  logging_file.print(",");

  logging_file.print(logdata.gps_latitude);
  logging_file.print(",");

  logging_file.print(logdata.gps_longitude);
  logging_file.print(",");

  logging_file.print(logdata.gps_altitude);
  logging_file.print(",");

  logging_file.print(logdata.battery_sensor);
  
  logging_file.print("\r\n");

  log_interval = now() + interval_secs;
}

void closeLogFile() {
  if(DEBUG_PRINTS) Serial.println("Closing log file");
  logging_file.close();
}

void openLogFile() {
  if(SD.exists(logging_path.c_str())) { 
    if(DEBUG_PRINTS) Serial.print("Opening ");
    if(DEBUG_PRINTS) Serial.println(logging_path);
    logging_file = SD.open(logging_path.c_str(), FILE_WRITE);
  } else {
    if(DEBUG_PRINTS) Serial.println("Was unable to open, creating new one");
    createLogFile();
  }
}

void logFileTimer() {
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

void createLogFile() {

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
      log_file_timeout = log_file_started + (timeout_mins * 60) + timeout_secs;
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
    log_interval = now() + interval_secs;
    if(DEBUG_PRINTS) Serial.println("Logged the first line");
  } else {
    if(DEBUG_PRINTS) Serial.println("Error opening the log file");
  }

}

