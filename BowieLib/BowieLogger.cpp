#include "BowieLogger.h"

BowieLogger::BowieLogger() {

  // how often to write a new file (mins)
  TIMEOUT_MINS = 15;
  
  // how often to write a new file (seconds)
  TIMEOUT_SECS = 0;
  
  // how often to log data (seconds)
  INTERVAL_SECS = 1;

  log_headers[0] = "Time";
  log_headers[1] = "Motor A Speed";
  log_headers[2] = "Motor A Dir";
  log_headers[3] = "Motor B Speed";
  log_headers[4] = "Motor B Dir";
  log_headers[5] = "Motor Current Sensor";
  log_headers[6] = "Servo Pos - Arm L";
  log_headers[7] = "Servo Pos - Arm R";
  log_headers[8] = "Servo Pos - End";
  log_headers[9] = "Servo Pos - Hopper";

  log_headers[10] = "Servo Pos - Lid";
  log_headers[11] = "Servo Pos - Extra";
  log_headers[12] = "Servo Current Sensor";
  log_headers[13] = "LED - Front L";
  log_headers[14] = "LED - Front R";
  log_headers[15] = "LED - Back L";
  log_headers[16] = "LED - Back R";
  log_headers[17] = "IMU Pitch";
  log_headers[18] = "IMU Roll";
  log_headers[19] = "IMU Yaw";

  log_headers[20] = "Compass Heading";
  log_headers[21] = "GPS Latitude";
  log_headers[22] = "GPS Longitude";
  log_headers[23] = "GPS Altitude";
  log_headers[24] = "Battery";
  log_headers[25] = "Comm Latency";

  logdata = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  LOGGING = false;
  logging_path = "";
  last_time_check = 0;

  LOG_LED = 13;

}

void BowieLogger::setLoggingLed(int pin) {
  LOG_LED = pin;
}

void BowieLogger::initLogging() {
  pinMode(LOG_LED, OUTPUT);

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

void BowieLogger::updateLogging() {
  if(LOGGING) {

    uint32_t difference = (uint32_t)(log_interval - now());
    uint8_t difference_sec = difference % 60;

    if(millis()-last_time_check >= 1000) {
      logFileTimer();
      last_time_check = millis();
    }
  
    if(difference_sec <= 0) {
  
      // get the data
      // NOTE: we are assuming this is done from the sketch side
  
      // open the file
      openLogFile();
  
      // log it to file
      if(DEBUG_PRINTS) Serial.println("Logging data");
      writeLogData();
  
      // close the file
      closeLogFile();
      log_interval = now() + INTERVAL_SECS;
       
    }

    digitalWrite(LOG_LED, HIGH);

  } else {
    digitalWrite(LOG_LED, LOW);
  }
}



