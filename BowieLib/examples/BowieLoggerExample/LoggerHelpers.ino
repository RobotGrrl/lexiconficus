void updateLogSensorData() {

  bowielogger.setLogData_t(LOG_TIME, now());

  
}

void initTime() {
  // from the TimeTeensy3 example code
  // most likely by Paul S
  
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
}

time_t getTeensy3Time() {
  return Teensy3Clock.get();
}


