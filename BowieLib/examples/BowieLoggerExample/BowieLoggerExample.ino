#include "BowieLogger.h"

BowieLogger bowielogger = BowieLogger();

void setup() {
  Serial.begin(9600);

  initTime(); // should always be first

  bowielogger.setLoggingLed(13);
  bowielogger.initLogging();
  
}

void loop() {

  updateLogSensorData();
  bowielogger.updateLogging();

}


