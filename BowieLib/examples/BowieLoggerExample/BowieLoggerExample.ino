/*
 * Bowie Logger Example
 * --------------------
 * 
 * Test the logging of data to the robot's micro SD card.
 * (Only for Mega Bowie, with Teensy 3.6 and RTC)
 * 
 * Erin RobotGrrl for RobotMissions
 * Dec. 23rd, 2017
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "BowieLogger.h"

BowieLogger bowielogger = BowieLogger();

void setup() {
  Serial.begin(9600);

  initTime(); // should always be first

  bowielogger.begin();
  bowielogger.setLoggingLed(13);
  bowielogger.initLogging();
  
}

void loop() {

  updateLogSensorData();
  bowielogger.updateLogging();

}


