void compassInit() {

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_COMPASS)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    COMPASS_ENABLED = false;
  }

  if(COMPASS_ENABLED) {
    // calibration data configured from the BNO055_Calibration sketch
    byte c_data[22] = {0, 0, 0, 0, 0, 0, 188, 254, 244, 0, 116, 255, 0, 0, 0, 0, 0, 0, 232, 3, 44, 3};
    bno.setCalibData(c_data);
    
    delay(1000);
    bno.setExtCrystalUse(true);
    delay(20);
    compassNeopixelsInit();
  }
  
}

void compassUpdate() {

  if(!COMPASS_ENABLED) return;
  
  sensors_event_t event;
  bno.getEvent(&event);

  compass_heading = event.orientation.x;
  compass_roll = event.orientation.y;
  compass_pitch = event.orientation.z;

  //print the euler angles for reference
  Serial << "Heading: " << event.orientation.x, 4;
  Serial << "\t Roll: " << event.orientation.y, 4;
  Serial << "\t Pitch: " << event.orientation.z, 4;
  Serial << endl;

//  Display calibration status for each sensor.
//  uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);
  
}

void compassTrackHeading() {

  if(!COMPASS_ENABLED) return;

  if(!at_heading) {

    int difference = distanceAngles((int)floor(tracking_heading), (int)floor(compass_heading));
    Serial << "diff: " << difference << " a = " << tracking_heading << " b = " << compass_heading << endl;
    
    if(abs(difference) <= 10) {
      //at_heading = true;

      Serial << "~~~~~~~~~~~~~ forward!" << endl;

      // TODO: go forward
      delay(80);
      
    } else if( difference > 0 ) {
      // turn to the right
      Serial << "~~~~~~~~~~~~~ right! " << endl;

      if(abs(difference) < 30.0) {
        // TODO
//        bowie.motor_setDir(0, MOTOR_DIR_FWD);
//        bowie.motor_setSpeed(0, 120);
//        bowie.motor_setDir(1, MOTOR_DIR_REV);
//        bowie.motor_setSpeed(1, 120);
        delay(20);
      } else {
        // TODO
//        bowie.motor_setDir(0, MOTOR_DIR_FWD);
//        bowie.motor_setSpeed(0, 200);
//        bowie.motor_setDir(1, MOTOR_DIR_REV);
//        bowie.motor_setSpeed(1, 200);
        delay(80);
      }
      
    } else if( difference < 0) {
      // turn to the left
      Serial << "~~~~~~~~~~~~~ left!" << endl;

      if(abs(difference) < 30.0) {
        // TODO
//        bowie.motor_setDir(0, MOTOR_DIR_REV);
//        bowie.motor_setSpeed(0, 120);
//        bowie.motor_setDir(1, MOTOR_DIR_FWD);
//        bowie.motor_setSpeed(1, 120);
        delay(20);
      } else {
        // TODO
//        bowie.motor_setDir(0, MOTOR_DIR_REV);
//        bowie.motor_setSpeed(0, 200);
//        bowie.motor_setDir(1, MOTOR_DIR_FWD);
//        bowie.motor_setSpeed(1, 200);
        delay(80);
      }
      
    }

    
  } else {
    /*
    // uncomment this block for testing mode
    if(BRIGHT_MODE) rainbowCycle(80);
    if(!BRIGHT_MODE) simpleTest(10);
    tracking_heading = random(0,360);
    at_heading = false;
    */
  }
  
}

int distanceAngles(int alpha, int beta) {
  int phi = abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
  int distance = phi > 180 ? 360 - phi : phi;
  //calculate sign 
  int sign = (alpha - beta >= 0 && alpha - beta <= 180) || (alpha - beta <= -180 && alpha - beta >= -360) ? 1 : -1; 
  distance *= sign;
  return distance;
}


