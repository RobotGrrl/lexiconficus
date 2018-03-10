void compassInit() {

  // TODO - write retry code
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_COMPASS)) {
    Serial.print("Oops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    COMPASS_ENABLED = false;
    compassNeopixelsInit(20, 0, 0);
  }

  if(COMPASS_ENABLED) {
    // calibration data configured from the BNO055_Calibration sketch
    byte c_data[22] = {0, 0, 0, 0, 0, 0, 188, 254, 244, 0, 116, 255, 0, 0, 0, 0, 0, 0, 232, 3, 44, 3};
    bno.setCalibData(c_data);
    
    delay(1000);
    bno.setExtCrystalUse(true);
    delay(20);
    compassNeopixelsInit(0, 20, 0);
  }

  emptyPixels();
  
}

void emptyPixels() {
  for(int i=0; i<NUMPIXELS; i++) {
    pixel_colours[i] = 0;
    pixel_states[i] = 0;
  }
}

void compassUpdate() {

  if(!COMPASS_ENABLED) return;
  
  sensors_event_t event;
  bno.getEvent(&event);

  compass_heading = event.orientation.x;
  compass_roll = event.orientation.y;
  compass_pitch = event.orientation.z;

  // print the euler angles for reference
  if(current_time-last_imu_print >= 1000) {
    Serial << "HEADING: " << event.orientation.x, 4;
    Serial << "\t ROLL: " << event.orientation.y, 4;
    Serial << "\t PITCH: " << event.orientation.z, 4;
    Serial << endl;
    last_imu_print = current_time;
  }

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

void navigateHeadingDirection() {

  if(!COMPASS_ENABLED) return;

  PREV_DRIVE_STATE = DRIVE_STATE;

  // calculate the difference in angle
  int difference = distanceAngles((int)floor(tracking_heading), (int)floor(compass_heading));
  if(current_time-last_diff_print >= 100) {
    Serial << "### DIFF: " << difference << " TRACK = " << tracking_heading << " COMPASS = " << compass_heading << endl;
    last_diff_print = current_time;
  }

  double THRESH_DEG = 50;
  
  if(abs(difference) <= THRESH_DEG) {
    // go forward
    DRIVE_STATE = FORWARD;
    at_heading = true;
    
  } else if(difference > 0) {
    // turn to the right
    if(abs(difference) < THRESH_DEG+20.0) {
      DRIVE_STATE = RIGHT;
    } else {
      DRIVE_STATE = HARD_RIGHT;
    }
    
  } else if(difference < 0) {
    // turn to the left
    if(abs(difference) < THRESH_DEG+20.0) {
      DRIVE_STATE = LEFT;
      delay(20);
    } else {
      DRIVE_STATE = HARD_LEFT;
    }
    
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


