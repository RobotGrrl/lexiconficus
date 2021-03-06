void scoopSequenceFast2() {

  bool DEBUGGING_ANIMATION = false;

  int temp_arm_pos = bowie.getArmPos();
  long start_ms = 0;
  long end_ms = 0;
  long total_start_ms = 0;
  long total_end_ms = 0;
  bool was_hopper_parked = bowie.getHopperParked();
  bool was_lid_parked = bowie.getLidParked();

  
  /*
  // -- calibrate the touchdown location
  // send end pos before arm
  bowie.moveEnd(END_MIN+200, 1, 1);
  
  // lift arm a bit
  bowie.moveArm(ARM_MIN+200, 1, 3);
  
  // move backwards a bit first
  bowie.motor_setDir(0, MOTOR_DIR_REV);
  bowie.motor_setSpeed(0, 255);
  bowie.motor_setDir(1, MOTOR_DIR_REV);
  bowie.motor_setSpeed(1, 255);
  delay(250);

  bowie.motor_setDir(0, MOTOR_DIR_REV);
  bowie.motor_setSpeed(0, 0);
  bowie.motor_setDir(1, MOTOR_DIR_REV);
  bowie.motor_setSpeed(1, 0);
  delay(10);
  
  // now calibrate
  calibrateTouchdown();

  // lift arm a bit
  bowie.moveArm(ARM_MIN+200, 1, 3);

  // move forwards again
  bowie.motor_setDir(0, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(0, 255);
  bowie.motor_setDir(1, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(1, 255);
  delay(150);

  bowie.motor_setDir(0, MOTOR_DIR_REV);
  bowie.motor_setSpeed(0, 0);
  bowie.motor_setDir(1, MOTOR_DIR_REV);
  bowie.motor_setSpeed(1, 0);
  delay(10);
  // --
  
  if(DEBUGGING_ANIMATION) delay(3000);
  */

  total_start_ms = millis();

  bowie.moveEnd(bowie.END_TOUCHDOWN, 1, 1);

  if(DEBUGGING_ANIMATION) delay(3000);

  if(was_hopper_parked) bowie.unparkHopper();
  if(was_lid_parked) bowie.unparkLid();
  
  // move the arm down a bit
  bowie.moveArm(ARM_MIN+100, 1, 3);

  if(DEBUGGING_ANIMATION) delay(3000);

  // -- digging

  // wiggle while digging down
  bowie.moveEnd(END_HOME);
  for(int j=0; j<5; j++) {
    Serial << "Going to END_HOME+100...";
    start_ms = millis();
    bowie.moveEnd(END_HOME+100, 2, 1);
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(80);

    Serial << "Going to END_HOME+300...";
    start_ms = millis();
    bowie.moveEnd(END_HOME+300, 2, 1);
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(80);
  }

  if(DEBUGGING_ANIMATION) delay(3000);

  
  // drive forward a bit to dig inwards a bit
  Serial << "Going to MOTORS FWD 255...";
  start_ms = millis();
  for(int i=100; i<255; i+=20) {
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, i);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, i);  
    delay(10);
  }
  bowie.motor_setDir(0, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(0, 255);
  bowie.motor_setDir(1, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(1, 255);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(200);

  // stop motors!
  Serial << "Going to MOTORS FWD 0...";
  start_ms = millis();
  for(int i=255; i>100; i-=10) {
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, i);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, i);
    delay(5);  
  }
  bowie.motor_setDir(0, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(0, 0);
  bowie.motor_setDir(1, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(1, 0);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;

  if(DEBUGGING_ANIMATION) delay(3000);
  
  // move arm down completely
  bowie.moveArm(ARM_MIN, 1, 3);

  if(DEBUGGING_ANIMATION) delay(3000);

  // go back to ground position
  Serial << "Going to END_TOUCHDOWN+100...";
  start_ms = millis();
  bowie.OVERRIDE_CHECK = true;
  bowie.moveEnd(bowie.END_TOUCHDOWN, 2, 1);
  bowie.OVERRIDE_CHECK = false;
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  
  if(DEBUGGING_ANIMATION) delay(3000);

  // drive forward a bit
  Serial << "Going to MOTORS FWD 255...";
  start_ms = millis();
  for(int i=100; i<255; i+=20) {
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, i);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, i);  
    delay(10);
  }
  bowie.motor_setDir(0, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(0, 255);
  bowie.motor_setDir(1, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(1, 255);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(1000);

  // stop motors!
  Serial << "Going to MOTORS FWD 0...";
  start_ms = millis();
  for(int i=255; i>100; i-=10) {
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, i);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, i);
    delay(5);  
  }
  bowie.motor_setDir(0, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(0, 0);
  bowie.motor_setDir(1, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(1, 0);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;

  if(DEBUGGING_ANIMATION) delay(3000);
  
  // -- end digging

  // move arm up a bit
  bowie.moveArm(ARM_MIN+100, 1, 3);

  if(DEBUGGING_ANIMATION) delay(3000);

  // tilt the scoop upwards to avoid losing the items
  Serial << "Going to END_PARALLEL_BOTTOM-700...";
  start_ms = millis();
  bowie.moveEnd(END_PARALLEL_BOTTOM-700, 1, 3); // todo - check this again, its less than end min
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(20);

  if(DEBUGGING_ANIMATION) delay(3000);

  // lift arm with scoop parallel to ground
  Serial << "Going to ARM_HOME...";
  start_ms = millis();
  bowie.moveArmAndEnd(ARM_HOME, 5, 3, ARM_MIN, ARM_HOME, END_PARALLEL_BOTTOM-700, END_HOME-200);//END_PARALLEL_BOTTOM-400);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;

  if(DEBUGGING_ANIMATION) delay(3000);
  
  // lift arm with scoop parallel to ground
  Serial << "Going to ARM_MAX...";
  start_ms = millis();
  bowie.moveArmAndEnd(ARM_MAX, 5, 3, ARM_HOME, ARM_MAX, END_HOME-200, END_PARALLEL_TOP-100);//END_PARALLEL_BOTTOM-400, END_PARALLEL_TOP-100);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  if(DEBUGGING_ANIMATION) delay(3000);

  // open lid
  Serial << "Going to LID_MIN...";
  start_ms = millis();
  bowie.moveLid(LID_MIN, 5, 1);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  if(DEBUGGING_ANIMATION) delay(3000);

  // dump scoop
  Serial << "Going to END_MIN...";
  start_ms = millis();
  bowie.moveEnd(END_MIN, 8, 3);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  if(DEBUGGING_ANIMATION) delay(3000);

  // bring scoop back to position
  Serial << "Going to END_PARALLEL_TOP...";
  start_ms = millis();
  bowie.moveEnd(END_PARALLEL_TOP, 8, 3);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  if(DEBUGGING_ANIMATION) delay(3000);

  // close lid
  Serial << "Going to LID_MAX...";
  start_ms = millis();
  bowie.moveLid(LID_MAX, 10, 1);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  if(DEBUGGING_ANIMATION) delay(3000);

  // park arm
  Serial << "Parking arm...";
  start_ms = millis();
  bowie.parkArm();
  bowie.parkEnd();  
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;

  if(DEBUGGING_ANIMATION) delay(3000);

  // lower arm
  /*
  Serial << "Going to ARM_MIN or temp_arm_pos...";
  start_ms = millis();
  bowie.moveArmAndEnd(ARM_MIN, 4, 3, ARM_MIN, ARM_MAX, END_PARALLEL_BOTTOM, END_PARALLEL_TOP);
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  if(DEBUGGING_ANIMATION) delay(3000);
  */

  total_end_ms = millis();

  Serial << "--------- Sequence complete in " << total_end_ms-total_start_ms << " ms \n\n";

  if(was_hopper_parked) bowie.parkHopper();
  if(was_lid_parked) bowie.parkLid();
  
}
