void turnSequence(bool dir) {
  
    // turn a portion
    if(dir) { // right
      bowie.motor_setDir(0, MOTOR_DIR_FWD);
      bowie.motor_setSpeed(0, 255);
      bowie.motor_setDir(1, MOTOR_DIR_REV);
      bowie.motor_setSpeed(1, 255);
    } else {
      bowie.motor_setDir(0, MOTOR_DIR_REV);
      bowie.motor_setSpeed(0, 255);
      bowie.motor_setDir(1, MOTOR_DIR_FWD);
      bowie.motor_setSpeed(1, 255);
    }
    delay(400);
    

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
    delay(50);

    // drive forward a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 255);
    delay(300);

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
    delay(50);

    // drive backward a bit
    bowie.motor_setDir(0, MOTOR_DIR_REV);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_REV);
    bowie.motor_setSpeed(1, 255);
    delay(450);

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
    delay(50);
    
}

void scoopSequenceSlow() {

  int temp_arm_pos = bowie.getArmPos();
  long start_ms = 0;
  long end_ms = 0;
  long total_start_ms = 0;
  long total_end_ms = 0;
  int ARM_DELAY = 3; // usually 3, with 2 servos working
  bowie.unparkHopper();

  total_start_ms = millis();
  
  if(temp_arm_pos < ARM_HOME) { // only dig when arm is down

    // wiggle while digging down
    for(int j=0; j<5; j++) {
      Serial << "Going to END_HOME+200...";
      start_ms = millis();
      for(int i=END_HOME; i<END_HOME+200; i+=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      end_ms = millis();
      Serial << " done " << (end_ms-start_ms) << "ms" << endl;
      delay(80);

      Serial << "Going to END_HOME+500...";
      start_ms = millis();
      for(int i=END_HOME+200; i>END_HOME+700; i-=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      end_ms = millis();
      Serial << " done " << (end_ms-start_ms) << "ms" << endl;
      delay(80);
    }

    // drive forward a bit
    Serial << "Going to MOTORS FWD 255...";
    start_ms = millis();
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
    for(int i=255; i>0; i-=5) {
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
    delay(10);

    // then move scoop parallel to ground
    Serial << "Going to END_PARALLEL_BOTTOM...";
    start_ms = millis();
    for(int i=END_HOME+700; i>END_PARALLEL_BOTTOM-100; i-=50) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(100);

    // drive forward a bit
    Serial << "Going to MOTORS FWD 255...";
    start_ms = millis();
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 255);
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(400);

    // stop motors!
    Serial << "Going to MOTORS FWD 0...";
    start_ms = millis();
    for(int i=255; i>0; i-=5) {
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
    delay(10);

  }

  if(temp_arm_pos < ARM_HOME) { // only do this if it's below ARM_HOME
    // tilt the scoop upwards to avoid losing the items
    Serial << "Going to END_PARALLEL_BOTTOM-500...";
    start_ms = millis();
    for(int i=END_PARALLEL_BOTTOM; i>END_PARALLEL_BOTTOM-500; i-=20) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(100);

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_HOME...";
    start_ms = millis();
    for(int i=temp_arm_pos; i<ARM_HOME; i++) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelValBounds( i, temp_arm_pos, ARM_HOME, END_PARALLEL_BOTTOM-500, END_PARALLEL_BOTTOM-200) );
      delay(ARM_DELAY);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  
    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX...";
    start_ms = millis();
    for(int i=ARM_HOME; i<ARM_MAX; i++) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelValBounds( i, ARM_HOME, ARM_MAX, END_PARALLEL_BOTTOM-200, END_PARALLEL_TOP-100) );
      delay(ARM_DELAY);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(100);
  
  } else { // otherwise, lift it to ARM_MAX from where it's at

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX...";
    start_ms = millis();
    for(int i=temp_arm_pos; i<ARM_MAX; i++) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelValBounds( i, ARM_HOME, ARM_MAX, END_PARALLEL_BOTTOM-200, END_PARALLEL_TOP-100) );
      delay(ARM_DELAY);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(100);
    
  }

  // open lid
  Serial << "Going to LID_MIN...";
  start_ms = millis();
  for(int i=LID_MAX; i>LID_MIN; i-=2) {
    bowie.lid.writeMicroseconds(i);
    delay(1);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(100);

  // dump scoop
  Serial << "Going to END_MIN...";
  start_ms = millis();
  for(int i=END_PARALLEL_TOP; i>END_MIN; i--) {
    bowie.end.writeMicroseconds(i);
    delay(1);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(100);

  // bring scoop back to position
  Serial << "Going to END_PARALLEL_TOP...";
  start_ms = millis();
  for(int i=END_MIN; i<END_PARALLEL_TOP; i++) {
    bowie.end.writeMicroseconds(i);
    delay(1);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(100);

  // close lid
  Serial << "Going to LID_MAX...";
  start_ms = millis();
  for(int i=LID_MIN; i<LID_MAX; i+=2) {
    bowie.lid.writeMicroseconds(i);
    delay(1);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(100);

  // lower arm
  /*
  Serial << "Going to ARM_MIN or temp_arm_pos...";
  start_ms = millis();
  for(int i=ARM_MAX; i>temp_arm_pos; i--) {
    bowie.moveArm(i);
    bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
    delay(ARM_DELAY);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(100);
  */

  Serial << "Parking the arm" << endl;
  start_ms = millis();
  bowie.parkArm();
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  delay(100);

  total_end_ms = millis();

  Serial << "--------- Sequence complete in " << total_end_ms-total_start_ms << " ms \n\n";

  bowie.parkHopper();
    
}


void scoopSequenceFast() {

  int temp_arm_pos = bowie.getArmPos();
  long start_ms = 0;
  long end_ms = 0;
  long total_start_ms = 0;
  long total_end_ms = 0;
  bowie.unparkHopper();

  total_start_ms = millis();
  
  if(temp_arm_pos < ARM_HOME) { // only dig when arm is down

    // wiggle while digging down
    for(int j=0; j<5; j++) {
      Serial << "Going to END_HOME+100...";
      start_ms = millis();
      for(int i=END_HOME; i<END_HOME+100; i+=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      end_ms = millis();
      Serial << " done " << (end_ms-start_ms) << "ms" << endl;
      delay(80);

      Serial << "Going to END_HOME+500...";
      start_ms = millis();
      for(int i=END_HOME+100; i>END_HOME+500; i-=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      end_ms = millis();
      Serial << " done " << (end_ms-start_ms) << "ms" << endl;
      delay(80);
    }

    // drive forward a bit
    Serial << "Going to MOTORS FWD 255...";
    start_ms = millis();
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
    for(int i=255; i>0; i-=5) {
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
    
    // then move scoop parallel to ground
    Serial << "Going to END_PARALLEL_BOTTOM...";
    start_ms = millis();
    for(int i=END_HOME+500; i>END_PARALLEL_BOTTOM-100; i-=50) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(20);

    // drive forward a bit
    Serial << "Going to MOTORS FWD 255...";
    start_ms = millis();
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 255);
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(400);

    // stop motors!
    Serial << "Going to MOTORS FWD 0...";
    start_ms = millis();
    for(int i=255; i>0; i-=5) {
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
    
  }

  if(temp_arm_pos < ARM_HOME) { // only do this if it's below ARM_HOME
    // tilt the scoop upwards to avoid losing the items
    Serial << "Going to END_PARALLEL_BOTTOM-500...";
    start_ms = millis();
    for(int i=END_PARALLEL_BOTTOM; i>END_PARALLEL_BOTTOM-500; i-=20) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    delay(20);

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_HOME...";
    start_ms = millis();
    for(int i=temp_arm_pos; i<ARM_HOME; i+=10) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelValBounds( i, temp_arm_pos, ARM_HOME, END_PARALLEL_BOTTOM-500, END_PARALLEL_BOTTOM-200) );
      delay(3);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    
    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX...";
    start_ms = millis();
    for(int i=ARM_HOME; i<ARM_MAX; i+=5) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelValBounds( i, ARM_HOME, ARM_MAX, END_PARALLEL_BOTTOM-200, END_PARALLEL_TOP-100) );
      delay(3);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    //delay(20);

  } else { // otherwise, lift it to ARM_MAX from where it's at

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX...";
    start_ms = millis();
    for(int i=temp_arm_pos; i<ARM_MAX; i+=5) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelValBounds( i, ARM_HOME, ARM_MAX, END_PARALLEL_BOTTOM-200, END_PARALLEL_TOP-100) );
      delay(3);
    }
    end_ms = millis();
    Serial << " done " << (end_ms-start_ms) << "ms" << endl;
    //delay(20);
    
  }

  // open lid
  Serial << "Going to LID_MIN...";
  start_ms = millis();
  for(int i=LID_MAX; i>LID_MIN; i-=10) {
    bowie.lid.writeMicroseconds(i);
    delay(1);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  // dump scoop
  Serial << "Going to END_MIN...";
  start_ms = millis();
  for(int i=END_PARALLEL_TOP; i>END_MIN; i-=8) {
    bowie.end.writeMicroseconds(i);
    delay(3);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  // bring scoop back to position
  Serial << "Going to END_PARALLEL_TOP...";
  start_ms = millis();
  for(int i=END_MIN; i<END_PARALLEL_TOP; i+=10) {
    bowie.end.writeMicroseconds(i);
    delay(1);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  // close lid
  Serial << "Going to LID_MAX...";
  start_ms = millis();
  for(int i=LID_MIN; i<LID_MAX; i+=10) {
    bowie.lid.writeMicroseconds(i);
    delay(1);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  // lower arm
  Serial << "Going to ARM_MIN or temp_arm_pos...";
  start_ms = millis();
  for(int i=ARM_MAX; i>temp_arm_pos; i-=5) {
    bowie.moveArm(i);
    //TODO bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
    delay(3);
  }
  end_ms = millis();
  Serial << " done " << (end_ms-start_ms) << "ms" << endl;
  //delay(20);

  total_end_ms = millis();

  Serial << "--------- Sequence complete in " << total_end_ms-total_start_ms << " ms \n\n";

  bowie.parkHopper();
    
}


void deposit() {

  bowie.unparkHopper();

  // if the arm is up let's move it to home
  int temp_arm = bowie.getArmPos();
  if(temp_arm > 2000) {
    for(int i=temp_arm; i>ARM_HOME; i-=5) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(3);
    }
  }

  // open lid
  Serial << "Going to LID_MIN";
  for(int i=LID_MAX; i>LID_MIN; i-=5) {
    bowie.lid.writeMicroseconds(i);
    delay(1);
  }
  Serial << " done" << endl;
  delay(10);

  Serial << "Going to TILT_MIN";
  for(int i=TILT_MAX; i>TILT_MIN; i--) {
    bowie.tilt.writeMicroseconds(i);
    delay(3);
  }
  Serial << " done" << endl;
  delay(100);

  delay(1000);

  Serial << "Going to TILT_MAX";
  for(int i=TILT_MIN; i<TILT_MAX; i++) {
    bowie.tilt.writeMicroseconds(i);
    delay(3);
  }
  Serial << " done" << endl;
  delay(100);

  // close lid
  Serial << "Going to LID_MAX";
  for(int i=LID_MIN; i<LID_MAX; i+=5) {
    bowie.lid.writeMicroseconds(i);
    delay(1);
  }
  Serial << " done" << endl;
  delay(10);

  // moving the arm back
  if(temp_arm > 2000) {
    for(int i=ARM_HOME; i<temp_arm; i+=5) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
  }

  bowie.parkHopper();
  
}


void homePositions() {
  // yes it is recommended to be in this order!
  bowie.moveLid(LID_MAX);
  bowie.moveHopper(TILT_MAX);
  bowie.moveArm(ARM_HOME);
  bowie.moveEnd(END_HOME);
}

void dance() {

  bowie.unparkHopper();

  // -- go forwards
  
  // drive forward a bit
  Serial << "Going to MOTORS FWD 255";
  bowie.motor_setDir(0, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(0, 255);
  bowie.motor_setDir(1, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(1, 255);
  Serial << " done" << endl;
  delay(200);

  // stop motors!
  Serial << "Going to MOTORS FWD 0";
  for(int i=255; i>0; i-=5) {
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
  Serial << " done" << endl;
  delay(10);
    
  // -- tap lid 3 times
  for(int j=0; j<3; j++) {
    
    // open lid a peek
    Serial << "Going to LID_MAX-500";
    for(int i=LID_MAX; i>(LID_MAX-500); i-=5) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);
  
    // close lid a peek
    Serial << "Going to LID_MAX";
    for(int i=(LID_MAX-500); i<LID_MAX; i+=5) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

  }

  // -- go backwards
  
  Serial << "Going to MOTORS FWD 192";
  bowie.motor_setDir(0, MOTOR_DIR_REV);
  bowie.motor_setSpeed(0, 192);
  bowie.motor_setDir(1, MOTOR_DIR_REV);
  bowie.motor_setSpeed(1, 192);
  Serial << " done" << endl;
  delay(200);

  // stop motors!
  Serial << "Going to MOTORS FWD 0";
  for(int i=192; i>0; i-=5) {
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
  Serial << " done" << endl;
  delay(10);

  // -- wave end effector
  
  // put the arm to the middle
  temp_arm_dance = bowie.getArmPos();
  if(temp_arm_dance < ARM_HOME) { // has to go up
    for(int i=temp_arm_dance; i<ARM_HOME; i++) {
      bowie.moveArm(i);  
      //TODO bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(2);
    }
  } else if(temp_arm_dance > ARM_HOME) { // has to go down
    for(int i=temp_arm_dance; i>ARM_HOME; i--) {
      bowie.moveArm(i);
      //TODO bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(2);
    }
  }

  // now wave
  int temp_end_pos = 0;//TODO bowie.clawParallelVal(ARM_HOME);
  for(int i=0; i<4; i++) {
    bowie.end.writeMicroseconds(temp_end_pos+200);
    delay(250);
    bowie.end.writeMicroseconds(temp_end_pos-200);
    delay(250);
  }

  // move arm back
  if(temp_arm_dance < ARM_HOME) { // has to move down
    for(int i=ARM_HOME; i>temp_arm_dance; i--) {
      bowie.moveArm(i);  
      //TODO bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(2);
    }
  } else if(temp_arm_dance > ARM_HOME) { // has to move up
    for(int i=ARM_HOME; i<temp_arm_dance; i++) {
      bowie.moveArm(i);  
      //TODO bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(2);
    }
  }

  bowie.parkHopper();

  // fin. --
  
}


void stopDance() {

  Serial << "stop dance does get called " << temp_arm_dance  << endl;

  /*
  // move arm back to position
  if(temp_arm_dance < ARM_HOME) { // has to move down
    for(int i=ARM_HOME; i>temp_arm_dance; i--) {
      bowie.moveArm(i);  
      bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(1);
    }
  } else if(temp_arm_dance > ARM_HOME) { // has to move up
    for(int i=ARM_HOME; i<temp_arm_dance; i++) {
      bowie.moveArm(i);  
      bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(1);
    }
  }
  */
  
}


