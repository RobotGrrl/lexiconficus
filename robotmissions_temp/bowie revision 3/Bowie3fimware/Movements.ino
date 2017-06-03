void scoopSequenceSlow() {

  // wiggle while digging down
    for(int j=0; j<5; j++) {
      Serial << "Going to END_HOME+100";
      for(int i=END_HOME; i<END_HOME+100; i+=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);

      Serial << "Going to END_HOME";
      for(int i=END_HOME+100; i>END_HOME; i-=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);
    }

    // then move scoop parallel to ground
    Serial << "Going to END_PARALLEL_BOTTOM";
    for(int i=END_HOME; i>END_PARALLEL_BOTTOM; i-=50) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    Serial << " done" << endl;
    delay(100);

    // drive forward a bit
    Serial << "Going to MOTORS FWD 192";
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 192);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 192);
    Serial << " done" << endl;
    delay(100);

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

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX";
    for(int i=ARM_MIN; i<ARM_MAX; i++) {
      bowie.arm.writeMicroseconds(i);
      bowie.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // open lid
    Serial << "Going to LID_MIN";
    for(int i=LID_MAX; i>LID_MIN; i--) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // dump scoop
    Serial << "Going to END_MIN";
    for(int i=END_PARALLEL_TOP; i>END_MIN; i--) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // bring scoop back to position
    Serial << "Going to END_PARALLEL_TOP";
    for(int i=END_MIN; i<END_PARALLEL_TOP; i++) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // close lid
    Serial << "Going to LID_MAX";
    for(int i=LID_MIN; i<LID_MAX; i++) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // lower arm
    Serial << "Going to ARM_MIN";
    for(int i=ARM_MAX; i>ARM_MIN; i--) {
      bowie.arm.writeMicroseconds(i);
      bowie.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);
    
}

void scoopSequenceFast() {
  
    // wiggle while digging down
    for(int j=0; j<5; j++) {
      Serial << "Going to END_HOME+100";
      for(int i=END_HOME; i<END_HOME+100; i+=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);

      Serial << "Going to END_HOME";
      for(int i=END_HOME+100; i>END_HOME; i-=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);
    }

    // then move scoop parallel to ground
    Serial << "Going to END_PARALLEL_BOTTOM";
    for(int i=END_HOME; i>END_PARALLEL_BOTTOM; i-=50) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    Serial << " done" << endl;
    delay(10);

    // drive forward a bit
    Serial << "Going to MOTORS FWD 192";
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 192);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 192);
    Serial << " done" << endl;
    delay(100);

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

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX";
    for(int i=ARM_MIN; i<ARM_MAX; i+=2) {
      bowie.arm.writeMicroseconds(i);
      bowie.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // open lid
    Serial << "Going to LID_MIN";
    for(int i=LID_MAX; i>LID_MIN; i-=5) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // dump scoop
    Serial << "Going to END_MIN";
    for(int i=END_PARALLEL_TOP; i>END_MIN; i-=2) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // bring scoop back to position
    Serial << "Going to END_PARALLEL_TOP";
    for(int i=END_MIN; i<END_PARALLEL_TOP; i+=5) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // close lid
    Serial << "Going to LID_MAX";
    for(int i=LID_MIN; i<LID_MAX; i+=5) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // lower arm
    Serial << "Going to ARM_MIN";
    for(int i=ARM_MAX; i>ARM_MIN; i-=2) {
      bowie.arm.writeMicroseconds(i);
      bowie.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

}



void scoopSequenceSlowB() {

  int temp_arm_pos = bowie.getArmPos();

  if(temp_arm_pos < ARM_HOME) { // only dig when arm is down

  // wiggle while digging down
    for(int j=0; j<5; j++) {
      Serial << "Going to END_HOME+100";
      for(int i=END_HOME; i<END_HOME+100; i+=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);

      Serial << "Going to END_HOME";
      for(int i=END_HOME+100; i>END_HOME; i-=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);
    }

    // then move scoop parallel to ground
    Serial << "Going to END_PARALLEL_BOTTOM";
    for(int i=END_HOME; i>END_PARALLEL_BOTTOM; i-=50) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    Serial << " done" << endl;
    delay(100);

    // drive forward a bit
    Serial << "Going to MOTORS FWD 192";
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 192);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 192);
    Serial << " done" << endl;
    delay(100);

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

  }

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX";
    for(int i=temp_arm_pos; i<ARM_MAX; i++) {
      bowie.moveArm(i);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // open lid
    Serial << "Going to LID_MIN";
    for(int i=LID_MAX; i>LID_MIN; i--) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // dump scoop
    Serial << "Going to END_MIN";
    for(int i=END_PARALLEL_TOP; i>END_MIN; i--) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // bring scoop back to position
    Serial << "Going to END_PARALLEL_TOP";
    for(int i=END_MIN; i<END_PARALLEL_TOP; i++) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // close lid
    Serial << "Going to LID_MAX";
    for(int i=LID_MIN; i<LID_MAX; i++) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);

    // lower arm
    Serial << "Going to ARM_MIN";
    for(int i=ARM_MAX; i>temp_arm_pos; i--) {
      bowie.moveArm(i);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(100);
    
}



void scoopSequenceFastB() {

  int temp_arm_pos = bowie.getArmPos();

  if(temp_arm_pos < ARM_HOME) { // only dig when arm is down
    
    // wiggle while digging down
    for(int j=0; j<5; j++) {
      Serial << "Going to END_HOME+100";
      for(int i=END_HOME; i<END_HOME+100; i+=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);

      Serial << "Going to END_HOME";
      for(int i=END_HOME+100; i>END_HOME; i-=20) {
        bowie.end.writeMicroseconds( i );
        delay(5);
      }
      Serial << " done" << endl;
      delay(80);
    }

    // then move scoop parallel to ground
    Serial << "Going to END_PARALLEL_BOTTOM";
    for(int i=END_HOME; i>END_PARALLEL_BOTTOM; i-=50) {
      bowie.end.writeMicroseconds( i );
      delay(5);
    }
    Serial << " done" << endl;
    delay(10);

    // drive forward a bit
    Serial << "Going to MOTORS FWD 192";
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 192);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 192);
    Serial << " done" << endl;
    delay(100);

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

  }

    // lift arm with scoop parallel to ground
    Serial << "Going to ARM_MAX";
    for(int i=temp_arm_pos; i<ARM_MAX; i+=2) {
      bowie.moveArm(i);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // open lid
    Serial << "Going to LID_MIN";
    for(int i=LID_MAX; i>LID_MIN; i-=5) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // dump scoop
    Serial << "Going to END_MIN";
    for(int i=END_PARALLEL_TOP; i>END_MIN; i-=2) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // bring scoop back to position
    Serial << "Going to END_PARALLEL_TOP";
    for(int i=END_MIN; i<END_PARALLEL_TOP; i+=5) {
      bowie.end.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // close lid
    Serial << "Going to LID_MAX";
    for(int i=LID_MIN; i<LID_MAX; i+=5) {
      bowie.lid.writeMicroseconds(i);
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

    // lower arm
    Serial << "Going to ARM_MIN";
    for(int i=ARM_MAX; i>temp_arm_pos; i-=2) {
      bowie.moveArm(i);
      bowie.end.writeMicroseconds( bowie.clawParallelVal(i) );
      delay(1);
    }
    Serial << " done" << endl;
    delay(10);

}




void deposit() {

  // if the arm is up let's move it to home
  int temp_arm = bowie.getArmPos();
  if(temp_arm > 2000) {
    for(int i=temp_arm; i>ARM_HOME; i-=5) {
      bowie.moveArm(i);
      delay(1);
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

  delay(2000);

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
      delay(1);
    }
  }
  
}


void homePositions() {

  bowie.moveArm(ARM_MIN);
  bowie.end.writeMicroseconds(END_HOME);
  bowie.tilt.writeMicroseconds(TILT_MAX);
  bowie.lid.writeMicroseconds(LID_MAX);
  
}

void dance() {

  // -- go forwards
  
  // drive forward a bit
  Serial << "Going to MOTORS FWD 192";
  bowie.motor_setDir(0, MOTOR_DIR_FWD);
  bowie.motor_setSpeed(0, 192);
  bowie.motor_setDir(1, MOTOR_DIR_FWD);
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
      bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(1);
    }
  } else if(temp_arm_dance > ARM_HOME) { // has to go down
    for(int i=temp_arm_dance; i>ARM_HOME; i--) {
      bowie.moveArm(i);
      bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(1);
    }
  }

  // now wave
  int temp_end_pos = bowie.clawParallelVal(ARM_HOME);
  for(int i=0; i<4; i++) {
    bowie.end.writeMicroseconds(temp_end_pos+200);
    delay(250);
    bowie.end.writeMicroseconds(temp_end_pos-200);
    delay(250);
  }

  // fin. --
  
}


void stopDance() {

  // move arm back to position
  if(ARM_HOME < temp_arm_dance) { // has to go up
    for(int i=ARM_HOME; i<temp_arm_dance; i++) {
      bowie.moveArm(i);  
      bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(1);
    }
  } else if(ARM_HOME > temp_arm_dance) { // has to go down
    for(int i=ARM_HOME; i>temp_arm_dance; i--) {
      bowie.moveArm(i);  
      bowie.end.writeMicroseconds(bowie.clawParallelVal(i));
      delay(1);
    }
  }
  
}


