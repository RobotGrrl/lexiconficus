
void trackingSig1(Block block) {

  // pink - follow

  if(PIXY_LED_ACTIVE) pixy.setLED(255,20,20);

  if(block.x > (160+20)) { // go left
  
    Serial << "Left" << endl;

    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_REV);
    bowie.motor_setSpeed(1, 255);
    delay(20);

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
    
  } else if(block.x < (160-20)) { // go right

    Serial << "Right" << endl;

    bowie.motor_setDir(0, MOTOR_DIR_REV);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 255);
    delay(20);

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
    
  } else {

    Serial << "W: " << block.width << " W avg: " << width_avg << endl;
    Serial << "H: " << block.height << " H avg: " << height_avg << endl;

    if(block.width > width_avg+2) {//width_avg+2 && block.height > height_avg+2) {
    // go backward
    Serial << "Go backward" << endl;
    bowie.motor_setDir(0, MOTOR_DIR_REV);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_REV);
    bowie.motor_setSpeed(1, 255);
    delay(20);

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
    
  } else if(block.width < width_avg-5) {//width_avg+5 && block.height < height_avg+5) {
    // go forward
    Serial << "Go forward" << endl;
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 255);
    delay(20);

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
  }

  }

}

void trackingSig2(Block block) {

  // green - turn left x4

  if(PIXY_LED_ACTIVE) pixy.setLED(0,255,0);

  //for(int i=0; i<3; i++) {
    // turn a portion
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_REV);
    bowie.motor_setSpeed(1, 255);
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
  //}

  currently_locked = false; // force this, since it is turning
  
}

void trackingSig3(Block block) {

  // blue - turn right x4

  if(PIXY_LED_ACTIVE) pixy.setLED(0,0,255);

  //for(int i=0; i<3; i++) {
    // turn a portion
    bowie.motor_setDir(0, MOTOR_DIR_REV);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 255);
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
  //}

  currently_locked = false; // force this, since it is turning
  
}

void trackingSig4(Block block) {

    // orange - forward

    if(PIXY_LED_ACTIVE) pixy.setLED(200,100,30);

    // drive forward a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 255);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 255);
    delay(3000);

    // stop a bit
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 0);
    delay(50);
  
}

void trackingSig5(Block block) {

  // yellow - scooop

  if(PIXY_LED_ACTIVE) pixy.setLED(100,100,30);

  scoopSequenceSlow();
  
}

void trackingSig6(Block block) {

  // white - dance mode

  if(PIXY_LED_ACTIVE) pixy.setLED(200,200,220);

  dance();
  
}



