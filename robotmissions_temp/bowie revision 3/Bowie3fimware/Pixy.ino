/*
 * Reminder: changed PixyUART.h to use Serial1
 * 
 * pixy colour codes
 * 1 = pink = follow
 * 2 = green = turn left x4
 * 3 = blue = turn right x4
 * 4 = orange = forward
 * 5 = yellow = scoop
 * 6 = white = dance mode
 */

void pixyUpdate() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  blocks = pixy.getBlocks();
  
  if (blocks) {
    i++;

   for (j=0; j<blocks; j++) {
     checkBlock(pixy.blocks[j]);
   }
    
   // do this (print) every 50 frames because printing every
   // frame would bog down the Arduino
   if (i%10==0) {
      //sprintf(buf, "Detected %d:\n", blocks);
      //Serial.print(buf);
      for (j=0; j<blocks; j++) {
        //sprintf(buf, "  block %d: ", j);
        //Serial.print(buf); 
        pixy.blocks[j].print();
      }
   }
   
  } 
}

void checkBlock(Block block) {

  if(!currently_locked) pixy.setLED(0,0,0); // sometimes the light messes up the detection
  
    if(waiting_to_lock == false && currently_locked == false) { // let's look in to this one...
      if(block.width >= 30 && block.height >= 30) { // if it's big enough, we'll check it
        Serial << "Going to wait" << endl;
        detected_signature = block.signature;
        first_notice = current_time;
        waiting_to_lock = true;
      }
    }
    
    if(waiting_to_lock == true && currently_locked == false) { // waiting to lock on the target (make sure it doesn't go away)
      if(block.signature == detected_signature && block.width >= 30 && block.height >= 30) {
        num_detects++;
        Serial << "Detects: " << num_detects << endl;
        if(PIXY_LED_ACTIVE) pixy.setLED(200,200,200);
        Serial << width_avg << endl;
        width_avg += block.width;
        height_avg += block.height;
        if(num_detects >= 30) { // 1 seconds and 40 detects for it to be locked
          currently_locked = true;
          width_avg = width_avg/(float)num_detects;
          height_avg = height_avg/(float)num_detects;
          Serial << "Going to track W: " << width_avg << " H: " << height_avg << endl;
          num_detects = 0;
          waiting_to_lock = false;
        } 
        if(current_time-first_notice >= 500 && num_detects < 10 && currently_locked == false) { // start over
          Serial << "Start over 1" << endl;
          if(PIXY_LED_ACTIVE) pixy.setLED(0,0,0);
          width_avg = 0;
          height_avg = 0;
          num_detects = 0;
          waiting_to_lock = false;
        } else if(current_time-first_notice >= 3000 && currently_locked == false) {
          Serial << "Start over 2" << endl;
          if(PIXY_LED_ACTIVE) pixy.setLED(0,0,0);
          width_avg = 0;
          height_avg = 0;
          num_detects = 0;
          waiting_to_lock = false;
        }
        Serial << current_time-first_notice << endl;
      }
    }

    // note- ignoring white (sig 6)
    if(currently_locked == true && block.signature == detected_signature && block.signature != 6) {
      if(block.width >= 30 && block.height >= 30) {
        Serial << "Tracking";
        last_lock_detect = current_time;

          if(block.signature == 1) {
            Serial << " [1] PINK" << endl;
            trackingSig1(block);
          } else if(block.signature == 2) {
            Serial << " [2] GREEN" << endl;
            trackingSig2(block);
          } else if(block.signature == 3) {
            Serial << " [3] BLUE" << endl;
            trackingSig3(block);
          } else if(block.signature == 4) {
            Serial << " [4] ORANGE" << endl;
            trackingSig4(block);
          } else if(block.signature == 5) {
            Serial << " [5] YELLOW" << endl;
            //trackingSig5(block);
          } else if(block.signature == 6) {
            Serial << " [6] WHITE" << endl;
            trackingSig6(block);
          }
         
      }
       
    } else if(currently_locked == true) {

      if(current_time-last_lock_detect >= 200) {
        Serial << "Preventitively stopping the motors" << endl;
        // make sure to stop the motors
        bowie.motor_setDir(0, MOTOR_DIR_FWD);
        bowie.motor_setSpeed(0, 0);
        bowie.motor_setDir(1, MOTOR_DIR_FWD);
        bowie.motor_setSpeed(1, 0);
      }
      
      if(current_time-last_lock_detect >= 2000) { // we lost the tracking
        Serial << "Lost tracking" << endl;
        if(PIXY_LED_ACTIVE) pixy.setLED(0,0,0);
        currently_locked = false;
        // reset the vars for turning
        bowie.resetTurnSequence();
      }
    }
  
}


