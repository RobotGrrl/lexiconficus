/*
 * Robot Missions Bowie Revision 3 Firmware
 * ----------------------------------------
 * 
 * May 26, 2017
 * Erin RobotGrrl for RobotMissions.org
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 */

#include "Bowie3.h"
#include <Streaming.h>
#include <XBee.h>
#include "PromulgateBig.h"
#include "PixyUART.h"

#define PROG_DEBUG false
#define COMM_DEBUG false

// ---- Promulgate
Promulgate promulgate = Promulgate(&Serial2, &Serial2);
unsigned long current_time = 0;
unsigned long diff_time = 0;
unsigned long last_rx = 0;
unsigned long last_transmit = 0;

void transmit_complete();
void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);

// ---- Xbee variables
#define XBEE_CONTROLLER_DH 0x0013A200
#define XBEE_CONTROLLER_DL 0x40DD9902
#define XBEE_ROBOT_DH 0x0013A200
#define XBEE_ROBOT_DL 0x40D96FC2

XBee xbee = XBee();
XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
XBeeAddress64 addr_controller = XBeeAddress64(XBEE_CONTROLLER_DH, XBEE_CONTROLLER_DL);
XBeeAddress64 addr_robot = XBeeAddress64(XBEE_ROBOT_DH, XBEE_ROBOT_DL);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ZBRxResponse rx = ZBRxResponse();
char message_tx[64];
char message_rx[64];
uint32_t msg_tx_count = 0;
uint32_t msg_rx_count = 0;
uint32_t msg_tx_err = 0;

// ---- Bowie
Bowie bowie = Bowie();

// ---- Messages
Msg msg_a = { true, '@', 'B', 0, 0, 'Z', 5, 5555, '!' };
Msg msg_b = { true, '@', 'B', 1, 1, 'Z', 0, 0, '!' };

bool a_or_b = false;
boolean light_on = false;

// ---- User-Defined Actions
boolean did_scoop = false;
boolean did_empty = false;
int temp_arm_dance = 0;

// ---- Pixy
PixyUART pixy;
bool waiting_to_lock = false;
bool currently_locked = false;
uint16_t detected_signature = 99;
long first_notice = 0;
uint16_t num_detects = 0;
long last_lock_detect = 0;
float width_avg = 0.0;
float height_avg = 0.0;
#define PIXY_LED_ACTIVE true
bool COMPUTER_VISION_ENABLED = true;


void setup() {
  // serial setup
  Serial.begin(9600);
  Serial2.begin(9600);
  delay(2000);
  xbee.begin(Serial2);
  Serial.println("Bowie the robot");
  
  // bowie setup
  bowie.init();
  bowie.enableRemoteOp();
  bowie.turnOnLights();

  // promulgate setup
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);
  promulgate.set_debug_stream(&Serial);

  // pixy setup
  // TODO: we need a way to not get stuck in looking for the pixy
  // (or any other sensor peripherals) if they are not connected,
  // and if they become disconnected in the field
  pixy.init();

  /*
   * notes
   * servo config instructions
   * 1. position servo arms 90 deg to base upwards to ARM_MAX-200
   * 2. check angle with arm_max (should be towards inner portion of base)
   * 3. check angle with arm_home (should be parallel to base)
   * 4. check angle with arm_min (should not be touching the base, pointed downwards)
   * 5. check sequence of movements - should be smooth
   * 6. go to arm_min & go to end_min+200
   * wrong needs revising 7. position end servo arm 90 deg to arm plate upwards
   * 8. check angle with end_max (should be parallel to arm)
   * 9. check angle with end_home (should be parallel to ground)
   * 10. adjust tilt_min to be perpendicular to the ground
   * 11. adjust tilt_max to be flush with the enclosure
   * 12. check angle lid max to be flush with the hopper
   * 13. check angle lid_min to be open (~45 deg from perpendicular to hopper)
   */

  
  

  bowie.LOG_CURRENT_WHILE_MOVING = false;
  //Serial << "Homing... " << endl;
  //homePositions();
  Serial << "Parking hopper..." << endl;
  bowie.parkHopper();
  Serial << "Parking lid..." << endl;
  bowie.parkLid();
  Serial << "Parking arm..." << endl;
  bowie.parkArm();
  Serial << "Parking end..." << endl;
  bowie.parkEnd();  
  Serial << "Ready" << endl;

  /*
  while(1<3) {

    bowie.lid.detach();
    bowie.tilt.detach();
//    bowie.arm.detach();
//    bowie.arm2.detach();
//    bowie.end.detach();

    //calibrateTouchdown();
    preScoop();
    delay(1000);

    for(int i=0; i<3; i++) {
      scoopSequenceFast2();
      delay(3000);
    }

  
  
//    bowie.monitorCurrent();
//    bowie.moveEnd(END_HOME, 1, 3);
//    bowie.moveArm(ARM_HOME, 1, 3);
//
//    bowie.turnOnLights();
//    delay(100);
//    bowie.turnOffLights();
//    delay(100);
    
  }
  */
  

//  delay(5000);
//  scoopSequenceFast2();
//  
//  while(1<3) {
//    scoopSequenceFast2();
//    delay(15000);
//  }

  
  //calibrateTouchdown();
  //calibrateTouchdownFast();
  //bowie.parkArm();
  //bowie.parkEnd();
  
}

void preScoop() {

  bool DEBUGGING_ANIMATION = false;
  
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
  
}

void calibrateTouchdown() {

  bool did_touchdown = false;
  bool last_try = false;
  bool verified_touch = false;
  int count = 0;
  int new_arm_pos = ARM_MIN+200;

  int end_bound_up = END_MIN+200;
  int end_bound_down = END_PARALLEL_BOTTOM;//+200;

  Serial << "Beginning touchdown calibration" << endl;

  bowie.moveEnd(END_MIN+200);
  bowie.moveArm(new_arm_pos);

  while(!did_touchdown) {

    for(int i=end_bound_up; i<end_bound_down; i+=2) {
      bowie.moveEnd(i, 1, 1);
      if(bowie.getScoopProbeL() == 1 || bowie.getScoopProbeR() == 1) { // detected an initial press
        // back up to beginning
        bowie.moveEnd(end_bound_up, 1, 1);
        // then come back
        bowie.moveEnd(i, 1, 1);
        // check if pressed again
        for(int j=0; j<25; j++) {
          // checking for 500ms
          //Serial << "Verifying touch..." << endl;
          if(bowie.getScoopProbeL() == 1 || bowie.getScoopProbeR() == 1) {
            verified_touch = true;
            delay(20); 
          } else {
            verified_touch = false;
            break;
          }
        }
        // only if the touch is verified
        if(verified_touch) {
          bowie.END_TOUCHDOWN = i;
          Serial << "Touchdown at " << i << ", " << i-END_PARALLEL_BOTTOM << " from END_PARALLEL_BOTTOM" << endl;
          did_touchdown = true;
          break;
        }
      }
    }

    if(did_touchdown) break;

    bowie.moveEnd(end_bound_up, 1, 3);

    if(last_try == true && did_touchdown == false) {
      Serial << "Could not find touchdown" << endl;
      break;
    } else {
      Serial << "Moving arm #" << count << endl;
      bowie.moveArm(new_arm_pos, 1, 2);  
    }

    count++;
    new_arm_pos -= (count*20);
    if(new_arm_pos <= ARM_MIN) {
      new_arm_pos = ARM_MIN;
      last_try = true;
    }
    
  }

  if(did_touchdown) {
    Serial << "Touchdown calibration complete" << endl;
    for(int i=0; i<20; i++) { // log a bit afterwards to see any changes
      bowie.servoInterruption(LOGGING_AFTER_KEY, 0);
      delay(3);
    }
  }
  
  Serial << "\nDone" << endl;
  
}

void calibrateTouchdownFast() {

  bool did_touchdown = false;
  int count = 0;
  int new_arm_pos = ARM_HOME-100;

  Serial << "Beginning touchdown calibration" << endl;

  bowie.moveEnd(END_MIN+200);
  bowie.moveArm(new_arm_pos);

  while(!did_touchdown) {

    for(int i=END_MIN+200; i<END_PARALLEL_BOTTOM; i+=10) {
      bowie.moveEnd(i, 5, 1);
      if(bowie.getScoopProbeL() == 1 && bowie.getScoopProbeR() == 1) {
        Serial << "Touchdown at " << i << ", " << i-END_PARALLEL_BOTTOM << " from END_PARALLEL_BOTTOM" << endl;
        did_touchdown = true;
        break;
      }
    }

    for(int i=END_PARALLEL_BOTTOM; i>END_MIN+200; i--) {
      bowie.moveEnd(i, 5, 1);
      if(bowie.getScoopProbeL() == 1 && bowie.getScoopProbeR() == 1) {
        Serial << "Touchdown at " << i << ", " << i-END_PARALLEL_BOTTOM << " from END_PARALLEL_BOTTOM" << endl;
        did_touchdown = true;
        break;
      }
    }

    if(did_touchdown) {
      Serial << "Touchdown calibration complete" << endl;
      for(int i=0; i<20; i++) { // log a bit afterwards to see any changes
        bowie.servoInterruption(LOGGING_AFTER_KEY, 0);
        delay(3);
      }
      break;
    }

    count++;
    new_arm_pos -= (count*50);
    if(new_arm_pos <= ARM_MIN && did_touchdown == false) {
      Serial << "Could not find touchdown" << endl;
      break;
    } else {
      Serial << "Moving arm #" << count << endl;
      bowie.moveArm(new_arm_pos, 5, 1);  
    }
    
  }
  
  Serial << "\nDone" << endl;
  
}


void loop() {

  //Serial << "\n";

  current_time = millis();

  if(bowie.getScoopProbeL() == 1 && bowie.getScoopProbeR() == 1) {
    Serial << "TOUCHED" << endl;
  }

  while(xbeeRead()) {
    for(int i=0; i<rx.getDataLength(); i++) {
      char c = message_rx[i];
      promulgate.organize_message(c);
      //Serial << c;
    }
    //Serial << "rx data len: " << rx.getDataLength() << endl;
  }

  // update pixy
  if(COMPUTER_VISION_ENABLED) {
    pixyUpdate();
  }

  // update robot
  if(!currently_locked) bowie.update(COMPUTER_VISION_ENABLED);

  delay(20);
  
}

void sendNextMsg() {
  Msg m = bowie.popNextMsg();
  xbeeSend(m.action, m.cmd, m.key, m.val, m.cmd2, m.key2, m.val2, m.delim);
}

void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  if(PROG_DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "command2: " << cmd2 << endl;
    Serial << "key2: " << key2 << endl;
    Serial << "val2: " << val2 << endl;
    Serial << "delim: " << delim << endl;
  }

  msg_rx_count++;
  
  if(light_on) {
    digitalWrite(BOARD_LED, LOW);
  } else {
    digitalWrite(BOARD_LED, HIGH);
  }
  light_on = !light_on;
  diff_time = current_time-last_rx;
  last_rx = current_time;
  //Serial << "Diff time: " << diff_time << " Msg count: " << msg_rx_count << endl;

  //if(cmd == '0') {
    // something was interrupted
  //}

  bowie.chooseNextMessage();
  sendNextMsg();

  // user defined actions
  if(action == '#') {

    // Green button = touchdown calibrate scoop (fast)
    if(cmd == 'G') {
      if(val == 1) {
        calibrateTouchdown();
//        if(!did_scoop) {
//          scoopSequenceFast();
//          did_scoop = true;
//        }
      } else if(val == 0) {
//        did_scoop = false;
      }
    } else if(cmd2 == 'G') {
      if(val2 == 1) {
        calibrateTouchdown();
//        if(!did_scoop) {
//          scoopSequenceFast();
//          did_scoop = true;
//        }
      } else if(val2 == 0) {
        calibrateTouchdown();
//        did_scoop = false;
      }
    }


    // White button = empty
    if(cmd == 'W') {
      if(val == 1) {
        if(!did_empty) {
          deposit2();
          did_empty = true;
        }
      } else if(val == 0) {
        did_empty = false;
      }
    } else if(cmd2 == 'W') {
      if(val2 == 1) {
        if(!did_empty) {
          deposit2();
          did_empty = true;
        }
      } else if(val2 == 0) {
        did_empty = false;
      }
    }


    // Blue button = scoop fast
    if(cmd == 'B') {
      if(val == 1) {
        if(!did_scoop) {
          scoopSequenceFast2();
          did_scoop = true;
        }
      } else if(val == 0) {
        did_scoop = false;
      }
    } else if(cmd2 == 'B') {
      if(val2 == 1) {
        if(!did_scoop) {
          scoopSequenceFast2();
          did_scoop = true;
        }
      } else if(val2 == 0) {
        did_scoop = false;
      }
    }

    // Black button = dance or computer vision
    if(cmd == 'N') {
      if(val == 1) {
        bowie.moveLid(LID_MIN);
//        Serial << "Enabling computer vision" << endl;
//        if(!COMPUTER_VISION_ENABLED) pixy.init();
//        COMPUTER_VISION_ENABLED = true;
      } else if(val == 0) {
        bowie.moveLid(LID_MAX);
        bowie.parkLid();
//        Serial << "Disabling computer vision" << endl;
//        COMPUTER_VISION_ENABLED = false;
      }
    } else if(cmd2 == 'N') {
      if(val2 == 1) {
        bowie.moveLid(LID_MIN);
//        Serial << "Enabling computer vision" << endl;
//        if(!COMPUTER_VISION_ENABLED) pixy.init();
//        COMPUTER_VISION_ENABLED = true;
      } else if(val2 == 0) {
        bowie.moveLid(LID_MAX);
        bowie.parkLid();
//        Serial << "Disabling computer vision" << endl;
//        COMPUTER_VISION_ENABLED = false;
      }
    }
    
    
  }
  
  // robot's control based on the messages
  if(!currently_locked) bowie.control(action, cmd, key, val, cmd2, key2, val2, delim);

}

void transmit_complete() {

}


