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

#define PROG_DEBUG true
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

void setup() {
  // serial setup
  Serial.begin(9600);
  Serial2.begin(9600);
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

  /*
   * notes
   * servo config instructions
   * 1. position servo arms 90 deg to base upwards to arm_max-200
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
  
  homePositions();
  delay(1000);

  /*
  while(1<3) {

    scoopSequenceFast();
    delay(2000);
    deposit();
    delay(2000);
    
  }
  */

}

void loop() {

  Serial << "\n";

  current_time = millis();

  while(xbeeRead()) {
    Serial << "xbee read" << endl;
    for(int i=0; i<rx.getDataLength(); i++) {
      char c = message_rx[i];
      promulgate.organize_message(c);
      Serial << c;
    }
    Serial << "rx data len: " << rx.getDataLength() << endl;
  }

  // update robot
  bowie.update();

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
  Serial << msg_rx_count << " " << cmd << endl;
  if(light_on) {
    digitalWrite(BOARD_LED, LOW);
  } else {
    digitalWrite(BOARD_LED, HIGH);
  }
  light_on = !light_on;
  diff_time = current_time-last_rx;
  last_rx = current_time;
  Serial << "diff time: " << diff_time << endl;

  if(cmd == '0') {
    // something was interrupted
  }

  bowie.chooseNextMessage();
  sendNextMsg();

  // user defined actions
  if(action == '#') {

    // scoop (fast)
    if(cmd == 'G') {
      if(val == 1) {
        if(!did_scoop) {
          scoopSequenceFastB();
          did_scoop = true;
        }
      } else if(val == 0) {
        did_scoop = false;
      }
    } else if(cmd2 == 'G') {
      if(val2 == 1) {
        if(!did_scoop) {
          scoopSequenceFastB();
          did_scoop = true;
        }
      } else if(val2 == 0) {
        did_scoop = false;
      }
    }


    // empty
    if(cmd == 'W') {
      if(val == 1) {
        if(!did_empty) {
          deposit();
          did_empty = true;
        }
      } else if(val == 0) {
        did_empty = false;
      }
    } else if(cmd2 == 'W') {
      if(val2 == 1) {
        if(!did_empty) {
          deposit();
          did_empty = true;
        }
      } else if(val2 == 0) {
        did_empty = false;
      }
    }


    // scoop (slow)
    if(cmd == 'B') {
      if(val == 1) {
        if(!did_scoop) {
          scoopSequenceSlowB();
          did_scoop = true;
        }
      } else if(val == 0) {
        did_scoop = false;
      }
    } else if(cmd2 == 'B') {
      if(val2 == 1) {
        if(!did_scoop) {
          scoopSequenceSlowB();
          did_scoop = true;
        }
      } else if(val2 == 0) {
        did_scoop = false;
      }
    }

    // dance
    if(cmd == 'N') {
      if(val == 1) {
        dance();
      } else if(val == 0) {
        stopDance();
      }
    } else if(cmd2 == 'N') {
      if(val2 == 1) {
        dance();
      } else if(val2 == 0) {
        stopDance();
      }
    }
    
    
  }
  
  // robot's control based on the messages
  bowie.control(action, cmd, key, val, cmd2, key2, val2, delim);

}

void transmit_complete() {

}


