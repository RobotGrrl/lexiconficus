/*
 * Robot Missions Robot Platform Firmware
 * --------------------------------------
 * 
 * March 20, 2017
 * Erin RobotGrrl for RobotMissions.org
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 */

#include "BowieSimple.h"
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

void setup() {
  // serial setup
  Serial.begin(9600);
  Serial2.begin(9600);
  xbee.begin(Serial2);
  Serial.println("Bowie Xbee Test");

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
  while(1<3) {
    //leftBork();
    bowie.motor_setDir(0, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(0, 60);
    bowie.motor_setDir(1, MOTOR_DIR_FWD);
    bowie.motor_setSpeed(1, 60);
    delay(5000);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setSpeed(1, 0);
    delay(2000);
    bowie.motor_setDir(0, MOTOR_DIR_REV);
    bowie.motor_setSpeed(0, 60);
    bowie.motor_setDir(1, MOTOR_DIR_REV);
    bowie.motor_setSpeed(1, 60);
    delay(2000);
    bowie.motor_setSpeed(0, 0);
    bowie.motor_setSpeed(1, 0);
    delay(2000);
  }
  */
  
  
}

void loop() {

  current_time = millis();

  while(xbeeRead()) {
    for(int i=0; i<rx.getDataLength(); i++) {
      char c = message_rx[i];
      promulgate.organize_message(c);
      //Serial << c;
    }
    //Serial << "rx data len: " << rx.getDataLength() << endl;
  }

  // update robot
  bowie.update();

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

  // robot's control based on the messages
  bowie.control(action, cmd, key, val, cmd2, key2, val2, delim);

}

void transmit_complete() {

}


