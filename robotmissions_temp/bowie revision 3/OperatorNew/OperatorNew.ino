/*
 * Robot Missions Operator Firmware (New)
 * --------------------------------
 * 
 * April 10, 2017
 * Erin RobotGrrl for RobotMissions.org
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 */

#include "OperatorNew.h"
#include <SoftwareSerial.h>
#include <Streaming.h>
#include <XBee.h>
#include "PromulgateBig.h"

#define PROG_DEBUG false

// ---- Operator
Operator controller = Operator();

// ---- Communication
// Xbee on Serial1
Promulgate promulgate = Promulgate(&Serial1, &Serial1);

void transmit_complete();
void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);

// ---- Xbee variables
XBee xbee = XBee();
XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
XBeeAddress64 addr_controller = XBeeAddress64(0x0013A200, 0x40DD9902);
XBeeAddress64 addr_robot = XBeeAddress64(0x0013A200, 0x40D96FC2);
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
ZBRxResponse rx = ZBRxResponse();
char message_tx[64];
char message_rx[64];
uint32_t msg_tx_count = 0;
uint32_t msg_rx_count = 0;
uint32_t msg_tx_err = 0;

// ---- Messages
Msg msg_none = { 9, '^', '0', 0, 0, '0', 0, 0, '!' };
Msg msg_a = { 8, '@', 'A', 1, 1, 'Z', 8, 8888, '!' };
Msg msg_b = { 8, '@', 'A', 0, 0, 'Z', 0, 0, '!' };

#define DEFAULT_RETRY_TIME 250
#define SECONDARY_RETRY_TIME 500

unsigned long last_rx = 0;
unsigned long current_time = 0;
unsigned long diff_time = 0;
uint16_t retry_time = DEFAULT_RETRY_TIME;

// ---- Other
bool a_or_b = false;
boolean light_on = false;
boolean has_started = false;


void setup() {
  // ---- 1. serial inits
  Serial.begin(9600);
  Serial1.begin(9600);
  xbee.begin(Serial1);
  Serial.println(F("Operator Xbee Test"));

  // ---- 2. promulgate
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);
  promulgate.set_debug_stream(&Serial);

  // ---- 3. operator
  controller.init();
  controller.breatheLeds();
  
}

void loop() {

  // ---- 0. current time
  current_time = millis();

  // ---- 1. custom code
  if(controller.redbouncer.fell()) {
    has_started = !has_started;
    controller.addNextMsg(msg_a);
    sendNextMsg();
  }

  // ---- 2. retry
  if(has_started) {
    if(current_time-last_rx >= retry_time) { // retry
      Serial.println(F("Retrying send"));
      sendNextMsg();
      retry_time = SECONDARY_RETRY_TIME;
      last_rx = current_time;
    }
  }

  // ---- 3. read from xbee
  while(xbeeRead()) {
    Serial << "\n<< ";
    for(int i=0; i<=rx.getDataLength(); i++) {
      char c = message_rx[i];
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }
  }

  // ---- 4. update
  controller.update();

  // ---- 5. (temporary) relay external commands
  if(Serial.available()) {
    char c = Serial.read();
    switch(c) {
      case 'A': // forward
      controller.addNextMsg( 3, '#', 'L', 1, 255, 'R', 1, 255, '!' );
      break;
      case 'B': // backward
      controller.addNextMsg( 3, '#', 'L', 0, 255, 'R', 0, 255, '!' );
      break;
      case 'C': // left
      controller.addNextMsg( 3, '#', 'L', 0, 255, 'R', 1, 255, '!' );
      break;
      case 'D': // right
      controller.addNextMsg( 3, '#', 'L', 1, 255, 'R', 0, 255, '!' );
      break;
      case 'E': // sequence
      controller.addNextMsg( 3, '#', 'G', 0, 1, '0', 0, 0, '!' );
      break;
    }
  }

}



void sendNextMsg() {
  Msg m = controller.popNextMsg();
  xbeeSend(m.action, m.cmd, m.key, m.val, m.cmd2, m.key2, m.val2, m.delim);
}

void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  val2+=1;//seems to only happen on this micro

  if(PROG_DEBUG) {
    Serial.println(delim); // do not delete this
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
  if(PROG_DEBUG) Serial << msg_rx_count << " " << cmd << endl;
  if(light_on) {
    digitalWrite(BOARD_LED, LOW);
  } else {
    digitalWrite(BOARD_LED, HIGH);
  }
  light_on = !light_on;
  diff_time = current_time-last_rx;
  last_rx = current_time;
  if(PROG_DEBUG) Serial << "diff time: " << diff_time << endl;
  retry_time = DEFAULT_RETRY_TIME;

  // send the next message now that we have received a reply from the robot
  sendNextMsg();

}

void transmit_complete() {

}


