/*
 * Bowie Multi Comms Example
 * -------------------------
 * 
 * Communicate through the robot's Xbee and also to
 * the connected Arduino (or similar). Easily send
 * commands to both, and display what was received.
 * 
 * Erin RobotGrrl for RobotMissions
 * Jan. 7th, 2018
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "BowieComms.h"

#define ROBOT_ID 3
#define COMM_LED 2

BowieComms bowiecomms_xbee;
BowieComms bowiecomms_arduino;

void receivedAction_Arduino(Msg m);
void commsTimeout_Arduino();
void controllerAdded_Arduino();
void controllerRemoved_Arduino();

void receivedAction_Xbee(Msg m);
void commsTimeout_Xbee();
void controllerAdded_Xbee();
void controllerRemoved_Xbee();

long current_time = 0;
long last_latency_print = 0;
long last_update = 0;

Msg random_periodic1 = bowiecomms_xbee.msg_none;
Msg random_periodic2 = bowiecomms_xbee.msg_none;

void setup() {
  Serial.begin(9600);

  bowiecomms_xbee = BowieComms();
  bowiecomms_xbee.begin();

  bowiecomms_arduino = BowieComms();
  bowiecomms_arduino.begin();
  
  // Init the Xbee comms
  xbeeCommsInit();

  // Init the Arduino comms
  arduinoCommsInit();
  
}

void loop() {

  delay(100);
  
  current_time = millis();

  bowiecomms_xbee.updateComms();
  bowiecomms_arduino.updateComms();

  if(current_time-last_update >= 1000) {
    updateRobotsPeriodicMessages();
    last_update = current_time;
  }

  while(Serial.available() > 0) {
    char c = Serial.read();
    if(c == 'A') {
      
      // Set the super bright leds to half brightness by
      // setting the Msg variables and adding it to the
      // queue.
      Msg m;
      m.priority = 10;
      m.action = '#';
      m.pck1.cmd = 'Q';
      m.pck1.key = 1;
      m.pck1.val = 128;
      m.pck2.cmd = 'Q';
      m.pck2.key = 2;
      m.pck2.val = 128;
      m.delim = '!';
      bowiecomms_xbee.addMsg(m);
      bowiecomms_arduino.addMsg(m);
      
    } else if(c == 'B') {
      
      // Set the super bright leds to high brightness by
      // inserting it to the queue
      Msg m;
      m.priority = 10;
      m.action = '#';
      m.pck1.cmd = 'Q';
      m.pck1.key = 1;
      m.pck1.val = 200;
      m.pck2.cmd = 'Q';
      m.pck2.key = 2;
      m.pck2.val = 200;
      m.delim = '!';
      bowiecomms_xbee.insertMsg(m);
      bowiecomms_arduino.insertMsg(m);
      
    } else if(c == 'C') {

      // Example below if you want to do a 'bruteforce' send
      // - which would bypass the message queue. Could have
      // unintended consequences (eg, if two messages are sent in
      // rapid succession, none may be delivered).
      bowiecomms_xbee.connSend('#', 'Q', 1, 0, '#', 'Q', 2, 0);
      bowiecomms_arduino.connSend('#', 'Q', 1, 0, '#', 'Q', 2, 0);
      
    }
  }
  
}


