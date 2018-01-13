#include "BowieComms.h"

#define COMM_LED 2

BowieComms bowiecomms_xbee = BowieComms();
BowieComms bowiecomms_arduino = BowieComms();

void commsTimeout();

long current_time = 0;
long last_latency_print = 0;
long last_update = 0;

Msg random_periodic1 = bowiecomms_xbee.msg_none;
Msg random_periodic2 = bowiecomms_xbee.msg_none;

void setup() {
  Serial.begin(9600);

  // Init the Xbee comms
  xbeeCommsInit();

  // Init the Arduino comms
  arduinoCommsInit();
  
}

void loop() {
  
  current_time = millis();
  
  bowiecomms_xbee.updateComms();
  bowiecomms_arduino.updateComms();

  if(current_time-last_latency_print >= 500) {
    Serial << "Xbee Comm latency: " << bowiecomms_xbee.getCommLatency() << endl;
    Serial << "Arduino Comm latency: " << bowiecomms_arduino.getCommLatency() << endl;
    last_latency_print = current_time;
  }

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


