#include "BowieComms.h"

#define COMM_LED 2

BowieComms bowiecomms = BowieComms();

void commsTimeout();

long current_time = 0;
long last_latency_print = 0;
long last_update = 0;

Msg random_periodic1 = bowiecomms.msg_none;
Msg random_periodic2 = bowiecomms.msg_none;

void setup() {
  Serial.begin(9600);

  bowiecomms.setCommLed(COMM_LED);
  bowiecomms.set_received_action_callback(receivedAction);
  bowiecomms.set_comms_timeout_callback(commsTimeout);
  bowiecomms.set_controller_added_callback(controllerAdded);
  bowiecomms.set_controller_removed_callback(controllerRemoved);
  

  bowiecomms.initComms();

  bowiecomms.addPeriodicMessage(random_periodic1);
  bowiecomms.addPeriodicMessage(random_periodic2);
  
}

void loop() {

  current_time = millis();
  
  bowiecomms.updateComms();

  if(current_time-last_latency_print >= 500) {
    Serial << "Comm latency: " << bowiecomms.getCommLatency() << endl;
    last_latency_print = current_time;
  }

  if(current_time-last_update >= 1000) {
    updateRobotsPeriodicMessages();
    last_update = current_time;
  }

  if(Serial.available() > 0) {
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
      bowiecomms.addMsg(m);
      
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
      bowiecomms.insertMsg(m);
      
    } else if(c == 'C') {

      // Example below if you want to do a 'bruteforce' send
      // - which would bypass the message queue. Could have
      // unintended consequences (eg, if two messages are sent in
      // rapid succession, none may be delivered).
      bowiecomms.xbeeSend('#', 'Q', 1, 0, '#', 'Q', 2, 0);
      
    }
  }

}

void receivedAction(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  Serial << "---RECEIVED ACTION---" << endl;
  Serial << "action: " << m.action << endl;
  Serial << "command: " << m.pck1.cmd << endl;
  Serial << "key: " << m.pck1.key << endl;
  Serial << "val: " << m.pck1.val << endl;
  Serial << "command: " << m.pck2.cmd << endl;
  Serial << "key: " << m.pck2.key << endl;
  Serial << "val: " << m.pck2.val << endl;
  Serial << "delim: " << m.delim << endl;
  
}

void commsTimeout() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.
}

void controllerAdded() {
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void controllerRemoved() {
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}

