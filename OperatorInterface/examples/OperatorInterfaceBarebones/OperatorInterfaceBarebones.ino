#include "OperatorInterface.h"

Operator opinterface = Operator();

void buttonChanged(int button, int val);
void receivedAction(Msg m);
void commsTimeout();
void controllerAdded();
void controllerRemoved();

void setup() {
  Serial.begin(9600);
  
  opinterface.initOperator(XBEE_CONN, 9600);
  opinterface.set_button_changed_callback(buttonChanged);
  // opinterface.setAutoconnect(FALSE); // uncomment if you want to select which robot to connect to
  opinterface.set_received_action_callback(receivedAction);
  opinterface.set_comms_timeout_callback(commsTimeout);
  opinterface.set_controller_added_callback(controllerAdded);
  opinterface.set_controller_removed_callback(controllerRemoved);
  
}

void loop() {

  opinterface.updateOperator();

}

void buttonChanged(int button, int val) {
  // This is called each time a button is pressed
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


