void arduinoCommsInit() {
  bowiecomms_arduino.setCommLed(COMM_LED);
  bowiecomms_arduino.set_received_action_callback(receivedAction_Arduino);
  bowiecomms_arduino.set_comms_timeout_callback(commsTimeout_Arduino);
  bowiecomms_arduino.set_controller_added_callback(controllerAdded_Arduino);
  bowiecomms_arduino.set_controller_removed_callback(controllerRemoved_Arduino);

  bowiecomms_arduino.initComms(ARDUINO_CONN);

  bowiecomms_arduino.addPeriodicMessage(random_periodic1);
  bowiecomms_arduino.addPeriodicMessage(random_periodic2);
}

void receivedAction_Arduino(Msg m) {
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

void commsTimeout_Arduino() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.
}

void controllerAdded_Arduino() {
  // This will never be called for the Arduino
  
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void controllerRemoved_Arduino() {
  // This will never be called for the Arduino
  
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}

