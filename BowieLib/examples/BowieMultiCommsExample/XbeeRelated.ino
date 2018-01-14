void xbeeCommsInit() {
  bowiecomms_xbee.setRobotID(ROBOT_ID);
  bowiecomms_xbee.setCommLed(COMM_LED);
  bowiecomms_xbee.set_received_action_callback(receivedAction_Xbee);
  bowiecomms_xbee.set_comms_timeout_callback(commsTimeout_Xbee);
  bowiecomms_xbee.set_controller_added_callback(controllerAdded_Xbee);
  bowiecomms_xbee.set_controller_removed_callback(controllerRemoved_Xbee);

  bowiecomms_xbee.initComms(XBEE_CONN, 9600);

  bowiecomms_xbee.addPeriodicMessage(random_periodic1);
  bowiecomms_xbee.addPeriodicMessage(random_periodic2);
}

void receivedAction_Xbee(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  /*
  Serial << "---RECEIVED ACTION FROM XBEE---" << endl;
  Serial << "action: " << m.action << endl;
  Serial << "command: " << m.pck1.cmd << endl;
  Serial << "key: " << m.pck1.key << endl;
  Serial << "val: " << m.pck1.val << endl;
  Serial << "command: " << m.pck2.cmd << endl;
  Serial << "key: " << m.pck2.key << endl;
  Serial << "val: " << m.pck2.val << endl;
  Serial << "delim: " << m.delim << endl;
  */
  
}

void commsTimeout_Xbee() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.
}

void controllerAdded_Xbee() {
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void controllerRemoved_Xbee() {
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}

