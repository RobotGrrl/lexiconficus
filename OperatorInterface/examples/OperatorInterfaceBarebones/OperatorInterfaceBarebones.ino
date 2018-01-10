/*
 * Operator Interface Barebones Example
 * ------------------------------------
 * 
 * Send the basic commands to the robot,
 * and display the main interface on the
 * screen.
 * 
 * Erin RobotGrrl for RobotMissions
 * Jan. 9th, 2018
 * --> http://RobotMissions.org
 * 
 * MIT license, check LICENSE for more information
 * All text above must be included in any redistribution
 * 
 */

#include "OperatorInterface.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 12
Adafruit_SSD1306 display(OLED_RESET);

#define OP_ID 50

OperatorInterface opinterface;

void buttonChanged(int button, int val);
void modeChanged(int mode);
void robotAdded();
void robotRemoved(bool connection);

void receivedAction(Msg m);
void commsTimeout();
void controllerAdded();
void controllerRemoved();

void setup() {
  delay(2000);
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();
  displayLogo();
  
  opinterface = OperatorInterface();

  opinterface.begin();
  opinterface.setOpID(OP_ID);
  opinterface.initOperator(XBEE_CONN, 9600, &Serial1);

  //opinterface.TESTING = true;
  opinterface.setAutoconnect(true);
  opinterface.set_received_action_callback(receivedAction);
  opinterface.set_comms_timeout_callback(commsTimeout);
  opinterface.set_controller_added_callback(controllerAdded);
  opinterface.set_controller_removed_callback(controllerRemoved);
  
  opinterface.set_button_changed_callback(buttonChanged);
  opinterface.set_mode_changed_callback(modeChanged);
  opinterface.set_robot_added_callback(robotAdded);
  opinterface.set_robot_removed_callback(robotRemoved);

  setButtonLabel("Drive", 0, 1);
  setButtonLabel("Arm", 1, 1);
  setButtonLabel("Empty", 2, 1);
  setButtonLabel("Scoop S", 3, 1);
  setButtonLabel("Scoop F", 4, 1);
  setButtonLabel("Dump", 5, 1);

  setModeLabel("Operator", 1);
  setModeLabel("Sensors", 2);
  setModeLabel("Autonomous", 3);


}

void loop() {

  opinterface.updateOperator();

  if(opinterface.isConnectedToRobot()) {
    mainMenu(opinterface.button_states, opinterface.CURRENT_MODE);
  } else {
    displaySearching(millis());
  }

  if(opinterface.isConnectedToRobot()) {
  
    if(opinterface.getCurrentMode() == 1) {
    
      if(opinterface.getButton(0) == 1) { // drive state
        opinterface.joystickDriveControl();
      }
  
      if(opinterface.getButton(1) == 1) { // arm state
        opinterface.joystickArmControl();
      }
        
      //if(opinterface.getButton(2) == 1) { // scoop
        // send scoop command
        // if, for example, you want to stop after some time...
        // opinterface.setButtonState(2, 0);
      //}
    
    }

  }
  
  //delay(50);

}

void buttonChanged(int button, int val) {
  // This is called each time a button is pressed
}

void modeChanged(int mode) {
  // This is called each time the mode switch is changed
}

void robotAdded() {
  // A new robot has been added to the list to select from.
  // If you want to select it, you could call 
  // opinterface.xbeeChooseRobotToConnect(),
  // but make sure that SELECTED_ROBOT is false first.
}

void robotRemoved(bool connection) {
  // A robot has been removed from the list to select from.
  // If connection == true, we are still connected to the robot we're
  // controlling. If not, you might want to prompt the user to select
  // a new robot. See comment in robotAdded() for details.
}

void receivedAction(Msg m) {
  // Received an action from the controller. The data is
  // packed into the Msg struct. Core actions with this data
  // will be done inside of the main robot mission program.
  // All Msgs will be passed through to this function, even
  // if there is / is not a core action associated with it.
  // You can do custom actions with this data here.

  /*
  Serial << "---RECEIVED ACTION---" << endl;
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


