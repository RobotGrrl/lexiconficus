#include "MegaBowieShoreline.h"

MegaBowieShoreline::MegaBowieShoreline() {

  // Instance of the class for the callbacks from Promulgate
  bowieInstance = this;

  REMOTE_OP_ENABLED = true;
  PREVENT_OVER_CURRENT = false;
  LOGGING_ENABLED = true;

  unlikely_count = 0;
  current_time = 0;
  last_ctrl = 0;
}

MegaBowieShoreline *MegaBowieShoreline::bowieInstance;

void MegaBowieShoreline::begin() {

  // Outputs
  pinMode(BOARD_LED, OUTPUT);
  pinMode(COMM_LED, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  
  // Arm
  bowiearm = BowieArm();

  bowiearm.setArm1ServoPin(SERVO_ARM1);
  bowiearm.setArm2ServoPin(SERVO_ARM2);
  bowiearm.setServoInterruptCallback(servoInterrupt);
  bowiearm.initServos();

  // Current Sensors
  servoCurrent = BowieCurrentSensor();
  motorCurrent = BowieCurrentSensor();

  servoCurrent.setCurrentSensePin(CURRENT_SERVO_SENS);
  servoCurrent.setCurrentSenseName("Servo");
  servoCurrent.initCurrentSensor();
  servoCurrent.set_waitingToCoolDown_callback(waitingToCoolDown_ServosCallback);
  servoCurrent.set_reactivateAfterCoolDown_callback(reactivateAfterCoolDown_ServosCallback);
  servoCurrent.set_overCurrentThreshold_callback(overCurrentThreshold_ServosCallback);
  
  motorCurrent.setCurrentSensePin(CURRENT_MOTOR_SENS);
  motorCurrent.setCurrentSenseName("Motor");
  motorCurrent.initCurrentSensor();
  motorCurrent.set_waitingToCoolDown_callback(waitingToCoolDown_MotorsCallback);
  motorCurrent.set_reactivateAfterCoolDown_callback(reactivateAfterCoolDown_MotorsCallback);
  motorCurrent.set_overCurrentThreshold_callback(overCurrentThreshold_MotorsCallback);

  // Drive
  bowiedrive = BowieDrive();

  bowiedrive.setMotorASpeedPin(MOTORA_SPEED);
  bowiedrive.setMotorBSpeedPin(MOTORB_SPEED);
  bowiedrive.setMotorACtrl1Pin(MOTORA_CTRL1);
  bowiedrive.setMotorACtrl2Pin(MOTORA_CTRL2);
  bowiedrive.setMotorBCtrl1Pin(MOTORB_CTRL1);
  bowiedrive.setMotorBCtrl2Pin(MOTORB_CTRL2);

  // Hopper
  bowiehopper = BowieHopper();

  bowiehopper.setServoHopperPivotPin(SERVO_HOPPER_PIVOT);
  bowiehopper.setServoHopperLidPin(SERVO_HOPPER_LID);

  bowiehopper.setServoInterruptCallback(servoInterrupt);

  bowiehopper.initServos();

  // Lights
  bowielights = BowieLights();

  bowielights.setFrontLeftPin(BRIGHT_LED_FRONT_LEFT);
  bowielights.setFrontRightPin(BRIGHT_LED_FRONT_RIGHT);
  bowielights.setBackLeftPin(BRIGHT_LED_BACK_LEFT);
  bowielights.setBackRightPin(BRIGHT_LED_BACK_RIGHT);

  bowielights.initLeds();

  // Logger
  bowielogger = BowieLogger();

  bowielogger.initTime();

  bowielogger.setLoggingLed(13);
  bowielogger.initLogging();

  // Scoop
  bowiescoop = BowieScoop();

  bowiescoop.setServoScoopPin(SERVO_END_EFFECTOR);
  bowiescoop.setProbeLPin(SCOOP_PROBE_LEFT);
  bowiescoop.setProbeRPin(SCOOP_PROBE_RIGHT);

  bowiescoop.setServoInterruptCallback(servoInterrupt);

  bowiescoop.initServos();

  // Comms
  bowiecomms_xbee = BowieComms();

  bowiecomms_xbee.setCommLed(COMM_LED);
  bowiecomms_xbee.set_received_action_callback(receivedAction_Xbee);
  bowiecomms_xbee.set_comms_timeout_callback(commsTimeout_Xbee);
  bowiecomms_xbee.set_controller_added_callback(controllerAdded_Xbee);
  bowiecomms_xbee.set_controller_removed_callback(controllerRemoved_Xbee);

  bowiecomms_xbee.initComms(XBEE_CONN);

  // TODO
  /*
  bowiecomms_xbee.addPeriodicMessage(random_periodic1);
  bowiecomms_xbee.addPeriodicMessage(random_periodic2);
  */

  // Arduino Comms
  bowiecomms_arduino = BowieComms();
  
  bowiecomms_arduino.setCommLed(COMM_LED);
  bowiecomms_arduino.set_received_action_callback(receivedAction_Arduino);
  bowiecomms_arduino.set_comms_timeout_callback(commsTimeout_Arduino);
  bowiecomms_arduino.set_controller_added_callback(controllerAdded_Arduino);
  bowiecomms_arduino.set_controller_removed_callback(controllerRemoved_Arduino);

  bowiecomms_arduino.initComms(ARDUINO_CONN);

}


/*

---- States ----

*/

void MegaBowieShoreline::enableRemoteOp() {
  REMOTE_OP_ENABLED = true;
}

void MegaBowieShoreline::disableRemoteOp() {
  REMOTE_OP_ENABLED = false;
}

void MegaBowieShoreline::enableLogging() {
  LOGGING_ENABLED = true;
}

void MegaBowieShoreline::disableLogging() {
  LOGGING_ENABLED = false;
}


/*

---- Bowie Comms Arduino Callbacks ----

*/

void MegaBowieShoreline::receivedAction_Arduino(Msg m) {
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

  bowieInstance->control(m);
  
}

void MegaBowieShoreline::commsTimeout_Arduino() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.

  // TODO

}

void MegaBowieShoreline::controllerAdded_Arduino() {
  // This will never be called for the Arduino
  
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void MegaBowieShoreline::controllerRemoved_Arduino() {
  // This will never be called for the Arduino
  
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}


/*

---- Bowie Comms Xbee Callbacks ----

*/

void MegaBowieShoreline::receivedAction_Xbee(Msg m) {
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

  bowieInstance->control(m);
  
}

void MegaBowieShoreline::commsTimeout_Xbee() {
  // The comms timed out. You can do an action here, such as
  // turning off the motors and sending them back to the 
  // home positions.

  // TODO

}

void MegaBowieShoreline::controllerAdded_Xbee() {
  // Called when receiving an Xbee response. The ID of the
  // controller will be sent via the received action. You
  // could do an action here, such as prepare the robot's
  // servos for moving.
}

void MegaBowieShoreline::controllerRemoved_Xbee() {
  // Called when the Xbee watchdog detects no messages
  // received from a controller after a given amount of time.
  // The ID of the controller could be deduced by not hearing
  // from it in the received action. You could do an action
  // here, such as the robot waving goodbye.
}


/*

---- Control ----

*/

void MegaBowieShoreline::update(bool force_no_sleep) {

  current_time = millis();

  // specific things to do if remote operation is enabled
  if(REMOTE_OP_ENABLED) {

    if(!force_no_sleep) {
      if(millis()-last_ctrl >= REMOTE_OP_TIMEOUT) {

        // TODO
        /*
        digitalWrite(COMM_LED, LOW);
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, 0);
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, 0);
        parkArm();
        parkEnd();
        parkHopper();
        parkLid();
        */

      }
    }
    
  }

  servoCurrent.updateCurrentSensor();
  motorCurrent.updateCurrentSensor();

  updateLogSensorData();
  bowielogger.updateLogging();

  // todo: monitoring current and logging
  
}

void MegaBowieShoreline::control(Msg m) {
  
  last_ctrl = millis();

  Packet packets[2] = { m.pck1, m.pck2 };

  // TODO
  /*
  if(COMM_DEBUG) {
    Serial << "*c1 cmd: " << packets[0].cmd << " key: " << packets[0].key << " val: " << packets[0].val << endl;
    Serial << "*c2 cmd: " << packets[1].cmd << " key: " << packets[1].key << " val: " << packets[1].val << endl;
  }
  */

  // we've seen this happen *sometimes*, and it is highly unlikely that this would be an
  // intentional command. let's make sure they mean this at least 2 times before listening
  
  // TODO
  /*
  if(val == 255 && val2 == 255 && cmd == 'L' && cmd2 == 'R') {
    unlikely_count++;
    if(unlikely_count <= 2) return;
  } else {
    unlikely_count = 0;
  }
  */

  // TODO
  /*

  if(action == '#') {

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'P') { // red button
        if(packets[i].val == 1) { // sends drive joystick cmds on operator side
        }
      }

      if(packets[i].cmd == 'Y') { // yellow button
        if(packets[i].val == 1) { // sends arm joystick cmds on operator side
        }
      }

      if(packets[i].cmd == 'G') { // green button
        if(packets[i].val == 1) { // collect (fast)
        }
      }

      if(packets[i].cmd == 'W') { // white button
        if(packets[i].val == 1) { // deposit
        }
      }

      if(packets[i].cmd == 'B') { // blue button
        if(packets[i].val == 1) { // collect (slow)
        }
      }

      if(packets[i].cmd == 'N') { // black button
        if(packets[i].val == 1) { // dance
        }
      }

    }
  } // -- end of '#' action specifier

  */

  // TODO
  /*

  if(action == '@') {

    if(TURN_SEQUENCE_MODE) {
      if(packets[0].cmd == 'L' && packets[0].key == 1 && packets[1].cmd == 'R' && packets[1].key == 0) {
        // turning right
        //last_activated_turn_sequence = millis();
        turnSequence(false);
        return; // we don't want the default stuff below in this mode
      } else if(packets[0].cmd == 'L' && packets[0].key == 0 && packets[1].cmd == 'R' && packets[1].key == 1) {
        // turning left
        //last_activated_turn_sequence = millis();
        turnSequence(true);
        return; // we don't want the default stuff below in this mode
      }
    }
    
    // stop the motors when zeroed
    if(packets[0].cmd == 'L' && packets[0].key == 0 && packets[0].val == 0 && packets[1].cmd == 'R' && packets[1].key == 0 && packets[1].val == 0) {
      motor_setDir(0, MOTOR_DIR_FWD);
      motor_setSpeed(0, 0);
      motor_setDir(1, MOTOR_DIR_FWD);
      motor_setSpeed(1, 0);
    }

    // if it reaches here, then we know we can reset this flag
    resetTurnSequence();

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'L') { // left motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          analogWrite(BRIGHT_LED_FRONT_LEFT, MAX_BRIGHTNESS);
          //leftBork();
          motor_setDir(0, MOTOR_DIR_FWD);
          motor_setSpeed(0, val);
        } else if(packets[i].key == 0) { // bwd
          analogWrite(BRIGHT_LED_FRONT_LEFT, MIN_BRIGHTNESS);
          //leftBork();
          motor_setDir(0, MOTOR_DIR_REV);
          motor_setSpeed(0, val);
        }
      }

      if(packets[i].cmd == 'R') { // right motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          digitalWrite(BRIGHT_LED_FRONT_RIGHT, HIGH);
          //leftBork();
          motor_setDir(1, MOTOR_DIR_FWD);
          motor_setSpeed(1, val);
        } else if(packets[i].key == 0) { // bwd
          digitalWrite(BRIGHT_LED_FRONT_RIGHT, LOW);
          //leftBork();
          motor_setDir(1, MOTOR_DIR_REV);
          motor_setSpeed(1, val);
        }
      }
      
      if(packets[i].cmd == 'S') { // arm (data from 0-100)
        
        int temp_pos = getArmPos();
        int new_pos = temp_pos;
        int the_increment = (int)map(val, 0, 100, 1, 70);

        if(packets[i].key == 0) {
          new_pos -= the_increment;
        } else if(packets[i].key == 1) {
          new_pos += the_increment;
        }

        if(new_pos < ARM_HOME) {
          moveArmAndEnd(new_pos, 3, 1, ARM_MIN, ARM_HOME, END_PARALLEL_BOTTOM-100, END_HOME-300); // END_PARALLEL_BOTTOM-700
        } else if(new_pos > ARM_HOME) {
          moveArmAndEnd(new_pos, 3, 1, ARM_HOME, ARM_MAX, END_HOME-300, END_PARALLEL_TOP-200);
        }

        Serial << "\narm angle: " << arm_position << endl;
      }
      
    }
  } // -- end of '@' action specifier

  */

}

void MegaBowieShoreline::servoInterrupt(int key, int val) {
  
  switch(key) {
    case SERVO_ARM_KEY:
    break;
    case SERVO_END_KEY:
    break;
    case SERVO_HOPPER_KEY:
    break;
    case SERVO_LID_KEY:
    break;
    case SERVO_EXTRA_KEY:
    break;
  }

}

void MegaBowieShoreline::updateLogSensorData() {
  // TODO all of the sensor data here

  bowielogger.setLogData_t(LOG_TIME, now());
}


/*

---- Current Sensors ----

*/

void MegaBowieShoreline::waitingToCoolDown_ServosCallback(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
}

void MegaBowieShoreline::reactivateAfterCoolDown_ServosCallback() {
 // you might want to re-attach the servos here
 // and send them back to a position
 // (or re-activate the dc motors)
}

void MegaBowieShoreline::overCurrentThreshold_ServosCallback(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
}

void MegaBowieShoreline::waitingToCoolDown_MotorsCallback(bool first) {
  // you might want to detach the servos here
  // (or de-activate the dc motors)
  // first == true when it's called the first time
}

void MegaBowieShoreline::reactivateAfterCoolDown_MotorsCallback() {
  // you might want to re-attach the servos here
  // and send them back to a position
  // (or re-activate the dc motors)
}

void MegaBowieShoreline::overCurrentThreshold_MotorsCallback(bool first) {
  // you might want to move the robot, it might
  // be in a bad position.
  // as well, de-activate the servos / motors.
  // PS: waitingToCoolDown() will be called prior to
  // this function
  // first == true when it's called the first time
}


void MegaBowieShoreline::waitingToCoolDown_Servos(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
}

void MegaBowieShoreline::reactivateAfterCoolDown_Servos() {
  if(!PREVENT_OVER_CURRENT) return;
}

void MegaBowieShoreline::overCurrentThreshold_Servos(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
}

void MegaBowieShoreline::waitingToCoolDown_Motors(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
}

void MegaBowieShoreline::reactivateAfterCoolDown_Motors() {
  if(!PREVENT_OVER_CURRENT) return;
}

void MegaBowieShoreline::overCurrentThreshold_Motors(bool first) {
  if(!PREVENT_OVER_CURRENT) return;
}


/*

---- Combined movements ----

*/

void MegaBowieShoreline::moveArmAndEnd(int armPos, int step, int del, int armMin, int armMax, int endMin, int endMax) {
  
  // TODO
  /*

  bool did_move_hopper = false;
  int hopper_original_pos = bowiehopper.getHopperPos();
  int endPos = 0;

  if(bowiearm.getArmPos() == ARM_MIN && bowiescoop.getScoopPos() < END_MAX) { // check if the arm is down and if the end is going past being down
    Serial << "!!! Cannot move end-effector here when arm down" << endl;
    return;
  }
  
  bowiearm.unparkArm();
  bowiescoop.unparkScoop();

  if(bowiehopper.getHopperPos() == TILT_MIN) { // check if the hopper is up
    bowiehopper.moveHopper(TILT_MAX); // move it flush if not
    did_move_hopper = true;
  }

  if(bowiearm.getArmPos() > armPos) { // headed towards ARM_MIN
    for(int i=bowiearm.getArmPos(); i>armPos; i-=step) {
      bowiearm.arm.writeMicroseconds(i);
      bowiearm.arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      endPos = clawParallelValBounds(i, armMin, armMax, endMin, endMax);
      bowiescoop.end.writeMicroseconds(endPos);
      arm_position = i;
      end_position = endPos;
      delay(del);
      servoInterruption(SERVO_ARM_AND_END_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  } else if(getArmPos() <= armPos) { // headed towards ARM_MAX
    for(int i=getArmPos(); i<armPos; i+=step) {
      arm.writeMicroseconds(i);
      arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      endPos = clawParallelValBounds(i, armMin, armMax, endMin, endMax);
      end.writeMicroseconds(endPos);
      arm_position = i;
      end_position = endPos;
      delay(del); 
      servoInterruption(SERVO_ARM_AND_END_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  }
  arm.writeMicroseconds(armPos);
  arm2.writeMicroseconds(SERVO_MAX_US - armPos + SERVO_MIN_US);
  endPos = clawParallelValBounds(armPos, armMin, armMax, endMin, endMax);
  end.writeMicroseconds(endPos);
  arm_position = armPos;
  end_position = endPos;
  delay(del);
  servoInterruption(SERVO_ARM_AND_END_KEY, armPos);
  if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving

  if(did_move_hopper) { // move hopper back to original position
    moveHopper(hopper_original_pos);
  }

  */

}

// ---------
// this code is from Micah Black, during Random Hacks of Kindness in Ottawa 2017!

//get a parallel claw value
int MegaBowieShoreline::clawParallelVal(int arm_Val) {
  return 1;
  // TODO
  //return (int)constrain( map(arm_Val, ARM_MIN, ARM_MAX, END_PARALLEL_BOTTOM, END_PARALLEL_TOP) , END_PARALLEL_BOTTOM, END_PARALLEL_TOP);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.

//get a parallel claw value
int MegaBowieShoreline::clawParallelValBounds(int arm_Val, int armMin, int armMax, int endMin, int endMax){
  return 1;
  // TODO
  //return (int)constrain( map(arm_Val, armMin, armMax, endMin, endMax) , endMin, endMax);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.



/*

---- Specific ----

*/

void MegaBowieShoreline::beep() {
  tone(SPEAKER, 150, 100);
}
