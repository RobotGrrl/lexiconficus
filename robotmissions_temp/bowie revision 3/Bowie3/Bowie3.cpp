#include "Bowie3.h"

// ---- Inits & Updates

Bowie::Bowie() {
  msgs_in_queue = 0;
  msg_send_index = 0;
  unlikely_count = 0;

  gyro_msg_x = 0;
  gyro_msg_y = 0;
  gyro_msg_z = 0;
  mag_msg_x = 0;
  mag_msg_y = 0;
  mag_msg_z = 0;
  accel_msg_x = 0;
  accel_msg_y = 0;
  accel_msg_z = 0;
  alt_msg = 0;
  temp_msg = 0;

  // current sensors
  current_servo_val = 0;
  current_motor_val = 0;
  current_servo = 0.0;
  current_motor = 0.0;
  current_servo_avg = 0.0;
  current_motor_avg = 0.0;
  high_current_detected = false;
  high_current_start = 0;
  servo_current_trigger = 0;
  SERVO_OVER_CURRENT_SHUTDOWN = false;
  num_over_current_shutdowns = 0;
  servo_shutdown_start = 0;

  GYRO_ENABLED = false;
  MAG_ENABLED = false;
  ACCEL_ENABLED = false;
  BMP_ENABLED = false;

  LOG_CURRENT_WHILE_MOVING = false;
  MONITOR_OVER_CURRENT = true;

  arm_position = ARM_PARK;
  end_position = END_PARALLEL_TOP;
  hopper_position = TILT_MAX;
  lid_position = LID_MAX;
  END_TOUCHDOWN = END_PARALLEL_BOTTOM;
  arm_parked = false;
  end_parked = false;
  hopper_parked = false;
  lid_parked = false;
}

void Bowie::init() {
  Serial << "Bowie init" << endl;

  initLeds();
  initMotors();
  initServos();
  initSensors();

}

void Bowie::update() {

  current_time = millis();

  // specific things to do if remote operation is enabled
  if(REMOTE_OP_ENABLED) {

    if(millis()-last_rx >= REMOTE_OP_TIMEOUT) {
      digitalWrite(COMM_LED, LOW);
      motor_setDir(0, MOTOR_DIR_REV);
      motor_setSpeed(0, 0);
      motor_setDir(1, MOTOR_DIR_REV);
      motor_setSpeed(1, 0);
    } else {
      digitalWrite(COMM_LED, HIGH);
    }

  }

  monitorCurrent();
  
  // TODO: if a button / sensor gets triggered here, 
  // send it via insertNextMsg();


  // update all the sensors
  /*
  sonar_val_left = analogRead(SONAR_LEFT);
  sonar_val_right = analogRead(SONAR_RIGHT);
  force_sensor_val_left = analogRead(FORCE_SENSOR_LEFT);
  force_sensor_val_right = analogRead(FORCE_SENSOR_RIGHT);
  if(gpio_pin1_input) gpio_pin1_val = analogRead(GPIO_PIN1);
  if(gpio_pin2_input) gpio_pin2_val = analogRead(GPIO_PIN2);
  if(gpio_pin3_input) gpio_pin3_val = analogRead(GPIO_PIN3);
  if(gpio_pin4_input) gpio_pin4_val = analogRead(GPIO_PIN4);
  if(gpio_pin5_input) gpio_pin5_val = analogRead(GPIO_PIN5);
  */

  // update the gyroscope sensor (L3GD20)
  if(GYRO_ENABLED) {
    //if(SENS_DEBUG) Serial << "gyro- gx: " << gx << " gy: " << gy << " gz: " << gz << endl;
    //if(SENS_DEBUG) Serial << "gyro- gx: " << gyro_msg_x << " gy: " << gyro_msg_y << " gz: " << gyro_msg_z << endl;
  }

  // update the magnetometer sensor (LSM303)
  if(MAG_ENABLED) {
    //if(SENS_DEBUG) Serial << "mag- mx: " << mx << " my: " << my << " mz: " << mz << endl;
    //if(SENS_DEBUG) Serial << "mag- mx: " << mag_msg_x << " my: " << mag_msg_y << " mz: " << mag_msg_z << endl;
  }

  if(ACCEL_ENABLED) {
    //if(SENS_DEBUG) Serial << "accel- ax: " << ax << " ay: " << ay << " az: " << az << endl;
    //if(SENS_DEBUG) Serial << "accel- ax: " << accel_msg_x << " ay: " << accel_msg_y << " az: " << accel_msg_z << endl;
  }

  if(BMP_ENABLED) {
    //if(SENS_DEBUG) Serial << "alt- " << alt << " temp- " << temperature << endl;
    //if(SENS_DEBUG) Serial << "alt- " << alt_msg << " temp- " << temp_msg << endl;
  }

}


/*

Sensor priority list (lower = bigger priority)

0 = none
1 = force & sonar sensors
2 = accel & gyro
3 = gpio
4 = mag

*/

void Bowie::chooseNextMessage() {

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

  switch(msg_send_index) {
    case 0:
      // accelerometer X & accelerometer Y
      m.priority = 2;
      m.action = '$';
      m.cmd = 'A';
      m.key = 1;
      m.val = accel_msg_x;
      m.cmd2 = 'A';
      m.key2 = 2;
      m.val2 = accel_msg_y;
    break;
    case 1:
      // accelerometer Z & gyro X
      m.priority = 2;
      m.action = '$';
      m.cmd = 'A';
      m.key = 3;
      m.val = accel_msg_z;
      m.cmd2 = 'O';
      m.key2 = 1;
      m.val2 = gyro_msg_x;
    break;
    case 2:
      // gyro Y & gyro Z
      m.priority = 2;
      m.action = '$';
      m.cmd = 'O';
      m.key = 2;
      m.val = gyro_msg_y;
      m.cmd2 = 'O';
      m.key2 = 3;
      m.val2 = gyro_msg_z;
    break;
    case 3:
      // force sensor L & R
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'F';
      m.key = 1;
      m.val = 0;//force_sensor_val_left;
      m.cmd2 = 'F';
      m.key2 = 2;
      m.val2 = 0;//force_sensor_val_right;
    break;
    case 4: 
      // sonar sensor L & R
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'U';
      m.key = 1;
      m.val = 0;//sonar_val_left;
      m.cmd2 = 'U';
      m.key2 = 2;
      m.val2 = 0;//sonar_val_right;
    break;
    case 5:
      // magnetometer X & Y
      m.priority = 2;
      m.action = '$';
      m.cmd = 'M';
      m.key = 1;
      m.val = mag_msg_x;
      m.cmd2 = 'M';
      m.key2 = 2;
      m.val2 = mag_msg_y;
    break;
    case 6:
      // magnetometer Z & altitude
      m.priority = 2;
      m.action = '$';
      m.cmd = 'M';
      m.key = 3;
      m.val = mag_msg_z;
      m.cmd2 = 'H';
      m.key2 = 1;
      m.val2 = alt_msg;
    break;
    case 7:
      // gpio 1 & 2
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 1;
      m.val = 0;//gpio_pin1_val;
      m.cmd2 = 'I';
      m.key2 = 2;
      m.val2 = 0;//gpio_pin2_val;
    break;
    case 8:
      // gpio 3 & 4
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 3;
      m.val = 0;//gpio_pin3_val;
      m.cmd2 = 'I';
      m.key2 = 4;
      m.val2 = 0;//gpio_pin4_val;
    break;
    case 9:
      // gpio 5 & temperature
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 5;
      m.val = 0;//gpio_pin5_val;
      m.cmd2 = 'T';
      m.key2 = 1;
      m.val2 = 0;//temp_msg;
    break;
  }

  addNextMsg(m);

  msg_send_index++;
  if(msg_send_index > 9) msg_send_index = 0;

}


// ---- States

void Bowie::enableRemoteOp() {
  REMOTE_OP_ENABLED = true;
}

void Bowie::disableRemoteOp() {
  REMOTE_OP_ENABLED = false;
}


// ---- Messages

uint8_t Bowie::getMsgQueueLength() {
  return msgs_in_queue;
}

Msg Bowie::popNextMsg() {
  struct Msg m = msg_queue[0];

  for(int i=0; i<msgs_in_queue-1; i++) {
    msg_queue[i] = msg_queue[i+1];
  }

  if(msgs_in_queue > 0) msgs_in_queue--;
  return m;
}

void Bowie::addNextMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  Serial.print("adding next message");
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(COMM_DEBUG) {
      Serial.print(F("Cannot add msg to queue, number of messages in queue: "));
      Serial.println(msgs_in_queue);
    }
  }
  msg_queue[msgs_in_queue].priority = priority;
  msg_queue[msgs_in_queue].action = action;
  msg_queue[msgs_in_queue].cmd = cmd;
  msg_queue[msgs_in_queue].key = key;
  msg_queue[msgs_in_queue].val = val;
  msg_queue[msgs_in_queue].cmd2 = cmd2;
  msg_queue[msgs_in_queue].key2 = key2;
  msg_queue[msgs_in_queue].val2 = val2;
  msg_queue[msgs_in_queue].delim = delim;
  msgs_in_queue++;
}

void Bowie::addNextMsg(Msg m) {
  Serial.print("adding next message");
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(COMM_DEBUG) {
      Serial.print(F("Cannot add msg to queue, number of messages in queue: "));
      Serial.println(msgs_in_queue);
    }
  }
  msg_queue[msgs_in_queue] = m;
  msgs_in_queue++;
}

void Bowie::insertNextMsg(Msg m) {

  Serial.print("inserting next message");

  if(msgs_in_queue == 0) {
    if(OP_DEBUG) Serial << "Adding it to index 0" << endl;
    msg_queue[0] = m;
    msgs_in_queue++;   
    return;
  }

  bool completed = false;

  for(int i=0; i<msgs_in_queue; i++) {
    if(m.priority < msg_queue[i].priority) { // overwrite messages that have a greater priority number (meaning they are less of a priority)
      if(OP_DEBUG) Serial << "A: Replacing " << i << " (" << msg_queue[i].priority << ") with new msg (" << m.priority << ")" << endl;
      msg_queue[i] = m;
      completed = true;
      break;
    } else if(m.priority == msg_queue[i].priority) { // if it's the same priority, and the same commands, overwrite with the newest version
      if(m.cmd == msg_queue[i].cmd && m.cmd2 == msg_queue[i].cmd2) {
        if(OP_DEBUG) Serial << "B: Replacing " << i << " (" << msg_queue[i].priority << ") with new msg (" << m.priority << ")" << endl;
        msg_queue[i] = m;
        completed = true;
        break;
      }
    }
  }

  if(!completed) { // if we can't insert it, at least add it
    addNextMsg(m);
  }

}



// ---- Communications API

/*   API
     
     Operator -> Robot
     -----------------
     L = left motor
     R = right motor
     S = arm
     C = claw
     P = red button
     G = green button
     Y = yellow button
     B = blue button
     W = white button
     Q = super bright leds

     Robot -> Operator
     -----------------
     A = accelerometer
     O = gyroscope
     F = force sensors
     U = sonar sensors
     M = magnetometer
     H = altitude
     I = gpio
     T = temperature
*/

void Bowie::control(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  
  last_rx = millis();

  Cmd c1 = { '0', 0, 0 };
  Cmd c2 = { '0', 0, 0 };
  c1.cmd = cmd;
  c1.key = key;
  c1.val = val;
  c2.cmd = cmd2;
  c2.key = key2;
  c2.val = val2;
  Cmd packets[2] = { c1, c2 };

  if(COMM_DEBUG) {
    Serial << "*c1 cmd: " << packets[0].cmd << " key: " << packets[0].key << " val: " << packets[0].val << endl;
    Serial << "*c2 cmd: " << packets[1].cmd << " key: " << packets[1].key << " val: " << packets[1].val << endl;
  }

  // we've seen this happen *sometimes*, and it is highly unlikely that this would be an
  // intentional command. let's make sure they mean this at least 2 times before listening
  
  if(val == 255 && val2 == 255 && cmd == 'L' && cmd2 == 'R') {
    unlikely_count++;
    if(unlikely_count <= 2) return;
  } else {
    unlikely_count = 0;
  }

  if(action == '#') {

    for(int i=0; i<2; i++) {

      /*
      if(packets[i].cmd == 'L') { // left motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          analogWrite(BRIGHT_LED_FRONT_LEFT, MAX_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_FWD);
          motor_setSpeed(0, val);
        } else if(packets[i].key == 0) { // bwd
          analogWrite(BRIGHT_LED_FRONT_LEFT, MIN_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_REV);
          motor_setSpeed(0, val);
        }
      }

      if(packets[i].cmd == 'R') { // right motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          analogWrite(BRIGHT_LED_FRONT_RIGHT, MAX_BRIGHTNESS);
          motor_setDir(1, MOTOR_DIR_FWD);
          motor_setSpeed(1, val);
        } else if(packets[i].key == 0) { // bwd
          analogWrite(BRIGHT_LED_FRONT_RIGHT, MIN_BRIGHTNESS);
          motor_setDir(1, MOTOR_DIR_REV);
          motor_setSpeed(1, val);
        }
      }
      */

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


  if(action == '@') {

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'L') { // left motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          analogWrite(BRIGHT_LED_FRONT_LEFT, MAX_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_FWD);
          motor_setSpeed(0, val);
        } else if(packets[i].key == 0) { // bwd
          analogWrite(BRIGHT_LED_FRONT_LEFT, MIN_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_REV);
          motor_setSpeed(0, val);
        }
      }

      if(packets[i].cmd == 'R') { // right motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          digitalWrite(BRIGHT_LED_FRONT_RIGHT, HIGH);
          motor_setDir(1, MOTOR_DIR_FWD);
          motor_setSpeed(1, val);
        } else if(packets[i].key == 0) { // bwd
          digitalWrite(BRIGHT_LED_FRONT_RIGHT, LOW);
          motor_setDir(1, MOTOR_DIR_REV);
          motor_setSpeed(1, val);
        }
      }
      
      if(packets[i].cmd == 'S') { // arm (data from 0-100)
        
        int the_pos = (int)map(val, 0, 100, ARM_MIN, ARM_MAX);
        int temp_pos = arm_position;
        
        moveArmAndEnd(the_pos, 1, 1, ARM_MIN, ARM_MAX, END_PARALLEL_BOTTOM, END_PARALLEL_TOP);

        /*
        if(abs(the_pos - temp_pos) >= 10) {
          if(the_pos > temp_pos) { // going up
            for(int i=temp_pos; i<the_pos; i+=1) {
              moveArm(i);
              end.writeMicroseconds(clawParallelVal(i));
              delay(3);
            }
          } else if(the_pos < temp_pos) { // going down
            for(int i=temp_pos; i>the_pos; i-=1) {
              moveArm(i);
              end.writeMicroseconds(clawParallelVal(i));
              delay(3);
            }
          }
          
        } else {
          moveArm(the_pos);
          end.writeMicroseconds(clawParallelVal(the_pos));
          delay(1);
        }
        */

        Serial << "\narm angle: " << arm_position << endl;
      }
      
      /*
      if(packets[i].cmd == 'C') { // claw / scoop
        int the_pos = (int)map(val, 0, 45, END_MIN, END_MAX);
        end.writeMicroseconds(the_pos);

        Serial << "\nclaw angle: " << the_pos << endl;
      }
      */

    }
  } // -- end of '@' action specifier
  
}




// ---- Init IO

void Bowie::initMotors() {
  pinMode(MOTORA_SPEED, OUTPUT);
  analogWrite(MOTORA_SPEED, 0);
  pinMode(MOTORB_SPEED, OUTPUT);
  analogWrite(MOTORB_SPEED, 0);
  pinMode(MOTORA_CTRL1, OUTPUT);
  digitalWrite(MOTORA_CTRL1, LOW);
  pinMode(MOTORA_CTRL2, OUTPUT);
  digitalWrite(MOTORA_CTRL2, LOW);
  pinMode(MOTORB_CTRL1, OUTPUT);
  digitalWrite(MOTORB_CTRL1, LOW);
  pinMode(MOTORB_CTRL2, OUTPUT);
  digitalWrite(MOTORB_CTRL2, LOW);
}

void Bowie::initServos() {
  pinMode(SERVO_ARM1, OUTPUT);
  pinMode(SERVO_ARM2, OUTPUT);
  pinMode(SERVO_END_EFFECTOR, OUTPUT);
  pinMode(SERVO_HOPPER_PIVOT, OUTPUT);
  pinMode(SERVO_HOPPER_LID, OUTPUT);
  pinMode(SERVO_EXTRA, OUTPUT);

  arm.attach(SERVO_ARM1);
  arm2.attach(SERVO_ARM2);
  end.attach(SERVO_END_EFFECTOR);
  tilt.attach(SERVO_HOPPER_PIVOT);
  lid.attach(SERVO_HOPPER_LID);
  extra.attach(SERVO_EXTRA);
}

void Bowie::initSensors() {
  pinMode(CURRENT_SERVO_SENS, INPUT);
  pinMode(CURRENT_MOTOR_SENS, INPUT);

  pinMode(SCOOP_PROBE_LEFT, INPUT);
  pinMode(SCOOP_PROBE_RIGHT, INPUT);
}

void Bowie::initLeds() {
  pinMode(BOARD_LED, OUTPUT);
  pinMode(COMM_LED, OUTPUT);
  pinMode(BRIGHT_LED_FRONT_LEFT, OUTPUT);
  pinMode(BRIGHT_LED_FRONT_RIGHT, OUTPUT);
  pinMode(BRIGHT_LED_BACK_LEFT, OUTPUT);
  pinMode(BRIGHT_LED_BACK_RIGHT, OUTPUT);
}


// ---- LEDs

void Bowie::turnOnLights() {
  analogWrite(BRIGHT_LED_FRONT_LEFT, MAX_BRIGHTNESS);
  analogWrite(BRIGHT_LED_FRONT_RIGHT, MAX_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_LEFT, MAX_BRIGHTNESS);
  analogWrite(BRIGHT_LED_BACK_RIGHT, MAX_BRIGHTNESS);
}


// ---- Scoop Probes

void Bowie::updateScoopProbes() {
  scoop_probe_left_val = digitalRead(SCOOP_PROBE_LEFT);
  scoop_probe_right_val = digitalRead(SCOOP_PROBE_RIGHT);
  //Serial << "Scoop probes- L: " << scoop_probe_left_val;
  //Serial << "Scoop probes- R: " << scoop_probe_right_val << endl;
}

bool Bowie::getScoopProbeL() {
  updateScoopProbes();
  return scoop_probe_left_val;
}

bool Bowie::getScoopProbeR() {
  updateScoopProbes();
  return scoop_probe_right_val;
}


// ---- Current Sensors

void Bowie::updateCurrentSensors() {
  current_servo_val = analogRead(CURRENT_SERVO_SENS);
  current_motor_val = analogRead(CURRENT_MOTOR_SENS);
  //Serial << "Current- Servos: " << current_servo_val;
  //Serial << " Motors: " << current_motor_val << endl;
}

uint16_t Bowie::getServoCurrentVal() {
  updateCurrentSensors();
  return current_servo_val;
}

uint16_t Bowie::getMotorCurrentVal() {
  updateCurrentSensors();
  return current_motor_val;
}

float Bowie::getServoCurrent() {
  updateCurrentSensors();
  return current_servo;
}

float Bowie::getMotorCurrent() {
  updateCurrentSensors();
  return current_motor;
}


// ---- Motors
// Thanks to Randy Glenn for writing this code before a Field Test in 2016!

void Bowie::motor_setDir(uint8_t motorNum, bool dir) {
  if(0 == motorNum) {
    if(dir) {
      digitalWrite(MOTORA_CTRL1, LOW);
      digitalWrite(MOTORA_CTRL2, HIGH);        
    } else {
      digitalWrite(MOTORA_CTRL1, HIGH);
      digitalWrite(MOTORA_CTRL2, LOW);
    }
  } else if(1 == motorNum) {
    if(dir) {
      digitalWrite(MOTORB_CTRL2, HIGH);
      digitalWrite(MOTORB_CTRL1, LOW);    
    } else {
      digitalWrite(MOTORB_CTRL2, LOW);
      digitalWrite(MOTORB_CTRL1, HIGH);    
    }    
  }
}

void Bowie::motor_setSpeed(uint8_t motorNum, uint8_t speed) {
  if(0 == motorNum) {
    analogWrite(MOTORA_SPEED, speed);
  } else if(1 == motorNum) {
    analogWrite(MOTORB_SPEED, speed);
  }
}

void Bowie::motor_setBrake(uint8_t motorNum) {
  motor_setSpeed(motorNum, 0);
}

void Bowie::motor_setCoast(uint8_t motorNum) {
  if(0 == motorNum) {
    digitalWrite(MOTORA_CTRL1, LOW);
    digitalWrite(MOTORA_CTRL2, LOW);
    digitalWrite(MOTORA_SPEED, HIGH);
  } else if(1 == motorNum) {
    digitalWrite(MOTORB_CTRL1, LOW);
    digitalWrite(MOTORB_CTRL2, LOW);
    digitalWrite(MOTORB_SPEED, HIGH);
  }
}

void Bowie::leftBork() {
  pinMode(MOTORA_CTRL1, OUTPUT);
  digitalWrite(MOTORA_CTRL1, LOW);
  pinMode(MOTORA_CTRL2, OUTPUT);
  digitalWrite(MOTORA_CTRL2, LOW);
  pinMode(MOTORA_SPEED, OUTPUT);
  analogWrite(MOTORA_SPEED, 0);
}

// ---- Servos

void Bowie::servoInterruption(int key, int val) {
  if(LOG_CURRENT_WHILE_MOVING) {
    updateCurrentSensors();
    Serial << "##,";
    Serial << millis() << ",";
    if(key == SERVO_ARM_KEY) Serial << "Arm,";
    if(key == SERVO_END_KEY) Serial << "End,";
    if(key == SERVO_HOPPER_KEY) Serial << "Hopper,";
    if(key == SERVO_LID_KEY) Serial << "Lid,";
    if(key == LOGGING_AFTER_KEY) Serial << "Finished,";
    if(key == SERVO_ARM_AND_END_KEY) Serial << "Arm & End,";
    Serial << val << ",";
    Serial << current_servo_val << ",";
    Serial << current_motor_val << endl;
  }

  if(MONITOR_OVER_CURRENT) {
    monitorCurrent();
  }

}

void Bowie::monitorCurrent() {

  updateCurrentSensors();

  current_time = millis();

  // notes on calibrating the values:
  // SERVO_CURRENT_THRESH_MAX should not be exceeded when the arm moves
  // regularly from MAX to MIN. there might be the occasional spike,
  // but servo_current_trigger should not exceed 2 or 3.
  // OVER_CURRENT_DELAY is how many of these triggers accumulate within
  // that set time. So if you want the shutdown to occur quickly, this
  // should be adjusted down. 
  // OVER_CURRENT_TRIG_THRESH is how many of the triggers will cause the
  // shutdown, within the delay time. 

  if(current_servo_val > max_current_reading) {
    max_current_reading = current_servo_val;
    Serial << "max: " << max_current_reading << endl;
  }

  //Serial << "Monitoring current " << current_servo_val << endl;

  // servo monitoring with the raw vals. we need this to be fast!
  // pre-check mode - done after servos cool off again (or before they heat up)
  if(!SERVO_OVER_CURRENT_SHUTDOWN) {

    // checking if the val is over the max or below the min - usually it is below the min
    if(current_servo_val >= SERVO_CURRENT_THRESH_MAX || current_servo_val <= SERVO_CURRENT_THRESH_MIN) {

      //Serial << "Monitoring current " << current_servo_val << endl;

      Serial << "!!! SERVO OVER CURRENT DETECTED !!!" << endl;

      // resetting our flags if this was first instance
      if(!high_current_detected) {
        high_current_start = current_time;
        servo_current_trigger = 0;
        high_current_detected = true;
      }

      // incrementing trigger count on detection
      if(high_current_detected) {
        servo_current_trigger++;
        Serial << "Servo over current trigger count: " << servo_current_trigger << endl;
      }

    }

    if(high_current_detected) {

      // counting how many times the over current threshold has been triggered
      // this seems to work better than going with a timed approach
      if(servo_current_trigger >= OVER_CURRENT_TRIG_THRESH) {

        // set the servo shutdown flags
        if(!SERVO_OVER_CURRENT_SHUTDOWN) servo_shutdown_start = current_time;
        SERVO_OVER_CURRENT_SHUTDOWN = true;

        Serial << "!!! Servos entering over current shutdown !!! " << servo_shutdown_start << endl;
        num_over_current_shutdowns++;

      } else { // if not, then we reset it after time
        if(current_time-high_current_start >= OVER_CURRENT_DELAY) {
          Serial << "reset" << endl;
          high_current_detected = false;
          servo_current_trigger = 0;
        }
      }

    }

  }

  // servo shutdown mode
  if(SERVO_OVER_CURRENT_SHUTDOWN) {

    // has this been multiple over-current instances in a short amount
    // of time? if so, the robot might be in a bad place. we can try
    // to reverse & wiggle its motors

    // checking number of times over current has happened
    if(num_over_current_shutdowns < NUM_OVER_CURRENT_THRESH) {

      // waiting for servo to cool off
      if(current_time-servo_shutdown_start <= OVER_CURRENT_TIMEOUT) {

        Serial << "waiting " << current_time-servo_shutdown_start << endl;

        arm.detach();
        arm2.detach();
        end.detach();

      } else { // now turn it back on

        Serial << "Going to turn on the servos" << endl;

        arm.attach(SERVO_ARM1);
        arm2.attach(SERVO_ARM2);
        end.attach(SERVO_END_EFFECTOR);

        // if only there was a way to detect where the servo is presently, 
        // and then send it slowly back to its original position...

        // reset vars
        SERVO_OVER_CURRENT_SHUTDOWN = false;
        high_current_detected = false;
        servo_current_trigger = 0;

        moveArm(ARM_HOME, 1, 5); // slowly
        moveEnd(END_HOME, 1, 5);
      }

    } else { // this is when the robot might be in a bad place

      Serial << "!!! Num over current > thresh, going to move robot !!!" << endl;

      // detach the servos again
      if(current_time-servo_shutdown_start <= OVER_CURRENT_TIMEOUT) { // waiting for servo to cool off
        arm.detach();
        arm2.detach();
        end.detach();
      }

      // move the robot
      // drive backward a bit
      for(int i=100; i<256; i+=5) {
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, i);
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, i);
        delay(2);
      }
      motor_setDir(0, MOTOR_DIR_REV);
      motor_setSpeed(0, 255);
      motor_setDir(1, MOTOR_DIR_REV);
      motor_setSpeed(1, 255);
      delay(1000);
      
      // stop
      motor_setDir(0, MOTOR_DIR_FWD);
      motor_setSpeed(0, 0);
      motor_setDir(1, MOTOR_DIR_FWD);
      motor_setSpeed(1, 0);
      delay(50);

      Serial << "Going to turn on the servos" << endl;
      delay(1000);

      // start the servos
      arm.attach(SERVO_ARM1);
      arm2.attach(SERVO_ARM2);
      end.attach(SERVO_END_EFFECTOR);

      // if only there was a way to detect where the servo is presently, 
      // and then send it slowly back to its original position...

      // reset vars
      SERVO_OVER_CURRENT_SHUTDOWN = false;
      high_current_detected = false;
      servo_current_trigger = 0;
      // reset the counter to 0 after moving
      num_over_current_shutdowns = 0;

      // slowly move
      moveArm(ARM_HOME, 1, 5);
      moveEnd(END_HOME, 1, 5);

    }
   
  }

  // the num of shutdowns times out after 1 min
  if(current_time-servo_shutdown_start > NUM_OVER_TIMEOUT && num_over_current_shutdowns != 0) {
    Serial << "Number of over current shutdowns is cleared" << endl;
    num_over_current_shutdowns = 0;
  }

  // todo- dc motor monitoring

}

// - Arm
void Bowie::moveArm(int armPos) {
  moveArm(armPos, 1, 3);
}

void Bowie::moveArm(int armPos, int step, int del) {

  if(SERVO_OVER_CURRENT_SHUTDOWN) {
    Serial << "!!! Cannot move arm, in servo over current shutdown !!!" << endl;
    monitorCurrent();
    return;
  }

  bool did_move_hopper = false;
  bool was_hopper_parked = hopper_parked;
  int hopper_original_pos = getHopperPos();
  
  unparkArm();
  
  if(getHopperPos() == TILT_MIN) { // check if the hopper is up
    moveHopper(TILT_MAX); // move it flush if not
    did_move_hopper = true;
  }

  if(getArmPos() > armPos) { // headed towards ARM_MIN
    for(int i=getArmPos(); i>armPos; i-=step) {
      //Serial << i << endl;
      arm.writeMicroseconds(i);
      arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      arm_position = i;      
      delay(del);
      servoInterruption(SERVO_ARM_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  } else if(getArmPos() <= armPos) { // headed towards ARM_MAX
    for(int i=getArmPos(); i<armPos; i+=step) {
      //Serial << i << endl;
      arm.writeMicroseconds(i);
      arm2.writeMicroseconds(SERVO_MAX_US - i + SERVO_MIN_US);
      arm_position = i;
      delay(del); 
      servoInterruption(SERVO_ARM_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  }
  arm.writeMicroseconds(armPos);
  arm2.writeMicroseconds(SERVO_MAX_US - armPos + SERVO_MIN_US);
  arm_position = armPos;
  delay(del);
  servoInterruption(SERVO_ARM_KEY, armPos);
  if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving

  if(did_move_hopper) { // move hopper back to original position
    moveHopper(hopper_original_pos);
    if(was_hopper_parked) parkHopper();
  }

}

void Bowie::parkArm() {
  if(arm_parked) return;
  moveArm(ARM_PARK);
  arm.detach();
  arm2.detach();
  arm_parked = true;
}

void Bowie::unparkArm() {
  if(!arm_parked) return;
  arm.attach(SERVO_ARM1);
  arm2.attach(SERVO_ARM2);
  arm_parked = false;
  moveArm(getArmPos());
}

bool Bowie::getArmParked() {
  return arm_parked;
}

int Bowie::getArmPos() {
  return arm_position;
}

// - End
void Bowie::moveEnd(int endPos) {
  moveEnd(endPos, 1, 3);
}

void Bowie::moveEnd(int endPos, int step, int del) {

  if(SERVO_OVER_CURRENT_SHUTDOWN) {
    Serial << "!!! Cannot move end, in servo over current shutdown !!!" << endl;
    monitorCurrent();
    return;
  }

  //if(getArmPos() == ARM_MIN && endPos < END_MAX) { // check if the arm is down and if the end is going past being down
  // TODO1 fix this later
  if(getArmPos() == ARM_MIN && endPos > END_PARALLEL_BOTTOM) { // check if the arm is down and if the end is going past being down
    Serial << "!!! Cannot move end-effector here when arm down" << endl;
    return;
  }

  unparkEnd();

  if(getEndPos() > endPos) { // going towards END_MIN
    for(int i=getEndPos(); i>endPos; i-=step) {
      end.writeMicroseconds(i);
      end_position = i;
      delay(del);
      servoInterruption(SERVO_END_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  } else if(getEndPos() <= endPos) { // going towards END_MAX
    for(int i=getEndPos(); i<endPos; i+=step) {
      end.writeMicroseconds(i);
      end_position = i;
      delay(del);
      servoInterruption(SERVO_END_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  }
  end_position = endPos;
  end.writeMicroseconds(endPos);
  delay(del);
  servoInterruption(SERVO_END_KEY, endPos);
  if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving

}

void Bowie::parkEnd() {
  if(end_parked) return;
  moveEnd(END_PARALLEL_TOP);
  end.detach();
  end_parked = true;
}

void Bowie::unparkEnd() {
  if(!end_parked) return;
  end.attach(SERVO_END_EFFECTOR);
  end_parked = false;
  moveEnd(END_PARALLEL_TOP);
}

bool Bowie::getEndParked() {
  return end_parked;
}

int Bowie::getEndPos() {
  return end_position;
}

// - Hopper
void Bowie::moveHopper(int hopperPos) {
  moveHopper(hopperPos, 1, 3);
}

void Bowie::moveHopper(int hopperPos, int step, int del) {
  bool did_move_lid = false;
  bool was_lid_parked = lid_parked;
  int lid_original_pos = getLidPos();
  bool did_move_arm = false;
  bool was_arm_parked = arm_parked;
  int arm_original_pos = getArmPos();

  unparkHopper();

  if(hopperPos != TILT_MAX) { // if it's going anywhere other than flush
    if(getLidPos() != LID_MIN) { // check if the lid is not open
      moveLid(LID_MIN); // move the lid open
      did_move_lid = true;
    }
  }
  if(getArmPos() >= ARM_MAX) { // check if the arm is parked or max
    moveArm(ARM_MAX-200); // slightly move the arm out of the way
    did_move_arm = true;
  }
  
  if(getHopperPos() > hopperPos) { // towards TILT_MIN
    for(int i=getHopperPos(); i>hopperPos; i-=step) {
      tilt.writeMicroseconds(i);
      hopper_position = i;
      delay(del);
      servoInterruption(SERVO_HOPPER_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  } else if(getHopperPos() <= hopperPos) { // towards TILT_MAX
    for(int i=getHopperPos(); i<hopperPos; i+=step) {
      tilt.writeMicroseconds(i);
      hopper_position = i;
      delay(del);
      servoInterruption(SERVO_HOPPER_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  }
  tilt.writeMicroseconds(hopperPos);
  hopper_position = hopperPos;
  delay(del);
  servoInterruption(SERVO_HOPPER_KEY, hopperPos);
  if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving

  if(did_move_lid) {
    moveLid(lid_original_pos);
    if(was_lid_parked) parkLid();
  }

  if(did_move_arm) {
    moveArm(arm_original_pos);
    if(was_arm_parked) parkArm();
  }

}

void Bowie::parkHopper() {
  if(hopper_parked) return;
  moveHopper(TILT_MAX);
  tilt.detach();
  hopper_parked = true;
}

void Bowie::unparkHopper() {
  if(!hopper_parked) return;
  tilt.attach(SERVO_HOPPER_PIVOT);
  hopper_parked = false;
  moveHopper(TILT_MAX);
}

bool Bowie::getHopperParked() {
  return hopper_parked;
}

int Bowie::getHopperPos() {
  return hopper_position;
}

// - Lid
void Bowie::moveLid(int lidPos) {
  moveLid(lidPos, 1, 3);
}

void Bowie::moveLid(int lidPos, int step, int del) {
  bool did_move_arm = false;
  bool was_arm_parked = arm_parked;
  int arm_original_pos = getArmPos();

  if(getHopperPos() == TILT_MIN) { // check if the hopper is up
    Serial << "!!! Cannot move lid when hopper is up" << endl;
    return;
  }

  unparkLid();

  if(getArmPos() >= ARM_MAX) { // check if the arm is parked or max
    moveArm(ARM_MAX-200); // slightly move the arm out of the way
    did_move_arm = true;
  }
  
  if(getLidPos() > lidPos) { // going to LID_MIN
    for(int i=getLidPos(); i>lidPos; i-=step) {
      lid.writeMicroseconds(i);
      lid_position = i;
      delay(del);
      servoInterruption(SERVO_LID_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  } else if(getLidPos() <= lidPos) {
    for(int i=getLidPos(); i<lidPos; i+=step) {
      lid.writeMicroseconds(i);
      lid_position = i;
      delay(del);
      servoInterruption(SERVO_LID_KEY, i);
      if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving
    }
  }
  lid.writeMicroseconds(lidPos);
  lid_position = lidPos;
  delay(del);
  servoInterruption(SERVO_LID_KEY, lidPos);
  if(SERVO_OVER_CURRENT_SHUTDOWN) return; // break out of here so the pos doesn't keep moving

  if(did_move_arm) {
    moveArm(arm_original_pos);
    if(was_arm_parked) parkArm();
  }

}

void Bowie::parkLid() {
  if(lid_parked) return;
  moveLid(LID_MAX);
  lid.detach();
  lid_parked = true;
}

void Bowie::unparkLid() {
  if(!lid_parked) return;
  lid.attach(SERVO_HOPPER_LID);
  lid_parked = false;
  moveLid(LID_MAX);
}

bool Bowie::getLidParked() {
  return lid_parked;
}

int Bowie::getLidPos() {
  return lid_position;
}

// - Other
void Bowie::moveArmAndEnd(int armPos, int step, int del, int armMin, int armMax, int endMin, int endMax) {
  
  if(SERVO_OVER_CURRENT_SHUTDOWN) {
    Serial << "!!! Cannot move arm, in servo over current shutdown !!!" << endl;
    monitorCurrent();
    return;
  }

  bool did_move_hopper = false;
  int hopper_original_pos = getHopperPos();
  int endPos = 0;

  if(getArmPos() == ARM_MIN && getEndPos() < END_MAX) { // check if the arm is down and if the end is going past being down
    Serial << "!!! Cannot move end-effector here when arm down" << endl;
    return;
  }
  
  unparkArm();
  unparkEnd();

  if(getHopperPos() == TILT_MIN) { // check if the hopper is up
    moveHopper(TILT_MAX); // move it flush if not
    did_move_hopper = true;
  }

  if(getArmPos() > armPos) { // headed towards ARM_MIN
    for(int i=getArmPos(); i>armPos; i-=step) {
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

}

// ---------
// this code is from Micah Black, during Random Hacks of Kindness in Ottawa 2017

//get a parallel claw value
int Bowie::clawParallelVal(int arm_Val){
  return (int)constrain( map(arm_Val, ARM_MIN, ARM_MAX, END_PARALLEL_BOTTOM, END_PARALLEL_TOP) , END_PARALLEL_BOTTOM, END_PARALLEL_TOP);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.

//get a parallel claw value
int Bowie::clawParallelValBounds(int arm_Val, int armMin, int armMax, int endMin, int endMax){
  return (int)constrain( map(arm_Val, armMin, armMax, endMin, endMax) , endMin, endMax);
} //constrain to make sure that it does not result in a value less than 800 - could make the servo rotate backwards.


//this is a combination of the two indivual movement functions
//next step - make the servos reach their endpoints at the same time
//could also add controling it by degrees - if target is <= 180, map to uS, then continue
//to add - keep claw parallel if only moving the arm
//to add - if the end value is parallel for the claw, move the arm and keep the claw parallel
void Bowie::moveScoop(int targetArmuS, int targetClawuS){ //arm first, claw second

  //claw values
  
  int current_Claw_Val = 1800;

  targetClawuS = constrain(targetClawuS, END_MAX, END_MIN); //claw max is smaller
  int ClawDiff = targetClawuS - current_Claw_Val;
  int ClawDirection;
  int ClawStartuS = current_Claw_Val;
  
  int ClawuSfromStart;
  int ClawuSfromEnd;
  float ClawuSfromPoint;
  
  int ClawIncrement;
  
  //arm values
  int current_Arm_Val = getArmPos();
  targetArmuS = constrain(targetArmuS, ARM_MIN, ARM_MAX);
  int ArmDiff = targetArmuS - current_Arm_Val;
  int ArmDirection;
  int ArmstartuS = current_Arm_Val;
  
  int ArmuSfromStart;
  int ArmuSfromEnd;
  float ArmuSfromPoint;
  
  int ArmIncrement;
  
  //Claw Direction
  if (ClawDiff == 0)
    ClawDirection = 0;
  else if (ClawDiff < 0) //if target is less than current, go backwards
    ClawDirection  = -1;
  else if (ClawDiff > 0)
    ClawDirection = 1; //if target is greater than 0, go forwards (increase servo value)
  else 
    ClawDirection = 0;
  
  //Arm Direction
  if (ArmDiff == 0)
    ArmDirection = 0;
  else if (ArmDiff < 0) //if target is less than current, go backwards
    ArmDirection  = -1;
  else if (ArmDiff > 0)
    ArmDirection = 1; //if target is greater than 0, go forwards (increase servo value)
  else
    ArmDirection = 0;
  
  
  while((current_Arm_Val != targetArmuS) || (current_Claw_Val != targetClawuS)){ //must put both into this loop
    ArmuSfromStart = current_Arm_Val - ArmstartuS;
    ArmuSfromEnd = targetArmuS - current_Arm_Val;
    ArmuSfromStart = abs(ArmuSfromStart); //abs must be comuted on a number, any math should be done outside the function
    ArmuSfromEnd = abs(ArmuSfromEnd);
    ArmuSfromPoint = (float)min(ArmuSfromStart, ArmuSfromEnd);
    
    ClawuSfromStart = current_Claw_Val - ClawStartuS;
    ClawuSfromEnd = targetClawuS - current_Claw_Val;
    ClawuSfromStart = abs(ClawuSfromStart); //abs mClawuSt be comuted on a number, any math should be done outside the function
    ClawuSfromEnd = abs(ClawuSfromEnd);
    ClawuSfromPoint = (float)min(ClawuSfromStart, ClawuSfromEnd);
    
    ClawIncrement = (int)constrain((ClawuSfromPoint / SERVO_MAX_US), 1, 15); //9 is the max servo travel in uS during a 5ms delay
    current_Claw_Val += ClawIncrement * ClawDirection; //forwards or backwards
    
    ArmIncrement = (int)constrain((ArmuSfromPoint / SERVO_MAX_US), 1, 15); //9 is the max servo travel in uS during a 5ms delay
    current_Arm_Val += ArmIncrement * ArmDirection; //forwards or backwards
    
    arm.writeMicroseconds(current_Arm_Val);
    arm2.writeMicroseconds(SERVO_MAX_US-current_Arm_Val+SERVO_MIN_US);
    end.writeMicroseconds(current_Claw_Val);
    delay(1);
  }
  

}




