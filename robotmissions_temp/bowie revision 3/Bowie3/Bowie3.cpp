#include "Bowie3.h"

// ---- Inits & Updates

Bowie::Bowie() {
  msgs_in_queue = 0;
  msg_send_index = 0;
  unlikely_count = 0;

  gpio_pin1_input = true;
  gpio_pin2_input = true;
  gpio_pin3_input = true;
  gpio_pin4_input = true;
  gpio_pin5_input = true;

  current_motor = 0;
  current_servo = 0;
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
  sonar_val_left = 0;
  sonar_val_right = 0;
  force_sensor_val_left = 0;
  force_sensor_val_right = 0;
  gpio_pin1_val = 0;
  gpio_pin2_val = 0;
  gpio_pin3_val = 0;
  gpio_pin4_val = 0;
  gpio_pin5_val = 0;

  GYRO_ENABLED = false;
  MAG_ENABLED = false;
  ACCEL_ENABLED = false;
  BMP_ENABLED = false;
}

void Bowie::init() {
  Serial << "Bowie init" << endl;

  initLeds();
  initMotors();
  initServos();
  //initSensors();
  
  //initGPIO(1, 1);
  //initGPIO(2, 1);
  //initGPIO(3, 1);
  //initGPIO(4, 1);
  //initGPIO(5, 1);

}

void Bowie::update() {


  // while(1<3) {

  //   for(int i=ARM_MIN; i<ARM_MAX; i++) {
  //     arm.write(i);
  //     //arm2.write(180-i);
  //     delay(10);
  //   }

  //   for(int i=ARM_MAX; i>ARM_MIN; i--) {
  //     arm.write(i);
  //     //arm2.write(180-i);
  //     delay(10);
  //   }

  //   for(int i=CLAW_MAX; i<CLAW_MIN; i++) {
  //     claw.writeMicroseconds(i);
  //     delay(10);
  //   }

  //   for(int i=CLAW_MIN; i>CLAW_MAX; i--) {
  //     claw.writeMicroseconds(i);
  //     delay(10);
  //   } 

  // }


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

  // TODO: if a button / sensor gets triggered here, 
  // send it via insertNextMsg();


  // update all the sensors
  sonar_val_left = analogRead(SONAR_LEFT);
  sonar_val_right = analogRead(SONAR_RIGHT);
  force_sensor_val_left = analogRead(FORCE_SENSOR_LEFT);
  force_sensor_val_right = analogRead(FORCE_SENSOR_RIGHT);
  if(gpio_pin1_input) gpio_pin1_val = analogRead(GPIO_PIN1);
  if(gpio_pin2_input) gpio_pin2_val = analogRead(GPIO_PIN2);
  if(gpio_pin3_input) gpio_pin3_val = analogRead(GPIO_PIN3);
  if(gpio_pin4_input) gpio_pin4_val = analogRead(GPIO_PIN4);
  if(gpio_pin5_input) gpio_pin5_val = analogRead(GPIO_PIN5);

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
      m.priority = 2;
      m.action = '$';
      m.cmd = 'F';
      m.key = 1;
      m.val = force_sensor_val_left;
      m.cmd2 = 'F';
      m.key2 = 2;
      m.val2 = force_sensor_val_right;
    break;
    case 4: 
      // sonar sensor L & R
      m.priority = 2;
      m.action = '$';
      m.cmd = 'U';
      m.key = 1;
      m.val = sonar_val_left;
      m.cmd2 = 'U';
      m.key2 = 2;
      m.val2 = sonar_val_right;
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
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 1;
      m.val = gpio_pin1_val;
      m.cmd2 = 'I';
      m.key2 = 2;
      m.val2 = gpio_pin2_val;
    break;
    case 8:
      // gpio 3 & 4
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 3;
      m.val = gpio_pin3_val;
      m.cmd2 = 'I';
      m.key2 = 4;
      m.val2 = gpio_pin4_val;
    break;
    case 9:
      // gpio 5 & temperature
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 5;
      m.val = gpio_pin5_val;
      m.cmd2 = 'T';
      m.key2 = 1;
      m.val2 = temp_msg;
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

  Serial << "insert ";

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

      if(packets[i].cmd == 'L') { // left motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          analogWrite(BRIGHT_LED_LEFT, MAX_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_FWD);
          motor_setSpeed(0, val);
        } else if(packets[i].key == 0) { // bwd
          analogWrite(BRIGHT_LED_LEFT, MIN_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_REV);
          motor_setSpeed(0, val);
        }
      }

      if(packets[i].cmd == 'R') { // right motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          analogWrite(BRIGHT_LED_RIGHT, MAX_BRIGHTNESS);
          motor_setDir(1, MOTOR_DIR_FWD);
          motor_setSpeed(1, val);
        } else if(packets[i].key == 0) { // bwd
          analogWrite(BRIGHT_LED_RIGHT, MIN_BRIGHTNESS);
          motor_setDir(1, MOTOR_DIR_REV);
          motor_setSpeed(1, val);
        }
      }

      if(packets[i].cmd == 'P') { // red button
        if(packets[i].val == 1) {
        }
      }

      if(packets[i].cmd == 'Y') { // yellow button
        if(packets[i].val == 1) { // arm down
          
          arm.write(ARM_HOME);
          arm2.write(180-ARM_HOME);
          delay(50);

          for(int i=ARM_HOME; i>ARM_MIN; i--) {
            arm.write(i);
            arm2.write(180-i);
            delay(10);
          }

          arm.write(ARM_MIN);
          arm2.write(180-ARM_MIN);

        } else if(packets[i].val == 0) { // back to home

          arm.write(ARM_MIN);
          arm2.write(180-ARM_MIN);
          delay(50);

          for(int i=ARM_MIN; i<ARM_HOME; i++) {
            arm.write(i);
            arm2.write(180-i);
            delay(10);
          }

          arm.write(ARM_HOME);
          arm2.write(180-ARM_HOME);

        }
      }

      if(packets[i].cmd == 'B') { // blue button
        if(packets[i].val == 1) { // arm up
          
          arm.write(ARM_HOME);
          arm2.write(180-ARM_HOME);
          delay(50);

          for(int i=ARM_HOME; i<ARM_MAX; i++) {
            arm.write(i);
            arm2.write(180-i);
            delay(10);
          }

          arm.write(ARM_MAX);
          arm2.write(180-ARM_MAX);

        } else if(packets[i].val == 0) { // arm home

          arm.write(ARM_MAX);
          arm2.write(180-ARM_MAX);
          delay(50);

          for(int i=ARM_MAX; i>ARM_HOME; i--) {
            arm.write(i);
            arm2.write(180-i);
            delay(10);
          }

          arm.write(ARM_HOME);
          arm2.write(180-ARM_HOME);

        }
      }

      if(packets[i].cmd == 'W') { // white button
        if(packets[i].val == 1) {
          analogWrite(BRIGHT_LED_RIGHT, MAX_BRIGHTNESS);
          analogWrite(BRIGHT_LED_LEFT, MAX_BRIGHTNESS);
          delay(500);
          analogWrite(BRIGHT_LED_RIGHT, MIN_BRIGHTNESS);
          analogWrite(BRIGHT_LED_LEFT, MIN_BRIGHTNESS);
          delay(500);
        }
      }

      if(packets[i].cmd == 'G') { // green button
        if(packets[i].val == 1) { // scoop up
         
          claw.writeMicroseconds(CLAW_HOME);
          delay(50);

          for(int i=CLAW_HOME; i>CLAW_MAX; i--) {
            claw.writeMicroseconds(i);
            delay(1);
          }

          claw.writeMicroseconds(CLAW_MAX);

        } else if(packets[i].val == 0) { // scoop home

          claw.writeMicroseconds(CLAW_MAX);
          delay(50);

          for(int i=CLAW_MAX; i<CLAW_HOME; i++) {
            claw.writeMicroseconds(i);
            delay(1);
          }

          claw.writeMicroseconds(CLAW_HOME);

        }
      }

      if(packets[i].cmd == 'N') { // black button
        if(packets[i].val == 1) { // scoop down
         
          claw.writeMicroseconds(CLAW_HOME);
          delay(50);

          for(int i=CLAW_HOME; i<CLAW_MIN; i++) {
            claw.writeMicroseconds(i);
            delay(1);
          }

          claw.writeMicroseconds(CLAW_MIN);

        } else if(packets[i].val == 0) { // scoop to home

          claw.writeMicroseconds(CLAW_MIN);
          delay(50);

          for(int i=CLAW_MIN; i>CLAW_HOME; i--) {
            claw.writeMicroseconds(i);
            delay(1);
          }

          claw.writeMicroseconds(CLAW_HOME);

        }
      }

      if(packets[i].cmd == 'Q') {
        if(packets[i].val == 0) {
          if(packets[i].key == 0) {
            digitalWrite(BRIGHT_LED_LEFT, LOW);
          } else if(packets[i].key == 1) {
            digitalWrite(BRIGHT_LED_RIGHT, LOW);
          }
        } else if(packets[i].val == 255) {
          if(packets[i].key == 0) {
            digitalWrite(BRIGHT_LED_LEFT, HIGH);
          } else if(packets[i].key == 1) {
            digitalWrite(BRIGHT_LED_RIGHT, HIGH);
          }
        } else {
          if(packets[i].key == 0) {
            analogWrite(BRIGHT_LED_LEFT, val);
          } else if(packets[i].key == 1) {
            analogWrite(BRIGHT_LED_RIGHT, val);
          }
        }
      }

    }
  } // -- end of '#' action specifier


  if(action == '@') {

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'L') { // left motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          analogWrite(BRIGHT_LED_LEFT, MAX_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_FWD);
          motor_setSpeed(0, val);
        } else if(packets[i].key == 0) { // bwd
          analogWrite(BRIGHT_LED_LEFT, MIN_BRIGHTNESS);
          motor_setDir(0, MOTOR_DIR_REV);
          motor_setSpeed(0, val);
        }
      }

      if(packets[i].cmd == 'R') { // right motor
        if(packets[i].val > 255) packets[i].key = 99; // something weird here, set key to skip
        if(packets[i].key == 1) { // fwd
          digitalWrite(BRIGHT_LED_RIGHT, HIGH);
          motor_setDir(1, MOTOR_DIR_FWD);
          motor_setSpeed(1, val);
        } else if(packets[i].key == 0) { // bwd
          digitalWrite(BRIGHT_LED_RIGHT, LOW);
          motor_setDir(1, MOTOR_DIR_REV);
          motor_setSpeed(1, val);
        }
      }

      if(packets[i].cmd == 'S') { // arm (data from 0-45)
        int the_pos = (int)map(val, 0, 45, ARM_MIN, ARM_MAX);

        arm.write(the_pos);
        arm2.write(180-the_pos);

        Serial << "\narm angle: " << the_pos << endl;
      }

      if(packets[i].cmd == 'C') { // claw / scoop
        int the_pos = (int)map(val, 0, 45, CLAW_MIN, CLAW_MAX);
        claw.writeMicroseconds(the_pos);

        Serial << "\nclaw angle: " << the_pos << endl;
      }

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
  hopper_pivot.attach(SERVO_HOPPER_PIVOT);
  hopper_lid.attach(SERVO_HOPPER_LID);
  extra.attach(SERVO_EXTRA);

  //arm.write(180-ARM_HOME);
  //arm2.write(ARM_HOME);
  //claw.writeMicroseconds(CLAW_HOME);
}

void Bowie::initSensors() {
  pinMode(CURRENT_SERVO_SENS, INPUT);
  pinMode(CURRENT_MOTOR_SENS, INPUT);

  /*
  pinMode(FORCE_SENSOR_LEFT, INPUT);
  pinMode(FORCE_SENSOR_RIGHT, INPUT);
  pinMode(SONAR_LEFT, INPUT);
  pinMode(SONAR_RIGHT, INPUT);
  */
}

void Bowie::initLeds() {
  pinMode(BOARD_LED, OUTPUT);
  pinMode(COMM_LED, OUTPUT);
  pinMode(BRIGHT_LED_FRONT_LEFT, OUTPUT);
  pinMode(BRIGHT_LED_FRONT_RIGHT, OUTPUT);
  pinMode(BRIGHT_LED_BACK_LEFT, OUTPUT);
  pinMode(BRIGHT_LED_BACK_RIGHT, OUTPUT);
}

void Bowie::initGPIO(uint8_t p, uint8_t state) {
  switch(p) {
    case 1:
      if(state == 0) {
        pinMode(GPIO_PIN1, OUTPUT);
      } else if(state == 1) {
        pinMode(GPIO_PIN1, INPUT);
      }
    break;
    case 2:
      if(state == 0) {
        pinMode(GPIO_PIN2, OUTPUT);
      } else if(state == 1) {
        pinMode(GPIO_PIN2, INPUT);
      }
    break;
    case 3:
      if(state == 0) {
        pinMode(GPIO_PIN3, OUTPUT);
      } else if(state == 1) {
        pinMode(GPIO_PIN3, INPUT);
      }
    break;
    case 4:
      if(state == 0) {
        pinMode(GPIO_PIN4, OUTPUT);
      } else if(state == 1) {
        pinMode(GPIO_PIN4, INPUT);
      }
    break;
    case 5:
      if(state == 0) {
        pinMode(GPIO_PIN5, OUTPUT);
      } else if(state == 1) {
        pinMode(GPIO_PIN5, INPUT);
      }
    break;
  }
}



// ---- LEDs

void Bowie::turnOnLights() {
  digitalWrite(BRIGHT_LED_FRONT_LEFT, HIGH);
  digitalWrite(BRIGHT_LED_FRONT_RIGHT, HIGH);
  digitalWrite(BRIGHT_LED_BACK_LEFT, HIGH);
  digitalWrite(BRIGHT_LED_BACK_RIGHT, HIGH);
}




// ---- Motors
// Thanks to Randy Glenn for writing this code before a Field Test in 2016!

void Bowie::motor_setDir(uint8_t motorNum, bool dir) {
  if(0 == motorNum) {
    if(dir) {
      digitalWrite(MOTORA_CTRL1, HIGH);
      digitalWrite(MOTORA_CTRL2, LOW);        
    } else {
      digitalWrite(MOTORA_CTRL1, LOW);
      digitalWrite(MOTORA_CTRL2, HIGH);
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



