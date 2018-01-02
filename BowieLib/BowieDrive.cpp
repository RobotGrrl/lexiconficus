#include "BowieDrive.h"

BowieDrive::BowieDrive() {

  // states
  TURN_SEQUENCE_MODE = true;

  // other
  current_time = 0;

  // info
  motor_a_speed = 0;
  motor_b_speed = 0;
  motor_a_dir = true;
  motor_b_dir = true;

  // driving algorithms
  turn_sequence_step = 0;
  restart_step_timer = true;
  step_start = 0;

}

void BowieDrive::initMotors() {
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

void BowieDrive::setMotorASpeedPin(uint8_t p) {
  MOTORA_SPEED = p;
}

void BowieDrive::setMotorBSpeedPin(uint8_t p) {
  MOTORB_SPEED = p;
}

void BowieDrive::setMotorACtrl1Pin(uint8_t p) {
  MOTORA_CTRL1 = p;
}

void BowieDrive::setMotorACtrl2Pin(uint8_t p) {
  MOTORA_CTRL2 = p;
}

void BowieDrive::setMotorBCtrl1Pin(uint8_t p) {
  MOTORB_CTRL1 = p;
}

void BowieDrive::setMotorBCtrl2Pin(uint8_t p) {
  MOTORB_CTRL2 = p;
}


/*

---- Motors ----
Thanks to Randy Glenn for writing this code before a Field Test in 2016!
*/

void BowieDrive::motor_setDir(uint8_t motorNum, bool dir) {
  if(0 == motorNum) {
    if(dir) {
      digitalWrite(MOTORA_CTRL1, LOW);
      digitalWrite(MOTORA_CTRL2, HIGH);
      motor_a_dir = true;
    } else {
      digitalWrite(MOTORA_CTRL1, HIGH);
      digitalWrite(MOTORA_CTRL2, LOW);
      motor_a_dir = false;
    }
  } else if(1 == motorNum) {
    if(dir) {
      digitalWrite(MOTORB_CTRL2, LOW);
      digitalWrite(MOTORB_CTRL1, HIGH);  
      motor_b_dir = true;  
    } else {
      digitalWrite(MOTORB_CTRL2, HIGH);
      digitalWrite(MOTORB_CTRL1, LOW);
      motor_b_dir = false;
    }    
  }
}

void BowieDrive::motor_setSpeed(uint8_t motorNum, uint8_t speed) {
  if(0 == motorNum) {
    analogWrite(MOTORA_SPEED, speed);
    motor_a_speed = speed;
  } else if(1 == motorNum) {
    analogWrite(MOTORB_SPEED, speed);
    motor_b_speed = speed;
  }
}

void BowieDrive::motor_setBrake(uint8_t motorNum) {
  motor_setSpeed(motorNum, 0);
}

void BowieDrive::motor_setCoast(uint8_t motorNum) {
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

void BowieDrive::leftBork() {
  pinMode(MOTORA_CTRL1, OUTPUT);
  digitalWrite(MOTORA_CTRL1, LOW);
  pinMode(MOTORA_CTRL2, OUTPUT);
  digitalWrite(MOTORA_CTRL2, LOW);
  pinMode(MOTORA_SPEED, OUTPUT);
  analogWrite(MOTORA_SPEED, 0);
}


/*

---- Driving Algorithms ----

*/

void BowieDrive::rampSpeed(bool dir, int start, int end, int step, int del) {
  if(start < end) {
    for(int i=start; i<end; i+=step) {

      if(dir) {
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, i);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, i);
      } else {
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, i);
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, i);
      }
      delay(del);

    }
  } else if(start > end) {
    for(int i=start; i>end; i-=step) {

      if(dir) {
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, i);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, i);
      } else {
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, i);
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, i);
      }
      delay(del);

    }
  }

  if(dir) {
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, end);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, end);
  } else {
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, end);
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, end);
  }

}

void BowieDrive::goSpeed(bool dir, int speed, int del) {
  if(dir) {
    motor_setDir(0, MOTOR_DIR_FWD);
    motor_setSpeed(0, speed);
    motor_setDir(1, MOTOR_DIR_FWD);
    motor_setSpeed(1, speed);
  } else {
    motor_setDir(0, MOTOR_DIR_REV);
    motor_setSpeed(0, speed);
    motor_setDir(1, MOTOR_DIR_REV);
    motor_setSpeed(1, speed);
  }
  delay(del);
}

void BowieDrive::turnSequence(bool dir) { // true = right, false = left

  current_time = millis();
  uint8_t max_steps = 5;

  // start the step timer whenever its a new sequence or flag set
  if(restart_step_timer == true) {
    step_start = current_time;
    restart_step_timer = false;
  }

  // reset the count
  if(turn_sequence_step > max_steps) {
    turn_sequence_step = 0;
  }

  Serial << "Turn sequence step " << turn_sequence_step << endl;

  // all the steps, at the end of each step, counter is incremented
  // and flag is set to restart the timer
  switch(turn_sequence_step) {
    case 0:
      if(current_time-step_start < 400) {
        // turn a portion
        Serial << "Turning" << endl;
        if(dir) { // right
          motor_setDir(0, MOTOR_DIR_FWD);
          motor_setSpeed(0, 255);
          motor_setDir(1, MOTOR_DIR_REV);
          motor_setSpeed(1, 255);
        } else {
          motor_setDir(0, MOTOR_DIR_REV);
          motor_setSpeed(0, 255);
          motor_setDir(1, MOTOR_DIR_FWD);
          motor_setSpeed(1, 255);
        }
      } else {
        turn_sequence_step++;
        restart_step_timer = true;
      }
    break;
    case 1:
      if(current_time-step_start < 50) {
        // stop a bit
        Serial << "Stopping" << endl;
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 0);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 0);
      } else {
        turn_sequence_step++;
        restart_step_timer = true;
      }
    break;
    case 2:
      if(current_time-step_start < 300) {
        // drive forward a bit
        Serial << "Driving forward" << endl;
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 255);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 255);
      } else {
        turn_sequence_step++;
        restart_step_timer = true;
      }
    break;
    case 3:
      if(current_time-step_start < 50) {
        // stop a bit
        Serial << "Stopping" << endl;
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 0);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 0);
      } else {
        turn_sequence_step++;
        restart_step_timer = true;
      }
    break;
    case 4:
      if(current_time-step_start < 450) {
        // drive backward a bit
        Serial << "Driving backward" << endl;
        motor_setDir(0, MOTOR_DIR_REV);
        motor_setSpeed(0, 255);
        motor_setDir(1, MOTOR_DIR_REV);
        motor_setSpeed(1, 255);
      } else {
        turn_sequence_step++;
        restart_step_timer = true;
      }
    break;
    case 5:
      if(current_time-step_start < 50) {
        // stop a bit
        Serial << "Stopping" << endl;
        motor_setDir(0, MOTOR_DIR_FWD);
        motor_setSpeed(0, 0);
        motor_setDir(1, MOTOR_DIR_FWD);
        motor_setSpeed(1, 0);
      } else {
        turn_sequence_step++;
        restart_step_timer = true;
      }
    break;
  }

}

void BowieDrive::resetTurnSequence() {
  restart_step_timer = true;
  turn_sequence_step = 0;
}


/*

---- Getters ----

*/

int BowieDrive::getMotorASpeed() {
  return motor_a_speed;
}

int BowieDrive::getMotorBSpeed() {
  return motor_b_speed;
}

bool BowieDrive::getMotorADir() {
  return motor_a_dir;
}

bool BowieDrive::getMotorBDir() {
  return motor_b_dir;
}

