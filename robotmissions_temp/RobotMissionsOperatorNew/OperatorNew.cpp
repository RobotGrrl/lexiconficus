#include "OperatorNew.h"

/*

Message priority list
0 = none
1 = button presses
2 = servos
3 = driving
...
9 = none

*/

// ---- Init & Updates

Operator::Operator() {

  // update
  current_time = 0;

  // msgs
  msgs_in_queue = 0;

  // joystick
  first_idle = false;
  joystick_idle = false;
  last_idle_update = 0;
  last_activity = 0;
  
  // modes
  turn_on_spot = true;
  digitalWrite(LED3, HIGH);
  slower_speed = false;
  slow_speed = 100;

  // buttons
  redbouncer = Bounce();
  greenbouncer = Bounce();
  yellowbouncer = Bounce();
  bluebouncer = Bounce();
  whitebouncer = Bounce();
  blackbouncer = Bounce();
  red_on = false;
  green_on = false;
  yellow_on = false;
  blue_on = false;
  white_on = false;
  black_on = false;
  auton_blink = false;
  last_auton_blink = 0;

}

void Operator::init() {
  initLeds();
  initButtons();
  initJoystick();
}

void Operator::update() {
  current_time = millis();
  updateButtons();

  // by slowing down the updates when the nunchuck is idle, we will receive the
  // replies from the robot less frequently, thereby letting us check the buttons
  // more frequently, so that the experience can be more responsive
  if(joystick_idle == true && current_time-last_idle_update >= IDLE_UPDATE_FREQ) {
    if(red_on) joystickControl();
    last_idle_update = current_time;
  } else {
    if(red_on) joystickControl();
  }
  
}

void Operator::breatheLeds() {

  for(int i=0; i<=256; i+=2) {
    analogWrite(LED1, i);
    analogWrite(LED2, i);
    analogWrite(LED3, i);
    analogWrite(LED4, i);
    analogWrite(LED5, i);
    analogWrite(LED6, i);
    delay(10);
  }

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, HIGH);
  digitalWrite(LED5, HIGH);
  digitalWrite(LED6, HIGH);
  delay(100);

  for(int i=256; i>=0; i-=2) {
    analogWrite(LED1, i);
    analogWrite(LED2, i);
    analogWrite(LED3, i);
    analogWrite(LED4, i);
    analogWrite(LED5, i);
    analogWrite(LED6, i);
    delay(10);
  }

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  delay(100);

}


// ---- Joystick

void Operator::joystickControl() {

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

  joy_x = analogRead(JOYSTICK_X);
  joy_y = analogRead(JOYSTICK_Y);
  joy_sw = analogRead(JOYSTICK_SW);

  //Serial << "X: " << joy_x << " Y: " << joy_y << " SW: " << joy_sw << endl;

  // checking to see if the nunchuck has been idle
  if(joy_y >= (HOME_Y-5) && joy_y <= (HOME_Y+5)
    && joy_x >= (HOME_X-5) && joy_x <= (HOME_Y+5) ) {
    if(first_idle) last_activity = current_time;

    if(current_time-last_activity >= ACTIVITY_TIMEOUT) {
      joystick_idle = true;
    }
  } else {
    first_idle = false;
    joystick_idle = false;
  }

  // drive

  int motor_speed = 0;
  int speed_r = 0;
  int speed_l = 0;
  boolean motor_dir = true;

  if(joy_y > MAX_Y) joy_y = MAX_Y;
  if(joy_y < MIN_Y) joy_y = MIN_Y;
  if(joy_x > MAX_X) joy_x = MAX_X;
  if(joy_x < MIN_X) joy_x = MIN_X;

  bool speed_adj = false;

  if(joy_y >= (HOME_Y-10) && joy_y <= (HOME_Y+10)
     && joy_x >= (HOME_X-10) && joy_x <= (HOME_X+10)) {
    
    speed_l = 0;
    speed_r = 0;

    // stand still
    m.priority = 3;
    m.action = '#';
    m.cmd = 'L';
    m.key = 0;
    m.val = speed_l;
    m.cmd2 = 'R';
    m.key2 = 0;
    m.val2 = speed_r;
    m.delim = '!';
    
  } else if(joy_x >= (MAX_X-100) && ( joy_y >= (HOME_Y-30) && joy_y <= (HOME_Y+30) ) ) {
  
    // hard turn to the left (so rev right higher)
    m.priority = 3;
    m.action = '#';
    m.cmd = 'L';
    m.key = 1;
    m.val = 255;
    m.cmd2 = 'R';
    m.key2 = 0;
    m.val2 = 255;
    m.delim = '!';

    if(OP_DEBUG) Serial << " hard left" << endl;
  
  } else if(joy_x <= (MIN_X+100) && ( joy_y >= (HOME_Y-30) && joy_y <= (HOME_Y+30) )) {

    // hard turn to the right (so rev left higher)

    m.priority = 3;
    m.action = '#';
    m.cmd = 'L';
    m.key = 0;
    m.val = 255;
    m.cmd2 = 'R';
    m.key2 = 1;
    m.val2 = 255;
    m.delim = '!';

    if(OP_DEBUG) Serial << " hard right" << endl;

  } else {

    if(joy_y > HOME_Y) {
      motor_speed = map(joy_y, HOME_Y, MAX_Y, 0, 255);
      motor_dir = true;
    } else if(joy_y < HOME_Y) {
      motor_speed = map(joy_y, HOME_Y, MIN_Y, 0, 255);
      motor_dir = false;
    }

    // sending the data
    if(motor_dir) {
      m.priority = 3;
      m.action = '#';
      m.cmd = 'L';
      m.key = 1;
      m.val = motor_speed;
      m.cmd2 = 'R';
      m.key2 = 1;
      m.val2 = motor_speed;
      m.delim = '!';
    } else {
      m.priority = 3;
      m.action = '@';
      m.cmd = 'L';
      m.key = 0;
      m.val = motor_speed;
      m.cmd2 = 'R';
      m.key2 = 0;
      m.val2 = motor_speed;
      m.delim = '!';
    }

    if(OP_DEBUG) Serial << " motor_speed: " << motor_speed << endl;

    /*
    float percent_l = (float)( map(joy_x, MIN_X, MAX_X, 0, 100) );
    percent_l /= 100.0;
    float percent_r = 1.0 - percent_l;

    speed_l = (int)(percent_l * (float)motor_speed);
    speed_r = (int)(percent_r * (float)motor_speed);    
    */

  }

  insertNextMsg(m);

}


// ---- Messages

uint8_t Operator::getMsgQueueLength() {
  return msgs_in_queue;
}

Msg Operator::popNextMsg() {
  struct Msg m = msg_queue[0];

  for(int i=0; i<msgs_in_queue-1; i++) {
    msg_queue[i] = msg_queue[i+1];
  }

  if(msgs_in_queue > 0) msgs_in_queue--;
  return m;
}

void Operator::addNextMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
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

void Operator::addNextMsg(Msg m) {
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(COMM_DEBUG) {
      Serial.print(F("Cannot add msg to queue, number of messages in queue: "));
      Serial.println(msgs_in_queue);
    }
  }
  msg_queue[msgs_in_queue] = m;
  msgs_in_queue++;
}

void Operator::insertNextMsg(Msg m) {

  if(msgs_in_queue == 0) {
    if(OP_DEBUG) Serial.println(F("Adding it to index 0"));
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


// ---- Buttons

void Operator::updateButtons() {
  redbouncer.update();
  greenbouncer.update();
  yellowbouncer.update();
  bluebouncer.update();
  whitebouncer.update();
  blackbouncer.update();

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

  if(redbouncer.fell()) {

    if(BUTTON_DEBUG) Serial.print(F("red button (P)"));

    red_on = !red_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'P';
    m.key = 0;

    if(red_on) {
      m.val = 1;
    } else {
      m.val = 0;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(red_on) {
      digitalWrite(LED1, HIGH);
    } else {
      digitalWrite(LED1, LOW);
    }

    addNextMsg(m);
    
  }
  
  if(greenbouncer.fell()) {

    if(BUTTON_DEBUG) Serial.print(F("green button (G)"));

    green_on = !green_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'G';
    m.key = 0;

    if(green_on) {
      m.val = 1;
    } else {
      m.val = 0;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(green_on) {
      digitalWrite(LED3, HIGH);
    } else {
      digitalWrite(LED3, LOW);
    }

    addNextMsg(m);

  }

  if(yellowbouncer.fell()) { // changing turn on the spot mode
    
    if(BUTTON_DEBUG) Serial.print(F("yellow button (Y)"));

    yellow_on = !yellow_on;
    
    m.priority = 1;
    m.action = '#';
    m.cmd = 'Y';
    m.key = 0;

    if(yellow_on) {
      m.val = 1;
    } else {
      m.val = 0;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';
    
    if(yellow_on) {
      digitalWrite(LED2, HIGH);
      //turn_on_spot = true;
    } else {
      digitalWrite(LED2, LOW);
      //turn_on_spot = false;
    }

    addNextMsg(m);

  }
  
  if(bluebouncer.fell()) { // changing slow speed mode
    
    if(BUTTON_DEBUG) Serial.print(F("blue button (B)"));

    blue_on = !blue_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'B';
    m.key = 0;

    if(blue_on) {
      m.val = 1;
    } else {
      m.val = 0;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(blue_on) {
      digitalWrite(LED5, HIGH);
      //slower_speed = true;
    } else {
      digitalWrite(LED5, LOW);
      //slower_speed = false;
    }

    addNextMsg(m);
    
  }
  
  if(whitebouncer.fell()) {

    if(BUTTON_DEBUG) Serial.print(F("white button (W)"));

    white_on = !white_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'W';
    m.key = 0;

    if(white_on) {
      m.val = 1;
    } else {
      m.val = 0;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(white_on) {
      digitalWrite(LED4, HIGH);
    } else {
      digitalWrite(LED4, LOW);
    }

    addNextMsg(m);
    
  }

  if(white_on) {
    if(current_time-last_auton_blink >= 500) {

      m.priority = 1;
      m.action = '#';
      m.cmd = 'Q';
      m.key = 1;
      m.val = 0;
      m.cmd2 = 'Q';
      m.key2 = 1;
      m.val2 = 0;
      m.delim = '!';

      if(auton_blink) {
        m.val = 255;
        m.val2 = 255;
      } else {
        m.val = 128;
        m.val2 = 128;
      }
      addNextMsg(m);
      last_auton_blink = current_time;
      auton_blink = !auton_blink;
    }
  }

  if(blackbouncer.fell()) {
    
    if(BUTTON_DEBUG) Serial.print(F("black button (N)"));

    black_on = !black_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'N';
    m.key = 0;

    if(black_on) {
      m.val = 1;
    } else {
      m.val = 0;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(black_on) {
      digitalWrite(LED6, HIGH);
    } else {
      digitalWrite(LED6, LOW);
    }

    addNextMsg(m);
    
  }

}

bool Operator::getRedButton() {
  if(red_on) return true;
  return false;
}

bool Operator::getGreenButton() {
  if(green_on) return true;
  return false;
}

bool Operator::getYellowButton() {
  if(yellow_on) return true;
  return false;
}

bool Operator::getBlueButton() {
  if(blue_on) return true;
  return false;
}

bool Operator::getWhiteButton() {
  if(white_on) return true;
  return false;
}

bool Operator::getBlackButton() {
  if(black_on) return true;
  return false;
}

// ---- Init IO

void Operator::initLeds() {
  pinMode(BOARD_LED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
}

void Operator::initButtons() {
  pinMode(REDBUTTON, INPUT);
  pinMode(GREENBUTTON, INPUT);
  pinMode(YELLOWBUTTON, INPUT);
  pinMode(BLUEBUTTON, INPUT);
  pinMode(WHITEBUTTON, INPUT);
  pinMode(BLACKBUTTON, INPUT);

  redbouncer.attach(REDBUTTON);
  redbouncer.interval(DEBOUNCE);

  greenbouncer.attach(GREENBUTTON);
  greenbouncer.interval(DEBOUNCE);

  yellowbouncer.attach(YELLOWBUTTON);
  yellowbouncer.interval(DEBOUNCE);

  bluebouncer.attach(BLUEBUTTON);
  bluebouncer.interval(DEBOUNCE);

  whitebouncer.attach(WHITEBUTTON);
  whitebouncer.interval(DEBOUNCE);

  blackbouncer.attach(BLACKBUTTON);
  blackbouncer.interval(DEBOUNCE);
}

void Operator::initJoystick() {
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_SW, INPUT);
}

