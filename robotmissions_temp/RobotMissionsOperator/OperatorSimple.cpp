#include "OperatorSimple.h"

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

  // nunchuck
  nunchuk = ArduinoNunchuk();
  first_idle = false;
  nunchuck_idle = false;
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
  red_on = false;
  green_on = false;
  yellow_on = false;
  blue_on = false;
  white_on = false;
  auton_blink = false;
  last_auton_blink = 0;

}

void Operator::init() {
  initLeds();
  initButtons();
  nunchuk.init();
}

void Operator::update() {
  current_time = millis();
  updateButtons();
  nunchuk.update();

  // by slowing down the updates when the nunchuck is idle, we will receive the
  // replies from the robot less frequently, thereby letting us check the buttons
  // more frequently, so that the experience can be more responsive
  if(nunchuck_idle == true && current_time-last_idle_update >= IDLE_UPDATE_FREQ) {
    nunchuckControl();
    last_idle_update = current_time;
  } else {
    nunchuckControl();
  }
  
}


// ---- Nunchuck

void Operator::nunchuckControl() {

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

  Serial << "X: " << nunchuk.analogX << " Y: " << nunchuk.analogY << " C: " << nunchuk.cButton << endl;

  // checking to see if the nunchuck has been idle
  if(nunchuk.analogY >= -5 && nunchuk.analogY <= 5
    && nunchuk.analogX >= -5 && nunchuk.analogX <= 5) {
    if(first_idle) last_activity = current_time;

    if(current_time-last_activity >= ACTIVITY_TIMEOUT) {
      nunchuck_idle = true;
    }
  } else {
    first_idle = false;
    nunchuck_idle = false;
  }

  if(nunchuk.zButton == 0 && nunchuk.cButton == 0) { // drive

    int motor_speed = 0;
    boolean motor_dir = true;

    if(nunchuk.analogY > MAX_Y) nunchuk.analogY = MAX_Y;
    if(nunchuk.analogY < MIN_Y) nunchuk.analogY = MIN_Y;
    if(nunchuk.analogX > MAX_X) nunchuk.analogX = MAX_X;
    if(nunchuk.analogX < MIN_X) nunchuk.analogX = MIN_X;

    bool speed_adj = false;

    if(nunchuk.analogY >= (HOME_Y-10) && nunchuk.analogY <= (HOME_Y+10)
       && nunchuk.analogX >= (HOME_X-10) && nunchuk.analogX <= (HOME_X+10)) {
      
      // stand still
      m.priority = 3;
      m.action = '#';
      m.cmd = 'L';
      m.key = 0;
      m.val = 0;
      m.cmd2 = 'R';
      m.key2 = 0;
      m.val2 = 0;
      m.delim = '!';
      
    } else if(nunchuk.analogY >= (HOME_Y-10) && nunchuk.analogY <= (HOME_Y+10)) { 
      
      // turning
      
      if(nunchuk.analogX >= (MIN_X+10)) {
        if(!turn_on_spot) {
          m.priority = 3;
          m.action = '@';
          m.cmd = 'L';
          m.key = 1;
          m.val = 64;
          m.cmd2 = 'R';
          m.key2 = 1;
          m.val2 = 255;
          m.delim = '!';
        } else {
          // previously:
          m.priority = 3;
          m.action = '@';
          m.cmd = 'L';
          m.key = 0;
          m.val = 255;
          m.cmd2 = 'R';
          m.key2 = 1;
          m.val2 = 255;
          m.delim = '!';
        }
      }
      if(nunchuk.analogX <= (MAX_X-10)) {
        if(!turn_on_spot) {
          m.priority = 3;
          m.action = '@';
          m.cmd = 'L';
          m.key = 1;
          m.val = 255;
          m.cmd2 = 'R';
          m.key2 = 1;
          m.val2 = 64;
          m.delim = '!';
        } else {
          // previously:
          m.priority = 3;
          m.action = '@';
          m.cmd = 'L';
          m.key = 1;
          m.val = 255;
          m.cmd2 = 'R';
          m.key2 = 0;
          m.val2 = 255;
          m.delim = '!';
        }
      }
      
    } else if(nunchuk.analogY >= HOME_Y) { // fwd

      motor_speed = map(nunchuk.analogY, HOME_Y, MAX_Y, 0, 255);
      motor_dir = true;
      speed_adj = true;

      if(slower_speed) motor_speed = map(nunchuk.analogY, HOME_Y, MAX_Y, 0, slow_speed);

    } else { // bwd
      
      motor_speed = map(nunchuk.analogY, MIN_Y, HOME_Y, 255, 0);
      motor_dir = false;
      speed_adj = true;

      if(slower_speed) motor_speed = map(nunchuk.analogY, MIN_Y, HOME_Y, slow_speed, 0);

    }

    if(speed_adj) { // calculate the motor speed for L and R
      
      float percent_r = (float)map(nunchuk.analogX, MIN_X, MAX_X, 0, 100);
      percent_r /= 100.0;
      float percent_l = 1.0-percent_r;
  
      int speed_r = (int)((float)motor_speed * percent_r);
      int speed_l = (int)((float)motor_speed * percent_l);
  
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

      if(OP_DEBUG) Serial << " speed L: " << speed_l << " R: " << speed_r << endl;

    }

    if(OP_DEBUG) {
      if(motor_dir) {
        Serial << "FWD- ";
      } else {
        Serial << "BWD- ";
      }
    }
    
  } else if(nunchuk.zButton == 1 && nunchuk.cButton == 0) { // arm

    uint8_t servo_pos = map(nunchuk.analogY, MIN_Y, MAX_Y, 0, 45);

    m.priority = 2;
    m.action = '#';
    m.cmd = 'S';
    m.key = 0;
    m.val = servo_pos;
    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(OP_DEBUG) Serial << "arm pos: " << servo_pos << endl;
    
  } else if(nunchuk.zButton == 0 && nunchuk.cButton == 1) {

    uint8_t servo_pos = map(nunchuk.analogY, MIN_Y, MAX_Y, 0, 45);

    m.priority = 2;
    m.action = '#';
    m.cmd = 'C';
    m.key = 0;
    m.val = servo_pos;
    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(OP_DEBUG) Serial << "claw pos: " << servo_pos << endl;
    
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

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

  if(redbouncer.fell()) {

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
      digitalWrite(LED2, HIGH);
    } else {
      digitalWrite(LED2, LOW);
    }

    addNextMsg(m);

  }

  if(yellowbouncer.fell()) { // changing turn on the spot mode
    
    yellow_on = !yellow_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'Y';
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
    
    if(yellow_on) {
      digitalWrite(LED3, HIGH);
      turn_on_spot = true;
    } else {
      digitalWrite(LED3, LOW);
      turn_on_spot = false;
    }

    addNextMsg(m);

  }
  
  if(bluebouncer.fell()) { // changing slow speed mode
    
    blue_on = !blue_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'B';
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

    if(blue_on) {
      digitalWrite(LED4, HIGH);
      slower_speed = true;
    } else {
      digitalWrite(LED4, LOW);
      slower_speed = false;
    }

    addNextMsg(m);
    
  }
  
  if(whitebouncer.fell()) {

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
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, HIGH);
    } else {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
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
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
        digitalWrite(LED4, HIGH);
        m.val = 255;
        m.val2 = 255;
      } else {
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        digitalWrite(LED4, LOW);
        m.val = 128;
        m.val2 = 128;
      }
      addNextMsg(m);
      last_auton_blink = current_time;
      auton_blink = !auton_blink;
    }
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


// ---- Init IO

void Operator::initLeds() {
  pinMode(BOARD_LED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
}

void Operator::initButtons() {
  pinMode(REDBUTTON, INPUT);
  pinMode(GREENBUTTON, INPUT);
  pinMode(YELLOWBUTTON, INPUT);
  pinMode(BLUEBUTTON, INPUT);
  pinMode(WHITEBUTTON, INPUT);

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
}

