#include "OperatorInterface.h"

Operator::Operator() : display(OLED_RESET) {

  // Instance of the class for the callbacks from Promulgate
  opInstance = this;

  // LED
  COMM_LED = 13;
  CONN_TYPE = XBEE_CONN;
  AUTOCONNECT = true;

  // Promulgate
  current_time = 0;
  diff_time = 0;
  last_rx_msg = 0;
  last_transmit = 0;
  use_base64_parsing = false;

  // Xbee
  xbee = XBee();
  addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
  txStatus = ZBTxStatusResponse();
  rx = ZBRxResponse();
  for(int i=0; i<64; i++) {
    message_tx[i] = '0';
    message_rx[i] = '0';
  }
  msg_tx_count = 0;
  msg_rx_count = 0;
  msg_tx_err = 0;
  last_rx = 0;
  last_tx = 0;
  last_retry_time = 0;

  // Xbee Robots
  num_addrs = 0;
  ind_addr_sent = 0;
  for(int i=0; i<MAX_ROBOTS; i++) {
    addr_all_robots[i] = XBeeAddress64(0, 0);
    ids_of_all_robots[i] = 0;
    failed_send_count[i] = 0;
    last_rx_all[i] = 0;
    SELECTED_ROBOT_ID[i] = 0;
  }
  last_rx_check = 0;
  last_led_blink = 0;
  led_on = false;
  last_retry_time = 0;
  SELECTED_ROBOT = false;
  num_robot_conn = 0;

  // Messages
  msgs_in_queue = 0;
  msg_send_index = 0;
  last_rx_comms = 0;
  unlikely_count = 0;
  
  // Op-specific Comms
  retry_time = DEFAULT_RETRY_TIME;
  retry_count = 0;

  // Display
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)  
  display.clearDisplay(); // Clear the buffer.
  display.display();

  // Pins
  button_pins[0] = BUTTON1;
  button_pins[1] = BUTTON2;
  button_pins[2] = BUTTON3;
  button_pins[3] = BUTTON4;
  button_pins[4] = BUTTON5;
  button_pins[5] = BUTTON6;
  button_pins[6] = JOYSTICK_SW;

  led_pins[0] = LED1;
  led_pins[1] = LED2;
  led_pins[2] = LED3;
  led_pins[3] = LED4;
  led_pins[4] = LED5;
  led_pins[5] = LED6;
  led_pins[6] = BOARD_LED;

  for(int i=0; i<7; i++) {
    button_states[i] = 0;
    bounce_buttons[i] = Bounce();
  }

  // State
  GO_TIME = false;
  CURRENT_STATE = IDLE_STATE;
  LAST_STATE = CURRENT_STATE;

  // Mode
  CURRENT_MODE = MODE1;

  // Joystick
  HOME_X = 347;
  HOME_Y = 356;
  first_idle = false;
  joystick_idle = false;
  last_idle_update = 0;
  last_activity = 0;
  arm_pos = 50;
  
  // Misc
  turn_on_spot = true;
  slower_speed = false;
  slow_speed = 100;
  letter_itr = 0;
  last_letter_itr = 0;

  // Control
  last_increment = 0;
  scrolling_up = false;
  turn_speed = 0;

  // LEDs
  last_led_blink = 0;
  led_on = false;
}

void Operator::set_comms_timeout_callback( void (*commsTimeoutCallback)() ) {
  _commsTimeoutCallback = commsTimeoutCallback;
}

void Operator::set_controller_added_callback( void (*controllerAddedCallback)() ) {
  _controllerAddedCallback = controllerAddedCallback;
}

void Operator::set_controller_removed_callback( void (*controllerRemovedCallback)() ) {
  _controllerRemovedCallback = controllerRemovedCallback;
}

void Operator::set_received_action_callback( void (*receivedActionCallback)(Msg m) ) {
  _receivedActionCallback = receivedActionCallback;
}

void Operator::set_button_changed_callback( void (*buttonChangedCallback)(int button, int value) ) {
  _buttonChangedCallback = buttonChangedCallback;
}

void Operator::setCommLed(uint8_t pin) {
  COMM_LED = pin;
}

void Operator::setAutoconnect(bool b) {
  AUTOCONNECT = b;
}

unsigned long Operator::getCommLatency() {
  return diff_time;
}

unsigned long Operator::getLastRXTime() {
  return last_rx_comms;
}

void Operator::initOperator(int conn, int baud) {

  bool possible = false;

  if(conn == USB_CONN) {

    Serial1.begin(baud);
    // Promulgate
    promulgate = Promulgate(&Serial1, &Serial1);

    CONN_TYPE = USB_CONN;
    possible = true;

  } else if(conn == XBEE_CONN) {
    
    // Start xbee's serial
    Serial1.begin(baud);
    xbee.begin(Serial1);

    // Promulgate
    promulgate = Promulgate(&Serial1, &Serial1);

    CONN_TYPE = XBEE_CONN;
    possible = true;

  } else if(conn == BT_CONN) {

    Serial1.begin(baud);
    // Promulgate
    promulgate = Promulgate(&Serial1, &Serial1);

    CONN_TYPE = BT_CONN;
    possible = true;

  }

  if(possible) {
    Serial.println("Connection set up OK");
  } else {
    Serial.println("ERROR SETTING UP CONNECTION - UART DOES NOT EXIST");
  }

  // promulgate setup
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);
  promulgate.set_debug_stream(&Serial);

  if(use_base64_parsing) {
    promulgate.useBase64Parsing(true);
  }

  initLeds();
  initButtons();
  initJoystick();
  displayLogo();
  initSpeaker();
  introLedSequence();
}

void Operator::updateOperator() {

  // Conn Comms
  connBlink();
  
  // Xbee Specific
  if(CONN_TYPE == XBEE_CONN) xbeeWatchdog();

  // Heartbeat
  // If we haven't selected a robot yet, send out our ID
  // more frequently.
  if(!SELECTED_ROBOT) {
    if(current_time-last_retry_time >= 500) {
      if(CONN_TYPE == XBEE_CONN) {
        // send it directly, bypassing the queue
        xbeeSendToList('$', 'X', 1, 50, 'X', 1, 50, '!');
      } else if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {
        connSend('$', 'X', 1, 50, 'X', 1, 50, '!');
      }
      last_retry_time = current_time;
    }
  } else {
    // If not, send our ID less frequently
    if(current_time-last_retry_time >= 2500) {
      addMsg( 3, '$', 'X', 1, 50, 'X', 1, 50, '!' );
      last_retry_time = current_time;
    }
  }

  // Reading the data from the Stream
  connRead();

  // Retry
  if(current_time-last_rx_msg >= retry_time && retry_count < 5) { // retry 5 times
    Serial.println(F("Retrying send"));
    Serial << getMsgQueueLength() << " msgs in queue" << endl;
    if(getMsgQueueLength() <= 0) {
      addMsg(msg_none);
    }
    sendNextMsg();
    retry_time = SECONDARY_RETRY_TIME;
    last_rx_msg = current_time;
    retry_count++;
  }
  
  // Comms have timed out
  if(millis()-last_rx_comms >= REMOTE_OP_TIMEOUT) {
    digitalWrite(COMM_LED, LOW);
    // callback that the comms has timed out
    _commsTimeoutCallback();
  } else {
    digitalWrite(COMM_LED, HIGH);
  }

  // Update our interface
  updateButtons();
  updateModeSwitch();

  // Display
  mainMenu();

  // Let's choose the robot in XBee mode
  xbeeChooseRobotToConnect();

  // TODO
  if(CURRENT_MODE == MODE1) {

    if(SELECTED_ROBOT) {
      if(CURRENT_STATE == DRIVE_STATE) joystickDriveControl();
      if(CURRENT_STATE == ARM_STATE) joystickArmControl();
    }

  }

}

void Operator::xbeeChooseRobotToConnect() {

  if(SELECTED_ROBOT == false && CONN_TYPE == XBEE_CONN) {
    
    // Say we're searching while waiting
    displaySearchingAnimation();

    for(int i=0; i<3; i++) {
      if(ids_of_all_robots[i] != 0) {
        display.setCursor(i*50,22);
        display.print(ids_of_all_robots[i]);
      }
    }

    for(int i=3; i<6; i++) {
      if(ids_of_all_robots[i] != 0) {
        display.setCursor(i*50,50);
        display.print(ids_of_all_robots[i]);
      }
    }

    display.display();

    if(!AUTOCONNECT) {

      if(getJoystickButton() == 1) {
        Serial << "They have chosen their robots!" << endl;

        for(int i=0; i<6; i++) {
          if(getButton(i) == 1) {
            SELECTED_ROBOT_ID[num_robot_conn] = ids_of_all_robots[i];
            num_robot_conn++;
          }
        }

      }

    } else {

      if(num_addrs > 0) { // as soon as we see one, connect to it
        SELECTED_ROBOT_ID[num_robot_conn] = ids_of_all_robots[0];
        num_robot_conn++;
      }

    }

    if(num_robot_conn > 0) {  
      SELECTED_ROBOT = true;
      CURRENT_STATE = IDLE_STATE;
      resetButtonStates();
    }

  }

}


/*

---- Joystick ----

*/

void Operator::calibrateHome() {

  int num = 10;
  int x_avg = 0;
  int y_avg = 0;
  for(int i=0; i<num; i++) {
    x_avg += getJoyX();
    y_avg += getJoyY();
    delay(20);
    Serial << i << " ";
  }

  HOME_X = (int)floor( (float)x_avg/(float)num );
  HOME_Y = (int)floor( (float)y_avg/(float)num );

}

void Operator::joystickDriveControl() {

  Msg m = msg_none;

  joy_x_prev = joy_x;
  joy_y_prev = joy_y;
  joy_x = analogRead(JOYSTICK_X);
  joy_y = analogRead(JOYSTICK_Y);

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

  int speed_r = 0;
  int speed_l = 0;
  boolean motor_dir = true;

  if(joy_y > MAX_Y) joy_y = MAX_Y;
  if(joy_y < MIN_Y) joy_y = MIN_Y;
  if(joy_x > MAX_X) joy_x = MAX_X;
  if(joy_x < MIN_X) joy_x = MIN_X;

  bool speed_adj = false;

  // TODO
  /*
  if(joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE)
     && joy_x >= (HOME_X-ZERO_ZONE) && joy_x <= (HOME_X+ZERO_ZONE)) {
    
    motor_speed = 0;
    //turn_speed = 0;

    // stand still
    m.priority = 3;
    m.action = '@';
    m.cmd = 'L';
    m.key = 0;
    m.val = motor_speed;
    m.cmd2 = 'R';
    m.key2 = 0;
    m.val2 = motor_speed;
    m.delim = '!';

    Serial << "stand still" << endl;
    
  } else if(joy_x >= (MAX_X-150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) ) ) {
  
    m.priority = 3;
    m.action = '@';
    m.cmd = 'L';
    m.key = 0;
    m.val = TURN_SPEED_REV;
    m.cmd2 = 'R';
    m.key2 = 1;
    m.val2 = TURN_SPEED_FWD;
    m.delim = '!';

    if(OP_DEBUG) Serial << " hard left" << endl;
  
  } else if(joy_x <= (MIN_X+150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) )) {

    m.priority = 3;
    m.action = '@';
    m.cmd = 'L';
    m.key = 1;
    m.val = TURN_SPEED_FWD;
    m.cmd2 = 'R';
    m.key2 = 0;
    m.val2 = TURN_SPEED_REV;
    m.delim = '!';

    if(OP_DEBUG) Serial << " hard right" << endl;

  } else {

    if(joy_y >= (HOME_Y+ZERO_ZONE)) { 

      // forwards

      if(current_time - last_increment > 10) {
        incr_speed = 4;//map(joy_x, (MIN_X+150), HOME_X, 1, 10);
        last_increment = current_time;
      } else {
        incr_speed = 0;  
      }
      
      if(joy_y >= joy_y_prev) { // going faster
        scrolling_up = true;
      } else if(joy_y < joy_y_prev) { // going slower
        scrolling_up = false;
      }

      if(scrolling_up) {
        motor_speed += incr_speed;
      } else {
        motor_speed -= incr_speed;
      }

      if(motor_speed > MAX_SPEED) motor_speed = MAX_SPEED;
      if(motor_speed < MIN_SPEED) motor_speed = MIN_SPEED;
      motor_dir = true;

      Serial << "motor_speed: " << motor_speed << endl;

    } else if(joy_y <= (HOME_Y-ZERO_ZONE)) { 

      // backwards

      if(current_time - last_increment > 10) {
        incr_speed = 2;//map(joy_x, (MIN_X+150), HOME_X, 1, 10);
        last_increment = current_time;
      } else {
        incr_speed = 0;  
      }
      
      if(joy_y <= joy_y_prev) { // going faster
        scrolling_up = false;
      } else if(joy_y > joy_y_prev) { // going slower
        scrolling_up = true;
      }

      if(scrolling_up) {
        motor_speed -= incr_speed;
      } else {
        motor_speed += incr_speed;
      }

      if(motor_speed > MAX_SPEED) motor_speed = MAX_SPEED;
      if(motor_speed < MIN_SPEED) motor_speed = MIN_SPEED;
      motor_dir = false;

      Serial << "motor_speed: " << motor_speed << endl;

    }

    // sending the data
    if(motor_dir) {
      m.priority = 3;
      m.action = '@';
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

  }
  */

  insertMsg(m);

}

void Operator::joystickArmControl() {

  current_time = millis();

  Msg m = msg_none;

  joy_x_prev = joy_x;
  joy_y_prev = joy_y;
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

  boolean motor_dir = true;

  if(joy_y > MAX_Y) joy_y = MAX_Y;
  if(joy_y < MIN_Y) joy_y = MIN_Y;
  if(joy_x > MAX_X) joy_x = MAX_X;
  if(joy_x < MIN_X) joy_x = MIN_X;

  // TODO
  /*
  if(joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE)
     && joy_x >= (HOME_X-ZERO_ZONE) && joy_x <= (HOME_X+ZERO_ZONE)) {
    
    incr_speed = 0;

    // stand still
    m.priority = 3;
    m.action = '@';
    m.cmd = 'S';
    m.key = 1;
    m.val = 0;//arm_pos;
    m.cmd2 = 'q';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';
    
  } else if(joy_x >= (MAX_X-150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) ) ) {
  
    // hard turn to the left
  
  } else if(joy_x <= (MIN_X+150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) )) {

    // hard turn to the right

  } else {


    if(joy_y >= (HOME_Y+ZERO_ZONE)) { 

      // forwards

      if(current_time - last_increment > 100) {
        //incr_speed++;
        incr_speed = (int)map(joy_y, HOME_Y, MAX_Y, 1, 100);
        last_increment = current_time;
      }
      
      if(incr_speed > 100) incr_speed = 100;

      Serial << "^ Arm incr: " << incr_speed << endl;
      m.key = 1;
      
    } else if(joy_y <= (HOME_Y-ZERO_ZONE)) { 

      // backwards

      if(current_time - last_increment > 100) {
        //incr_speed++;
        incr_speed = (int)map(joy_y, HOME_Y, MIN_Y, 1, 100);
        last_increment = current_time;
      }
      
      if(incr_speed > 100) incr_speed = 100;

      m.key = 0;

      Serial << "v Arm incr: " << incr_speed << endl;

    }

    // sending the data
    m.priority = 3;
    m.action = '@';
    m.cmd = 'S';
    
    m.val = incr_speed;
    m.cmd2 = 'q';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    if(OP_DEBUG) Serial << " incr_speed: " << incr_speed << endl;

  }
  */

  insertMsg(m);

}

int Operator::getJoyX() {
  return analogRead(JOYSTICK_X);
}

int Operator::getJoyY() {
  return analogRead(JOYSTICK_Y);
}


/*

---- Buttons ----

*/

void Operator::updateButtons() {
  
  Msg m = msg_none;
  bool did_change = false;

  for(int i=0; i<7; i++) {
    bounce_buttons[i].update();
    if(bounce_buttons[i].fell()) {
      if(i < 6) {
        Serial << "Button " << i << endl;
      } else {
        Serial << "Joystick button" << endl;
      }
      if(button_states[i] == 0) {
        button_states[i] = 1;
        did_change = true;
      } else if(button_states[i] == 1) {
        button_states[i] = 0;
        did_change = true;
      }
    }

    if(did_change) {

      _buttonChangedCallback(i, button_states[i]);

      m.priority = 1;
      m.action = '#';
      m.pck1.key = 0;
      m.pck1.val = button_states[i];
      m.delim = '!';
      switch(i) {
        case 0: // 'red'
          m.pck1.cmd = 'P';
        break;
        case 1: // yellow
          m.pck1.cmd = 'Y';
        break;
        case 2: // green
          m.pck1.cmd = 'G';
        break;
        case 3: // white
          m.pck1.cmd = 'W';
        break;
        case 4: // blue
          m.pck1.cmd = 'B';
        break;
        case 5: // black
          m.pck1.cmd = 'N';
        break;
        case 6: // joystick
          m.pck1.cmd = 'J';
        break;
      }
    }
    addMsg(m);
  }

}

bool Operator::getButton(uint8_t b) {
  if(button_states[b] == 1) return true;
  return false;
}

bool Operator::getJoystickButton() {
  if(button_states[6] == 1) return true;
  return false;
}

void Operator::resetButtonStates() {
  for(int i=0; i<7; i++) {
    button_states[i] = 0;
  }
  ledsOff();
}

/*

---- Mode Switch ----

*/

void Operator::updateModeSwitch() {

  int val = analogRead(MODE_SW);

  // TODO check previous & do a callback

  if(val >= MODE1_THRESH) {
    CURRENT_MODE = MODE1;
  } else if(val >= MODE2_THRESH) {
    CURRENT_MODE = MODE2;
  } else if(val >= MODE3_THRESH) {
    CURRENT_MODE = MODE3;
  }

}


/*

---- Speaker ----

*/

void Operator::buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000 / frequency / 2;
  long numCycles = frequency * length / 1000;
  for (long i = 0; i < numCycles; i++) { 
    digitalWrite(targetPin, HIGH); 
    delayMicroseconds(delayValue); 
    digitalWrite(targetPin, LOW); 
    delayMicroseconds(delayValue); 
  }
}


/*

---- Init ----

*/

void Operator::initLeds() {
  pinMode(COMM_LED, OUTPUT);
  for(int i=0; i<7; i++) {
    pinMode(led_pins[i], OUTPUT);
  }
}

void Operator::initButtons() {
  for(int i=0; i<7; i++) {
    pinMode(button_pins[i], INPUT_PULLUP);
    bounce_buttons[i].attach(button_pins[i]);
    bounce_buttons[i].interval(DEBOUNCE);
  }
  pinMode(MODE_SW, INPUT);
}

void Operator::initJoystick() {
  pinMode(JOYSTICK_X, INPUT_PULLUP);
  pinMode(JOYSTICK_Y, INPUT_PULLUP);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
}

void Operator::initSpeaker() {
  pinMode(SPEAKER, OUTPUT);
  // beep beep
  buzz(SPEAKER, 1568, 100);
  delay(100);
  buzz(SPEAKER, 2349, 100);
  delay(100);
  buzz(SPEAKER, 4699, 100);
  delay(100);
}


/*

---- Messages ----

*/

uint8_t Operator::getMsgQueueLength() {
  return msgs_in_queue;
}

// Retrieve the next message, and move the other messages behind it up
Msg Operator::popNextMsg() {
  struct Msg m = msg_queue[0];

  for(int i=0; i<msgs_in_queue-1; i++) {
    msg_queue[i] = msg_queue[i+1];
  }

  if(msgs_in_queue > 0) msgs_in_queue--;
  return m;
}

void Operator::addMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  //Serial.print("adding next message");
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(COMM_DEBUG) {
      Serial.print(F("Cannot add msg to queue, number of messages in queue: "));
      Serial.println(msgs_in_queue);
    }
  }
  msg_queue[msgs_in_queue].priority = priority;
  msg_queue[msgs_in_queue].action = action;
  msg_queue[msgs_in_queue].pck1.cmd = cmd;
  msg_queue[msgs_in_queue].pck1.key = key;
  msg_queue[msgs_in_queue].pck1.val = val;
  msg_queue[msgs_in_queue].pck2.cmd = cmd2;
  msg_queue[msgs_in_queue].pck2.key = key2;
  msg_queue[msgs_in_queue].pck2.val = val2;
  msg_queue[msgs_in_queue].delim = delim;
  msgs_in_queue++;
}

// If the queue is full, we don't add to it. We err this way because by the time
// the rest of the queue has been sent, chances are when we reach the message
// that was attempted to be added — it would be out of date already. With the cycle
// of sensor sends, there will be new data along its way shortly.
void Operator::addMsg(Msg m) {
  //Serial.print("adding next message");
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(COMM_DEBUG) {
      Serial.print(F("Cannot add msg to queue, number of messages in queue: "));
      Serial.println(msgs_in_queue);
    }
  }
  msg_queue[msgs_in_queue] = m;
  msgs_in_queue++;
}

// If the message to be inserted is more priority than the other messages in the queue, 
// then it will insert it at that index, and move the other messages back. This means the
// last message in the queue will be lost.
// If the message is the same priority, and has the same commands (for both packets in 
// the message), then it will replace it. 
// As a failsafe, if it can't be inserted, the message is at least added.
void Operator::insertMsg(Msg m) {

  Serial.print("inserting next message");

  if(msgs_in_queue == 0) {
    if(OP_DEBUG) Serial << "Adding it to index 0" << endl;
    msg_queue[0] = m;
    msgs_in_queue++;
    return;
  }

  bool completed = false;
  uint8_t insert_index = 0;

  for(int i=0; i<msgs_in_queue; i++) {
    if(m.priority < msg_queue[i].priority) { // insert messages that have a greater priority number (meaning they are less of a priority)
      insert_index = i;
      for(int j=msgs_in_queue-1; j>insert_index; j--) {
        msg_queue[j] = msg_queue[j-1];
      }
      if(OP_DEBUG) Serial << "A: Inserting " << i << " (" << msg_queue[i].priority << ") at index: " << insert_index << endl;
      msg_queue[insert_index] = m;
      completed = true;
      break;
    } else if(m.priority == msg_queue[i].priority) { // if it's the same priority, and the same commands, overwrite with the newest version
      if(m.pck1.cmd == msg_queue[i].pck1.cmd && m.pck2.cmd == msg_queue[i].pck2.cmd) {
        if(OP_DEBUG) Serial << "B: Replacing " << i << " (" << msg_queue[i].priority << ") with new msg (" << m.priority << ")" << endl;
        msg_queue[i] = m;
        completed = true;
        break;
      }
    }
  }

  if(!completed) { // if we can't insert it, at least add it
    addMsg(m);
  }

}


/*

---- Xbee ----

*/

void Operator::xbeeSendEasy(char c) {

  sprintf(message_tx,"%c", c);

  Serial << "Xbee TX: " << message_tx << endl;

  if(SELECTED_ROBOT) { // send to everyone as we're in discovery mode
  //if(num_addrs > 0) {
    
    for(int i=0; i<num_addrs; i++) {
      if(addr_all_robots[i].getMsb() != 0 && addr_all_robots[i].getLsb() != 0) {

        for(int j=0; j<num_robot_conn; j++) {
          if(SELECTED_ROBOT_ID[j] != 0) { // we have chosen a robot
            if(ids_of_all_robots[i] == SELECTED_ROBOT_ID[j]) { // this is its id
              Serial << "Sent to list ind " << i << endl;
              ZBTxRequest zbtx = ZBTxRequest(addr_all_robots[i], (uint8_t *)message_tx, strlen(message_tx));
              zbtx.setFrameId('0');
              xbee.send(zbtx); 
              ind_addr_sent = i;
            }
          }
        }

      }
    }

  } else {
    // by default, send to the coordinator of the network
    Serial << "Send to default A" << endl;
    ZBTxRequest zbtx = ZBTxRequest(addr_coord, (uint8_t *)message_tx, strlen(message_tx));
    zbtx.setFrameId('0');
    xbee.send(zbtx); 
  }
 
}

void Operator::xbeeSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  // Promulgate format:
  // *out_stream << action << cmd << key << "," << val << delim;
  
  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);

  if(SELECTED_ROBOT) { // send to everyone as we're in discovery mode
  //if(num_addrs > 0) {

    for(int i=0; i<num_addrs; i++) {
      if(addr_all_robots[i].getMsb() != 0 && addr_all_robots[i].getLsb() != 0) {

        for(int j=0; j<num_robot_conn; j++) {

          if(SELECTED_ROBOT_ID[j] != 0) { // we have chosen a robot

            if(ids_of_all_robots[i] == SELECTED_ROBOT_ID[j]) { // this is its id
              Serial << "Sent " << message_tx << " to list ind " << i << endl;
              ZBTxRequest zbtx = ZBTxRequest(addr_all_robots[i], (uint8_t *)message_tx, strlen(message_tx));
              zbtx.setFrameId('0');
              xbee.send(zbtx); 
              ind_addr_sent = i;
            }
          }
        }
        
      }
    }

  } else {
    // by default, send to the coordinator of the network
    Serial << "Send to default B" << endl;
    ZBTxRequest zbtx = ZBTxRequest(addr_coord, (uint8_t *)message_tx, strlen(message_tx));
    zbtx.setFrameId('0');
    xbee.send(zbtx); 
  }
 
}

void Operator::xbeeSendToList(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  // Promulgate format:
  // *out_stream << action << cmd << key << "," << val << delim;
  
  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);

  if(SELECTED_ROBOT) { // send to everyone as we're in discovery mode
  //if(num_addrs > 0) {

    for(int i=0; i<num_addrs; i++) {
      if(addr_all_robots[i].getMsb() != 0 && addr_all_robots[i].getLsb() != 0) {
      
        Serial << "nnnnnnnnnnnnnnn Sent to list ind " << i << endl;
        ZBTxRequest zbtx = ZBTxRequest(addr_all_robots[i], (uint8_t *)message_tx, strlen(message_tx));
        zbtx.setFrameId('0');
        xbee.send(zbtx); 
        ind_addr_sent = i;
        
      }
    }

  } else {
    // by default, send to the coordinator of the network
    Serial << "Send to default C" << endl;
    ZBTxRequest zbtx = ZBTxRequest(addr_coord, (uint8_t *)message_tx, strlen(message_tx));
    zbtx.setFrameId('0');
    xbee.send(zbtx); 
  }
 
}

void Operator::addXbeeToList(XBeeAddress64 newAddr) {
  Serial << "Sender address - High: ";
  print32Bits(newAddr.getMsb());
  Serial << " Low: ";
  print32Bits(newAddr.getLsb());
  Serial << endl;

  bool add_it_to_list = true;

  for(int i=0; i<MAX_ROBOTS; i++) {
    if(addr_all_robots[i].getMsb() == newAddr.getMsb() && addr_all_robots[i].getLsb() == newAddr.getLsb()) {
      // already on the list
      add_it_to_list = false;
    }
  }

  if(add_it_to_list) {
    Serial << "! New robot added to Xbee list !" << endl;
    addr_all_robots[num_addrs] = XBeeAddress64(newAddr.getMsb(), newAddr.getLsb());
    num_addrs++;
    // TODO send a callback _robotAddedCallback();
  }
  
}

void Operator::updateRxTime(XBeeAddress64 senderLongAddress) {
  for(int i=0; i<MAX_ROBOTS; i++) {
    if(addr_all_robots[i].getMsb() == senderLongAddress.getMsb() && addr_all_robots[i].getLsb() == senderLongAddress.getLsb()) {
      last_rx_all[i] = millis();
      return;
    }
  }
}

void Operator::xbeeWatchdog() {
  if(current_time-last_rx_check >= 10000) {
    for(int i=0; i<MAX_ROBOTS; i++) {
      if(current_time-last_rx_all[i] >= 11000) {
        if(addr_all_robots[i].getMsb() != 0 && addr_all_robots[i].getLsb() != 0) {
          // remove it from the list
          Serial << "Have not heard from: ";
          print32Bits(addr_all_robots[i].getMsb());
          Serial << " ";
          print32Bits(addr_all_robots[i].getLsb());
          Serial << " since " << current_time-last_rx_all[i] << "ms ago." << endl;
          Serial << "! Removing it from list !" << endl;
          Serial << "Its num was: " << ids_of_all_robots[i];
          // check to see if it was a robot we were connected to
          for(int j=0; j<num_robot_conn; j++) {
            // go through the list of all the robots we're
            // connected to
            if(ids_of_all_robots[i] == SELECTED_ROBOT_ID[j]) {
              Serial << " us: " << SELECTED_ROBOT_ID[j] << endl;
              Serial << "Resetting selected robot" << endl;
              SELECTED_ROBOT_ID[j] = 0;
              num_robot_conn--;
              // now we are not connected to any robot
              if(num_robot_conn < 0) num_robot_conn = 0; // just in case
            }
          }
          // reset if we're no longer connected to any robot
          if(num_robot_conn == 0) {
            // TODO
            //resetButtonStates();
            SELECTED_ROBOT = false;
            CURRENT_STATE = IDLE_STATE;
          }
          // remove it from the list
          addr_all_robots[i] = XBeeAddress64(0, 0);
          last_rx_all[i] = 0;
          ids_of_all_robots[i] = 0;
          // then push everything else up
          for(int j=i; j<MAX_ROBOTS-1; j++) {
            addr_all_robots[j] = addr_all_robots[j+1];
            last_rx_all[j] = last_rx_all[j+1];
            ids_of_all_robots[j] = ids_of_all_robots[j+1];
            SELECTED_ROBOT_ID[j] = SELECTED_ROBOT_ID[j+1];
          }
          // so that we can properly deincrement the num_addr counter
          num_addrs--;
          // TODO send a callback _robotRemovedCallback();
        }
      }
    }
    last_rx_check = current_time;
  }
}

bool Operator::xbeeRead() {
  
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) { // we got something

    Serial << "Xbee response: " << xbee.getResponse().getApiId() << endl;

    // --- Node Identifier Response
    // I'm not sure if this one will be good to have for the
    // operators (aka: routers), since then they would be
    // joining each other (??) -- Maybe for a p2p mode, for a
    // different purpose, like multiplayer pong.
    // Will need a way to filter out the other controllers
    if(xbee.getResponse().getApiId() == ZB_IO_NODE_IDENTIFIER_RESPONSE) {
      Serial << "Xbee node identifier response" << endl;
      xbee.getResponse().getZBRxResponse(rx);
      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      addXbeeToList(senderLongAddress);
    }

    // --- TX Response
    if(xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      uint16_t the_tx_addr = txStatus.getRemoteAddress();
      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        Serial << "Message delivered successfully! to: ";
        print16Bits(the_tx_addr);
        Serial << endl;
      } else {
        Serial << "Did not deliver message to: " << the_tx_addr << endl;
      }
      
    }

    // --- RX Response
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { // got an rx packet
      xbee.getResponse().getZBRxResponse(rx);

      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      Serial.print(msg_rx_count);
      Serial.print(" >>>> Received data from ");
      print32Bits(senderLongAddress.getMsb());
      print32Bits(senderLongAddress.getLsb());
      Serial.println(": ");

      addXbeeToList(senderLongAddress);
      updateRxTime(senderLongAddress);

      for(int i=0; i<64; i++) {
        message_rx[i] = ' ';
      }

      // ascii representation of text sent through xbee
      for (int i=0; i <rx.getDataLength(); i++){
        if (!iscntrl(rx.getData()[i])) {
          message_rx[i] = (char)rx.getData()[i];
          Serial.write(message_rx[i]);
        }
      }
      Serial.println();
      msg_rx_count++;
      last_rx = millis();
      return true;
    }
  }

  return false;
  
}

// these routines are just to print the data with
// leading zeros and allow formatting such that it
// will be easy to read.
void Operator::print32Bits(uint32_t dw){
  print16Bits(dw >> 16);
  print16Bits(dw & 0xFFFF);
}

void Operator::print16Bits(uint16_t w){
  print8Bits(w >> 8);
  print8Bits(w & 0x00FF);
}

void Operator::print8Bits(byte c){
  uint8_t nibble = (c >> 4);
  if (nibble <= 9)
    Serial.write(nibble + 0x30);
  else
    Serial.write(nibble + 0x37);
      
  nibble = (uint8_t) (c & 0x0F);
  if (nibble <= 9)
    Serial.write(nibble + 0x30);
  else
    Serial.write(nibble + 0x37);
}


/*

---- Generic Conn Related ----

*/

void Operator::connRead() {

  char c;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    while(Serial1.available()) {
      c = Serial1.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

  } else if(CONN_TYPE == XBEE_CONN) {

    while(xbeeRead()) {
      //Serial << "\n<< ";
      for(int i=0; i<=rx.getDataLength(); i++) {
        c = message_rx[i];
        promulgate.organize_message(c);
        Serial << c;
        if(c == '!' || c == '?' || c == ';') Serial << endl;
      }
    }

  }

}

void Operator::connBlink() {
  if(current_time-last_led_blink >= 100) {
    if(current_time-last_rx <= 6000 && current_time > 6000) {
      // TODO
      //digitalWrite(led, led_on);
      //led_on = !led_on;
    } else {
      // TODO
      //digitalWrite(led, LOW);
    }
    last_led_blink = current_time;
  }
}

void Operator::connSend(Msg m) {

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", m.action, m.pck1.cmd, m.pck1.key, m.pck1.val, m.pck2.cmd, m.pck2.key, m.pck2.val, m.delim);
  
  Serial << "Conn TX: " << message_tx << endl;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    xbeeSend(m.action, m.pck1.cmd, m.pck1.key, m.pck1.val, m.pck2.cmd, m.pck2.key, m.pck2.val, m.delim);
    last_tx = current_time;

  }

}

void Operator::connSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);

  Serial << "Conn TX: " << message_tx << endl;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    xbeeSend(action, cmd, key, val, cmd2, key2, val2, delim);
    last_tx = current_time;

  }

}

// this is only used in case of debugging
void Operator::connSendEasy(char c) {

  sprintf(message_tx,"%c", c);

  Serial << "Conn TX: " << message_tx << endl;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    xbeeSendEasy(c);
    last_tx = current_time;

  }

}


/*

---- Promulgate Related ----

*/

Operator *Operator::opInstance;

void Operator::received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  Packet p1 = { cmd, key, val };
  Packet p2 = { cmd2, key2, val2 };
  Msg m = { 99, action, p1, p2, delim };

  opInstance->processAction(m);
}

void Operator::transmit_complete() {
  opInstance->transmitDidComplete();
}

void Operator::processAction(Msg m) {

  // TODO
  //if(PROG_DEBUG) {
    Serial << "---PROCESS ACTION---" << endl;
    Serial << "action: " << m.action << endl;
    Serial << "command: " << m.pck1.cmd << endl;
    Serial << "key: " << m.pck1.key << endl;
    Serial << "val: " << m.pck1.val << endl;
    Serial << "command: " << m.pck2.cmd << endl;
    Serial << "key: " << m.pck2.key << endl;
    Serial << "val: " << m.pck2.val << endl;
    Serial << "delim: " << m.delim << endl;
  //}

  // Sending the ID
  if(m.action == '$') {

    // TODO improve
    //for(int i=0; i<2; i++) {
      if(m.pck1.cmd == 'W' || m.pck2.cmd == 'W') {
        //Serial << "ID of the robot: " << val << endl;
        long most_recent_time = 0;
        int assoc_ind = 0;
        
        for(int i=0; i<MAX_ROBOTS; i++) {
          // we'll assume the one to send last is who this
          // id is associated with. makes sense
          if(last_rx_all[i] > most_recent_time) {
            most_recent_time = last_rx_all[i];
            assoc_ind = i;
          }
        }
  
        //Serial << "ID at ind " << assoc_ind << " is: " << val << "(num_addrs: " << num_addrs << endl;
        ids_of_all_robots[assoc_ind] = m.pck1.val;
        //break;
      }
    //}

  }

  // } else {
  //   chooseNextMessage();
  //   sendNextMsg();
  // }
  
  _receivedActionCallback(m);

  msg_rx_count++;
  diff_time = current_time-last_rx_msg;
  last_rx_msg = current_time;
  Serial << "COMMS- Roundtrip latency (ms): " << diff_time << " Msg count: " << msg_rx_count << endl;

}

void Operator::transmitDidComplete() {
  // Not really sure what to put here right now, so built in in for the future
}

void Operator::sendNextMsg() {
  Msg m = popNextMsg();
  connSend(m);
}


/*

---- Display ----

*/

void Operator::displaySearchingAnimation() {

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0);
  //display.print("Searching");
  switch(letter_itr) {
    case 0:
    display.print("S");
    break;
    case 1:
    display.print("Se");
    break;
    case 2:
    display.print("Sea");
    break;
    case 3:
    display.print("Sear");
    break;
    case 4:
    display.print("Searc");
    break;
    case 5:
    display.print("Search");
    break;
    case 6:
    display.print("Searchi");
    break;
    case 7:
    display.print("Searchin");
    break;
    case 8:
    display.print("Searching");
    break;
  }
  if(current_time-last_letter_itr >= 200) {
    letter_itr++;
    if(letter_itr > 8) letter_itr = 0;
    last_letter_itr = current_time;
  }

}

void Operator::mainMenu() {

  display.clearDisplay();

  displayTitleBar();

  uint8_t r = 5;
  uint8_t y = 20;
  uint8_t sp = 27;

  display.drawCircle((display.width()/2)-50, y, r, WHITE);
  display.drawCircle(display.width()/2, y, r, WHITE);
  display.drawCircle((display.width()/2)+50, y, r, WHITE);

  display.drawCircle((display.width()/2)-50, y+sp, r, WHITE);
  display.drawCircle(display.width()/2, y+sp, r, WHITE);
  display.drawCircle((display.width()/2)+50, y+sp, r, WHITE);


  display.setTextSize(1);

  if(CURRENT_MODE == MODE1) {

    display.setCursor(0,y+10);
    display.println("Drive");

    display.setCursor((display.width()/2)-15,y+10);
    display.println("Arm");

    display.setCursor((display.width()/2)+32,y+10);
    display.println("Empty");

    display.setCursor(0,y+sp+10);
    display.println("Scoop S");

    display.setCursor((display.width()/2)-15,y+sp+10);
    display.println("Scoop F");

    display.setCursor((display.width()/2)+32,y+sp+10);
    display.println("Dump");

  } else if(CURRENT_MODE == MODE2) {

    display.setCursor(0,y+10);
    display.println("Mot. A");

    display.setCursor((display.width()/2)-15,y+10);
    display.println("Serv. A");

    display.setCursor((display.width()/2)+32,y+10);
    display.println("Touch");

    display.setCursor(0,y+sp+10);
    display.println("");

    display.setCursor((display.width()/2)-15,y+sp+10);
    display.println("");

    display.setCursor((display.width()/2)+32,y+sp+10);
    display.println("");

  } else if(CURRENT_MODE == MODE3) {

    display.setCursor(0,y+10);
    display.println("Square");

    display.setCursor((display.width()/2)-15,y+10);
    display.println("Wave");

    display.setCursor((display.width()/2)+32,y+10);
    display.println("Dance");

    display.setCursor(0,y+sp+10);
    display.println("GPS");

    display.setCursor((display.width()/2)-15,y+sp+10);
    display.println("Comp.");

    display.setCursor((display.width()/2)+32,y+sp+10);
    display.println("");

  }

  display.display();

}

void Operator::displayTitleBar() {

  display.drawLine(0, 10, display.width()-1, 10, WHITE);

  display.setCursor(5,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if(CURRENT_MODE == MODE1) {
    display.println("Operator");
  } else if(CURRENT_MODE == MODE2) {
    display.println("Sensors");
  } else if(CURRENT_MODE == MODE3) {
    display.println("Autonomous");
  }

  display.setCursor(80,0);
  display.println("Batt:");
  display.setCursor(108,0);
  uint8_t batt_val = 100;
  display.println(batt_val);

}

void Operator::displayLogo() {
  display.drawBitmap(0, 0,  robot_missions_logo, 128, 64, 1);
  display.display();
}

void Operator::scrollLogo() {
  display.drawBitmap(0, 0,  robot_missions_logo, 128, 64, 1);
  display.display();
  
  display.startscrolldiagright(0x00, 0x0F);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
}


/*

---- LEDs ----

*/

void Operator::introLedSequence() {

  for(int i=0; i<6; i++) {
    analogWrite(led_pins[i], 255);
    delay(80);
  }

  for(int j=0; j<3; j++) {
    for(int i=0; i<6; i++) {
      analogWrite(led_pins[i], 0);
    }
    delay(100);
    for(int i=0; i<6; i++) {
      analogWrite(led_pins[i], 255);
    }
    delay(100);
  }

  for(int i=6; i>=0; i--) {
    analogWrite(led_pins[i], 0);
    delay(80);
  }
  
}

void Operator::breatheLeds() {

  for(int i=0; i<=256; i+=2) {
    for(int j=0; j<6; j++) {
      analogWrite(led_pins[j], i);
    }
    delay(10);
  }

  for(int j=0; j<6; j++) {
    digitalWrite(led_pins[j], HIGH);
  }
  delay(100);

  for(int i=256; i>=0; i-=2) {
    for(int j=0; j<6; j++) {
      analogWrite(led_pins[j], i);
    }
    delay(10);
  }

  for(int j=0; j<6; j++) {
    digitalWrite(led_pins[j], LOW);
  }
  delay(100);

}

void Operator::ledsOff() {
  for(int j=0; j<6; j++) {
    digitalWrite(led_pins[j], LOW);
  }
}



