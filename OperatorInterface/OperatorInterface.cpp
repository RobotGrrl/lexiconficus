#include "OperatorInterface.h"

OperatorInterface *OperatorInterface::opInstance;

// TODO add an api for controlling the op's speaker

OperatorInterface::OperatorInterface() {

}

void OperatorInterface::begin() {

  // Instance of the class for the callbacks from Promulgate
  opInstance = this;
  TESTING = false;
  OP_ID = 50;

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
  addr_coord = XBeeAddress64(XBEE_COORDINATOR_DH, XBEE_COORDINATOR_DL);
  addr_robot = XBeeAddress64(0x0013A200, 0x40D96FC2);
  addr_fake = XBeeAddress64(0x13131313, 0x13131313);
  txStatus = ZBTxStatusResponse();
  rx = ZBRxResponse();
  for(int i=0; i<32; i++) {
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
    addr_all_robots[i] = XBeeAddress64(addr_fake.getMsb(), addr_fake.getLsb());
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
  unlikely_count = 0;
  
  // Op-specific Comms
  retry_time = DEFAULT_RETRY_TIME;
  retry_count = 0;

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

  // Mode & State
  CURRENT_STATE = IDLE_STATE;
  CURRENT_MODE = MODE1;

  // Joystick
  HOME_X = (int)((MAX_X-MIN_X)/2);
  HOME_Y = (int)((MAX_Y-MIN_Y)/2);
  last_increment = 0;
  last_activity = 0;
  scrolling_up = false;
  
  // Misc
  turn_on_spot = true;
  slower_speed = false;
  slow_speed = 100;
  letter_itr = 0;
  last_letter_itr = 0;
  sticky_buttons = false;

  // Control
  motor_l_dir = false;
  motor_r_dir = false;
  motor_l_speed = 0;
  motor_r_speed = 0;
  motor_speed = 0;  
  turn_speed = 0;
  incr_speed = 0;
  arm_pos = 50;

  // LEDs
  last_led_blink = 0;
  led_on = false;

}

void OperatorInterface::setOpID(uint8_t the_op_id) {
  OP_ID = the_op_id;
}

void OperatorInterface::setCommLed(uint8_t pin) {
  COMM_LED = pin;
}

void OperatorInterface::setAutoconnect(bool b) {
  AUTOCONNECT = b;
}

unsigned long OperatorInterface::getCommLatency() {
  return diff_time;
}

unsigned long OperatorInterface::getLastRXTime() {
  return last_rx_msg;
}

int OperatorInterface::getMotorSpeed(int m) { // 1 = left, 2 = right
  if(m == 1) {
    return motor_l_speed;
  } else if(m == 2) {
    return motor_r_speed;
  }
  return 0;
}

bool OperatorInterface::getMotorDir(int m) {
  if(m == 1) {
    return motor_l_dir;
  } else if(m == 2) {
    return motor_r_dir;
  }
  return false;
}

int OperatorInterface::getCurrentMode() {
  return CURRENT_MODE;
}

bool OperatorInterface::isConnectedToRobot() {
  return SELECTED_ROBOT;
}


/*

---- #Callbacks ----

*/

void OperatorInterface::set_comms_timeout_callback( void (*commsTimeoutCallback)() ) {
  _commsTimeoutCallback = commsTimeoutCallback;
}

void OperatorInterface::set_controller_added_callback( void (*controllerAddedCallback)() ) {
  _controllerAddedCallback = controllerAddedCallback;
}

void OperatorInterface::set_controller_removed_callback( void (*controllerRemovedCallback)() ) {
  _controllerRemovedCallback = controllerRemovedCallback;
}

void OperatorInterface::set_received_action_callback( void (*receivedActionCallback)(Msg m) ) {
  _receivedActionCallback = receivedActionCallback;
}

void OperatorInterface::set_button_changed_callback( void (*buttonChangedCallback)(int button, int value) ) {
  _buttonChangedCallback = buttonChangedCallback;
}

void OperatorInterface::set_mode_changed_callback( void (*modeChangedCallback)(int mode) ) {
  _modeChangedCallback = modeChangedCallback;
}

void OperatorInterface::set_robot_added_callback( void (*robotAddedCallback)() ) {
  _robotAddedCallback = robotAddedCallback;
}

void OperatorInterface::set_robot_removed_callback( void (*robotRemovedCallback)(bool still_connected) ) {
  _robotRemovedCallback = robotRemovedCallback;
}


/*

---- #Init #Update ----

*/

void OperatorInterface::initOperator(int conn, long baud, HardwareSerial *serial) {

  serialyeah = serial;
  
  // Promulgate
  promulgate = Promulgate(serialyeah, serialyeah);

  // Xbee
  xbee = XBee();
  addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
  addr_coord = XBeeAddress64(XBEE_COORDINATOR_DH, XBEE_COORDINATOR_DL);
  addr_robot = XBeeAddress64(0x0013A200, 0x40D96FC2);
  addr_fake = XBeeAddress64(0x13131313, 0x13131313);
  txStatus = ZBTxStatusResponse();
  rx = ZBRxResponse();


  for(int i=0; i<MAX_ROBOTS; i++) {
    addr_all_robots[i] = XBeeAddress64(addr_fake.getMsb(), addr_fake.getLsb());
    ids_of_all_robots[i] = 0;
    failed_send_count[i] = 0;
    last_rx_all[i] = 0;
    SELECTED_ROBOT_ID[i] = 0;
  }

  for(int i=0; i<32; i++) {
    message_tx[i] = '0';
    message_rx[i] = '0';
  }

  calibrateHome();

  // ---

  bool possible = false;

  if(conn == USB_CONN) {

    Serial1.begin(baud);

    CONN_TYPE = USB_CONN;
    SELECTED_ROBOT = true; // connected by default
    possible = true;

  } else if(conn == XBEE_CONN) {
    
    pinMode(SPEAKER, OUTPUT);
    for(int i=0; i<3; i++) {
      buzz(NOTE_A6, 80);
      delay(40);
    }

    serialyeah->begin(baud);
    delay(50);
    xbee.begin(*serialyeah);

    CONN_TYPE = XBEE_CONN;
    possible = true;

  } else if(conn == BT_CONN) {

    Serial1.begin(baud);

    CONN_TYPE = BT_CONN;
    SELECTED_ROBOT = true; // connected by default
    possible = true;

  }

  if(possible) {
    if(OP_DEBUG) Serial.println("Connection set up OK");
  } else {
    if(OP_DEBUG) Serial.println("ERROR SETTING UP CONNECTION - UART DOES NOT EXIST");
  }

  // promulgate setup
  promulgate.useBase64Parsing(false);
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(this->received_action);
  promulgate.set_tx_callback(this->transmit_complete);
  promulgate.set_debug_stream(&Serial);

  initLeds();
  initButtons();
  initJoystick();
  initSpeaker();
  introLedSequence();

}

void OperatorInterface::updateOperator() {

  opInstance = this;
  current_time = millis();

  // Conn Comms
  connBlink();

  // Xbee Specific
  if(CONN_TYPE == XBEE_CONN) xbeeWatchdog();

  // Heartbeat
  // If we haven't selected a robot yet, send out our ID
  // more frequently.
  if(!SELECTED_ROBOT) {
    if(current_time-last_retry_time >= 500) {
      connSend('$', 'X', 1, OP_ID, 'X', 1, OP_ID, '!');
      last_retry_time = current_time;
    }
  } else {
    // If not, send our ID less frequently
    if(current_time-last_retry_time >= 2500) {
      addMsg( 3, '$', 'X', 1, OP_ID, 'X', 1, OP_ID, '!' );
      //connSend('$', 'X', 1, OP_ID, 'X', 1, OP_ID, '!');
      last_retry_time = current_time;
    }
  }

  // Reading the data from the Stream
  connRead();

  // Retry sending messages if there are any
  connRetrySend();
  
  // Comms have timed out
  if(millis()-last_rx_msg >= REMOTE_OP_TIMEOUT && SELECTED_ROBOT == true) {
    if(CONN_DEBUG) Serial << "REMOTE OP TIMEOUT" << endl;
    digitalWrite(COMM_LED, LOW);
    // callback that the comms has timed out
    _commsTimeoutCallback();
  }

  // Update our interface
  updateButtons();
  updateModeSwitch();
  
  if(SELECTED_ROBOT == false) {
    // Let's choose the robot in XBee mode
    // This will activate whenever SELECTED_ROBOT == false 
    // and whenever CONN_TYPE == XBEE_CONN
    chooseRobotToConnect();
  }

  if(CONN_TYPE == BT_CONN || CONN_TYPE == USB_CONN) {
    // Send the messages from the operator without waiting for
    // messages from the robot. Doing this because assuming that
    // these types of connections would most likely be to a computer
    // or terminal.
    sendNextMsg();
  }
  
}


/*

---- #Joystick ----

*/

void OperatorInterface::updateJoystick() {
  joy_x_prev = joy_x;
  joy_y_prev = joy_y;
  joy_x = getJoyX();
  joy_y = getJoyY();

  if(OP_DEBUG) Serial << "X: " << joy_x << " Y: " << joy_y << endl;

  if(joy_y > MAX_Y) joy_y = MAX_Y;
  if(joy_y < MIN_Y) joy_y = MIN_Y;
  if(joy_x > MAX_X) joy_x = MAX_X;
  if(joy_x < MIN_X) joy_x = MIN_X;
}

void OperatorInterface::calibrateHome() {

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

int OperatorInterface::getJoyX() {
  return analogRead(JOYSTICK_X);
}

int OperatorInterface::getJoyY() {
  return analogRead(JOYSTICK_Y);
}


/*

---- #Control ----

*/

void OperatorInterface::joystickDriveControl() {

  Msg m = msg_none;
  bool motor_dir = true;
  bool send_motor_message = false;

  updateJoystick();

  if(joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE)
     && joy_x >= (HOME_X-ZERO_ZONE) && joy_x <= (HOME_X+ZERO_ZONE)) {
    
    if(OP_DEBUG) Serial << "home" << endl;

    motor_speed = 0;
    motor_l_dir = false;
    motor_l_speed = 0;
    motor_r_dir = false;
    motor_r_speed = 0;

    send_motor_message = true;
    
  } else if(joy_x >= (MAX_X-150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) ) ) {
  
    motor_l_dir = false;
    motor_l_speed = TURN_SPEED_REV;
    motor_r_dir = true;
    motor_r_speed = TURN_SPEED_FWD;

    send_motor_message = true;
  
  } else if(joy_x <= (MIN_X+150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) )) {

    motor_l_dir = true;
    motor_l_speed = TURN_SPEED_FWD;
    motor_r_dir = false;
    motor_r_speed = TURN_SPEED_REV;

    send_motor_message = true;

  } else {

    if(joy_y >= (HOME_Y+ZERO_ZONE)) { 

      // forwards

      if(current_time - last_increment > 10) {
        incr_speed = 10;
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

      send_motor_message = true;

    } else if(joy_y <= (HOME_Y-ZERO_ZONE)) { 

      // backwards

      if(current_time - last_increment > 10) {
        incr_speed = 10;
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

      send_motor_message = true;
    }

    // sending the data
    motor_l_speed = motor_speed;
    motor_r_speed = motor_speed;
    if(motor_dir) {
      motor_l_dir = true;
      motor_r_dir = true;
    } else {
      motor_l_dir = false;
      motor_r_dir = false;
    }
    
  }

  if(send_motor_message) {

    if(OP_DEBUG) Serial << "Motor speed L (" << motor_l_dir << "): " << motor_l_speed;
    if(OP_DEBUG) Serial << "Motor speed R (" << motor_r_dir << "): " << motor_r_speed;
    if(OP_DEBUG) Serial << endl;

    m.priority = 3;
    m.action = '@';
    m.pck1.cmd = 'L';
    m.pck1.key = motor_l_dir;
    m.pck1.val = motor_l_speed;
    m.pck2.cmd = 'R';
    m.pck2.key = motor_r_dir;
    m.pck2.val = motor_r_speed;
    m.delim = '!';

    insertMsg(m);

  }
  
}

void OperatorInterface::joystickArmControl() {

  Msg m = msg_none;
  current_time = millis();
  bool send_arm_msg = false;
  bool arm_dir = true;

  updateJoystick();

  if(joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE)
     && joy_x >= (HOME_X-ZERO_ZONE) && joy_x <= (HOME_X+ZERO_ZONE)) {
    
    incr_speed = 0;
    arm_dir = true;

    send_arm_msg = true;
    
  } else if(joy_x >= (MAX_X-150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) ) ) {
  
    // hard turn to the left
  
  } else if(joy_x <= (MIN_X+150) && ( joy_y >= (HOME_Y-ZERO_ZONE) && joy_y <= (HOME_Y+ZERO_ZONE) )) {

    // hard turn to the right

  } else {

    if(joy_y >= (HOME_Y+ZERO_ZONE)) { 

      // forwards
      if(current_time - last_increment > 100) {
        incr_speed = (int)map(joy_y, HOME_Y, MAX_Y, 4, 10);
        last_increment = current_time;
      }
      if(incr_speed > 10) incr_speed = 10;
      arm_dir = true;
      
      send_arm_msg = true;
      
    } else if(joy_y <= (HOME_Y-ZERO_ZONE)) { 

      // backwards
      if(current_time - last_increment > 100) {
        incr_speed = (int)map(joy_y, HOME_Y, MIN_Y, 4, 10);
        last_increment = current_time;
      }
      if(incr_speed > 10) incr_speed = 10;
      arm_dir = false;

      send_arm_msg = true;

    }

  }

  if(send_arm_msg) {

    if(OP_DEBUG) Serial << " Arm incr_speed (" << arm_dir << "): " << incr_speed << endl;

    m.priority = 3;
    m.action = '@';
    m.pck1.cmd = 'S';
    m.pck1.key = arm_dir;
    m.pck1.val = incr_speed;
    m.delim = '!';

    insertMsg(m);

  }

}


/*

---- #Buttons ----

*/

void OperatorInterface::updateButtons() {
  
  if(current_time < 4000) return; // something odd happens on startup

  Msg m = msg_none;
  bool did_change = false;

  uint8_t prev_button_states[7];
  for(int i=0; i<7; i++) {
    prev_button_states[i] = button_states[i];
  }

  for(int i=0; i<7; i++) {
    did_change = false;
    bounce_buttons[i].update();
    if(bounce_buttons[i].fell()) {
      if(i < 6) {
        if(OP_DEBUG) Serial << "Button " << i;
      } else {
        if(OP_DEBUG) Serial << "Joystick button";
      }
      Serial << " pressed " << endl;
      
      if(prev_button_states[i] == 0) { // before, it was off

        if(sticky_buttons) {
          // no need to do anything extra
        } else {
          // clear the other ones first
          for(int j=0; j<6; j++) {
            if(button_states[j] == 1) {
              ledQuickFade(led_pins[j], 255, 0);
              button_states[j] = 0;
            }
          }
          resetButtonStates();
          delay(100);
        }
        button_states[i] = 1;

        if(i<6) ledQuickFade(led_pins[i], 0, 255);
        if(i>=6) ledQuickPulseAll();
        did_change = true;
        
      } else if(prev_button_states[i] == 1) { // before, it was on

        if(sticky_buttons) {
          // no need to do anything extra
        } else {
          resetButtonStates();
          delay(100);
        }
        button_states[i] = 0;
        if(i<6) ledQuickFade(led_pins[i], 255, 0);
        if(i>=6) ledQuickPulseAll();
        did_change = true;
        
      }
    }

    if(did_change) {

      m.priority = 1;
      m.action = '#';
      m.pck1.key = CURRENT_MODE;
      m.pck1.val = button_states[i];
      m.pck2.cmd = 'N';
      m.pck2.key = CURRENT_MODE;
      m.pck2.val = CURRENT_STATE;
      m.delim = '!';
      switch(i) {
        case 0: // 'red'
          m.pck1.cmd = 'P';
          if(button_states[i] == 1) {
            buzz(NOTE_C6, 80);
          } else if(button_states[i] == 0) {
            buzz(NOTE_C2, 80);
          }
        break;
        case 1: // yellow
          m.pck1.cmd = 'Y';
          if(button_states[i] == 1) {
            buzz(NOTE_E6, 80);
          } else if(button_states[i] == 0) {
            buzz(NOTE_E2, 80);
          }
        break;
        case 2: // green
          m.pck1.cmd = 'G';
          if(button_states[i] == 1) {
            buzz(NOTE_G6, 80);
          } else if(button_states[i] == 0) {
            buzz(NOTE_G2, 80);
          }
        break;
        case 3: // white
          m.pck1.cmd = 'W';
          if(button_states[i] == 1) {
            buzz(NOTE_C7, 80);
          } else if(button_states[i] == 0) {
            buzz(NOTE_C3, 80);
          }
        break;
        case 4: // blue
          m.pck1.cmd = 'B';
          if(button_states[i] == 1) {
            buzz(NOTE_E7, 80);
          } else if(button_states[i] == 0) {
            buzz(NOTE_E3, 80);
          }
        break;
        case 5: // black
          m.pck1.cmd = 'N';
          if(button_states[i] == 1) {
            buzz(NOTE_G7, 80);
          } else if(button_states[i] == 0) {
            buzz(NOTE_G3, 80);
          }
        break;
        case 6: // joystick
          m.pck1.cmd = 'J';
          if(button_states[i] == 1) {
            buzz(NOTE_A7, 80);
          } else if(button_states[i] == 0) {
            buzz(NOTE_A3, 80);
          }
        break;
      }

      insertMsg(m);
      _buttonChangedCallback(i, button_states[i]);

    }

  }

  // update the state
  bool all_zero = true;
  for(int i=0; i<6; i++) {
    if(button_states[i] == 1) {
      CURRENT_STATE = ACTIVE_STATE;
      all_zero = false;
      break;
    }
  }
  if(all_zero) CURRENT_STATE = IDLE_STATE;

}

bool OperatorInterface::getButton(uint8_t b) {
  if(button_states[b] == 1) return true;
  return false;
}

void OperatorInterface::setButtonState(uint8_t b, uint8_t state) {
  button_states[b] = state;
}

bool OperatorInterface::getJoystickButton() {
  if(button_states[6] == 1) return true;
  return false;
}

void OperatorInterface::resetButtonStates() {
  for(int i=0; i<7; i++) {
    button_states[i] = 0;
  }
  ledsOff();
}


/*

---- #Mode Switch ----

*/

void OperatorInterface::updateModeSwitch() {

  int val = analogRead(MODE_SW);

  if(val >= MODE1_THRESH) {
    if(CURRENT_MODE != MODE1) {
      buzz(NOTE_E6, 80);
      delay(80);
      buzz(NOTE_F6, 80);
      delay(80);
      buzz(NOTE_E6, 80);
      resetButtonStates();
      _modeChangedCallback(MODE1);
    }
    CURRENT_MODE = MODE1;
  } else if(val >= MODE2_THRESH) {
    if(CURRENT_MODE != MODE2) {
      buzz(NOTE_C6, 80);
      delay(80);
      buzz(NOTE_G6, 80);
      delay(80);
      buzz(NOTE_C6, 80);
      resetButtonStates();
      _modeChangedCallback(MODE2);
    }
    CURRENT_MODE = MODE2;
  } else if(val >= MODE3_THRESH) {
    if(CURRENT_MODE != MODE3) {
      buzz(NOTE_A6, 80);
      delay(80);
      buzz(NOTE_D6, 80);
      delay(80);
      buzz(NOTE_A6, 80);
      resetButtonStates();
      _modeChangedCallback(MODE3);
    }
    CURRENT_MODE = MODE3;
  }

}


/*

---- #Speaker ----

*/

void OperatorInterface::buzz(long frequency, long length) {
  long delayValue = 1000000 / frequency / 2;
  long numCycles = frequency * length / 1000;
  for (long i = 0; i < numCycles; i++) { 
    digitalWrite(SPEAKER, HIGH); 
    delayMicroseconds(delayValue); 
    digitalWrite(SPEAKER, LOW); 
    delayMicroseconds(delayValue); 
  }
}


/*

---- #Init ----

*/

void OperatorInterface::initLeds() {
  pinMode(COMM_LED, OUTPUT);
  for(int i=0; i<7; i++) {
    pinMode(led_pins[i], OUTPUT);
  }
}

void OperatorInterface::initButtons() {
  for(int i=0; i<7; i++) {
    pinMode(button_pins[i], INPUT_PULLUP);
    bounce_buttons[i].attach(button_pins[i]);
    bounce_buttons[i].interval(DEBOUNCE);
  }
  pinMode(MODE_SW, INPUT);
}

void OperatorInterface::initJoystick() {
  pinMode(JOYSTICK_X, INPUT_PULLUP);
  pinMode(JOYSTICK_Y, INPUT_PULLUP);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
}

void OperatorInterface::initSpeaker() {
  pinMode(SPEAKER, OUTPUT);
  // beep beep
  buzz(NOTE_G6, 100);
  delay(100);
  buzz(NOTE_D7, 100);
  delay(100);
  buzz(NOTE_D8, 100);
  delay(100);
}


/*

---- #Messages ----

*/

uint8_t OperatorInterface::getMsgQueueLength() {
  return msgs_in_queue;
}

// Retrieve the next message, and move the other messages behind it up
Msg OperatorInterface::popNextMsg() {

  if(msgs_in_queue <= 0) return msg_none;

  struct Msg m = msg_queue[0];

  for(int i=0; i<msgs_in_queue-1; i++) {
    msg_queue[i] = msg_queue[i+1];
  }

  msgs_in_queue--;

  if(msgs_in_queue <= 0) {
    msgs_in_queue = 0;
    addMsg(msg_none);
  }

  return m;
}

void OperatorInterface::addMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  //Serial.print("adding next message");
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(MSG_DEBUG) {
      Serial.print(F("Cannot add msg to queue, number of messages in queue: "));
      Serial.println(msgs_in_queue);
    }
    return;
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
void OperatorInterface::addMsg(Msg m) {
  //if(OP_DEBUG) Serial << "adding next msg " << msgs_in_queue << endl;
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(MSG_DEBUG) {
      Serial.print(F("Cannot add msg to queue, number of messages in queue: "));
      Serial.println(msgs_in_queue);
    }
    return;
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
void OperatorInterface::insertMsg(Msg m) {

  if(MSG_DEBUG) Serial.println("inserting next message");

  bool completed = false;
  uint8_t insert_index = 0;

  for(int i=0; i<msgs_in_queue; i++) {
    if(m.priority < msg_queue[i].priority) { // insert messages that have a greater priority number (meaning they are less of a priority)
      insert_index = i;
      for(int j=msgs_in_queue-1; j>insert_index; j--) {
        msg_queue[j] = msg_queue[j-1];
      }
      if(MSG_DEBUG) Serial << "A: Inserting " << i << " (" << msg_queue[i].priority << ") at index: " << insert_index << endl;
      msg_queue[insert_index] = m;
      completed = true;
      break;
    } else if(m.priority == msg_queue[i].priority) { // if it's the same priority, and the same commands, overwrite with the newest version
      if(m.pck1.cmd == msg_queue[i].pck1.cmd && m.pck2.cmd == msg_queue[i].pck2.cmd) {
        if(MSG_DEBUG) Serial << "B: Replacing " << i << " (" << msg_queue[i].priority << ") with new msg (" << m.priority << ")" << endl;
        msg_queue[i] = m;
        completed = true;
        break;
      }
    }
  }

  if(!completed) { // if we can't insert it, at least add it
    if(MSG_DEBUG) Serial << "if we can't insert it, at least add it" << endl;
    addMsg(m);
  }

}


/*

---- #Xbee ----

*/

void OperatorInterface::xbeeSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);

  if(SELECTED_ROBOT) { // send to everyone as we're in discovery mode
  
    for(int i=0; i<num_addrs; i++) {
      if(num_addrs >= MAX_ROBOTS-1) break; // something went wrong here

      if(addr_all_robots[i].getMsb() != 0 && addr_all_robots[i].getLsb() != 0) {
        if(addr_all_robots[i].getMsb() != addr_fake.getMsb() && addr_all_robots[i].getLsb() != addr_fake.getLsb()) {
      
          if(XBEE_DEBUG) Serial << "Sent to list ind " << i << endl;
          ZBTxRequest zbtx = ZBTxRequest(addr_all_robots[i], (uint8_t *)message_tx, strlen(message_tx));
          zbtx.setFrameId('0');
          xbee.send(zbtx); 
          ind_addr_sent = i;
        
        }
      }
    }

  } else {
    // by default, send to the coordinator of the network
    if(XBEE_DEBUG) Serial << "Send to default coordinator" << endl;
    ZBTxRequest zbtx = ZBTxRequest(addr_coord, (uint8_t *)message_tx, strlen(message_tx));
    //ZBTxRequest zbtx = ZBTxRequest(addr_robot, (uint8_t *)message_tx, strlen(message_tx));
    zbtx.setFrameId('0');
    xbee.send(zbtx); 
  }
 
}

void OperatorInterface::xbeeSendEasy(char c) {

  sprintf(message_tx,"%c", c);

  if(SELECTED_ROBOT) { // send to everyone as we're in discovery mode
  
    for(int i=0; i<num_addrs; i++) {
      if(num_addrs >= MAX_ROBOTS-1) break; // something went wrong here

      if(addr_all_robots[i].getMsb() != 0 && addr_all_robots[i].getLsb() != 0) {
        if(addr_all_robots[i].getMsb() != addr_fake.getMsb() && addr_all_robots[i].getLsb() != addr_fake.getLsb()) {
      
          if(XBEE_DEBUG) Serial << "Sent to list ind " << i << endl;
          ZBTxRequest zbtx = ZBTxRequest(addr_all_robots[i], (uint8_t *)message_tx, strlen(message_tx));
          zbtx.setFrameId('0');
          xbee.send(zbtx); 
          ind_addr_sent = i;
        
        }
      }
    }

  } else {
    // by default, send to the coordinator of the network
    if(XBEE_DEBUG) Serial << "Send to default coordinator" << endl;
    ZBTxRequest zbtx = ZBTxRequest(addr_coord, (uint8_t *)message_tx, strlen(message_tx));
    //ZBTxRequest zbtx = ZBTxRequest(addr_robot, (uint8_t *)message_tx, strlen(message_tx));
    zbtx.setFrameId('0');
    xbee.send(zbtx); 
  }
 
}

void OperatorInterface::addXbeeToList(XBeeAddress64 newAddr) {
  
  if(XBEE_DEBUG) Serial << "Sender address - High: ";
  if(XBEE_DEBUG) print32Bits(newAddr.getMsb());
  if(XBEE_DEBUG) Serial << " Low: ";
  if(XBEE_DEBUG) print32Bits(newAddr.getLsb());
  if(XBEE_DEBUG) Serial << endl;

  bool add_it_to_list = true;

  for(int i=0; i<MAX_ROBOTS; i++) {
    if(addr_all_robots[i].getMsb() == newAddr.getMsb() && addr_all_robots[i].getLsb() == newAddr.getLsb()) {
      if(XBEE_DEBUG) Serial << "Not adding to the list - already there" << endl;
      // already on the list
      add_it_to_list = false;
    }
  }

  if(add_it_to_list) {
    if(num_addrs >= MAX_ROBOTS-1) {
      if(XBEE_DEBUG) Serial << "! Reached max number of robots !" << endl;
      return;
    }
    if(XBEE_DEBUG) Serial << "! New robot added to Xbee list !" << endl;
    addr_all_robots[num_addrs] = XBeeAddress64(newAddr.getMsb(), newAddr.getLsb());
    
    if(XBEE_DEBUG) {
      for(int i=0; i<MAX_ROBOTS; i++) {
        Serial << i << ": ";
        print32Bits(addr_all_robots[i].getMsb());
        Serial << " ";
        print32Bits(addr_all_robots[i].getLsb());
        Serial << endl;
      }
    }
    
    num_addrs++;
    _robotAddedCallback();
  }
  
}

void OperatorInterface::updateRxTime(XBeeAddress64 senderLongAddress) {
  for(int i=0; i<MAX_ROBOTS; i++) {
    if(addr_all_robots[i].getMsb() == senderLongAddress.getMsb() && addr_all_robots[i].getLsb() == senderLongAddress.getLsb()) {
      last_rx_all[i] = millis();
      return;
    }
  }
}

void OperatorInterface::xbeeWatchdog() {
  if(current_time-last_rx_check >= 10000) {
    for(int i=0; i<MAX_ROBOTS; i++) {
      if(current_time-last_rx_all[i] >= 11000) {
        if(addr_all_robots[i].getMsb() != 0 && addr_all_robots[i].getLsb() != 0) {
          if(addr_all_robots[i].getMsb() != addr_fake.getMsb() && addr_all_robots[i].getLsb() != addr_fake.getLsb()) {
            // remove it from the list
            if(XBEE_DEBUG) Serial << "Have not heard from: ";
            if(XBEE_DEBUG) print32Bits(addr_all_robots[i].getMsb());
            if(XBEE_DEBUG) Serial << " ";
            if(XBEE_DEBUG) print32Bits(addr_all_robots[i].getLsb());
            if(XBEE_DEBUG) Serial << " since " << current_time-last_rx_all[i] << "ms ago." << endl;
            if(XBEE_DEBUG) Serial << "! Removing it from list !" << endl;
            if(XBEE_DEBUG) Serial << "Its num was: " << ids_of_all_robots[i];
            // check to see if it was a robot we were connected to
            for(int j=0; j<num_robot_conn; j++) {
              // go through the list of all the robots we're
              // connected to
              if(ids_of_all_robots[i] == SELECTED_ROBOT_ID[j]) {
                if(XBEE_DEBUG) Serial << " us: " << SELECTED_ROBOT_ID[j] << endl;
                if(XBEE_DEBUG) Serial << "Resetting selected robot" << endl;
                SELECTED_ROBOT_ID[j] = 0;
                num_robot_conn--;
                // now we are not connected to any robot
                if(num_robot_conn < 0) num_robot_conn = 0; // just in case
              }
            }
            // reset if we're no longer connected to any robot
            //if(num_robot_conn == 0) {
              resetButtonStates();
              SELECTED_ROBOT = false;
              CURRENT_STATE = SEARCHING_STATE;
            //}
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

            if(XBEE_DEBUG) {
              for(int i=0; i<MAX_ROBOTS; i++) {
                Serial << i << ": ";
                print32Bits(addr_all_robots[i].getMsb());
                Serial << " ";
                print32Bits(addr_all_robots[i].getLsb());
                Serial << endl;
              }
            }

            // so that we can properly deincrement the num_addr counter
            num_addrs--;
            if(num_addrs < 0) num_addrs = 0;
            _robotRemovedCallback(SELECTED_ROBOT);
          }
        }
      }
    }
    last_rx_check = current_time;
  }
}

bool OperatorInterface::xbeeRead() {

  //if(XBEE_DEBUG) Serial << "Xbee read" << endl;
  
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) { // we got something

    if(XBEE_DEBUG) Serial << "Xbee response: " << xbee.getResponse().getApiId();

    // --- Node Identifier Response
    // I'm not sure if this one will be good to have for the
    // operators (aka: routers), since then they would be
    // joining each other (??) -- Maybe for a p2p mode, for a
    // different purpose, like multiplayer pong.
    // Will need a way to filter out the other controllers
    if(xbee.getResponse().getApiId() == ZB_IO_NODE_IDENTIFIER_RESPONSE) {
      if(XBEE_DEBUG) Serial << "Node identifier" << endl;
      xbee.getResponse().getZBRxResponse(rx);
      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      addXbeeToList(senderLongAddress);
    }
    
    // --- TX Response
    if(xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      if(XBEE_DEBUG) Serial << "TX" << endl;
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      uint16_t the_tx_addr = txStatus.getRemoteAddress();
    }

    // --- RX Response
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { // got an rx packet
      if(XBEE_DEBUG) Serial << "RX" << endl;

      xbee.getResponse().getZBRxResponse(rx);

      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      if(XBEE_DEBUG) Serial.print(msg_rx_count);
      if(XBEE_DEBUG) Serial.print(" >>>> Received data from ");
      if(XBEE_DEBUG) print32Bits(senderLongAddress.getMsb());
      if(XBEE_DEBUG) Serial << " ";
      if(XBEE_DEBUG) print32Bits(senderLongAddress.getLsb());
      if(XBEE_DEBUG) Serial << "Len = " << rx.getDataLength() << endl;

      addXbeeToList(senderLongAddress);
      updateRxTime(senderLongAddress);

      for(int i=0; i<32; i++) {
        message_rx[i] = ' ';
      }

      // ascii representation of text sent through xbee
      for (int i=0; i <rx.getDataLength(); i++){
        if (!iscntrl(rx.getData()[i])) {
          message_rx[i] = (char)rx.getData()[i];
          if(XBEE_DEBUG) Serial.write(message_rx[i]);
        }
      }
      if(XBEE_DEBUG) Serial.println();
      last_rx = millis();
      return true;
    }
  }

  return false;
  
}

// these routines are just to print the data with
// leading zeros and allow formatting such that it
// will be easy to read.
void OperatorInterface::print32Bits(uint32_t dw){
  print16Bits(dw >> 16);
  print16Bits(dw & 0xFFFF);
}

void OperatorInterface::print16Bits(uint16_t w){
  print8Bits(w >> 8);
  print8Bits(w & 0x00FF);
}

void OperatorInterface::print8Bits(byte c){
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

---- #Conn Generic Related ----

*/

void OperatorInterface::connRead() {

  char c;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    while(Serial1.available()) {
      c = Serial1.read();
      promulgate.organize_message(c);
      if(CONN_DEBUG) Serial << c;
      if(CONN_DEBUG) { if(c == '!' || c == '?' || c == ';') Serial << endl; }
    }

  } else if(CONN_TYPE == XBEE_CONN) {

    while(xbeeRead()) {
      if(CONN_DEBUG) Serial << "Read... (" << rx.getDataLength() << ")\n";
      for(int i=0; i<=rx.getDataLength(); i++) {
        c = message_rx[i];
        promulgate.organize_message(c);
        if(CONN_DEBUG) Serial << c;
        if(CONN_DEBUG) { if(c == '!' || c == '?' || c == ';') Serial << endl; }
      }
    }

  }

}

void OperatorInterface::connBlink() {
  if(current_time-last_led_blink >= 100) {
    if(current_time-last_rx <= 6000 && current_time > 6000) {
      digitalWrite(COMM_LED, led_on);
      led_on = !led_on;
    } else {
      digitalWrite(COMM_LED, LOW);
    }
    last_led_blink = current_time;
  }
}

void OperatorInterface::connSend(Msg m) {

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", m.action, m.pck1.cmd, m.pck1.key, m.pck1.val, m.pck2.cmd, m.pck2.key, m.pck2.val, m.delim);
  
  Serial << "Conn TX ----> " << message_tx << endl;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    xbeeSend(m.action, m.pck1.cmd, m.pck1.key, m.pck1.val, m.pck2.cmd, m.pck2.key, m.pck2.val, m.delim);
    last_tx = current_time;

  }

}

void OperatorInterface::connSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);

  Serial << "Conn TX ----> " << message_tx << endl;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    xbeeSend(action, cmd, key, val, cmd2, key2, val2, delim);
    last_tx = current_time;

  }

}

// this is only used in case of debugging
void OperatorInterface::connSendEasy(char c) {

  sprintf(message_tx,"%c", c);

  Serial << "Conn TX ----> " << message_tx << endl;

  if(CONN_TYPE == USB_CONN || CONN_TYPE == BT_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    xbeeSendEasy(c);
    last_tx = current_time;

  }

}

void OperatorInterface::connRetrySend() {
  if(current_time-last_rx_msg >= retry_time && retry_count < 5) { // retry 5 times
    if(CONN_DEBUG) Serial.println(F("Retrying send"));
    if(CONN_DEBUG) Serial << getMsgQueueLength() << " msgs in queue" << endl;
    if(getMsgQueueLength() <= 0) {
      addMsg(msg_none);
    }
    sendNextMsg();
    retry_time = SECONDARY_RETRY_TIME;
    last_rx_msg = current_time;
    retry_count++;
  }
}

void OperatorInterface::chooseRobotToConnect() {

  if(CONN_TYPE == XBEE_CONN) {

    if(AUTOCONNECT == false) {

      sticky_buttons = true;

      if(getJoystickButton() == 1) {
        if(CONN_DEBUG) Serial << "They have chosen their robots!" << endl;
        
        for(int i=0; i<6; i++) {
          if(getButton(i) == 1) {
            SELECTED_ROBOT_ID[num_robot_conn] = ids_of_all_robots[i];
            num_robot_conn++;
          }
        }

      }

    } else if(AUTOCONNECT == true) {

      sticky_buttons = false;

      if(num_addrs > 0) { // as soon as we see one, connect to it
        if(CONN_DEBUG) Serial << "We are going to connect to this robot!" << endl;
        SELECTED_ROBOT_ID[num_robot_conn] = ids_of_all_robots[0];
        num_robot_conn++;
      }

    }

    if(num_robot_conn > 0) {  
      SELECTED_ROBOT = true;
      CURRENT_STATE = IDLE_STATE;
      resetButtonStates();
      sticky_buttons = false;
    }

  }

}



/*

---- #Promulgate Related ----

*/

void OperatorInterface::received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  Packet p1 = { cmd, key, val };
  Packet p2 = { cmd2, key2, val2 };
  Msg m = { 99, action, p1, p2, delim };

  opInstance->processAction(m);
}

void OperatorInterface::transmit_complete() {
  opInstance->transmitDidComplete();
}

void OperatorInterface::processAction(Msg m) {

  Serial << "Conn RX <---- " << m.action << m.pck1.cmd << m.pck1.key << ",";
  Serial << m.pck1.val << "," << m.pck2.cmd << m.pck2.key << ",";
  Serial << m.pck2.val << m.delim << endl;

  if(COMM_DEBUG) {
    Serial << "---PROCESS ACTION---" << endl;
    Serial << "action: " << m.action << endl;
    Serial << "command: " << m.pck1.cmd << endl;
    Serial << "key: " << m.pck1.key << endl;
    Serial << "val: " << m.pck1.val << endl;
    Serial << "command: " << m.pck2.cmd << endl;
    Serial << "key: " << m.pck2.key << endl;
    Serial << "val: " << m.pck2.val << endl;
    Serial << "delim: " << m.delim << endl;
  }

  // Sending the ID
  if(m.action == '$') {

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
    }
    
  }

  _receivedActionCallback(m);

  msg_rx_count++;
  diff_time = millis()-last_rx_msg;
  last_rx_msg = current_time;
  Serial << "COMMS- Roundtrip latency (ms): " << diff_time << " Msg count: " << msg_rx_count << endl;

  // send the next message now that we have received a reply from the robot
  sendNextMsg();

}

void OperatorInterface::transmitDidComplete() {
  // Not really sure what to put here right now, so built in in for the future
}

void OperatorInterface::sendNextMsg() {
  Msg m = popNextMsg();
  connSend(m);
}


/*

---- #LEDs ----

*/

void OperatorInterface::introLedSequence() {

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

void OperatorInterface::breatheLeds() {

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

void OperatorInterface::ledsOff() {
  for(int j=0; j<6; j++) {
    digitalWrite(led_pins[j], LOW);
  }
}


void OperatorInterface::ledQuickFade(uint8_t pin, uint8_t from, uint8_t to) {
  
  if(from < to) {
    for(int i=from; i<to; i+=3) {
      analogWrite(pin, i);
      delay(1);
    }
  } else if(from > to) {
    for(int i=from; i>to; i-=3) {
      analogWrite(pin, i);
      delay(1);
    }
  }

  analogWrite(pin, to);

}

void OperatorInterface::ledQuickPulseAll() {

  for(int i=0; i<255; i+=5) {
    for(int j=0; j<6; j++) {
      analogWrite(led_pins[j], i);
      delay(1);
    }
  }
  for(int j=0; j<6; j++) {
    analogWrite(led_pins[j], 255);
    delay(1);
  }

  for(int i=255; i>0; i-=5) {
    for(int j=0; j<6; j++) {
      analogWrite(led_pins[j], i);
      delay(1);
    }
  }
  for(int j=0; j<6; j++) {
    analogWrite(led_pins[j], 0);
    delay(1);
  }

}


