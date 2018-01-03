#include "OperatorInterface.h"

/*

Message priority list
0 = none
1 = button presses
2 = servos
3 = driving
...
9 = none

*/

uint8_t led_pins[7] = { LED1, LED2, LED3, LED4, LED5, LED6, BOARD_LED };
uint8_t button_states[7] = { 0, 0, 0, 0, 0, 0, 0 };
Bounce bounce_buttons[7] = { Bounce(), Bounce(), Bounce(), Bounce(), Bounce(), Bounce(), Bounce() };


// ---- Init & Updates

Operator::Operator() : display(OLED_RESET) {

  // display
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)  
  display.clearDisplay(); // Clear the buffer.
  display.display();

  // update
  current_time = 0;

  // state
  GO_TIME = false;
  CURRENT_STATE = IDLE_STATE;
  LAST_STATE = CURRENT_STATE;

  // mode
  CURRENT_MODE = MODE1;

  // joystick changes its home position
  HOME_X = 347;
  HOME_Y = 356;

  // msgs
  msgs_in_queue = 0;

  // joystick
  first_idle = false;
  joystick_idle = false;
  last_idle_update = 0;
  last_activity = 0;
  arm_pos = 50;
  
  // modes
  turn_on_spot = true;
  slower_speed = false;
  slow_speed = 100;

  // joystick
  last_increment = 0;
  scrolling_up = false;
  turn_speed = 0;
}

void Operator::init() {
  initLeds();
  initButtons();
  initJoystick();
  displayLogo();
  initSpeaker();
  introLedSequence();
}

void Operator::update() {

  current_time = millis();
  updateButtons();
  updateModeSwitch();

  mainMenu();

  // leaving this out for now, as MODE1 is the only mode
  //if(CURRENT_MODE == MODE1) {

    // wait for go time (connecting to robots) from the sketch
    if(GO_TIME) {
      if(CURRENT_STATE == DRIVE_STATE) joystickDriveControl();
      if(CURRENT_STATE == ARM_STATE) joystickArmControl();
    }

  //}

  /*
  // todo: debug this
  // by slowing down the updates when the nunchuck is idle, we will receive the
  // replies from the robot less frequently, thereby letting us check the buttons
  // more frequently, so that the experience can be more responsive
  if(joystick_idle == true && current_time-last_idle_update >= IDLE_UPDATE_FREQ) {
    if(red_on) joystickDriveControl();
    last_idle_update = current_time;
  } else {
    if(red_on) joystickControl();
  }
  */
  
}

// ---- Display

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
  //delay(300);

  display.startscrolldiagright(0x00, 0x0F);
  // TODO: Make this better
  delay(2000);
  display.startscrolldiagleft(0x00, 0x0F);
  // TODO: Make this better
  delay(2000);
  display.stopscroll();
}


// ---- LEDs

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

void Operator::ledsOff() {
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
}

void Operator::buttonsOff() {
  red_on = false;
  yellow_on = false;
  green_on = false;
  white_on = false;
  blue_on = false;
  black_on = false;
  ledsOff();
}


// ---- Joystick

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

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

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

  insertNextMsg(m);

}

void Operator::joystickArmControl() {

  current_time = millis();

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

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

  insertNextMsg(m);

}

int Operator::getJoyX() {
  return analogRead(JOYSTICK_X);
}

int Operator::getJoyY() {
  return analogRead(JOYSTICK_Y);
}




// ---- Buttons

void Operator::updateButtons() {
  
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
      } else if(button_states[i] == 1) {
        button_states[i] = 0;
      }
    }
  }

  for(int i=0; i<6; i++) {
    if(button_states[i] == 1) {
      analogWrite(led_pins[i], 255);
    } else {
      analogWrite(led_pins[i], 0);
    }
  }
  
  /*

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

  // drive state
  if(bouncer1.fell()) {

    if(BUTTON_DEBUG) Serial.print(F("red button (P)"));

    if(reset_leds) {
      ledsOff();
      yellow_on = false;
      green_on = false;
      white_on = false;
      blue_on = false;
      black_on = false;
    }
    
    red_on = !red_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'P';
    m.key = 0;

    if(red_on) {
      m.val = 1;
      digitalWrite(LED1, HIGH);
      CURRENT_STATE = DRIVE_STATE;
    } else {
      m.val = 0;
      digitalWrite(LED1, LOW);
      CURRENT_STATE = IDLE_STATE;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    addNextMsg(m);

  }
  
  // arm state
  if(bouncer2.fell()) { 
    
    if(BUTTON_DEBUG) Serial.print(F("yellow button (Y)"));

    if(reset_leds) {
      ledsOff();
      red_on = false;
      green_on = false;
      white_on = false;
      blue_on = false;
      black_on = false;
    }

    yellow_on = !yellow_on;
    
    m.priority = 1;
    m.action = '#';
    m.cmd = 'Y';
    m.key = 0;

    if(yellow_on) {
      m.val = 1;
      digitalWrite(LED2, HIGH);
      CURRENT_STATE = ARM_STATE;
    } else {
      m.val = 0;
      digitalWrite(LED2, LOW);
      CURRENT_STATE = IDLE_STATE;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    addNextMsg(m);

  }

  if(bouncer3.fell()) {

    if(BUTTON_DEBUG) Serial.print(F("green button (G)"));

    if(reset_leds) {
      ledsOff();
      red_on = false;
      yellow_on = false;
      white_on = false;
      blue_on = false;
      black_on = false;
    }

    green_on = !green_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'G';
    m.key = 0;

    if(green_on) {
      m.val = 1;
      digitalWrite(LED3, HIGH);
      CURRENT_STATE = DUMP_STATE;
    } else {
      m.val = 0;
      digitalWrite(LED3, LOW);
      CURRENT_STATE = IDLE_STATE;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    addNextMsg(m);

  }
  
  if(bouncer4.fell()) {

    if(BUTTON_DEBUG) Serial.print(F("white button (W)"));

    if(reset_leds) {
      ledsOff();
      red_on = false;
      yellow_on = false;
      green_on = false;
      blue_on = false;
      black_on = false;
    }

    white_on = !white_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'W';
    m.key = 0;

    if(white_on) {
      m.val = 1;
      digitalWrite(LED4, HIGH);
      CURRENT_STATE = SCOOP_S_STATE;
    } else {
      m.val = 0;
      digitalWrite(LED4, LOW);
      CURRENT_STATE = IDLE_STATE;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    addNextMsg(m);
    
  }

  if(bouncer5.fell()) { 
    
    if(BUTTON_DEBUG) Serial.print(F("blue button (B)"));

    if(reset_leds) {
      ledsOff();
      red_on = false;
      yellow_on = false;
      green_on = false;
      white_on = false;
      black_on = false;
    }

    blue_on = !blue_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'B';
    m.key = 0;

    if(blue_on) {
      m.val = 1;
      digitalWrite(LED5, HIGH);
      CURRENT_STATE = SCOOP_F_STATE;
    } else {
      m.val = 0;
      digitalWrite(LED5, LOW);
      CURRENT_STATE = IDLE_STATE;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    addNextMsg(m);
    
  }

  if(bouncer6.fell()) {
    
    if(BUTTON_DEBUG) Serial.print(F("black button (N)"));

    if(reset_leds) {
      red_on = false;
      yellow_on = false;
      green_on = false;
      white_on = false;
      blue_on = false;
    }

    black_on = !black_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'N';
    m.key = 0;

    if(black_on) {
      m.val = 1;
      digitalWrite(LED6, HIGH);
      CURRENT_STATE = EMPTY_STATE;
    } else {
      m.val = 0;
      digitalWrite(LED6, LOW);
      CURRENT_STATE = IDLE_STATE;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    addNextMsg(m);
    
  }

  if(joystick_button.fell()) {
    
    //if(BUTTON_DEBUG) 
    Serial.print(F("joystick button (J)"));

    // for the joystick, this defaults to true
    // because of the initialisation sequence

    joystick_on = !joystick_on;

    m.priority = 1;
    m.action = '#';
    m.cmd = 'J';
    m.key = 0;

    if(joystick_on) {
      m.val = 1;
    } else {
      m.val = 0;
    }

    m.cmd2 = '0';
    m.key2 = 0;
    m.val2 = 0;
    m.delim = '!';

    addNextMsg(m);
    
  }

  */


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

bool Operator::getJoystickButton() {
  if(joystick_on) return true;
  return false;
}

// ---- Mode Switch

void Operator::updateModeSwitch() {

  int val = analogRead(MODE_SW);

  if(val >= MODE1_THRESH) {
    CURRENT_MODE = MODE1;
  } else if(val >= MODE2_THRESH) {
    CURRENT_MODE = MODE2;
  } else if(val >= MODE3_THRESH) {
    CURRENT_MODE = MODE3;
  }

}

// ---- Speaker

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
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
  pinMode(BUTTON5, INPUT_PULLUP);
  pinMode(BUTTON6, INPUT_PULLUP);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  pinMode(MODE_SW, INPUT);

  bounce_buttons[0].attach(BUTTON1);
  bounce_buttons[0].interval(DEBOUNCE);
  bounce_buttons[1].attach(BUTTON2);
  bounce_buttons[1].interval(DEBOUNCE);
  bounce_buttons[2].attach(BUTTON3);
  bounce_buttons[2].interval(DEBOUNCE);
  bounce_buttons[3].attach(BUTTON4);
  bounce_buttons[3].interval(DEBOUNCE);
  bounce_buttons[4].attach(BUTTON5);
  bounce_buttons[4].interval(DEBOUNCE);
  bounce_buttons[5].attach(BUTTON6);
  bounce_buttons[5].interval(DEBOUNCE);
  bounce_buttons[6].attach(JOYSTICK_SW);
  bounce_buttons[6].interval(DEBOUNCE);

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


// ---- Messages

uint8_t Operator::getMsgQueueLength() {
  return msgs_in_queue;
}

Msg Operator::popNextMsg() {
  struct Msg m = msg_queue[0];

  if(msgs_in_queue > 0) {
    for(int i=0; i<msgs_in_queue-1; i++) {
      msg_queue[i] = msg_queue[i+1];
    }
    msgs_in_queue--;
  }
  return m;
}

void Operator::addNextMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  Serial << "msgs in queue: " << msgs_in_queue << endl;
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    popNextMsg();
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
  Serial << "msgs in queue: " << msgs_in_queue << endl;
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    popNextMsg();
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

