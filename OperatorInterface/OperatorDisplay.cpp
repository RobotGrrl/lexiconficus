#include "OperatorDisplay.h"

OperatorDisplay::OperatorDisplay() {
  
}

void OperatorDisplay::begin() {
  
  // Display
  //display = Adafruit_SSD1306(OLED_RESET);
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)  
  display.clearDisplay(); // Clear the buffer.
  display.display();
  for(int i=0; i<6; i++) {
    for(int j=0; j<3; j++) {
      buttonLabels[i][j] = " ";
    }
  }
  for(int i=0; i<3; i++) {
    modeLabels[i] = " ";
  }
  last_letter_itr = 0;
  letter_itr = 0;

}

void OperatorDisplay::setButtonLabel(String label, int button, int mode) {
  buttonLabels[button][mode-1] = label;
}

void OperatorDisplay::setModeLabel(String label, int mode) {
  modeLabels[mode-1] = label;
}

void OperatorDisplay::displaySearching(unsigned long current_time) {

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(10,10);
  
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

void OperatorDisplay::mainMenu(uint8_t button_states[], uint8_t CURRENT_MODE) {

  display.clearDisplay();

  displayTitleBar(CURRENT_MODE);

  uint8_t r = 5;
  uint8_t y = 20;
  uint8_t sp = 27;

  if(button_states[0] == 1) {
    display.fillRect(((display.width()/2)-50)-r, y-r, r*2, r*2, WHITE);
  } else {
    display.drawCircle((display.width()/2)-50, y, r, WHITE);
  }
  
  if(button_states[1] == 1) {
    display.fillRect((display.width()/2)-r, y-r, r*2, r*2, WHITE);
  } else {
    display.drawCircle(display.width()/2, y, r, WHITE);
  }

  if(button_states[2] == 1) {
    display.fillRect(((display.width()/2)+50)-r, y-r, r*2, r*2, WHITE);
  } else {
    display.drawCircle((display.width()/2)+50, y, r, WHITE);
  }


  if(button_states[3] == 1) {
    display.fillRect(((display.width()/2)-50)-r, y+sp-r, r*2, r*2, WHITE);
  } else {
    display.drawCircle((display.width()/2)-50, y+sp, r, WHITE);
  }

  if(button_states[4] == 1) {
    display.fillRect((display.width()/2)-r, y+sp-r, r*2, r*2, WHITE);
  } else {
    display.drawCircle(display.width()/2, y+sp, r, WHITE);
  }

  if(button_states[5] == 1) {
    display.fillRect(((display.width()/2)+50)-r, y+sp-r, r*2, r*2, WHITE);
  } else {
    display.drawCircle((display.width()/2)+50, y+sp, r, WHITE);
  }

  display.setTextSize(1);

  if(CURRENT_MODE == MODE1) {

    display.setCursor(0,y+10);
    display.println(buttonLabels[0][0]);

    display.setCursor((display.width()/2)-15,y+10);
    display.println(buttonLabels[1][0]);

    display.setCursor((display.width()/2)+32,y+10);
    display.println(buttonLabels[2][0]);

    display.setCursor(0,y+sp+10);
    display.println(buttonLabels[3][0]);

    display.setCursor((display.width()/2)-15,y+sp+10);
    display.println(buttonLabels[4][0]);

    display.setCursor((display.width()/2)+32,y+sp+10);
    display.println(buttonLabels[5][0]);

  } else if(CURRENT_MODE == MODE2) {

    display.setCursor(0,y+10);
    display.println(buttonLabels[0][1]);

    display.setCursor((display.width()/2)-15,y+10);
    display.println(buttonLabels[1][1]);

    display.setCursor((display.width()/2)+32,y+10);
    display.println(buttonLabels[2][1]);

    display.setCursor(0,y+sp+10);
    display.println(buttonLabels[3][1]);

    display.setCursor((display.width()/2)-15,y+sp+10);
    display.println(buttonLabels[4][1]);

    display.setCursor((display.width()/2)+32,y+sp+10);
    display.println(buttonLabels[5][1]);

  } else if(CURRENT_MODE == MODE3) {

    display.setCursor(0,y+10);
    display.println(buttonLabels[0][2]);

    display.setCursor((display.width()/2)-15,y+10);
    display.println(buttonLabels[1][2]);

    display.setCursor((display.width()/2)+32,y+10);
    display.println(buttonLabels[2][2]);

    display.setCursor(0,y+sp+10);
    display.println(buttonLabels[3][2]);

    display.setCursor((display.width()/2)-15,y+sp+10);
    display.println(buttonLabels[4][2]);

    display.setCursor((display.width()/2)+32,y+sp+10);
    display.println(buttonLabels[5][2]);

  }

  display.display();

}

void OperatorDisplay::displayTitleBar(uint8_t CURRENT_MODE) {

  display.drawLine(0, 10, display.width()-1, 10, WHITE);

  display.setCursor(5,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if(CURRENT_MODE == MODE1) {
    display.println(modeLabels[0]);
  } else if(CURRENT_MODE == MODE2) {
    display.println(modeLabels[1]);
  } else if(CURRENT_MODE == MODE3) {
    display.println(modeLabels[2]);
  }

  display.setCursor(80,0);
  display.println("Batt:");
  display.setCursor(108,0);
  uint8_t batt_val = 100;
  display.println(batt_val);

}

void OperatorDisplay::displayLogo() {
  display.drawBitmap(0, 0,  robot_missions_logo, 128, 64, 1);
  display.display();
}

void OperatorDisplay::scrollLogo() {
  display.drawBitmap(0, 0,  robot_missions_logo, 128, 64, 1);
  display.display();
  
  display.startscrolldiagright(0x00, 0x0F);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
}

void OperatorDisplay::clearTheDisplay() {
  display.clearDisplay();
}

void OperatorDisplay::displaySearchingRobotIDs(uint16_t ids_of_all_robots[]) {

  display.setTextColor(WHITE);

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
}
