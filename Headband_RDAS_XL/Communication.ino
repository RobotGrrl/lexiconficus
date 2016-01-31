void initComm() {
  xbee_promulgate.LOG_LEVEL = Promulgate::ERROR_;
  xbee_promulgate.set_rx_callback(received_action);
  xbee_promulgate.set_tx_callback(transmit_complete);

  serial_promulgate.LOG_LEVEL = Promulgate::ERROR_;
  serial_promulgate.set_rx_callback(received_action);
  serial_promulgate.set_tx_callback(transmit_complete); 
}


void checkCommunication() {
  
  while(xbee.available()) {
    char c = xbee.read();
    Serial << c;
    xbee_promulgate.organize_message(c);
  }
  
  if(Serial.available() > 0) {
    char c = Serial.read();
    serial_promulgate.organize_message(c);    
  }
   
}


void received_action(char action, char cmd, uint8_t key, uint16_t val, char delim) {
  
  if(DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "delim: " << delim << endl;
  }
  
  // notes:
  // #P<s>,<d>!
  // <s> from 0-255, <d> from 0-65536
  // P: goPlaces
  // F: goForwards
  // B: goBackwards
  // L: turnLeft
  // R: turnRight
  // S: stopMotors
  
  // this is sent to the sensor board when the motors are finished moving:
  // ^Z1,1!
  
}

void transmit_complete() {
  if(DEBUG) {
    Serial << "--transmit complete--" << endl;
  }
}



