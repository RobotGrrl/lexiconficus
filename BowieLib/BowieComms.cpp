#include "BowieComms.h"

BowieComms *BowieComms::bcInstance;

BowieComms::BowieComms() {
  
}

void BowieComms::begin() {

  // Instance of the class for the callbacks from Promulgate
  bcInstance = this;
  ROBOT_ID = 3;

  // LED
  COMM_LED = 12;
  CONN_TYPE = XBEE_CONN;

  // Promulgate
  current_time = 0;
  diff_time = 0;
  last_rx_msg = 0;
  last_transmit = 0;
  use_base64_parsing = false;

  // Xbee
  xbee = XBee();
  addr64 = XBeeAddress64(0x00000000, 0x0000ffff);
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

  // Xbee Operator Controllers
  XBeeAddress64 addr_all_controllers[MAX_CONTROLLERS];
  num_addrs = 0;
  ind_addr_sent = 0;
  for(int i=0; i<MAX_CONTROLLERS; i++) {
    addr_all_controllers[i] = XBeeAddress64(addr_fake.getMsb(), addr_fake.getLsb());
    addr_remote_controllers[i] = 0;
    failed_send_count[i] = 0;
    last_rx_all[i] = 0;
  }
  last_rx_check = 0;
  last_led_blink = 0;
  led_on = false;
  last_retry_time = 0;

  // Messages
  msgs_in_queue = 0;
  msg_send_index = 0;
  last_rx_comms = 0;
  unlikely_count = 0;
  for(int i=0; i<MAX_PERIODIC_MESSAGES; i++) {
    periodic_messages[i] = msg_none;
  }
  msg_send_items = 0;

}

void BowieComms::setRobotID(uint8_t the_robot_id) {
  ROBOT_ID = the_robot_id;
}

void BowieComms::set_comms_timeout_callback( void (*commsTimeoutCallback)() ) {
  _commsTimeoutCallback = commsTimeoutCallback;
}

void BowieComms::set_controller_added_callback( void (*controllerAddedCallback)() ) {
  _controllerAddedCallback = controllerAddedCallback;
}

void BowieComms::set_controller_removed_callback( void (*controllerRemovedCallback)() ) {
  _controllerRemovedCallback = controllerRemovedCallback;
}

void BowieComms::set_received_action_callback( void (*receivedActionCallback)(Msg m) ) {
  _receivedActionCallback = receivedActionCallback;
}

void BowieComms::initComms(int conn, int baud) {


  for(int i=0; i<MAX_CONTROLLERS; i++) {
    addr_all_controllers[i] = XBeeAddress64(addr_fake.getMsb(), addr_fake.getLsb());
    addr_remote_controllers[i] = 0;
    failed_send_count[i] = 0;
    last_rx_all[i] = 0;
  }



  pinMode(COMM_LED, OUTPUT);

  // found these defs for serial ports here:
  // https://github.com/PaulStoffregen/cores/tree/master/teensy3

  bool possible = false;

  if(conn == PI_CONN) {

    Serial1.begin(baud);
    // Promulgate
    promulgate = Promulgate(&Serial1, &Serial1);

    CONN_TYPE = PI_CONN;
    possible = true;

  } else if(conn == XBEE_CONN) {
    
    delay(100);

    // Start xbee's serial
    Serial2.begin(baud);
    xbee.begin(Serial2);

    // Promulgate
    promulgate = Promulgate(&Serial2, &Serial2);

    CONN_TYPE = XBEE_CONN;
    possible = true;

  } else if(conn == GPS_CONN) {

    Serial3.begin(baud);
    // Promulgate
    promulgate = Promulgate(&Serial3, &Serial3);

    CONN_TYPE = GPS_CONN;
    possible = true;

  } else if(conn == PIXY_CONN) {

    #ifdef HAS_KINETISK_UART3

    Serial4.begin(baud);
    // Promulgate
    promulgate = Promulgate(&Serial4, &Serial4);

    CONN_TYPE = PIXY_CONN;
    possible = true;

    #endif

  } else if(conn == BT_CONN) {

    #ifdef HAS_KINETISK_UART4

    Serial5.begin(baud);
    // Promulgate
    promulgate = Promulgate(&Serial5, &Serial5);

    CONN_TYPE = BT_CONN;
    possible = true;

    #endif

  } else if(conn == ARDUINO_CONN) {

    #if defined(HAS_KINETISK_UART5) || defined (HAS_KINETISK_LPUART0)

    Serial6.begin(baud);
    // Promulgate
    promulgate = Promulgate(&Serial6, &Serial6);

    CONN_TYPE = ARDUINO_CONN;
    possible = true;

    #endif

  }

  // promulgate setup
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(this->received_action);
  promulgate.set_tx_callback(this->transmit_complete);
  promulgate.set_debug_stream(&Serial);

  if(use_base64_parsing) {
    promulgate.useBase64Parsing(true);
  } else {
    promulgate.useBase64Parsing(false);
  }

  if(possible) {
    Serial.println("Connection set up OK");
  } else {
    Serial.println("ERROR SETTING UP CONNECTION - UART DOES NOT EXIST");
  }

}

void BowieComms::updateComms() {

  bcInstance = this;
  current_time = millis();

  // Conn Comms
  connBlink();
  
  // Xbee Specific
  if(CONN_TYPE == XBEE_CONN) xbeeWatchdog();

  // Sending a heartbeat (with the robot's ID) if we haven't heard recently.
  // This is important, because the robot only sends if it receives an action
  // from the controller. So if we don't hear, we will actively try to re-establish
  // the comms by sending this.
  if(current_time-last_tx >= HEARTBEAT_MS) {
    if(COMM_DEBUG) Serial << "Sending a heartbeat" << endl;
    connSend('$', 'W', 1, ROBOT_ID, 'W', 1, ROBOT_ID, '!' );
    last_tx = current_time;
  }

  // Reading the data from the Stream
  connRead();

  // comms have timed out
  if(millis()-last_rx_comms >= REMOTE_OP_TIMEOUT) {
    digitalWrite(COMM_LED, LOW);
    // callback that the comms has timed out
    _commsTimeoutCallback();
  } else {
    digitalWrite(COMM_LED, HIGH);
  }

}

void BowieComms::setCommLed(uint8_t pin) {
  COMM_LED = pin;
}

unsigned long BowieComms::getCommLatency() {
  return diff_time;
}

unsigned long BowieComms::getLastRXTime() {
  return last_rx_comms;
}

void BowieComms::setConnType(uint8_t t) {
  CONN_TYPE = t;
  if(CONN_DEBUG) Serial << "setConnType " << t << " CONN_TYPE " << CONN_TYPE << endl;
}


/*

---- Xbee Comms ----

*/

void BowieComms::addXbeeToList(XBeeAddress64 newAddr) {
  if(XBEE_DEBUG) Serial << "Sender address - High: ";
  if(XBEE_DEBUG) print32Bits(newAddr.getMsb());
  if(XBEE_DEBUG) Serial << " Low: ";
  if(XBEE_DEBUG) print32Bits(newAddr.getLsb());
  if(XBEE_DEBUG) Serial << endl;

  bool add_it_to_list = true;

  for(int i=0; i<MAX_CONTROLLERS; i++) {
    if(addr_all_controllers[i].getMsb() == newAddr.getMsb() && addr_all_controllers[i].getLsb() == newAddr.getLsb()) {
      Serial << "Not adding to the list - already there" << endl;
      // already on the list
      add_it_to_list = false;
    }
  }

  if(add_it_to_list) {
    if(num_addrs >= MAX_CONTROLLERS-1) {
      Serial << "! Reached max number of controllers !" << endl;
      return;
    }
    Serial << "! New controller added to Xbee list !" << endl;
    addr_all_controllers[num_addrs] = XBeeAddress64(newAddr.getMsb(), newAddr.getLsb());
    num_addrs++;
    Serial.println(num_addrs);
    for(int i=0; i<MAX_CONTROLLERS; i++) {
      Serial << i << ": ";
      print32Bits(addr_all_controllers[i].getMsb());
      Serial << " ";
      print32Bits(addr_all_controllers[i].getLsb());
      Serial << endl;
    }
    // immediately reply with an identifier for the robot
    connSend('$', 'W', 1, ROBOT_ID, 'W', 1, ROBOT_ID, '!' );
    _controllerAddedCallback();
  }
  
}

void BowieComms::updateRxTime(XBeeAddress64 senderLongAddress) {
  for(int i=0; i<MAX_CONTROLLERS; i++) {
    if(addr_all_controllers[i].getMsb() == senderLongAddress.getMsb() && addr_all_controllers[i].getLsb() == senderLongAddress.getLsb()) {
      last_rx_all[i] = millis();
      return;
    }
  }
}

void BowieComms::xbeeWatchdog() {
  if(current_time-last_rx_check >= 10000) {
    for(int i=0; i<MAX_CONTROLLERS; i++) {
      if(current_time-last_rx_all[i] >= 11000) {
        if(addr_all_controllers[i].getMsb() != 0 && addr_all_controllers[i].getLsb() != 0) {
          if(addr_all_controllers[i].getMsb() != addr_fake.getMsb() && addr_all_controllers[i].getLsb() != addr_fake.getLsb()) {
            // remove it from the list
            if(XBEE_DEBUG) Serial << "Have not heard from: ";
            if(XBEE_DEBUG) print32Bits(addr_all_controllers[i].getMsb());
            if(XBEE_DEBUG) Serial << " ";
            if(XBEE_DEBUG) print32Bits(addr_all_controllers[i].getLsb());
            if(XBEE_DEBUG) Serial << " since " << current_time-last_rx_all[i] << "ms ago." << endl;
            if(XBEE_DEBUG) Serial << "! Removing it from list !" << endl;
            addr_all_controllers[i] = XBeeAddress64(addr_fake.getMsb(), addr_fake.getLsb());
            last_rx_all[i] = 0;
            // then push everything else up
            for(int j=i; j<MAX_CONTROLLERS-1; j++) {
              addr_all_controllers[j] = addr_all_controllers[j+1];
              last_rx_all[j] = last_rx_all[j+1];
            }
            // so that we can properly deincrement the num_addr counter
            num_addrs--;
            for(int i=0; i<MAX_CONTROLLERS; i++) {
              Serial << i << ": ";
              print32Bits(addr_all_controllers[i].getMsb());
              Serial << " ";
              print32Bits(addr_all_controllers[i].getLsb());
              Serial << endl;
            }
            if(num_addrs < 0) num_addrs = 0;
            _controllerRemovedCallback();
          }
        }
      }
    }
    last_rx_check = current_time;
  }
}

bool BowieComms::xbeeRead() {

  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) { // we got something

    if(XBEE_DEBUG) Serial << "Xbee response: " << xbee.getResponse().getApiId();

    // --- Node Identifier Response
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
      if(addr_remote_controllers[ind_addr_sent] == 0) {
        addr_remote_controllers[ind_addr_sent] = the_tx_addr;
      }
    }

    // --- RX Response
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) { // got an rx packet
      if(XBEE_DEBUG) Serial << "RX" << endl;

      xbee.getResponse().getZBRxResponse(rx);

      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      if(XBEE_DEBUG) Serial.print(msg_rx_count);
      if(XBEE_DEBUG) Serial.print(" >>>> Received data from ");
      if(XBEE_DEBUG) print32Bits(senderLongAddress.getMsb());
      if(XBEE_DEBUG) print32Bits(senderLongAddress.getLsb());
      if(XBEE_DEBUG) Serial.println(": ");

      addXbeeToList(senderLongAddress);
      updateRxTime(senderLongAddress);

      for(int i=0; i<32; i++) {
        message_rx[i] = ' ';
      }

      // ascii representation of text sent through xbee
      if(XBEE_DEBUG) Serial << "This is the data: ";
      for (int i=0; i <rx.getDataLength(); i++){
        if (!iscntrl(rx.getData()[i])) {
          message_rx[i] = (char)rx.getData()[i];
          if(XBEE_DEBUG) Serial.write(message_rx[i]);
        }
      }
      Serial.println();
      for(int i=0; i<10; i++) {
        if(XBEE_DEBUG) Serial << (char)message_rx[i];
      }
      last_rx = millis();
      return true;
    }
  }

  return false;
  
}

// these routines are just to print the data with
// leading zeros and allow formatting such that it
// will be easy to read. huzzah!
void BowieComms::print32Bits(uint32_t dw){
  print16Bits(dw >> 16);
  print16Bits(dw & 0xFFFF);
}

void BowieComms::print16Bits(uint16_t w){
  print8Bits(w >> 8);
  print8Bits(w & 0x00FF);
}

void BowieComms::print8Bits(byte c){
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

void BowieComms::connRead() {

  char c;

  if(CONN_TYPE == PI_CONN) {

    while(Serial1.available()) {
      c = Serial1.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

  } else if(CONN_TYPE == XBEE_CONN) {

    if(XBEE_DEBUG) Serial << "Conn read" << endl;

    while(xbeeRead()) {
      if(XBEE_DEBUG) Serial << "\nRead...<< ";
      for(int i=0; i<=rx.getDataLength(); i++) {
        c = message_rx[i];
        promulgate.organize_message(c);
        if(XBEE_DEBUG) Serial << c;
        if(XBEE_DEBUG) { if(c == '!' || c == '?' || c == ';') Serial << endl; }
      }
    }

  } else if(CONN_TYPE == GPS_CONN) {

    while(Serial3.available()) {
      c = Serial3.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

  } else if(CONN_TYPE == PIXY_CONN) {

    #ifdef HAS_KINETISK_UART3

    while(Serial4.available()) {
      c = Serial4.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

    #endif

  } else if(CONN_TYPE == BT_CONN) {

    #ifdef HAS_KINETISK_UART4

    while(Serial5.available()) {
      c = Serial5.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

    #endif

  } else if(CONN_TYPE == ARDUINO_CONN) {

    #if defined(HAS_KINETISK_UART5) || defined (HAS_KINETISK_LPUART0)

    while(Serial6.available()) {
      c = Serial6.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

    #endif

  }

}

void BowieComms::connBlink() {
  if(current_time-last_led_blink >= 100) {
    if(current_time-last_rx <= 6000 && current_time > 6000) { // the last bit is to make sure it doesn't turn on on startup
      digitalWrite(COMM_LED, led_on);
      led_on = !led_on;
    } else {
      digitalWrite(COMM_LED, LOW);
    }
    last_led_blink = current_time;
  }
}

void BowieComms::connSend(Msg m) {

  if(CONN_DEBUG) Serial << "Entering connSend m" << endl;

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", m.action, m.pck1.cmd, m.pck1.key, m.pck1.val, m.pck2.cmd, m.pck2.key, m.pck2.val, m.delim);
  
  if(CONN_DEBUG) Serial << "Conn TX: " << message_tx << endl;
  if(CONN_DEBUG) Serial << "CONN_TYPE = " << CONN_TYPE << endl;
  
  if(CONN_TYPE == PI_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    //Serial << "num_addrs = " << num_addrs << endl;

    for(int i=0; i<num_addrs; i++) {
      if(i >= MAX_CONTROLLERS) break;
      if(addr_all_controllers[i].getMsb() != 0 && addr_all_controllers[i].getLsb() != 0) {
        if(CONN_DEBUG) Serial << "Xbee TX: " << message_tx << endl;
        // crash here while printing the above
        ZBTxRequest zbtx = ZBTxRequest(addr_all_controllers[i], (uint8_t *)message_tx, strlen(message_tx));
        zbtx.setFrameId('0');
        xbee.send(zbtx); 
        ind_addr_sent = i;
        last_tx = current_time;
      }
    }
    last_tx = current_time;

  } else if(CONN_TYPE == GPS_CONN) {

    Serial3.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == PIXY_CONN) {

    #ifdef HAS_KINETISK_UART3
    Serial4.print(message_tx);
    last_tx = current_time;
    #endif

  } else if(CONN_TYPE == BT_CONN) {

    #ifdef HAS_KINETISK_UART4
    Serial5.print(message_tx);
    last_tx = current_time;
    #endif

  } else if(CONN_TYPE == ARDUINO_CONN) {

    #if defined(HAS_KINETISK_UART5) || defined (HAS_KINETISK_LPUART0)
    Serial6.print(message_tx);
    last_tx = current_time;
    #endif

  }

}

// TODO combine these two functions
void BowieComms::connSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  if(CONN_DEBUG) Serial << "Entering connSend long" << endl;

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);

  if(CONN_DEBUG) Serial.print("Conn TX ");
  if(CONN_DEBUG) Serial.print(message_tx);
  if(CONN_DEBUG) Serial.print("\n");
  if(CONN_DEBUG) Serial << "CONN_TYPE = " << CONN_TYPE << endl;
  
  if(CONN_TYPE == PI_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    //Serial << "num_addrs = " << num_addrs << endl;
    //num_addrs = 1;
    //Serial << "num_addrs = " << num_addrs << endl;

    if(CONN_DEBUG) Serial.print("NUM ADDRS: ");
    if(CONN_DEBUG) Serial.print(num_addrs);
    if(CONN_DEBUG) Serial.print("\n");

    for(int i=0; i<num_addrs; i++) {
      if(i >= MAX_CONTROLLERS) break;
      if(addr_all_controllers[i].getMsb() != 0 && addr_all_controllers[i].getLsb() != 0) {
        if(CONN_DEBUG) Serial.print("Xbee TX ");
        if(CONN_DEBUG) Serial.print(message_tx);
        Serial.print("\n");
        //Serial << "Xbee TX: " << message_tx << endl;
        ZBTxRequest zbtx = ZBTxRequest(addr_all_controllers[i], (uint8_t *)message_tx, strlen(message_tx));
        zbtx.setFrameId('0');
        xbee.send(zbtx); 
        ind_addr_sent = i;
        last_tx = current_time;
      }
    }
    last_tx = current_time;

  } else if(CONN_TYPE == GPS_CONN) {

    Serial3.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == PIXY_CONN) {

    #ifdef HAS_KINETISK_UART3
    Serial4.print(message_tx);
    last_tx = current_time;
    #endif

  } else if(CONN_TYPE == BT_CONN) {

    #ifdef HAS_KINETISK_UART4
    Serial5.print(message_tx);
    last_tx = current_time;
    #endif

  } else if(CONN_TYPE == ARDUINO_CONN) {

    #if defined(HAS_KINETISK_UART5) || defined (HAS_KINETISK_LPUART0)
    Serial6.print(message_tx);
    last_tx = current_time;
    #endif

  }
 
}

// this is only used in case of debugging
void BowieComms::connSendEasy(char c) {

  sprintf(message_tx,"%c", c);

  if(CONN_DEBUG) Serial << "Conn TX: " << message_tx << endl;

  if(CONN_TYPE == PI_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    for(int i=0; i<num_addrs; i++) {
      if(addr_all_controllers[i].getMsb() != 0 && addr_all_controllers[i].getLsb() != 0) {
        if(CONN_DEBUG) Serial << "[" << i << "] Xbee TX: " << message_tx << " - ";
        if(CONN_DEBUG) print32Bits(addr_all_controllers[i].getMsb());
        if(CONN_DEBUG) Serial << " ";
        if(CONN_DEBUG) print32Bits(addr_all_controllers[i].getLsb());
        if(CONN_DEBUG) Serial << endl;
              
        ZBTxRequest zbtx = ZBTxRequest(addr_all_controllers[i], (uint8_t *)message_tx, strlen(message_tx));
        zbtx.setFrameId('0');
        xbee.send(zbtx); 
        ind_addr_sent = i;
      }
    }
    last_tx = current_time;

  } else if(CONN_TYPE == GPS_CONN) {

    Serial3.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == PIXY_CONN) {

    #ifdef HAS_KINETISK_UART3
    Serial4.print(message_tx);
    last_tx = current_time;
    #endif

  } else if(CONN_TYPE == BT_CONN) {

    #ifdef HAS_KINETISK_UART4
    Serial5.print(message_tx);
    last_tx = current_time;
    #endif

  } else if(CONN_TYPE == ARDUINO_CONN) {

    #if defined(HAS_KINETISK_UART5) || defined (HAS_KINETISK_LPUART0)
    Serial6.print(message_tx);
    last_tx = current_time;
    #endif

  }

}


/*

---- Promulgate Related ----

*/

void BowieComms::received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  Packet p1 = { cmd, key, val };
  Packet p2 = { cmd2, key2, val2 };
  Msg m = { 99, action, p1, p2, delim };

  bcInstance->processAction(m);
}

void BowieComms::transmit_complete() {
  bcInstance->transmitDidComplete();
}

void BowieComms::processAction(Msg m) {

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
    if(m.pck1.cmd == 'X' || m.pck2.cmd == 'X') {
      // reply with our id
      delay(20);
      connSend('$', 'W', 1, ROBOT_ID, 'W', 1, ROBOT_ID, '!' );
    }
  } else {
    chooseNextMessage();
    sendNextMsg();
  }
  
  _receivedActionCallback(m);

  msg_rx_count++;
  diff_time = millis()-last_rx_msg;
  last_rx_msg = current_time;
  Serial << "COMMS- Roundtrip latency (ms): " << diff_time << " Msg count: " << msg_rx_count << endl;

}

int BowieComms::getMsgRxCount() {
  return msg_rx_count;
}

void BowieComms::transmitDidComplete() {
  // Not really sure what to put here right now, so built in in for the future
}

void BowieComms::sendNextMsg() {
  Msg m = popNextMsg();
  connSend(m);
}



/*

---- Messages ----

*/

uint8_t BowieComms::getMsgQueueLength() {
  return msgs_in_queue;
}

// Retrieve the next message, and move the other messages behind it up
Msg BowieComms::popNextMsg() {

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

void BowieComms::addMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  //Serial.print("adding next message");
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(COMM_DEBUG) {
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
void BowieComms::addMsg(Msg m) {
  //Serial.print("adding next message");
  if(msgs_in_queue > MSG_QUEUE_SIZE-1) {
    if(COMM_DEBUG) {
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
void BowieComms::insertMsg(Msg m) {

  if(MSG_DEBUG) Serial.print("inserting next message");

  if(msgs_in_queue == 0) {
    if(MSG_DEBUG) Serial << "Adding it to index 0" << endl;
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
    addMsg(m);
  }

}

void BowieComms::chooseNextMessage() {

  addMsg(periodic_messages[msg_send_index]);
  
  msg_send_index++;
  if(msg_send_index > msg_send_items) msg_send_index = 0;

}

void BowieComms::addPeriodicMessage(Msg m) {
  if(msg_send_items < MAX_PERIODIC_MESSAGES) { // only add if there's enough room
    periodic_messages[msg_send_items] = m;
    msg_send_items++;
  }
}

void BowieComms::updatePeriodicMessage(Msg m) {
  for(int i=0; i<msg_send_items; i++) {
    Msg temp = periodic_messages[i];
    if(temp.pck1.cmd == m.pck1.cmd && temp.pck2.cmd == m.pck2.cmd) { // they both match
      periodic_messages[i] = m;
    }
  }
}

void BowieComms::removePeriodicMessage(uint8_t remove_ind) {
  if(remove_ind != 99) {
    for(int j=remove_ind; j<msg_send_items-1; j++) {
      periodic_messages[j] = periodic_messages[j+1];
    }
    msg_send_items--;
    if(msg_send_items < 0) msg_send_items = 0;
  }
}

void BowieComms::removePeriodicMessage(Msg m) {
  uint8_t remove_ind = 99;
  for(int i=0; i<msg_send_items; i++) {
    Msg temp = periodic_messages[i];
    if(temp.pck1.cmd == m.pck1.cmd && temp.pck2.cmd == m.pck2.cmd) { // they both match
      remove_ind = i;
      break;
    }
  }

  if(remove_ind != 99) {
    for(int j=remove_ind; j<msg_send_items-1; j++) {
      periodic_messages[j] = periodic_messages[j+1];
    }
    msg_send_items--;
    if(msg_send_items < 0) msg_send_items = 0;
  }

}
