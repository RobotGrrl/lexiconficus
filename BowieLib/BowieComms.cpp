#include "BowieComms.h"

BowieComms::BowieComms() {
  
  // Instance of the class for the callbacks from Promulgate
  bcInstance = this;

  // LED
  COMM_LED = 12;

  // Promulgate
  current_time = 0;
  diff_time = 0;
  last_rx_msg = 0;
  last_transmit = 0;

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

  // Xbee Operator Controllers
  XBeeAddress64 addr_all_controllers[MAX_CONTROLLERS];
  num_addrs = 0;
  ind_addr_sent = 0;
  for(int i=0; i<MAX_CONTROLLERS; i++) {
    addr_all_controllers[i] = XBeeAddress64(0, 0);
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

void BowieComms::initComms(int conn) {
  pinMode(COMM_LED, OUTPUT);

  // found these defs for serial ports here:
  // https://github.com/PaulStoffregen/cores/tree/master/teensy3

  bool possible = false;

  if(conn == PI_CONN) {

    Serial1.begin(9600);
    // Promulgate
    promulgate = Promulgate(&Serial1, &Serial1);

    CONN_TYPE = PI_CONN;
    possible = true;

  } else if(conn == XBEE_CONN) {
    
    // Start xbee's serial
    Serial2.begin(9600);
    xbee.begin(Serial2);

    // Promulgate
    promulgate = Promulgate(&Serial2, &Serial2);

    CONN_TYPE = XBEE_CONN;
    possible = true;

  } else if(conn == GPS_CONN) {

    Serial3.begin(9600);
    // Promulgate
    promulgate = Promulgate(&Serial3, &Serial3);

    CONN_TYPE = GPS_CONN;
    possible = true;

  } else if(conn == PIXY_CONN) {

    #ifdef HAS_KINETISK_UART3

    Serial4.begin(9600);
    // Promulgate
    promulgate = Promulgate(&Serial4, &Serial4);

    CONN_TYPE = PIXY_CONN;
    possible = true;

    #endif

  } else if(conn == BT_CONN) {

    #ifdef HAS_KINETISK_UART4

    Serial5.begin(9600);
    // Promulgate
    promulgate = Promulgate(&Serial5, &Serial5);

    CONN_TYPE = BT_CONN;
    possible = true;

    #endif

  } else if(conn == ARDUINO_CONN) {

    #if defined(HAS_KINETISK_UART5) || defined (HAS_KINETISK_LPUART0)

    Serial6.begin(9600);
    // Promulgate
    promulgate = Promulgate(&Serial6, &Serial6);

    CONN_TYPE = ARDUINO_CONN;
    possible = true;

    #endif

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

}

void BowieComms::updateComms() {

  // Conn Comms
  connBlink();
  
  // Xbee Specific
  if(CONN_TYPE == XBEE_CONN) xbeeWatchdog();

  // Sending a heartbeat (with the robot's ID) if we haven't heard recently.
  // This is important, because the robot only sends if it receives an action
  // from the controller. So if we don't hear, we will actively try to re-establish
  // the comms by sending this.
  if(current_time-last_tx >= HEARTBEAT_MS) {
    Serial << "Sending a heartbeat" << endl;
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


/*

---- Xbee Comms ----

*/

void BowieComms::addXbeeToList(XBeeAddress64 newAddr) {
  Serial << "Sender address - High: ";
  print32Bits(newAddr.getMsb());
  Serial << " Low: ";
  print32Bits(newAddr.getLsb());
  Serial << endl;

  bool add_it_to_list = true;

  for(int i=0; i<MAX_CONTROLLERS; i++) {
    if(addr_all_controllers[i].getMsb() == newAddr.getMsb() && addr_all_controllers[i].getLsb() == newAddr.getLsb()) {
      // already on the list
      add_it_to_list = false;
    }
  }

  if(add_it_to_list) {
    Serial << "! New controller added to Xbee list !" << endl;
    addr_all_controllers[num_addrs] = XBeeAddress64(newAddr.getMsb(), newAddr.getLsb());
    num_addrs++;
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
          // remove it from the list
          Serial << "Have not heard from: ";
          print32Bits(addr_all_controllers[i].getMsb());
          Serial << " ";
          print32Bits(addr_all_controllers[i].getLsb());
          Serial << " since " << current_time-last_rx_all[i] << "ms ago." << endl;
          Serial << "! Removing it from list !" << endl;
          addr_all_controllers[i] = XBeeAddress64(0, 0);
          last_rx_all[i] = 0;
          // then push everything else up
          for(int j=i; j<MAX_CONTROLLERS-1; j++) {
            addr_all_controllers[j] = addr_all_controllers[j+1];
            last_rx_all[j] = last_rx_all[j+1];
          }
          // so that we can properly deincrement the num_addr counter
          num_addrs--;
          _controllerRemovedCallback();
        }
      }
    }
    last_rx_check = current_time;
  }
}

bool BowieComms::xbeeRead() {

  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) { // we got something

    Serial << "Xbee response: " << xbee.getResponse().getApiId() << endl;

    // --- Node Identifier Response
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
      if(addr_remote_controllers[ind_addr_sent] == 0) {
        addr_remote_controllers[ind_addr_sent] = the_tx_addr;
      }
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

    while(xbeeRead()) {
      //Serial << "\n<< ";
      for(int i=0; i<=rx.getDataLength(); i++) {
        c = message_rx[i];
        promulgate.organize_message(c);
        Serial << c;
        if(c == '!' || c == '?' || c == ';') Serial << endl;
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

    while(Serial4.available()) {
      c = Serial4.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

  } else if(CONN_TYPE == BT_CONN) {

    while(Serial5.available()) {
      c = Serial5.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

  } else if(CONN_TYPE == ARDUINO_CONN) {

    while(Serial6.available()) {
      c = Serial6.read();
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }

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

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", m.action, m.pck1.cmd, m.pck1.key, m.pck1.val, m.pck2.cmd, m.pck2.key, m.pck2.val, m.delim);
  
  Serial << "Conn TX: " << message_tx << endl;

  if(CONN_TYPE == PI_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    for(int i=0; i<num_addrs; i++) {
      if(addr_all_controllers[i].getMsb() != 0 && addr_all_controllers[i].getLsb() != 0) {
        Serial << "Xbee TX: " << message_tx << endl;
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

void BowieComms::connSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);

  Serial << "Conn TX: " << message_tx << endl;

  if(CONN_TYPE == PI_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    for(int i=0; i<num_addrs; i++) {
      if(addr_all_controllers[i].getMsb() != 0 && addr_all_controllers[i].getLsb() != 0) {
        Serial << "Xbee TX: " << message_tx << endl;
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

  Serial << "Conn TX: " << message_tx << endl;

  if(CONN_TYPE == PI_CONN) {

    Serial1.print(message_tx);
    last_tx = current_time;

  } else if(CONN_TYPE == XBEE_CONN) {

    for(int i=0; i<num_addrs; i++) {
      if(addr_all_controllers[i].getMsb() != 0 && addr_all_controllers[i].getLsb() != 0) {
        Serial << "[" << i << "] Xbee TX: " << message_tx << " - ";
        print32Bits(addr_all_controllers[i].getMsb());
        Serial << " ";
        print32Bits(addr_all_controllers[i].getLsb());
        Serial << endl;
              
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

BowieComms *BowieComms::bcInstance;

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

  if(PROG_DEBUG) {
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
  diff_time = current_time-last_rx_msg;
  last_rx_msg = current_time;
  Serial << "COMMS- Roundtrip latency (ms): " << diff_time << " Msg count: " << msg_rx_count << endl;

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
  struct Msg m = msg_queue[0];

  for(int i=0; i<msgs_in_queue-1; i++) {
    msg_queue[i] = msg_queue[i+1];
  }

  if(msgs_in_queue > 0) msgs_in_queue--;
  return m;
}

void BowieComms::addMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
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
void BowieComms::addMsg(Msg m) {
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
void BowieComms::insertMsg(Msg m) {

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
