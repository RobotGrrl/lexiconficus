#include "BowieComms.h"

BowieComms::BowieComms() {
  // start xbee's serial
  Serial2.begin(9600);
  xbee.begin(Serial2);

  COMM_LED = 12;

  // Promulgate
  promulgate = Promulgate(&Serial2, &Serial2);
  current_time = 0;
  diff_time = 0;
  last_rx_msg = 0;
  last_transmit = 0;

  // promulgate setup
  promulgate.LOG_LEVEL = Promulgate::ERROR_;
  promulgate.set_rx_callback(received_action);
  promulgate.set_tx_callback(transmit_complete);
  promulgate.set_debug_stream(&Serial);

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

  XBeeAddress64 addr_all_controllers[MAX_CONTROLLERS];
  num_addrs = 0;
  ind_addr_sent = 0;
  for(int i=0; i<MAX_CONTROLLERS; i++) {
    addr_all_controllers[i] = XBeeAddress64(0x00000000, 0x0000ffff);
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

}

void BowieComms::set_comms_timeout_callback( void (*commsTimeoutCallback)() ) {
  _commsTimeoutCallback = commsTimeoutCallback;
}

void BowieComms::initComms() {
  pinMode(COMM_LED, OUTPUT);
}

void BowieComms::updateComms() {

  // XBee Comms
  xbeeBlink();
  xbeeWatchdog();

  if(current_time-last_tx >= 1000) {
    // why would this happen? let's prevent a timeout
    Serial << "Timeout?" << endl;
    xbeeSend('$', 'W', 1, ROBOT_ID, 'W', 1, ROBOT_ID, '!' );
    last_tx = current_time;
  }

  while(xbeeRead()) {
    //Serial << "\n<< ";
    for(int i=0; i<=rx.getDataLength(); i++) {
      char c = message_rx[i];
      promulgate.organize_message(c);
      Serial << c;
      if(c == '!' || c == '?' || c == ';') Serial << endl;
    }
  }

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



/*

  Xbee Comms

*/

void BowieComms::xbeeSend(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  // Promulgate format:
  // *out_stream << action << cmd << key << "," << val << delim;
  
  sprintf(message_tx,"%c%c%d,%d,%c%d,%d%c", action, cmd, key, val, cmd2, key2, val2, delim);
  
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
 
}

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
    //xbeeSendEasy('W');
    xbeeSend('$', 'W', 1, ROBOT_ID, 'W', 1, ROBOT_ID, '!' );
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

void BowieComms::xbeeBlink() {
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

void BowieComms::xbeeVarsInit() {
  for(int i=0; i<MAX_CONTROLLERS; i++) {
    addr_all_controllers[i] = XBeeAddress64(0, 0);
    addr_remote_controllers[i] = 0;
    failed_send_count[i] = 0;
    last_rx_all[i] = 0;
  }
}

// this is only used in case of debugging
void BowieComms::xbeeSendEasy(char c) {

  sprintf(message_tx,"%c", c);

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

}

// these routines are just to print the data with
// leading zeros and allow formatting such that it
// will be easy to read.
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

Promulgate related


*/



void BowieComms::sendNextMsg() {
  Msg m = popNextMsg();
  xbeeSend(m.action, m.cmd, m.key, m.val, m.cmd2, m.key2, m.val2, m.delim);
}

void BowieComms::received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

  Cmd c1 = { '0', 0, 0 };
  Cmd c2 = { '0', 0, 0 };
  c1.cmd = cmd;
  c1.key = key;
  c1.val = val;
  c2.cmd = cmd2;
  c2.key = key2;
  c2.val = val2;
  Cmd packets[2] = { c1, c2 };

  if(PROG_DEBUG) {
    Serial << "---CALLBACK---" << endl;
    Serial << "action: " << action << endl;
    Serial << "command: " << cmd << endl;
    Serial << "key: " << key << endl;
    Serial << "val: " << val << endl;
    Serial << "command2: " << cmd2 << endl;
    Serial << "key2: " << key2 << endl;
    Serial << "val2: " << val2 << endl;
    Serial << "delim: " << delim << endl;
  }

  if(action == '$') {
    for(int i=0; i<2; i++) {
      if(packets[i].cmd == 'X') {
        // reply with our id
        delay(20);
        xbeeSend('$', 'W', 1, ROBOT_ID, 'W', 1, ROBOT_ID, '!' );
        break;
      }
    }
  } else {
    chooseNextMessage();
    sendNextMsg();
  }
  
  // TODO: send a callback here with the Cmd structs
  // inside of this callback will call the control function in MegaBowieShoreline

  msg_rx_count++;
  diff_time = current_time-last_rx_msg;
  last_rx_msg = current_time;
  Serial << "COMMS- Roundtrip latency (ms): " << diff_time << " Msg count: " << msg_rx_count << endl;

}

void BowieComms::transmit_complete() {

}



/*

Sensor priority list (lower # = bigger priority)

0 = none
1 = force & sonar sensors
2 = accel & gyro
3 = gpio
4 = mag

*/

void BowieComms::chooseNextMessage() {

  Msg m = {0, '^', '0', 0, 0, '0', 0, 0, '!'};

  switch(msg_send_index) {
    case 0:
      // accelerometer X & accelerometer Y
      m.priority = 2;
      m.action = '$';
      m.cmd = 'A';
      m.key = 1;
      m.val = accel_msg_x;
      m.cmd2 = 'A';
      m.key2 = 2;
      m.val2 = accel_msg_y;
    break;
    case 1:
      // accelerometer Z & gyro X
      m.priority = 2;
      m.action = '$';
      m.cmd = 'A';
      m.key = 3;
      m.val = accel_msg_z;
      m.cmd2 = 'O';
      m.key2 = 1;
      m.val2 = gyro_msg_x;
    break;
    case 2:
      // gyro Y & gyro Z
      m.priority = 2;
      m.action = '$';
      m.cmd = 'O';
      m.key = 2;
      m.val = gyro_msg_y;
      m.cmd2 = 'O';
      m.key2 = 3;
      m.val2 = gyro_msg_z;
    break;
    case 3:
      // force sensor L & R
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'F';
      m.key = 1;
      m.val = 0;//force_sensor_val_left;
      m.cmd2 = 'F';
      m.key2 = 2;
      m.val2 = 0;//force_sensor_val_right;
    break;
    case 4: 
      // sonar sensor L & R
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'U';
      m.key = 1;
      m.val = 0;//sonar_val_left;
      m.cmd2 = 'U';
      m.key2 = 2;
      m.val2 = 0;//sonar_val_right;
    break;
    case 5:
      // magnetometer X & Y
      m.priority = 2;
      m.action = '$';
      m.cmd = 'M';
      m.key = 1;
      m.val = mag_msg_x;
      m.cmd2 = 'M';
      m.key2 = 2;
      m.val2 = mag_msg_y;
    break;
    case 6:
      // magnetometer Z & altitude
      m.priority = 2;
      m.action = '$';
      m.cmd = 'M';
      m.key = 3;
      m.val = mag_msg_z;
      m.cmd2 = 'H';
      m.key2 = 1;
      m.val2 = alt_msg;
    break;
    case 7:
      // gpio 1 & 2
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 1;
      m.val = 0;//gpio_pin1_val;
      m.cmd2 = 'I';
      m.key2 = 2;
      m.val2 = 0;//gpio_pin2_val;
    break;
    case 8:
      // gpio 3 & 4
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 3;
      m.val = 0;//gpio_pin3_val;
      m.cmd2 = 'I';
      m.key2 = 4;
      m.val2 = 0;//gpio_pin4_val;
    break;
    case 9:
      // gpio 5 & temperature
      // TODO
      m.priority = 2;
      m.action = '$';
      m.cmd = 'I';
      m.key = 5;
      m.val = 0;//gpio_pin5_val;
      m.cmd2 = 'T';
      m.key2 = 1;
      m.val2 = 0;//temp_msg;
    break;
  }

  addNextMsg(m);

  msg_send_index++;
  if(msg_send_index > 9) msg_send_index = 0;

}

// ---- Messages

uint8_t BowieComms::getMsgQueueLength() {
  return msgs_in_queue;
}

Msg BowieComms::popNextMsg() {
  struct Msg m = msg_queue[0];

  for(int i=0; i<msgs_in_queue-1; i++) {
    msg_queue[i] = msg_queue[i+1];
  }

  if(msgs_in_queue > 0) msgs_in_queue--;
  return m;
}

void BowieComms::addNextMsg(uint8_t priority, char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  //Serial.print("adding next message");
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

void BowieComms::addNextMsg(Msg m) {
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

void BowieComms::insertNextMsg(Msg m) {

  Serial.print("inserting next message");

  if(msgs_in_queue == 0) {
    if(OP_DEBUG) Serial << "Adding it to index 0" << endl;
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


