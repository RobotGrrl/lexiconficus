#include "BowieComms.h"

#define COMM_LED 2

void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim);

BowieComms bowiecomms = BowieComms();

void commsTimeout();


void setup() {
  Serial.begin(9600);

  bowiecomms.setCommLed(COMM_LED);
  bowiecomms.set_comms_timeout_callback(commsTimeout);

  bowiecomms.promulgate.set_rx_callback(received_action);

  bowiecomms.initComms();
  
}

void loop() {
  bowiecomms.updateComms();

}

void commsTimeout() {
  
}

void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

}

