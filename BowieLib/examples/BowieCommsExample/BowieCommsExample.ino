#include "BowieComms.h"

#define COMM_LED 2

BowieComms bowiecomms = BowieComms();

void commsTimeout();


void setup() {
  Serial.begin(9600);

  bowiecomms.setCommLed(COMM_LED);
  bowiecomms.set_comms_timeout_callback(commsTimeout);

  bowiecomms.initComms();
  
}

void loop() {
  bowiecomms.updateComms();

  if(Serial.available() > 0) {
    char c = Serial.read();
    if(c == 'A') {
      // Uncomment the below if you want to do a 'bruteforce' send
      // - which would bypass the message queue, though could have
      // unintended consequences (eg, if two messages are sent in
      // rapid succession, none may be delivered).
      // bowiecomms.xbeeSend('#', 'Q', 1, 128, '#', 'Q', 2, 128);
    } else if(c == 'B') {
      //bowiecomms.xbeeSend();
    }
  }

}

void commsTimeout() {
  
}

/*
void received_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {

}
*/

