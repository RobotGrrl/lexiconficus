#include "MegaBowieShoreline.h"

MegaBowieShoreline::MegaBowieShoreline() {
  
}

void MegaBowieShoreline::control(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  
  last_rx_comms = millis();

  Cmd c1 = { '0', 0, 0 };
  Cmd c2 = { '0', 0, 0 };
  c1.cmd = cmd;
  c1.key = key;
  c1.val = val;
  c2.cmd = cmd2;
  c2.key = key2;
  c2.val = val2;
  Cmd packets[2] = { c1, c2 };

  if(COMM_DEBUG) {
    Serial << "*c1 cmd: " << packets[0].cmd << " key: " << packets[0].key << " val: " << packets[0].val << endl;
    Serial << "*c2 cmd: " << packets[1].cmd << " key: " << packets[1].key << " val: " << packets[1].val << endl;
  }

  // we've seen this happen *sometimes*, and it is highly unlikely that this would be an
  // intentional command. let's make sure they mean this at least 2 times before listening
  
  if(val == 255 && val2 == 255 && cmd == 'L' && cmd2 == 'R') {
    unlikely_count++;
    if(unlikely_count <= 2) return;
  } else {
    unlikely_count = 0;
  }


  if(action == '#') {

    for(int i=0; i<2; i++) {

      if(packets[i].cmd == 'P') { // red button
        if(packets[i].val == 1) { // sends drive joystick cmds on operator side
        }
      }

    }
  } // -- end of '#' action specifier

}
