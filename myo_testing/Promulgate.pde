
// this is where we parse the received info
// from the Arduino!
void received_action(char action, char cmd, int key_msg, int val, char delim) {
  
  if(DEBUG) {
    println("---RECEIVED ACTION!---");
    println("action: " + action);
    println("cmd: " + cmd);
    println("key: " + key_msg);
    println("val: " + val);
    println("delim: " + delim);
  }
  
}


void transmit_complete() {
  if(DEBUG) println("message sent");
}


void transmit_action(char action, char cmd, int key_msg, int val, char delim) {
  
  if(key_msg > 256) {
    println("promulgate error: key has to be <= 256");
    return;
  }
  
  if(val > 1023) {
    println("promulgate error: val has to be <= 1023");
    return;
  }
  
  String s = (action + "" + cmd + "" + key_msg + "," + val + "" + delim);
  println(s);
  if(connected) {
    arduino.write(s);
    transmit_complete();
  }
  
}