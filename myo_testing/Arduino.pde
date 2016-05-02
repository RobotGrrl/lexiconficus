// -- ARDUINO -- //
Serial arduino;
int port = 99;
boolean connected = false;

// -- PARSING -- //
int msgLen = 20;
int msgIndex = 0;
char[] msg = new char[msgLen];
boolean reading = false;
boolean completed = false;
int receivedValue = 0;


void readData() {
 
  if(connected) {
    // let's read the data
    char c;
    
    while(arduino.available() > 0) {
      
      c = arduino.readChar();
      
      if(c == '~' || c == '@' || c == '#' || c == '^' || c == '&') {
        reading = true;
        completed = false; 
      }
      
      if(reading) {
        if(DEBUG) println("readChar(): " + c + " msgIndex: " + msgIndex);
        
        msg[msgIndex++] = c;
      
        if(c == '!' || c == '?' || c == ';') {
          if(msgIndex >= 6-1) {
            completed = true;
          } else {
            if(DEBUG) println("promulgate error: received delimeter before the message was long enough"); 
          }
        }
        
        if(completed == true) {
          msgLen = msgIndex;
          msgIndex = 0;
          reading = false;
          parse_message();
          return;
        }
        
        if(msgIndex >= 20-1) {
          if(DEBUG) println("promulgate warning: index exceeded max message length");
          msgIndex = 0;
          msgLen = 0;
        }
    
      }
      
    }
    
    
  }
  
}


void parse_message() {
  
  if(DEBUG) {
    println("parsing message now");
    for(int i=0; i<msgLen; i++) {
      print(msg[i]);
    }
    println("\n(end)");
  }
  
  char action = '0';
  char cmd = '0';
  int key_msg = 0;
  int val = 0;
  char delim = '0';
  
  String temp_key = "";
  String temp_val = "";
  int comma_index = 4;
  int delim_index = msgLen-1;
  
  // setting the action and command
  action = msg[0];
  cmd = msg[1];
  
  
  // finding the key
  for(int i=2; i<msgLen; i++) {
    
    if(msg[i] == ',') {
      comma_index = i;
      break;
    } else {
      temp_key = temp_key + msg[i];
    }
    
  }
  
  try {
    key_msg = (int)Integer.parseInt(temp_key);
  } catch(Exception e) {
    println("promulgate error: failed to parse int for key");
    println(e);
    return;
  }
  
  
  // finding the val
  for(int i=comma_index+1; i<msgLen-1; i++) {
    
    char c = msg[i];
    
    if(c == '!' || c == '?' || c == ';') {
      delim_index = i;
      break;
    } else {
      temp_val = temp_val + msg[i];
    }
    
  }
  
  try {
    val = (int)Integer.parseInt(temp_val);
  } catch(Exception e) {
    println("promulgate error: failed to parse int for val");
    println(e);
    return;
  }
  
  // setting the delimeter
  delim = msg[delim_index];
  
  
  // finally done!
  received_action(action, cmd, key_msg, val, delim);
  
}