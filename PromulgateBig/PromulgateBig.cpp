/*******************
Promulgate
-----------

This library lets other devices trigger actions using a very simple API.
You can also send commands to other devices to trigger actions.

Erin K / RobotGrrl - May 21, 2014
--> http://RobotGrrl.com/blog
--> http://RoboBrrd.com

MIT license, check LICENSE for more information
All text above must be included in any redistribution
*******************/

#include "PromulgateBig.h"
#include "base64.hpp"

Promulgate::Promulgate() {
  
}


Promulgate::Promulgate(Stream *in, Stream *out) {
  in_stream = in;
  out_stream = out;
  begin();
}

void Promulgate::begin() {

  LOG_LEVEL = WARN;

  use_base64_parsing = false;
  set_debug_stream(out_stream);
  reading_message = false;
  ser_len = 0;

  reset_buffer();

}

void Promulgate::set_debug_stream(Stream *d) {
  debug_stream = d;
}

void Promulgate::set_rx_callback( void (*rxCallback)(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) ) {
  _rxCallback = rxCallback;
}

void Promulgate::set_tx_callback( void (*txCallback)() ) {
  _txCallback = txCallback;
}

void Promulgate::useBase64Parsing(bool b) {
  use_base64_parsing = b;
}



// Message parsing

void Promulgate::parse_message(char msg[], uint8_t len) {
  
  //if(LOG_LEVEL >= DEBUG) *debug_stream << "parsing message" << endl;
  
  if(len < 3) return; // we are not looking for short messages...
  
  // get the action specifier
  char action = msg[0];
  
  if(action != '0') {
  
    // get the command
    char cmd = msg[1];
    
    // find the , to see if there is a value for the key
    uint8_t comma = 0;
    for(uint8_t i=2; i<len-2; i++) {
      if(msg[i] == ',') {
        comma = i;
        break;
      }
    }
    
    // there is no val
    boolean find_val = true;
    if(comma == 0) {
      comma = len-1;
      find_val = false;
    }
    
    // get the key number
    uint8_t key = 0;
    uint8_t lb = 2; // index of leading digit
    uint8_t ub = comma-1; // index of last digit
    
    for(uint8_t i=lb; i<=ub; i++) {
      key += ( msg[lb + (i-lb)] - '0') * pow(10, ub-i);
    }

    // find the next ,
    uint8_t comma2 = comma;
    for(uint8_t i=comma+1; i<len-2; i++) {
      if(msg[i] == ',') {
        comma2 = i;
        break;
      }
    }

    // get the val number
    uint16_t val = 0;
    lb = comma+1;
    ub = comma2-1;
 
    if(find_val) {
  
      for(uint8_t i=lb; i<=ub; i++) {
        val += ( msg[lb + (i-lb)] - '0' ) * pow(10, ub-i);
      }
      
    }
    
    // get the 2nd command
    char cmd2 = msg[comma2+1];

    // find the 3rd ,
    uint8_t comma3 = comma2;
    for(uint8_t i=comma2+1; i<len-2; i++) {
      if(msg[i] == ',') {
        comma3 = i;
        break;
      }
    }

    // get the 2nd key number
    uint8_t key2 = 0;
    lb = comma2+2; // index of leading digit
    ub = comma3-1; // index of last digit
    
    for(uint8_t i=lb; i<=ub; i++) {
      key2 += ( msg[lb + (i-lb)] - '0') * pow(10, ub-i);
    }

    // get the 2nd val number
    uint16_t val2 = 0;
    lb = comma3+1;
    ub = len-2;
 
    for(uint8_t i=lb; i<=ub; i++) {
      val2 += ( msg[lb + (i-lb)] - '0' ) * pow(10, ub-i);
    }
    
    // get the delimeter
    char delim = msg[len-1];
    
    // print it for debugging
    
    if(LOG_LEVEL >= DEBUG) {
      *debug_stream << "---RECEIVED---" << endl;
      *debug_stream << "Action specifier: " << action << endl;
      *debug_stream << "Command: " << cmd << endl;
      *debug_stream << "Key: " << key << endl;
      *debug_stream << "Value: " << val << endl;
      *debug_stream << "Command2: " << cmd2 << endl;
      *debug_stream << "Key2: " << key2 << endl;
      *debug_stream << "Value2: " << val2 << endl;
      *debug_stream << "Delim: " << delim << endl;
    }
    
    _rxCallback(action, cmd, key, val, cmd2, key2, val2, delim);
    
  }

}


void Promulgate::organize_message(char c) {

  if(!reading_message) {
    
    // check for the action specifier
    if(c == '~' || c == '@' || c == '#' || c == '^' || c == '&' || c == '$') {
      reading_message = true;
      ser[ser_len] = c;
      ser_len++;
    }
    
  } else {
   
    ser[ser_len] = c;
    ser_len++;
    
    // check for the delimeter
    if(c == '!' || c == '?' || c == ';') {
      reading_message = false;
      if(use_base64_parsing) {
        //Serial << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PARSING 64 - I DUNNO WHY" << endl;
        parse_message64(ser, ser_len);
      } else {
        //Serial << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! NOT PARSING 64 - ITS OK" << endl;
        parse_message(ser, ser_len);
      }
      reset_buffer();
    }
    
  }

  if(ser_len >= 29) {
    // something bad has happened if we are here...
    ser_len = 0;
    if(LOG_LEVEL >= WARN) {
      *debug_stream << "Message > 30 chars, resetting and not parsing" << endl;
    }
    reading_message = false;
    reset_buffer();
  }
  
}

// This is a revised parser by David Kavanagh to handle receiving 
// base64 encoded messages (for example, from a computer).
// Also uses fixed length for the parsing.
void Promulgate::parse_message64(char msg[], uint8_t len) {
 
 //if(LOG_LEVEL >= DEBUG) *debug_stream << "parsing message" << endl;
 
 if(len < 3) return; // we are not looking for short messages...

 char binary[64];
 char tmp[len];
 // extract encoded packet from between $ and !
 for (int i=1; i<(len-1); i++) {
   tmp[i-1] = msg[i];
 }
 unsigned int b64_len = decode_base64(tmp, binary);
 if (b64_len > 0) {
   Serial << "b64 length = " << b64_len << " data " << binary << endl;
   msg = binary;
   len = b64_len;
 }
 // get the action specifier
 char action = msg[0];
 
 if(action != '0') {

   *debug_stream << "msg : ";
   char buf [2];
   for (int i=0; i<len; ++i) {
     sprintf(buf, "%02x", (char)(msg[i]));
     *debug_stream << buf << ':';
   }
   *debug_stream << "\n";
   
   // get the command
   char cmd = msg[1];
   
   // find the , to see if there is a value for the key
   uint8_t comma = 0;
   for(uint8_t i=2; i<len-2; i++) {
     if(msg[i] == ',') {
       comma = i;
       break;
     }
   }
   
   // there is no val
   boolean find_val = true;
   if(comma == 0) {
     comma = len-1;
     find_val = false;
   }
   
   // get the key number
   uint8_t key = msg[2];
   
   Serial << "key1 = " << key << endl;

   // find the next ,
   uint8_t comma2 = comma;
   for(uint8_t i=comma+1; i<len-2; i++) {
     if(msg[i] == ',') {
       comma2 = i;
       break;
     }
   }

   // get the val number
   uint16_t val = msg[comma+1] + (msg[comma+2] * 256);
   
   Serial << "val1 = " << val << endl;
   
   // get the 2nd command
   char cmd2 = msg[comma2+1];

   // find the 3rd ,
   uint8_t comma3 = comma2;
   for(uint8_t i=comma2+1; i<len-2; i++) {
     if(msg[i] == ',') {
       comma3 = i;
       break;
     }
   }

   // get the 2nd key number
   uint8_t key2 = msg[comma2+2];
   
   Serial << "key2 = " << key2 << endl;

   // get the 2nd val number
   uint16_t val2 = msg[comma3+1] + (msg[comma3+2] * 256);
   
   Serial << "val2 = " << val2 << endl;
   
   // get the delimeter
   char delim = msg[len-1];
   
   // print it for debugging
   
   if(LOG_LEVEL >= DEBUG) {
     *debug_stream << "---RECEIVED---" << endl;
     *debug_stream << "Action specifier: " << action << endl;
     *debug_stream << "Command: " << cmd << endl;
     *debug_stream << "Key: " << key << endl;
     *debug_stream << "Value: " << val << endl;
     *debug_stream << "Command2: " << cmd2 << endl;
     *debug_stream << "Key2: " << key2 << endl;
     *debug_stream << "Value2: " << val2 << endl;
     *debug_stream << "Delim: " << delim << endl;
   }
   
   _rxCallback(action, cmd, key, val, cmd2, key2, val2, delim);
   
 }

}



// Send Reply

void Promulgate::transmit_action(char action, char cmd, uint8_t key, uint16_t val, char cmd2, uint8_t key2, uint16_t val2, char delim) {
  
  *out_stream << action << cmd << key << "," << val << "," << cmd2 << key2 << "," << val << delim;

  if(LOG_LEVEL >= DEBUG) {
    *debug_stream << "---TRANSMITTED---" << endl;
    *debug_stream << "Action specifier: " << action << endl;
    *debug_stream << "Command: " << cmd << endl;
    *debug_stream << "Key: " << key << endl;
    *debug_stream << "Value: " << val << endl;
    *debug_stream << "Command2: " << cmd2 << endl;
    *debug_stream << "Key2: " << key2 << endl;
    *debug_stream << "Value2: " << val2 << endl;
    *debug_stream << "Delim: " << delim << endl;
  }

  _txCallback();

}



// Init / Reset

void Promulgate::reset_buffer() {
  
 for(uint8_t i=0; i<30; i++) {
    ser[i] = 0;
  }
  
  ser_len = 0; 
}

