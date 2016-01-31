void initWings() {
  servo_L.attach(SERVO_L_PIN, SERVO_L_MIN, SERVO_L_MAX);
  servo_R.attach(SERVO_R_PIN, SERVO_R_MIN, SERVO_R_MAX);
  
  servo_L.write(SERVO_L_HOME);
  servo_R.write(SERVO_R_HOME); 
}


void heartbeatBehaviour() {
 
  if(QS == true) {                       // Quantified Self flag is true when arduino finds a heartbeat
    
    if(DEBUG) Serial << "BPM: " << BPM << " IBI: " << IBI << endl;
  
    //xbee_promulgate.transmit_action('#', 'F', 200, 100, '!');
    
    if(current_time-last_servo_update >= 1000) {
    strip.clear();
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 10, 100, 10);
    }
    strip.show();
    delay(20);
    }
       
    QS = false;                      // reset the Quantified Self flag for next time    
  
  } else {
    
   if(current_time-last_led_update >= 500) {
     strip.clear();
     for(int i=0; i<N_LEDS; i++) {
       strip.setPixelColor(i, (int)(random(0,255)/10), (int)(random(0,255)/10), (int)(random(0,255)/10));
     }
     strip.show();
     delay(20);
     last_led_update = current_time;
   }
    
  }
  
}


void wingWiggleBehaviour() {
 
  // waiting for the 7 seconds after the last servo update,
  // as long as it's 3 seconds after the last gyro trigger
  
  if(current_time-last_gyro_trig > 3000) {
    
    if(current_time-last_servo_update >= 7000) {
     
    servo_L.attach(SERVO_L_PIN);
    servo_R.attach(SERVO_R_PIN);
    
    int r = (int)random(0, 2);
    
    if(r == 0) {
    
      for(int i=0; i<30; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);
        delay(10);
      }
      
      for(int i=30; i>-30; i--) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      for(int i=-30; i<0; i++) {
        servo_L.write(SERVO_L_HOME+i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
    
    } else if(r == 1) {
      
      for(int i=0; i<30; i++) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);
        delay(10);
      }
      
      for(int i=30; i>-30; i--) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
      for(int i=-30; i<0; i++) {
        servo_L.write(SERVO_L_HOME-i);
        servo_R.write(SERVO_R_HOME+i);      
        delay(10);
      }
      
    }
    
    servo_L.detach();
    servo_R.detach();
      
    last_servo_update = current_time;  
    
    }
    
  }
  
}
