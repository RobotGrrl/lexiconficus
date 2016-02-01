void initMotion() {

  x_home = analogRead(X_PIN);
  y_home = analogRead(Y_PIN);
  z_home = analogRead(Z_PIN);  
  
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  
  gyro_home = analogRead(GYRO_PIN);
  gyro_zero_voltage = ( gyro_home * max_voltage ) / 1023;
  
}

void updateGyro() {

  gyro.read();
  
  Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print(" ");
  
}

void checkGyroBounds() {

  if(gyro.data.z < -100) z_val = -100;
  if(gyro.data.z > 100) z_val = 100;
  
  if(gyro.data.x < -100) x_val = -100;
  if(gyro.data.x > 100) x_val = 100;
  
}

void detectMotionZ() {

  if(gyro.data.z > 40) {
    //if(movement_num == 0 || movement_num == 1 || movement_num == 3 || movement_num == 4) {
    //  movement_num = 1;
      Serial << "forwards!" << endl;

      state = FRONT;
      refreshStates(true);
      
      //xbee_promulgate.transmit_action('#', 'F', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME-90);
      servo_R.write(SERVO_R_HOME+90);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 100, 255, 10);
      }
      strip.show();
      
      last_gyro_trig = current_time;
      
    //}
  } else if(gyro.data.z < -40) {
    //if(movement_num == 0 || movement_num == 2 || movement_num == 3 || movement_num == 4) {
      Serial << "backwards!" << endl;

      state = BACK;
      refreshStates(true);
      
      //xbee_promulgate.transmit_action('#', 'B', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME+60);
      servo_R.write(SERVO_R_HOME-60);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 10, 50, 200);
      }
      strip.show();
      
      last_gyro_trig = current_time;
      
    //  movement_num = 2;
    //}
  }
  
}

void detectMotionX() {

  if(gyro.data.x > 40) {
    //if(movement_num == 0 || movement_num == 1 || movement_num == 2 || movement_num == 3) {
      Serial << "left!" << endl;

      state = LEFT;
      refreshStates(true);
      
      //xbee_promulgate.transmit_action('#', 'L', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME+60);
      servo_R.write(SERVO_R_HOME+60);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 200, 200, 200);
      }
      strip.setPixelColor(1, 10, 255, 100);
      strip.setPixelColor(2, 10, 255, 100);
      strip.setPixelColor(3, 10, 255, 100);
      strip.show();
      
      last_gyro_trig = current_time;
      
    //  movement_num = 3;
    //}
  } else if(gyro.data.x < -40) {
    //if(movement_num == 0 || movement_num == 1 || movement_num == 2 || movement_num == 4) {
      Serial << "right!" << endl;

      state = RIGHT;
      refreshStates(true);
      
      //xbee_promulgate.transmit_action('#', 'R', MOTOR_SPEED, 500, '!');
      
      servo_L.attach(SERVO_L_PIN);
      servo_R.attach(SERVO_R_PIN);
      
      servo_L.write(SERVO_L_HOME-60);
      servo_R.write(SERVO_R_HOME-60);
      delay(250);
      
      servo_L.detach();
      servo_R.detach();
      
      for(int i=0; i<N_LEDS; i++) {
      strip.setPixelColor(i, 200, 200, 200);
      }
      strip.setPixelColor(3, 10, 255, 100);
      strip.setPixelColor(4, 10, 255, 100);
      strip.setPixelColor(5, 10, 255, 100);
      strip.show();
      
      last_gyro_trig = current_time;
      
    //  movement_num = 4;
    //}
  }
  
}

void wingWiggleBehaviour() {

  if(current_time-last_gyro_trig > 3000) {
    
    state = NONE;
    refreshStates(true);
    
    
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
    
    
    
    if(QS == true) {                       // Quantified Self flag is true when arduino finds a heartbeat
      
      Serial << "BPM: " << BPM << " IBI: " << IBI << endl;
    
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
    
  
}

void heartbeatBehaviour() {
  // meep
}



