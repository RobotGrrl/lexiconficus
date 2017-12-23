/*
 * Handy Current Monitor Functions
 * --------------------------------
 * 
 * Using Bowie's brain board v1.0 w/Teensy 3.6 and current sensors
 * 
 * Erin RobotGrrl
 * Dec. 23, 2017
 * 
 */

#include <Streaming.h>

#define CURRENT_SERVO_SENS 15
#define CURRENT_MOTOR_SENS 14

// set this to true if you want to
// constantly monitor and protect for over current
bool MONITOR_OVER_CURRENT = true;

// over current thresholds
int CURRENT_THRESH_MAX = 880;   // analog read value
int CURRENT_THRESH_MIN = 400;   // analog read value

// how many of these triggers accumulate within
// that set time. So if you want the shutdown to occur quickly, 
// this should be adjusted down.
int OVER_CURRENT_DELAY = 2000;

// how many of the triggers will cause the
// shutdown, within the delay time. 
int OVER_CURRENT_TRIG_THRESH = 3;

// the number of times the servo has gone over current before
// entering into 'cool down' mode.
int NUM_OVER_CURRENT_THRESH = 3;

// the amount of time to wait for the servo to 'cool down',
// before resuming normal operation.
int OVER_CURRENT_TIMEOUT = 2500;

// number of over current shutdowns is cleared after 
// this much time
long NUM_OVER_TIMEOUT = 65000;

// pin of our current sensor
int CURRENT_SENSOR_PIN = 15;

// current reading
uint16_t current_val = 0;
int AVG_WINDOW = 100;
float current_val_avg = 0.0;
int current_val_avg_bucket = 0;
int current_avg_count = 0;
uint16_t max_current_reading = 0;
uint16_t min_current_reading = 0;

// current monitoring
bool high_current_detected = false;
long high_current_start = 0;
uint16_t this_current_trigger = 0;
bool THIS_OVER_CURRENT_SHUTDOWN = false;
uint8_t num_over_current_shutdowns = 0;
long this_shutdown_start = 0;
bool first_wait = true;
bool first_over = true;

// time
long current_time = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);
  initCurrentSensors();
}

void loop() {

  if(MONITOR_OVER_CURRENT) {
    monitorCurrent();
  } else {
    updateCurrentSensor();
  }

}


void initCurrentSensors() {
  pinMode(CURRENT_SENSOR_PIN, INPUT);
}

void updateCurrentSensor() {

  current_val = getCurrentSensorReading();
  
  current_val_avg_bucket += current_val;
  if(current_avg_count > AVG_WINDOW-2) {
    current_val_avg = (float)current_val_avg_bucket / (float)(AVG_WINDOW-1);
    Serial << "Monitoring current avg " << current_val_avg << endl;
    current_val_avg_bucket = 0;
    current_avg_count = 0;
  }
  current_avg_count++;
  
}

float getCurrentSensorAvg() {
  return current_val_avg;
}

int getCurrentSensorReading() {

  current_val = analogRead(CURRENT_SENSOR_PIN);

  if(current_val > max_current_reading) {
    max_current_reading = current_val;
    Serial << "Current sensor max: " << max_current_reading << endl;
  }

  if(current_val < min_current_reading) {
    min_current_reading = current_val;
    Serial << "Current sensor min: " << min_current_reading << endl;
  }
  
  return current_val;
}

void monitorCurrent() {

  updateCurrentSensor();

  current_time = millis();

  // notes on calibrating the values:
  // CURRENT_THRESH_MAX should not be exceeded when the arm moves
  // regularly from MAX to MIN. there might be the occasional spike,
  // but this_current_trigger should not exceed 2 or 3.
  // OVER_CURRENT_DELAY is how many of these triggers accumulate within
  // that set time. So if you want the shutdown to occur quickly, this
  // should be adjusted down. 
  // OVER_CURRENT_TRIG_THRESH is how many of the triggers will cause the
  // shutdown, within the delay time. 

  // servo monitoring with the raw vals. we need this to be fast!
  // pre-check mode - done after servos cool off again (or before they heat up)
  if(!THIS_OVER_CURRENT_SHUTDOWN) {

    // checking if the val is over the max or below the min - usually it is below the min
    if(current_val >= CURRENT_THRESH_MAX || current_val <= CURRENT_THRESH_MIN) {

      //Serial << "Monitoring current " << current_val << endl;

      Serial << "!!! OVER CURRENT DETECTED !!! " << current_val << endl;

      // resetting our flags if this was first instance
      if(!high_current_detected) {
        high_current_start = current_time;
        this_current_trigger = 0;
        high_current_detected = true;
      }

      // incrementing trigger count on detection
      if(high_current_detected) {
        this_current_trigger++;
        Serial << "This over current trigger count: " << this_current_trigger << endl;
      }

    }

    if(high_current_detected) {

      // counting how many times the over current threshold has been triggered
      // this seems to work better than going with a timed approach
      if(this_current_trigger >= OVER_CURRENT_TRIG_THRESH) {

        // set the servo shutdown flags
        if(!THIS_OVER_CURRENT_SHUTDOWN) this_shutdown_start = current_time;
        THIS_OVER_CURRENT_SHUTDOWN = true;

        Serial << "!!! This is entering over current shutdown !!! " << this_shutdown_start << endl;
        num_over_current_shutdowns++;

      } else { // if not, then we reset it after time
        if(current_time-high_current_start >= OVER_CURRENT_DELAY) {
          Serial << "reset" << endl;
          high_current_detected = false;
          this_current_trigger = 0;
        }
      }

    }

  }

  // servo shutdown mode
  if(THIS_OVER_CURRENT_SHUTDOWN) {

    // has this been multiple over-current instances in a short amount
    // of time? if so, the robot might be in a bad place. we can try
    // to reverse & wiggle its motors

    // checking number of times over current has happened
    if(num_over_current_shutdowns < NUM_OVER_CURRENT_THRESH) {

      // waiting for servo to cool off
      if(current_time-this_shutdown_start <= OVER_CURRENT_TIMEOUT) {

        Serial << "Waiting to cool down " << current_time-this_shutdown_start << endl;

        // callback
        waitingToCoolDown(first_wait);

        if(first_wait) first_wait = !first_wait;

      } else { // now turn it back on

        Serial << "Cool down complete, ready to reactivate" << endl;

        // callback
        reactivateAfterCoolDown();

        // reset vars
        THIS_OVER_CURRENT_SHUTDOWN = false;
        high_current_detected = false;
        this_current_trigger = 0;
        first_wait = true;

      }

    } else { // this is when the robot might be in a bad place

      Serial << "!!! Num over current > thresh !!!" << endl;

      // detach the servos again
      if(current_time-this_shutdown_start <= OVER_CURRENT_TIMEOUT) { // waiting for servo to cool off
        // callback
        waitingToCoolDown(first_wait);
        if(first_wait) first_wait = !first_wait;
      } else {
        // callback
        reactivateAfterCoolDown();

        // reset vars
        THIS_OVER_CURRENT_SHUTDOWN = false;
        high_current_detected = false;
        this_current_trigger = 0;
        first_wait = true;

        // reset the counter to 0 after this
        num_over_current_shutdowns = 0;
        first_over = true;
      }

      // callback
      overCurrentThreshold(first_over);
      if(first_over) first_over = !first_over;

    }
   
  }

  // the num of shutdowns times out after 1 min
  if(current_time-this_shutdown_start > NUM_OVER_TIMEOUT && num_over_current_shutdowns != 0) {
    Serial << "Number of over current shutdowns is cleared" << endl;
    num_over_current_shutdowns = 0;
  }

}



