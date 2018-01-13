#include "Arduino.h"
#include <Streaming.h>

#ifndef _BOWIECURRENTSENSOR_H_
#define _BOWIECURRENTSENSOR_H_

#define CUR_DEBUG false

class BowieCurrentSensor {

  public:
    BowieCurrentSensor();
    void begin();

    bool MONITOR_OVER_CURRENT;
    int CURRENT_THRESH_MAX;
    int CURRENT_THRESH_MIN;
    int OVER_CURRENT_DELAY;
    int OVER_CURRENT_TRIG_THRESH;
    int NUM_OVER_CURRENT_THRESH;
    int OVER_CURRENT_TIMEOUT;
    long NUM_OVER_TIMEOUT;

    int CURRENT_SENSOR_PIN;
    String CURRENT_SENSOR_NAME;

    void initCurrentSensor();
    void refreshCurrentSensor();
    void updateCurrentSensor();

    void setCurrentSensePin(int pin);
    void setCurrentSenseName(String name);
    void setCurrentThreshMax(int val);
    void setCurrentThreshMin(int val);
    float getCurrentSensorAvg();
    int getCurrentSensorReading();

    void set_waitingToCoolDown_callback( void (*waitingToCoolDown)(bool first) );
    void set_reactivateAfterCoolDown_callback( void (*reactivateAfterCoolDown)() );
    void set_overCurrentThreshold_callback( void (*overCurrentThreshold)(bool first) );

  private:

    void monitorCurrent();

    // current reading
    uint16_t current_val;
    int AVG_WINDOW = 100;
    float current_val_avg;
    int current_val_avg_bucket;
    int current_avg_count;
    uint16_t max_current_reading;
    uint16_t min_current_reading;

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

    // callbacks
    void (*_waitingToCoolDown)(bool first);
    void (*_reactivateAfterCoolDown)();
    void (*_overCurrentThreshold)(bool first);

};

#endif