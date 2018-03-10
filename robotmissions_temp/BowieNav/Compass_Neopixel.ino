void compassNeopixelsInit(uint8_t r, uint8_t g, uint8_t b) {
  pixels.begin();
  simpleTest(80, r, g, b);
}

void simpleTest(uint8_t wait, uint8_t r, uint8_t g, uint8_t b) {
  for(int i=0; i<pixels.numPixels(); i++){
    pixels.setPixelColor(i, pixels.Color(r,g,b));
    pixels.show();
    delay(wait);
  }
  for(int i=pixels.numPixels(); i>=0; i--) {
    pixels.setPixelColor(i, pixels.Color(r,g,b));
    pixels.show();
    delay(wait);
  }
}

void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  //for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixels.numPixels(); i++) {
      pixels.setPixelColor(i, Wheel(((i * 256 / pixels.numPixels())) & 255));
      pixels.show();
    delay(wait);
    }
  //}
}

void compassIndicatorLeds() {

  for(int i=NUMPIXELS; i>=0; i--) {
    pixels.setPixelColor(i, pixels.Color(0,0,0));
  }

  // the neopixel directly infront of the robot on the
  // compass viewer
  robot_direction_indicator = 2;

  // divide compass heading by 15 to map to the 
  // number of leds (because 360/15 == 24).
  // then subtract 24, to get the led number to
  // be lit. 24-(360/15) == 0.
  north_indicator = 24-floor(compass_heading/15);
  north_indicator += robot_direction_indicator-1;
  if(north_indicator < 0) north_indicator = 0;
  if(north_indicator >= 24) north_indicator = 0;

  // default tracking heading is 45
  heading_indicator = 0;
  heading_indicator = floor(tracking_heading/15);
  heading_indicator += north_indicator;
  heading_indicator += robot_direction_indicator-1;
  if(heading_indicator < 0) heading_indicator = 0;
  if(heading_indicator > 24) heading_indicator -= 24;
  
  // north = blue
  pixel_states[north_indicator] = 1;
  pixel_colours[north_indicator] = pixels.Color(0,0,LED_BRIGHTNESS);
  
  // heading = green
  pixel_states[heading_indicator] = 1;
  pixel_colours[heading_indicator] = pixels.Color(0,LED_BRIGHTNESS,0);
  
  // robot = red
  pixel_states[robot_direction_indicator] = 1;
  pixel_colours[robot_direction_indicator] = pixels.Color(LED_BRIGHTNESS,0,0);
  
  // pixels will be updated in the main loop
  
}

void compassDriveLeds() {
  
  int new_colour = LED_BRIGHTNESS-60;
  if(new_colour < 15) new_colour = 15;

  uint32_t compass_pixel_colour = pixels.Color(new_colour,new_colour,new_colour);
  
  switch(DRIVE_STATE) {
    case IDLE:
    break;
    case FORWARD:
      // the span of leds we want to update
      for(int i=0; i<=4; i++) {
        // check to make sure these aren't being lit as an indicator
        if(pixel_states[i] != 1) {
          pixel_states[i] = 2;
          pixel_colours[i] = compass_pixel_colour;
        }
      }
    break;
    case RIGHT:
      for(int i=6; i<=10; i++) {
        if(pixel_states[i] != 1) {
          pixel_states[i] = 2;
          pixel_colours[i] = compass_pixel_colour;
        }
      }
    break;
    case HARD_RIGHT:
      for(int i=4; i<=12; i++) {
        if(pixel_states[i] != 1) {
          pixel_states[i] = 2;
          pixel_colours[i] = compass_pixel_colour;
        }
      }
    break;
    case LEFT:
      for(int i=18; i<=22; i++) {
        if(pixel_states[i] != 1) {
          pixel_states[i] = 2;
          pixel_colours[i] = compass_pixel_colour;
        }
      }
    break;
    case HARD_LEFT:
      for(int i=16; i<=23; i++) {
        if(pixel_states[i] != 1) {
          pixel_states[i] = 2;
          pixel_colours[i] = compass_pixel_colour;
        }
      }
      if(pixel_states[0] != 1) {
        pixel_states[0] = 2;
        pixel_colours[0] = compass_pixel_colour;
      }
    break;
    case REVERSE:
    break;
  }

  // now update the pixels
  for(int i=0; i<NUMPIXELS; i++) {
    if(pixel_states[i] > 0) {
      pixels.setPixelColor(i, pixel_colours[i]);
    }
  }
  pixels.show();
  emptyPixels();
  
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

