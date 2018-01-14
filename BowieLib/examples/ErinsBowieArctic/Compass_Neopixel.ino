void compassNeopixelsInit() {
  pixels.begin();
  simpleTest(80);
}

void simpleTest(uint8_t wait) {
  for(int i=0; i<pixels.numPixels(); i++){
    pixels.setPixelColor(i, pixels.Color(0,15,0));
    pixels.show();
    delay(wait);
  }
  for(int i=pixels.numPixels(); i>=0; i--) {
    pixels.setPixelColor(i, pixels.Color(0,0,0));
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

void compassLeds() {

  for(int i=NUMPIXELS; i>=0; i--) {
    pixels.setPixelColor(i, pixels.Color(0,0,0));
  }

  int compass_indicator = 0;
  compass_indicator = floor(compass_heading/15);

  int north_indicator = 24-floor(compass_heading/15);
  if(north_indicator < 0) north_indicator = 0;

  int heading_indicator = 0;
  heading_indicator = floor(tracking_heading/15);
  //heading_indicator = (north_indicator+2)+floor(tracking_heading/15);
  //heading_indicator = floor( distanceAngles((int)floor(tracking_heading), (int)floor(compass_heading) )/15)-compass_indicator;
  //if(heading_indicator >= 24) heading_indicator -= 24;

  int next = 0;

  Serial << "HEADING = " << compass_heading << endl;

  // north = blue
  next = north_indicator+0;
  if(next > 24) next-=24;
  pixels.setPixelColor(next, pixels.Color(0,0,15));
  pixels.setPixelColor(0, pixels.Color(0,15,0));
  /*
  next = north_indicator+6;
  if(next > 24) next-=24;
  pixels.setPixelColor(next, pixels.Color(0,15,0));
  next = north_indicator+12;
  if(next > 24) next-=24;
  pixels.setPixelColor(next, pixels.Color(0,15,0));
  next = north_indicator+18;
  if(next > 24) next-=24;
  pixels.setPixelColor(next, pixels.Color(0,15,0));
  */
  
  pixels.setPixelColor(heading_indicator, pixels.Color(15,15,0));
  pixels.setPixelColor(compass_indicator, pixels.Color(15,0,0));
    
  pixels.show();
  
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

