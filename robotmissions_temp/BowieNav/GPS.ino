void gpsInit() {
  Serial3.begin(GPSBaud);
  pinMode(GPS_FIX_PIN, INPUT);

  for(int i=0; i<MAX_POINTS; i++) {
    POINTS_LAT[i] = 0.0;
    POINTS_LON[i] = 0.0;
    POINTS_ACTION[i] = 0;
  }
  
  //loadBaseballCourse();
  //loadTestPoint();
}

void loadTestPoint() {
  POINTS_LAT[0] = 44.291548;
  POINTS_LON[0] = -76.309864;
  POINTS_ACTION[0] = 1;
}

void loadBaseballCourse() {
  POINTS_LAT[0] = 44.214520;
  POINTS_LON[0] = -76.525332;
  POINTS_ACTION[0] = 1;
  
  POINTS_LAT[1] = 44.214625;
  POINTS_LON[1] = -76.525256;
  POINTS_ACTION[1] = 2;
  
  POINTS_LAT[2] = 44.214696;
  POINTS_LON[2] = -76.525384;
  POINTS_ACTION[2] = 2;
  
  POINTS_LAT[3] = 44.214577;
  POINTS_LON[3] = -76.525473;
  POINTS_ACTION[3] = 2;

  POINTS_LAT[4] = 44.214520;
  POINTS_LON[4] = -76.525332;
  POINTS_ACTION[4] = 3;
}

void gpsRobotNavigation() {

  double distanceMetresToPoint =
    TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      POINTS_LAT[current_point], 
      POINTS_LON[current_point]);
  
  double courseToPoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      POINTS_LAT[current_point], 
      POINTS_LON[current_point]);

  Serial << "\n\n" << distanceMetresToPoint << "m away to point #" << current_point << " at: " << courseToPoint << " deg" << endl;

  if(distanceMetresToPoint <= DISTANCE_THRESH) {
    simpleTest(20, 255, 255, 255);
    Serial << "WE ARE HERE!" << endl;
    Serial << "We have hit waypoint #" << current_point << "!" << endl;
    delay(500);
    // not now current_point++; // on to the next point
    if(current_point >= MAX_POINTS) {
      Serial << "Circuit completed!" << endl;
      current_point = MAX_POINTS;
    }
  } else {

    // TODO: Drive & turn the robot according to compass heading

    tracking_heading = courseToPoint;
    // TODO - update this with the new code
    //compassTrackHeading();
    
  }

  for(int i=0; i<MAX_POINTS; i++) {
    unsigned long dist =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      POINTS_LAT[i], 
      POINTS_LON[i]);
    if(dist <= DISTANCE_THRESH) {
      gpsWaypointAction();
    }
  }
  
}

void gpsWaypointAction() {

  switch(POINTS_ACTION[current_point]) {
    case 0:
    // nothing
    break;
    case 1: // start
      Serial << "BOOP!" << endl;
      digitalWrite(COMM_LED, HIGH);
      delay(100);
      digitalWrite(COMM_LED, LOW);
      delay(100);
    break;
    case 2: // mid
      Serial << "BLONK!" << endl;
      digitalWrite(COMM_LED, HIGH);
      tone(SPEAKER, 300, 100);
      delay(100);
      digitalWrite(COMM_LED, LOW);
      delay(100);
    break;
    case 3: // end
      Serial << "BLONK!" << endl;
      digitalWrite(COMM_LED, HIGH);
      tone(SPEAKER, 200, 80);
      delay(80);
      digitalWrite(COMM_LED, LOW);
      delay(80); 
    break;
  }
  
}

void gpsUpdate() {
  readGPS(100);

  if(gps.satellites.isValid()) {
    gpsPrintData();
    // TODO: Uncomment for nav
    /*
    if(following_course) {
      gpsRobotNavigation();
    }
    */
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }
}

void gpsPrintData() {

  if(printCount > 20) {
    Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
    Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
    Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));
    printCount = 0;
  }
  
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  printCount++;
  
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void readGPS(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  readGPS(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  readGPS(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid()) {
    Serial.print(F("******** "));
  } else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  readGPS(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  readGPS(0);
}

