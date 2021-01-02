// GPS functions

void initGPS() {
  Serial.begin(UBLOX_BAUD);                                       //initiate GPS
  gps.init();                                                          //Initiate GPS Data lines

  Serial.println(F("GPS initialized..."));
  delay(50);
  if(gps.setAirborne()){
    Serial.println(F("Airborne mode set..."));
  }
  Serial.println(F("GPS configured"));
}


void updateTelemetry() {

  for(int i=SIZE-2; i>=0; i--) {
    // "push back" array indices by one to make room for new values
    alt[i+1] = alt[i];
    latitude[i+1] = latitude[i];
    longitude[i+1] = longitude[i];
    timeStamp[i+1] = timeStamp[i];
    fixStatus[i+1] = fixStatus[i];
  }

  timeStamp[0] = millis();  // get most recent time stamp
  sats = gps.getSats();     // get most recent number of satellites

  float dt = (timeStamp[0] - timeStamp[9])/1000;

  checkFix();

  if(fixStatus[0] == FIX) {
    // get GPS data if GPS has a fix
    latitude[0] = gps.getLat();
    longitude[0] = gps.getLon();
    alt[0] =  gps.getAlt_feet();
    Serial.print(F("Got fix, fix alt = "));
    Serial.println(alt[0]);

    if(fixStatus[0] == FIX && fixStatus[9] == FIX) {
      // only get these values if the GPS had a fix for inputs
      ascentRate = getAscentRate(alt[0],alt[9],timeStamp[0],timeStamp[9]);
      groundSpeed = getGroundSpeed(latitude[0],latitude[9],longitude[0],longitude[9],timeStamp[0],timeStamp[9]);
      heading = getHeading(latitude[0],latitude[9],longitude[0],longitude[9]);
      detData.Usage = 0x01;
    }
  }
  else {
    // use linear regressions to predict geo-coordinates
    latitude[0] = getNextLat(latitude[1],heading,dt,groundSpeed);
    longitude[0] = getNextLong(longitude[1],latitude[1],heading,dt,groundSpeed);
    alt[0] = getNextAlt(ascentRate,dt,alt[1]);
    detData.Usage = 0x05;
  }
  GPSdata.alt = alt[0];
  GPSdata.latitude = latitude[0];
  GPSdata.longitude = longitude[0];
  GPSdata.AR = ascentRate;
}
  

void checkFix() {
  if(gps.getFixAge() < 4000) {
    fixStatus[0] = FIX;
  }
  else if(gps.getFixAge() > 4000) {
    fixStatus[0] = NOFIX; // CHANGE TO NOFIX
  }
  else{
    fixStatus[0] = NOFIX; // CHANGE TO NOFIX
  }
}

bool boundaryCheck() {
  // function to check if the payload is out of the flight boundaries
  if (longitude[0] > EASTERN_BOUNDARY) {
    cutReasonA = 0x10;
    return true;
  }
  else if (longitude[0] < WESTERN_BOUNDARY) {
    cutReasonA = 0x20;
    return true;
  }
  else if (latitude[0] > NORTHERN_BOUNDARY) {
    cutReasonA = 0x30;
    return true;
  }
  else if (latitude[0] < SOUTHERN_BOUNDARY) {
    cutReasonA = 0x40;
    return true; 
  }
  else {
    return false;
  } 
}

void fixGPSLEDSchema() {
  // for LED1:
  // fix LED timing schema
  // if fix on both, solid light
  // if fix only this one, blink one sec
  // if fix on only main, blink three sec
  // if no fix, blink five sec

  if(fixStatus[0] == FIX && inputPacket.fixStatus && !LEDOn){
    digitalWrite(LED,HIGH);
    LEDOn = true;
  }
  else if (fixStatus[0] == FIX && !inputPacket.fixStatus){
    if (!LEDOn && millis()-LEDOffStamp > 1000){
      digitalWrite(LED,HIGH);
      LEDOn = true;
      LEDOnStamp = millis();
    }
    else if (LEDOn && millis()-LEDOnStamp > 1000){
      digitalWrite(LED,LOW);
      LEDOn = false;
      LEDOffStamp = millis();
    }
  }
  else if (fixStatus[0] == NOFIX && inputPacket.fixStatus){
    if (!LEDOn && millis()-LEDOffStamp > 3000){
      digitalWrite(LED,HIGH);
      LEDOn = true;
      LEDOnStamp = millis();
    }
    else if (LEDOn && millis()-LEDOnStamp > 3000){
      digitalWrite(LED,LOW);
      LEDOn = false;
      LEDOffStamp = millis();
    }
  }
  else if (fixStatus[0] == NOFIX && !inputPacket.fixStatus){
    if (!LEDOn && millis()-LEDOffStamp > 5000){
      digitalWrite(LED,HIGH);
      LEDOn = true;
      LEDOnStamp = millis();
    }
    else if (LEDOn && millis()-LEDOnStamp > 5000){
      digitalWrite(LED,LOW);
      LEDOn = false;
      LEDOffStamp = millis();
    }
  }  
}


float getAscentRate(float alt1, float alt2, long time1, long time2) {
  float velocity = 0;

  time1 /= 1000;    // divide milliseconds into seconds
  time2 /= 1000;
    
  if(alt1 !=0 && alt2 != 0) {
    velocity = ((alt1-alt2)/(time1-time2)) * 60;
  }

  return velocity;      // returns a velocity in feet/minute
}


float getNextAlt(float ascentRate, float dt, float currentAlt) {
  float alt = 0;
  if(currentAlt != 0) {
    alt = ascentRate*dt + currentAlt;
  }

  return alt;
}


float getGroundSpeed(float lat1, float lat2, float long1, float long2, float time1, float time2) {
  float groundSpeed = 0;

  float miles_per_lat = 69; // roughly 69 miles per degree latitude // nice
  float miles_per_long = cos(lat1*D2R)*69.172; // miles per degree longitude depending on the current latitude

  time1 /= 1000;  // divide milliseconds into seconds
  time2 /= 1000;
  
  if(lat1 != 0 && lat2 != 0 && long1 != 0 && long2 != 0) {
    float lat_per_hour = (lat1 - lat2)/(time1 - time2) * SECONDS_PER_HOUR;  // degrees latitude per hour
    float long_per_hour = (long1 - long2)/(time1- time2) *  SECONDS_PER_HOUR; // degrees longitutde per hour

    groundSpeed = sqrt(pow(lat_per_hour*miles_per_lat,2) + pow(long_per_hour*miles_per_long,2));  // find magnitude of the velocity vector
  }

  return groundSpeed*FPM_PER_MPH;   // return in feet per minute
}


float getNextLat(float lat1, float heading, float dt, float groundSpeed) {
  float nextLat = 0;

  groundSpeed /= (60*5280);  // convert feet per minute to miles per second
  float dx = groundSpeed * dt;  // gives predicted displacement in miles

  float miles_per_lat = 69; // roughly consistent for the entire Earth
  
  if(lat1 != 0) {
    nextLat = lat1 + (dx/miles_per_lat)*sin(heading); // simple vector math to get next latitude
  }

  return nextLat;
}


float getNextLong(float long1, float lat1, float heading, float dt, float groundSpeed) {
  float nextLong = 0;
  
  groundSpeed /= (60*5280);  // convert feet per minute to miles per second
  float dx = groundSpeed * dt;  // gives predicted displacement in miles

  float miles_per_long = cos(lat1*D2R)*69.172;  // varies depending on latitude
  
  if(long1 != 0) {
    nextLong = long1 + (dx/miles_per_long)*cos(heading);
  }

  return nextLong;
}

float getHeading(float lat1, float lat2, float long1, float long2) {
  // position 1 is where you are, position 2 is where you were
  
  float dLong = long1 - long2;  // delta longitude
  float headingY = cos(lat1*D2R)*sin((dLong)*D2R);
  float headingX = cos(lat2*D2R)*sin(lat1*D2R) - sin(lat2*D2R)*cos(lat1*D2R)*cos((dLong)*D2R);

  float heading = atan2(headingX,headingY);

  return heading;
}
