/////////////////////////////////////////////////////////////////////////////////////////////////
///////////******************* BEGINNING OF CDU FUNCTIONS *************************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

 bool CompareGPS() {

  for(int i=SIZE-2; i>=0; i--) {
    // "push back" array indices by one to make room for new values
    Altitude[i+1] = Altitude[i];
    latitude[i+1] = latitude[i];
    longitude[i+1] = longitude[i];
    timeStamp[i+1] = timeStamp[i];
    fixStatus[i+1] = fixStatus[i];
  }
  timeStamp[0] = millis();
  if (stat > 7) { fixStatus[0] = FIX; }

  memcpy(&latA,&recoveredDataA[5],4); 
  memcpy(&latB,&recoveredDataB[5],4);
  Serial.print("CDU1 LAT: ");
  Serial.println(latA); 
  Serial.print("CDU2 LAT: ");
  Serial.println(latB);
  Serial.print("ERAU LAT: ");
  Serial.println(latC);
  //latC assigned when GPS fix recieved in main loop
  memcpy(&lonA,&recoveredDataA[9],4); 
  memcpy(&lonB,&recoveredDataB[9],4); 
  //lonC assigned when GPS fix recieved in main loop
  memcpy(&altA,&recoveredDataA[13],4);
  memcpy(&altB,&recoveredDataB[13],4);
  //altC assigned when GPS fix recieved in main loop
    
  AWorking = workingGPS(latAPrev, latA, lonAPrev, lonA, altAPrev, altA);
  BWorking = workingGPS(latBPrev, latB, lonBPrev, lonB, altBPrev, altB);
  CWorking = workingGPS(latCPrev, latC, lonCPrev, lonC, altCPrev, altC);

  heading = getHeading(latitude[1],latitude[9],longitude[1],longitude[9]);
  groundSpeed = getGroundSpeed(latitude[1],latitude[9],longitude[1],longitude[9],timeStamp[1],timeStamp[9]);
  ascentRate = getAscentRate(Altitude[1],Altitude[9],timeStamp[1],timeStamp[9]);
  
//  Serial.print("OG AR: ");
//  Serial.println(ascentRate);
  if (abs(ascentRate-ascentRatePrev)>100){ // throws out bad AR, which happens occasionally
    FARcounter++;
    if (FARcounter == 1) ascentRate = ascentRatePrev;
  }
  else FARcounter = 0;
  ascentRatePrev = ascentRate;
//  Serial.print("prev AR: ");
//  Serial.println(ascentRatePrev);
  
  dt = (timeStamp[1] - timeStamp[9])/1000;
  checkFix();
    
  if(!AWorking && !BWorking && !CWorking && tenGoodHits && Altitude[0]>= ALTITUDE_FLOOR) { // NEW-ISH
    // if no GPS are working AND we've had 10 good hits AND altitude is above minimum, use linear progression
    latitude[0] = getNextLat(latitude[1],heading,dt,groundSpeed);
    longitude[0] = getNextLong(longitude[1],latitude[1],heading,dt,groundSpeed);
    Altitude[0] = getNextAlt(ascentRate,dt,Altitude[1]);
    Serial.println(F("GPS NOT WORKING"));
    detData.Usage = 0x05; // NEW
    return false; // NEW
  }
  else if(AWorking && !BWorking && !CWorking) { // A only working
    latitude[0] = latA;
    longitude[0] = lonA;
    Altitude[0] = altA;
    detData.Usage = 0x01; // NEW
  }
  else if(!AWorking && BWorking && !CWorking) { // B only working
    latitude[0] = latB;
    longitude[0] = lonB;
    Altitude[0] = altB;
    detData.Usage = 0x01; // NEW
  }
  else if(!AWorking && !BWorking && CWorking) { // C only working
    latitude[0] = latC;
    longitude[0] = lonC;
    Altitude[0] = altC;
    detData.Usage = 0x01; // NEW
  }
  else if(AWorking && BWorking && !CWorking) { // A and B working
    latitude[0] = (latA+latB)/2;
    longitude[0] = (lonA+lonB)/2;
    Altitude[0] = (altA+altB)/2;
    detData.Usage = 0x02; // NEW
  }
  else if(AWorking && !BWorking && CWorking) { // A and C working
    latitude[0] = (latA+latC)/2;
    longitude[0] = (lonA+lonC)/2;
    Altitude[0] = (altA+altC)/2;
    detData.Usage = 0x02; // NEW
  }
  else if(!AWorking && BWorking && CWorking) { // B and C working
    latitude[0] = (latB+latC)/2;
    longitude[0] = (lonB+lonC)/2;
    Altitude[0] = (altB+altC)/2;
    detData.Usage = 0x02; // NEW
  }
  else if(AWorking && BWorking && CWorking) { // A B and C working
    latitude[0] = (latA+latB+latC)/3;
    longitude[0] = (lonA+lonB+lonC)/3;
    Altitude[0] = (altA+altB+altC)/3;
    detData.Usage = 0x03; // NEW
  }


    // Setting the previous values to the current values so they can be used for computations next time.
latAPrev = latA;
latBPrev = latB;
latCPrev = latC;
lonAPrev = lonA;
lonBPrev = lonB;
lonCPrev = lonC;
altAPrev = altA;
altBPrev = altB;
altCPrev = altC;

if (detData.Usage == 0x01 || detData.Usage == 0x02 || detData.Usage == 0x03) // ALL NEW
{
  tenGoodHitsCounter+=1;
  if (tenGoodHitsCounter == 10) tenGoodHits = true;
}

GPSdata.alt = Altitude[0]; // NEW
GPSdata.latitude = latitude[0]; // NEW
GPSdata.longitude = longitude[0]; // NEW
GPSdata.AR = ascentRate; // NEW

return true;

}

bool workingGPS(float latPrev, float latCurrent, float lonPrev, float lonCurrent, float altPrev, float altCurrent){
  float latDiff = abs(latPrev - latCurrent);
  float lonDiff = abs(lonPrev - lonCurrent);
  //Serial.println(altPrev);
  //Serial.println(altCurrent);
  float altDiff = abs(altPrev - altCurrent);
  if(latDiff == 0 && lonDiff == 0 && altDiff == 0) {
    return false;
  }
  else if(latDiff >= MAX_LAT_CHANGE || lonDiff >= MAX_LON_CHANGE || altDiff >= MAX_ALT_CHANGE) {
    return false;
  }
  return true;
}

float getAscentRate(float alt1, float alt2, long time1, long time2) {
  float velocity = 0;

  time1 /= 1000;    // divide milliseconds into seconds
  time2 /= 1000;
    
  if(alt1 !=0 && alt2 != 0) {
    velocity = ((alt1-alt2)/(time1-time2)) * 60;
  }

  return velocity;      // returns a vleocity in feet/minute
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

  float miles_per_lat = 69; // roughly 69 miles per degree latitude
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

void checkFix() {
  
  if(millis() - fix_timestamp < 4000) {
    fixStatus[0] = FIX;
  }
  else {
    fixStatus[0] = NOFIX;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////*********************** END OF CDU FUNCTIONS ***************************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
