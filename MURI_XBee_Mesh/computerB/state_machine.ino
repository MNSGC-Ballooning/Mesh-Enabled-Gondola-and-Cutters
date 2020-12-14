// State machine functions for the cutaway system

#define INITIALIZATION 0x00   // corresponding hex values are chosen so as to avoid bit-flipping in the stratosphere
#define ASCENT 0x01
#define SLOW_ASCENT 0x02
#define SLOW_DESCENT 0x04
#define DESCENT 0x08
#define FLOAT 0x10
#define OUT_OF_BOUNDARY 0x20
#define TEMPERATURE_FAILURE 0x40
#define BATTERY_FAILURE 0x60
#define RECOVERY 0x80

// CUT REASON KEY
// Ascent Timer ran out 0x00
// Termination altitude reached 0x01
// Slow ascent state 0x02
// Reached slow descent floor 0x03
// Float state 0x04
// Battery failure 0x05
// Reached eastern boundary 0x10
// Reached western boundary 0x20
// Reached Northern boundary 0x30
// Reached southern boundary 0x40
// Below min temp 0x50
// Above max temp 0x60


void stateMachine() {
  static bool initDone = false;
  static byte initCounter = 0;

  // ensure the state machine does not start until a certain intial altitude is reached
  if(!initDone && fixStatus[0] == FIX) {
    state = INITIALIZATION;
    stateString = F("Initialization");
    if(alt[0] > INIT_ALTITUDE) {
      initCounter++;
      if(initCounter >= 40) {
        initCounter = 0;
        Serial.println(F("Initialized"));
        initDone = true; 
      }
    }
    if(alt[0] < INIT_ALTITUDE){
      initCounter = 0;
    }
  }

  // run state switch function if the state machine is intialized
  if (initDone && fixStatus[0] == FIX) { 
    stateSwitch();
  }

  
  // run functions based off of the current state
  switch(state) {
    ///// Ascent /////
    case 0x01:
      stateString = F("Ascent");

      static unsigned long ascentStamp = millis();

      // cut balloon A if the ascent timer runs out

      // cut balloon A if the termination altitude is reached

      break;

    ///// Slow Ascent /////
    case 0x02:
      stateString = F("Slow Ascent");

      // cut both balloons as the stack is ascending too slowly
      requestCut();
      cutReasonB = 0x02;

      break;

    ///// Slow Descent /////   
    case 0x04:
      // organize timing schema for slow descent state
      stateString = F("Slow Descent");

      static unsigned long slowDescentStamp = millis(); // initializaed upon first time in this state
      static byte SDTerminationCounter = 0;

      if(millis() - slowDescentStamp > SLOW_DESCENT_INTERVAL*M2MS || (alt[0] < SLOW_DESCENT_FLOOR && alt[0] != 0)) {
        SDTerminationCounter++;

        if(SDTerminationCounter >= 40 && (millis() - slowDescentStamp >= SLOW_DESCENT_BUFFER*M2MS) ) {
          requestCut();
          cutReasonB = 0x03;
        }
      }

      break;

    ///// Descent /////
    case 0x08:
      // do nothing but note state
      stateString = F("Descent");

      static byte floorAltitudeCounter = 0;   // increments if the stack is below the slow descent altitude floor
      static bool cutCheck = false;

      if(alt[0] < SLOW_DESCENT_FLOOR && alt[0] != 0) {
        floorAltitudeCounter++;

        if(floorAltitudeCounter >= 40 && !cutCheck) {
          floorAltitudeCounter = 0;

          requestCut();
          cutReasonB = 0x03;
        }
      }

      break;

    ///// Float /////
    case 0x10:
      // abort flight
      stateString = F("Float");

      // cut both balloons as the stack is in a float state
      requestCut();
      cutReasonB = 0x04;      

      break;

    ///// Out of Boundary /////
    case 0x20:
      // cut resistor and note state
      stateString = F("Out of Boundary");
      
      // cut both balloons as the stack is out of the predefined flight boundaries
      requestCut();
      // cut reasons are more specifically defined in the boundaryCheck() function

      break;

    ///// Temperature Failure /////
//    case 0x40:
//      // cut resistor and note state
//      stateString = F("Temperature Failure");
//
//      // cut balloon as temps are at critical levels
//      cutResistorOnB(); // rather than requestCut, as this cannot be confirmed by main
//
//      // cut reasons defined within tempCheck() function
//
//      break;

    ///// Battery Failure /////
//    case 0x60:
//
//      stateString = F("Battery failure");
//      // only cut A, so slow descent can be entered
//          
//      break;

    ///// Recovery /////
    case 0x80:
      // reserved for any functions near the ground
      stateString = F("Recovery");

      break;

  }
  
}


void stateSwitch() {
  // initialize all counters as static bytes that begin at zero
  static byte ascentCounter = 0,  slowAscentCounter = 0,  descentCounter = 0, slowDescentCounter = 0, recoveryCounter = 0, boundaryCounter = 0;
  static byte tempCounter = 0; static byte voltCounter = 0;
  static uint16_t floatCounter = 0;

  if(stateSwitched) {     // reset all state counters if the state was just switched
    ascentCounter = 0;  slowAscentCounter = 0;  descentCounter  = 0;  slowDescentCounter = 0;   floatCounter  = 0; recoveryCounter = 0;  boundaryCounter = 0;
    tempCounter = 0; 
    stateSwitched = false;
  }
  if(ascentRate > MAX_SA_RATE && state != ASCENT) {
    ascentCounter++;
    Serial.println(F("Ascent Counter!"));
    if(ascentCounter >= 40) {
      state = ASCENT;
      ascentCounter = 0;
      Serial.println(F("ASCENT"));
      stateSwitched = true;
    }
  }
  else if(ascentRate <= MAX_SA_RATE && ascentRate > MAX_FLOAT_RATE && state != SLOW_ASCENT && alt[0] < 30000) {
    slowAscentCounter++;
    Serial.println(F("Slow Ascent Counter!"));
    if(slowAscentCounter >= 40) {
      state = SLOW_ASCENT;
      slowAscentCounter = 0;
      Serial.println(F("SLOW ASCENT"));
      stateSwitched = true;
    }
  }
  else if(ascentRate <= MAX_FLOAT_RATE && ascentRate >= MIN_FLOAT_RATE && state != FLOAT) {
    floatCounter++;
    Serial.println(F("Float Counter!"));
    if(floatCounter >= 1800) {
      state = FLOAT;
      floatCounter = 0;
      Serial.println(F("FLOAT"));
      stateSwitched = true;
    }
  }
  else if(ascentRate < MIN_FLOAT_RATE && ascentRate >= MIN_SD_RATE && state != SLOW_DESCENT) {
    slowDescentCounter++;
    Serial.println(F("Slow Descent Counter!"));
    if(slowDescentCounter >= 40) {
      state = SLOW_DESCENT;
      slowDescentCounter = 0;
      slowBuffer = millis();
      Serial.println(F("SLOW DESCENT"));
      stateSwitched = true;
    }
  }
  else if(ascentRate < MIN_SD_RATE && state !=DESCENT) {
    descentCounter++;
    Serial.println(F("Descent Counter!"));
    if(descentCounter >= 40) {
      state = DESCENT;
      descentCounter = 0;
      Serial.println(F("DESCENT"));
      stateSwitched = true;
    }
  }
  else if(state != RECOVERY && (state == DESCENT || state == SLOW_DESCENT) && alt[0] < RECOVERY_ALTITUDE) {
    recoveryCounter++;
    Serial.println(F("Recovery Counter!"));
    if(recoveryCounter >= 40) {
      state = RECOVERY;
      recoveryCounter = 0;
      Serial.println(F("RECOVERY"));
      stateSwitched = true;
    }
  }

  // part of a separate series of if/else statements as criteria for this state is different
  if(boundaryCheck() && state != OUT_OF_BOUNDARY) {
    boundaryCounter++;
    thisHit = 1;
    Serial.println(F("Boundary Counter!"));
    if(boundaryCounter >= 40) {
      state = OUT_OF_BOUNDARY;
      Serial.println(F("OUT OF BOUND"));
      boundaryCounter = 0;
      stateSwitched = true;
    }
  } 

//  if(tempCheck() && state != TEMPERATURE_FAILURE){
//    tempCounter++;
//    thisHit = 2;
//    Serial.println(F("Temp Counter!"));
//    if(tempCounter >= 40) {
//      state = TEMPERATURE_FAILURE;
//      Serial.println(F("TEMP FAILURE"));
//      tempCounter = 0;
//      stateSwitched = true;
//    }
//  }
//  if(2*analogRead(VOLTAGE_PIN) < MINIMUM_VOLTAGE && state != BATTERY_FAILURE){
//    voltCounter++;
//    thisHit = 3;
//    Serial.println(F("Volt Counter!"));
//    if (voltCounter >= 40) {
//      state = BATTERY_FAILURE;
//      voltCounter = 0;
//      Serial.println(F("BATTERY FAILURE"));
//      stateSwitched = true;
//    }
//  }

  // if two or more counters are greater than 0, reset all counters
  // resets counters if non-consecutive
  uint8_t statesHit = 0;
  if( ascentCounter > 0 ) statesHit++;
  if( slowAscentCounter > 0 ) statesHit++;
  if( descentCounter > 0 ) statesHit++;
  if( slowDescentCounter > 0 ) statesHit++;
  if( recoveryCounter > 0 ) statesHit++;
  //if( boundaryCounter > 0 ) statesHit++;
  //if( tempCounter > 0 ) statesHit++;
  //if( voltCounter > 0 ) statesHit++;
  if( floatCounter > 0 ) statesHit++;
  if( statesHit >= 2 ){
    ascentCounter = 0;
    slowAscentCounter = 0;
    descentCounter = 0;
    slowDescentCounter = 0;
    recoveryCounter = 0;
    //boundaryCounter = 0;
    //tempCounter = 0; 
    //voltCounter = 0;
    floatCounter = 0;
  }

  if( prevHit != thisHit )
  {
    boundaryCounter = 0;
    tempCounter = 0; 
    //voltCounter = 0;
  }

  prevHit = thisHit;
}
