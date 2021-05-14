// CUT REASON KEY
// No cut 0x22
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
// Master timer reached 0x70
///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////DETERMINATION///////////////////////////////////////////////////////////////
void Determination(){
  // takes in GPS data, pressure data, and current time
  // if GPS is valid, uses GPS to determine lat, long, alt, and AR
  // if GPS is not valid but pressure is, calculates altitude and AR (below 80000ft)
  // if neither are valid, and above floor with at least 10 good previous GPS hits, use linear progression
  // if below floor with bad GPS, throws error
  // uses hex indication as to whether it is using GPS, pressure, or linear progression


    detData.alt = GPSdata.alt;
    detData.latitude = GPSdata.latitude;
    detData.longitude = GPSdata.longitude;
    detData.AR = GPSdata.AR;
  if (fixStatus[0] != FIX && fixStatus[9] != FIX){ // have compareGPS return a bool - true if GPS data is good, false if not (all 3 GPS fail) SET DETDATA.USAGE TO 1, 2, or 3 in COMPAREGPS FUNCTIO
     Serial.println(F("GPS NOT WORKING"));} // outputs a warning if GPS not working while on the ground 

  if (detData.Usage!=0x05 && detData.Usage!=0x01){
    // THIS WILL IDEALLY NEVER TRIGGER
    detData.Usage = 0x00; // indicates error
  }
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////CONTROL////////////////////////////////////////////////////////////////////////////
void Control(){
  // uses detData to determine what state it thinks we should be in
  // worst case states take priority, but every possible state is outputted as hex

  if (detData.AR>=MAX_SA_RATE){
    stateSuggest = ASCENT;
  }
  if (detData.AR<MAX_SA_RATE && detData.AR>=MAX_FLOAT_RATE){
    stateSuggest = SLOW_ASCENT;
  }
  if (detData.AR<MAX_FLOAT_RATE && detData.AR>MIN_FLOAT_RATE){
    stateSuggest = FLOAT;
  }
  if (detData.AR<=MIN_FLOAT_RATE && detData.AR>MIN_SD_RATE){
    stateSuggest = SLOW_DESCENT;
  }
  if (detData.AR<=MIN_SD_RATE){
    stateSuggest = DESCENT;
  }

  /*if (tempfailure){
    stateSuggest = TEMP_FAILURE;
  }

  if (batteryFailure){
    stateSuggest = BATTERY_FAILURE;
  }*/

  if ((detData.longitude > EASTERN_BOUNDARY) || (detData.longitude < WESTERN_BOUNDARY) || (detData.latitude >NORTHERN_BOUNDARY) || (detData.latitude < SOUTHERN_BOUNDARY)){
    stateSuggest = OUT_OF_BOUNDS;  
  }

  if (detData.alt < ALTITUDE_FLOOR){
    stateSuggest = INITIALIZATION;
  }

  if (detData.Usage == 0x00){ // error state
    stateSuggest = ERROR_STATE; // doesn't exist - State will go to default
  }

  if (millis() > MASTER_TIMER){
    stateSuggest = PAST_TIMER;
  }
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////STATE/////////////////////////////////////////////////////////////////
void State(){
  Serial.println(stateSuggest);
  Serial.println(currentState);  // take in stateSuggest from control and switches into that state after a predetermined number of hits
  switch (stateSuggest){
    ////ASCENT////
    case ASCENT:

      Serial.println("Suggested State: Ascent");
      if (currentState!=ASCENT){ // criteria for entering Ascent functionality
        ascentCounter += 1; // increment ascent counter
        SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        Serial.println(ascentCounter);
        if (ascentCounter >= 30 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 30 consecutive state suggestions
          currentState = ASCENT;
          ascentStamp = millis();
        }
      }

      if (currentState==ASCENT){ // operations while in ascent
        
        if (detData.alt > ALTITUDE_CEILING || millis()-ascentStamp >= ASCENT_TIMER){ // if ceiling or timer is reached
          requestCut(); // only cuts A (large balloon) so slow descent can still be acheived
        }
      }
      break;

    ////SLOW ASCENT////
    case SLOW_ASCENT:
      Serial.println("SA");
      if (currentState!=SLOW_ASCENT){ // criteria for entering Slow Ascent functionality
          SAcounter += 1; // increment slow ascent counter
          ascentCounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
          tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
          
          if (SAcounter >= 60 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 60 consecutive state suggestions
            currentState = SLOW_ASCENT;
            SAstamp = millis();
          }
        }
  
        if (currentState==SLOW_ASCENT){ // operations while in slow ascent
          
          if (detData.alt > ALTITUDE_CEILING || millis()-SAstamp >= SA_TIMER){ // if ceiling or timer is reached
            requestCut(); // cut only A to enter slow descent
          }

          if (detData.alt < SA_FLOOR){ // cuts immediately under threshold
            requestCut(); // only cuts A (large balloon) so some slow descent can still be acheived
          }
        }
        break;

    ////FLOAT////
    case FLOAT:
      Serial.println("Suggested State: Float");
      if (currentState!=FLOAT){ // criteria for entering Float functionality
        floatCounter += 1; // increment float counter
        ascentCounter =0, SAcounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (floatCounter >= 180 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 180 consecutive state suggestions
          currentState = FLOAT;
          floatStamp = millis();
        }
      }

      if (currentState==FLOAT){ // operations while in float
        
        if (detData.alt > ALTITUDE_CEILING){ // if ceiling is reached
          requestCut(); // cut only A to attempt to enter slow descent
        }

        if (millis()-floatStamp >= FLOAT_TIMER){ // if timer reached
          requestCut(); // cut both balloons
        }
      }
      break;

    ////SLOW DESCENT////
    case SLOW_DESCENT:
      Serial.println("Suggested State: Slow Descent");
      if (currentState!=SLOW_DESCENT){ // criteria for entering Slow Descent functionality
        SDcounter += 1; // increment slow descent counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (SDcounter >= 30 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 30 consecutive state suggestions
          currentState = SLOW_DESCENT;
          SDstamp = millis();
        }
      }

      if (currentState==SLOW_DESCENT){ // operations while in slow descent
        
        if (detData.alt < SLOW_DESCENT_FLOOR || millis()-SDstamp >= SLOW_DESCENT_TIMER){ // if floor or timer is reached
          requestCut(); // cut both balloons (A is likely cut already, but just in case)
        }
      }
      break;


    ////DESCENT////
    case DESCENT:

      Serial.println("Suggested State: Descent");
      if (currentState!=DESCENT){ // criteria for entering Descent functionality
        descentCounter += 1; // increment descent counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0;
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (descentCounter >= 30 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 30 consecutive state suggestions
          currentState = DESCENT;
          descentStamp = millis();
        }
      }

      if (currentState==DESCENT){ // operations while in descent
        
        if (millis()-descentStamp >= SLOW_DESCENT_TIMER){ // reuses SD timer as a backup
          requestCut(); // cut both balloons
        }
      }
      break;

    ////TEMPERATURE FAILURE//// To be added later... REVIEW AND ADJUST BEFORE USING
    /* case TEMP_FAILURE:
    
      if (currentState!=TEMP_FAILURE){ // criteria for entering Temperature Failure functionality
        tempCounter += 1; // increment temperature failure counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
      }

      if (currentState==TEMP_FAILURE){ // operations while in temperature failure
        requestCut();
        cutResistorOnB();
      }*/

    ////BATTERY FAILURE////To be added later... REVIEW AND ADJUST BEFORE USING
    /*case BATTERY_FAILURE:
      if (currentState!=BATTERY_FAILURE){ // criteria for entering Battery Failure functionality
        battCounter += 1; // increment battery failure counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
      }

      if (currentState==BATTERY_FAILURE){ // operations while in battery failure
        requestCut();
        cutResistorOnB();
      }*/
        
    ////OUT OF BOUNDARY////
    case OUT_OF_BOUNDS:
      Serial.println("Suggested State: Out of Bounds");
      if (currentState!=OUT_OF_BOUNDS){ // criteria for entering Out of Boundary functionality
        boundCounter += 1; // increment out of boundary counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (boundCounter >= 180 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 180 consecutive state suggestions
          currentState = OUT_OF_BOUNDS;
        }
      }

      if (currentState==OUT_OF_BOUNDS){ // operations while out of boundary
        requestCut(); // cut both balloons
      }
      break;

    ////MASTER TIMER REACHED////
    case PAST_TIMER:
      Serial.println("Suggested State: Past Timer");
      if (currentState!=PAST_TIMER){ // criteria for entering Master Timer Reached functionality
        timerCounter += 1; // increment ascent counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0; // reset all other state counters
        
        if (timerCounter >= 10){ // activates after 10 consecutive state suggestions, regardless of altitude
          currentState = PAST_TIMER;
        }
      }

      if (currentState==PAST_TIMER){ // operations after Master Timer reached
        
        requestCut();
        // wait to cut B (hopefully to get slow descent data)
      }

      break;

    ////DEFAULT////
    default: 

    Serial.println("HERE 0");
    // move outside of switch-case
    // has to get through initialization to trigger states
    // if initialization never triggers, moves to the next of the function where it cuts A then B

      if (currentState==INITIALIZATION){ // currentState is initialized as INITIALIZATION, no other states have been activated
        Serial.println("Suggested State: Initialization");
        if (initCounter==1) {
          defaultStamp = millis();
          initCounter++;
        }
        
        //Serial.println(defaultStamp);
        if ( (millis()-defaultStamp) >= (INITIALIZATION_TIME + ASCENT_TIMER) ){ // gives an extra time buffer to leave initialization
         requestCut();
         // wait to cut B (hopefully to get slow descent data)
        }
      }
        else {
          Serial.println("Suggested State: Error");
          // if it's not in initialization, means it entered another state at some point, then everything stopped working and stateSuggest = ERROR_STATE now
          // should wait a while to see if it will enter a state again, then do the same cut strategy, hoping for some slow descent
          defaultStamp2 = millis();
          if ( (millis()-defaultStamp2) >= (DEFAULT_TIME + ASCENT_TIMER) ){ // gives an extra time buffer to leave default

            requestCut();
            // wait to cut B (hopefully to get slow descent data)
            }
        }
        
  break;
}
}

void printState(){
  switch (currentState){
    case ASCENT: 
      Serial.println("ASCENT");
      break;
    case INITIALIZATION:
      Serial.println("INITIALIZATION");
      break;
    case SLOW_ASCENT:
      Serial.println("SLOW_ASCENT");
      break;
    case FLOAT:
      Serial.println("FLOAT");
      break;
    case SLOW_DESCENT:
      Serial.println("SLOW_DESCENT");
      break;
    case DESCENT:
      Serial.println("DESCENT");
      break;
    case TEMP_FAILURE:
      Serial.println("TEMP_FAILURE");
      break;
    case BATTERY_FAILURE:
      Serial.println("BATTERY_FAILURE");
      break;
    case OUT_OF_BOUNDS:
      Serial.println("OUT_OF_BOUNDS");
      break;
    case PAST_TIMER:
      Serial.println("PAST_TIMER");
      break;
    case ERROR_STATE:
      Serial.println("ERROR_STATE");
      break;
  }
}
