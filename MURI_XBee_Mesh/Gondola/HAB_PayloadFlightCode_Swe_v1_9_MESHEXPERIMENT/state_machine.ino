///////////////////////////////////////////////////////////////////////////////////////////////////
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ALL NEW~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
// CUT REASON KEY
// Ascent Timer ran out 0x00
// Termination altitude reached 0x01
// Slow ascent timer ran out 0x02
// In slow ascent, under threshold 0x03
// Float timer ran out 0x04
// Reached slow descent floor 0x05
// Slow descent timer ran out 0x06
// Out of bounds 0x07
// Master timer ran out 0x08
// Error - default state timers 0x09
//////////////////////////////////DETERMINATION////////////////////////////////////////////////////
void Determination(){
  // takes in GPS data, pressure data, and current time
  // if GPS is valid, uses GPS to determine lat, long, alt, and AR
  // if GPS is not valid but pressure is, calculates altitude and AR (below 80000ft)
  // if neither are valid, and above floor with at least 10 good previous GPS hits, use linear progression
  // if below floor with bad GPS, throws error
  // uses hex indication as to whether it is using GPS, pressure, or linear progression

  if (CompareGPS()){ // have compareGPS return a bool - true if GPS data is good, false if not (all 3 GPS fail) SET DETDATA.USAGE TO 1, 2, or 3 in COMPAREGPS FUNCTION
    
    detData.alt = GPSdata.alt;
    detData.latitude = GPSdata.latitude;
    detData.longitude = GPSdata.longitude;
    detData.AR = GPSdata.AR;
    
  }
  
  /*else if (pressureValid()){ // need a way to determine if the pressure is valid

    Serial.println(F("GPS NOT WORKING");
    detData.pressure = pressure;
    detData.alt = GetAltFromPressure();
    detData.latitude = 0; // or other alternative - 0 may be unreliable
    detData.longitude = 0;
    detData.AR = GetARFromPressure();
    detData.Usage = 0x04; // using pressure for determination
      
  }*/

  /*else if (tenGoodHits && (Altitude[0] >= ALTITUDE_FLOOR)){ 

    THIS FUNCTIONALITY CAN BE FOUND IN COMPARISON FOR IF NO GPS IS WORKING
    maybe this should be if tenGoodHits is false, and assume AR = 800 ft/min

    Serial.println(F("GPS NOT WORKING");
    linearProgression();
    detData.Usage = 0x05; // using linear progression
    
  }*/
  
  else Serial.println(F("GPS NOT WORKING")); // outputs a warning if GPS not working while on the ground 

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

  if (detData.AR>=375){
    stateSuggest = ASCENT;
  }
  if (detData.AR<375 && detData.AR>=100){
    stateSuggest = SLOW_ASCENT;
  }
  if (detData.AR<100 && detData.AR>-100){
    stateSuggest = FLOAT;
  }
  if (detData.AR<=-100 && detData.AR>-375){
    stateSuggest = SLOW_DESCENT;
  }
  if (detData.AR<=-375){
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
    Serial.println("Suggested State: Initialization (or error or past timer if that comes next)");
  }

  if (detData.Usage == 0x00){ // error state
    stateSuggest = ERROR_STATE; // doesn't exist - State will go to default
    Serial.println("Suggested State: Error (or past timer if that comes next)");
  }

  if (millis() > MASTER_TIMER){
    stateSuggest = PAST_TIMER;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////STATE/////////////////////////////////////////////////////////////////
void State(){
  // take in stateSuggest from control and switches into that state after a predetermined number of hits
  switch (stateSuggest){
    ////ASCENT////
    case ASCENT:

      Serial.println("Suggested State: Ascent");
      if (currentState!=ASCENT){ // criteria for entering Ascent functionality
        ascentCounter += 1; // increment ascent counter
        SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (ascentCounter >= 30 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 30 consecutive state suggestions
          currentState = ASCENT;
          ascentStamp = millis();
        }
      }

      if (currentState==ASCENT){ // operations while in ascent
        Serial.println("Current State: Ascent");
        
        if (millis()-ascentStamp >= ASCENT_TIMER){ // if ascent timer is reached
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x00;
          
        }

        if (detData.alt > ALTITUDE_CEILING){ // if ceiling is reached
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x01;
        }
       
      }

      break;

    ////SLOW ASCENT////
    case SLOW_ASCENT:

      Serial.println("Suggested State: Slow Ascent");
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
          Serial.println("Current State: Slow Ascent");
          if (detData.alt > ALTITUDE_CEILING){ // if ceiling reached
            checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
            cdu1Packet_tx[4] = checksum >> 8;
            cdu1Packet_tx[5] = checksum;
            sendMeshData(cdu1Packet_tx,false);
            cutReasonA = 0x01;
          }
          if (millis()-SAstamp >= SA_TIMER){ // if slow ascent timer reached
            checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
            cdu1Packet_tx[4] = checksum >> 8;
            cdu1Packet_tx[5] = checksum;
            sendMeshData(cdu1Packet_tx,false);
            cutReasonA = 0x02;
          }

          if (detData.alt < SA_FLOOR){ // cuts immediately under threshold
            checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
            cdu1Packet_tx[4] = checksum >> 8;
            cdu1Packet_tx[5] = checksum;
            sendMeshData(cdu1Packet_tx,false);
            cutReasonA = 0x03;
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

        FARcounter = 0;
        
        if (floatCounter >= 180 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 180 consecutive state suggestions
          currentState = FLOAT;
          floatStamp = millis();
        }
      }

      if (currentState==FLOAT){ // operations while in float
      
        Serial.println("Current State: Float");
        if (detData.alt > ALTITUDE_CEILING){ // if ceiling is reached
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x01;
        }

        if (millis()-floatStamp >= FLOAT_TIMER){ // if timer reached
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x04;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);   
          cutReasonB = 0x04;
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
        Serial.println("Current State: Slow Descent");
        if (detData.alt < SLOW_DESCENT_FLOOR){ // if floor is reached
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x05;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);   
          cutReasonB = 0x05;
        }
        
        else if (millis()-SDstamp >= SLOW_DESCENT_TIMER){ // if timer is reached
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x06;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);   
          cutReasonB = 0x06;
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
        Serial.println("Current State: Descent");
        if (millis()-descentStamp >= SLOW_DESCENT_TIMER){ // reuses SD timer as a backup
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x06;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);   
          cutReasonB = 0x06;
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
        cutResistorOnA();
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
        cutResistorOnA();
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
        Serial.println("Current State: Out of Bounds");
        checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x07;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);   
          cutReasonB = 0x07;
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
        Serial.println("Current State: Past Timer");
        checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
        cdu1Packet_tx[4] = checksum >> 8;
        cdu1Packet_tx[5] = checksum;
        sendMeshData(cdu1Packet_tx,false);
        cutReasonA = 0x08;
        if (timeCounter==1) {
          timerStampCutA = millis();
          timeCounter++;
        }
        if (millis() - timerStampCutA >= SLOW_DESCENT_TIMER){ // wait to cut B (hopefully to get slow descent data)
          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);   
          cutReasonB = 0x08;
        }
      }

      break;

    ////DEFAULT////
    default: 

    // if initialization never triggers, moves to the next of the function where it cuts A then B

      if (currentState==INITIALIZATION){ // currentState is initialized as INITIALIZATION, no other states have been activated
        Serial.println("Current State: Initialization");
        if (initCounter==1) {
          defaultStamp = millis();
          initCounter++;
        }
        if ( (millis()-defaultStamp) >= (INITIALIZATION_TIME + ASCENT_TIMER) ){ // gives an extra time buffer to leave initialization

         checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
         cdu1Packet_tx[4] = checksum >> 8;
         cdu1Packet_tx[5] = checksum;
         sendMeshData(cdu1Packet_tx,false);
         cutReasonA = 0x09;
         if (initCounter2==1) {
          defaultStampCutA = millis();
          initCounter2++;
        }
         if (millis() - defaultStampCutA >= SLOW_DESCENT_TIMER){ // wait to cut B (hopefully to get slow descent data)
          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);    
          cutReasonB = 0x09;
         }
        }
          
        }

        else {
          Serial.println("Current State: Error");
          // if it's not in initialization, means it entered another state at some point, then everything stopped working and stateSuggest = ERROR_STATE now
          // should wait a while to see if it will enter a state again, then do the same cut strategy, hoping for some slow descent
          defaultStamp2 = millis();
          if ( (millis()-defaultStamp2) >= (DEFAULT_TIME + ASCENT_TIMER) ){ // gives an extra time buffer to leave default

            checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
            cdu1Packet_tx[4] = checksum >> 8;
            cdu1Packet_tx[5] = checksum;
            sendMeshData(cdu1Packet_tx,false);
            cutReasonA = 0x09;
            defaultStampCutA = millis();
            if (millis() - defaultStampCutA >= SLOW_DESCENT_TIMER){ // wait to cut B (hopefully to get slow descent data)
              checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
              cdu2Packet_tx[4] = checksum >> 8;
              cdu2Packet_tx[5] = checksum;
              sendMeshData(cdu2Packet_tx,true);    
              cutReasonB = 0x09;
            }
          }
        }

        break;
        
  }
  Serial.println();
  if (cutReasonA != 0x20 || cutReasonB != 0x20){
    Serial.print("CUT REASON A: ");
    Serial.print(cutReasonA,HEX);
    Serial.print(", CUT REASON B: ");
    Serial.println(cutReasonB,HEX);
    Serial.println();
  }
  Serial.println(detData.alt);
  Serial.println(detData.AR);
}
