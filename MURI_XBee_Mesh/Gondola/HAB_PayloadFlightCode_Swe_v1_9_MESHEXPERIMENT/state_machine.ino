/////////////////////////////////////////////////////////////////////////////////////////////////
///////////******************* BEGINNING OF STATE MACHINE FUNCTIONS ***************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
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
  // ensure the state machine does not start until a certain intial altitude is reached
  static bool initDone = false;
  static bool initComplete = false;
  static byte initCounter = 0;
  CompareGPS();
  if(!initComplete){
    state = INITIALIZATION;
    stateString = F("Initialization");
    if(Altitude[1] > INIT_ALTITUDE) {
      initCounter++;
      if(initCounter >= 40) {
        initCounter = 0;
        initComplete = true; 
      }
    }
    if(Altitude[1] < INIT_ALTITUDE){
      initCounter = 0;
    }
  }
  // run state switch function if the state machine is intialized
  if (initComplete) { 
    stateSwitch();
  }

  uint16_t checksum;  
  // run functions based off of the current state
  switch(state) {
    ///// Ascent /////
    case 0x01:
      stateString = F("Ascent");

      static unsigned long ascentStamp = millis();

      // cut balloon A if the ascent timer runs out
      if(millis() - ascentStamp > ASCENT_INTERVAL*M2MS) {
        //******************* SEND CUT COMMAND TO CUTTER COMPUTER A ******************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
        cutReasonA = 0x00;
      }
      // cut balloon A if the termination altitude is reached
      if (Altitude[0] > SLOW_DESCENT_CEILING) {
        //******************* SEND CUT COMMAND TO CUTTER COMPUTER A *********************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
        cutReasonA = 0x01;
      }
      //else cutReasonA = F("0");

      break;

    ///// Slow Ascent /////
    case 0x02:
      // cut the resistor and note state
      stateString = F("Slow Ascent");

      // cut both balloons as the stack is ascending too slowly
      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x02;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);   
          cutReasonB = 0x02;

      break;

    ///// Slow Descent /////      
    case 0x04:
      // organize timing schema for slow descent state
      stateString = F("Slow Descent");

      static unsigned long slowDescentStamp = millis(); // initializaed upon first time in this state
      static byte SDTerminationCounter = 0;

      if(millis() - slowDescentStamp > SLOW_DESCENT_INTERVAL*M2MS || (Altitude[0] < SLOW_DESCENT_FLOOR && Altitude[0] != 0)) {
        SDTerminationCounter++;

        if(SDTerminationCounter >= 40) {
          //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ******************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x03;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);    
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

      if(Altitude[0] < SLOW_DESCENT_FLOOR && Altitude[0] != 0) {
        floorAltitudeCounter++;

        if(floorAltitudeCounter >= 40 && !cutCheck) {
          floorAltitudeCounter = 0;

          //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B *******************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
          cutReasonA = 0x03;

          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);    
          cutReasonB = 0x03;
        }
      }

      break;

    ///// Float /////
    case 0x10:
      // abort flight
      stateString = F("Float");

      // cut both balloons as the stack is in a float state
      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
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

      break;

    ///// Out of Boundary /////
    case 0x20:
      // cut resistor and note state
      stateString = F("Out of Boundary");
      
      // cut both balloons as the stack is out of the predefined flight boundaries
      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
          checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
          cdu1Packet_tx[4] = checksum >> 8;
          cdu1Packet_tx[5] = checksum;
          sendMeshData(cdu1Packet_tx,false);
                    
      
          checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
          cdu2Packet_tx[4] = checksum >> 8;
          cdu2Packet_tx[5] = checksum;
          sendMeshData(cdu2Packet_tx,true);
       // cut reasons are more specifically defined in the boundaryCheck() function

      break;

    ///// Temperature Failure /////
//    case 0x40:
//      // cut resistor and note state
//      stateString = F("Temperature Failure");
//
//      // cut balloon as temps are at critical levels
//      //******************* SEND CUT COMMAND TO CUTTER COMPUTERS A AND B ********************////////////
//      checksum = cdu1Packet_tx[0] + cdu1Packet_tx[1] + cdu1Packet_tx[2] + cdu1Packet_tx[3];
//          cdu1Packet_tx[4] = checksum >> 8;
//          cdu1Packet_tx[5] = checksum;
//          Serial4.write(cdu1Packet_tx, CDU_TX_SIZE);
//      
//      checksum = cdu2Packet_tx[0] + cdu2Packet_tx[1] + cdu2Packet_tx[2] + cdu2Packet_tx[3];
//          cdu2Packet_tx[4] = checksum >> 8;
//          cdu2Packet_tx[5] = checksum;
//          Serial5.write(cdu2Packet_tx, CDU_TX_SIZE);
//      
//      cutReasonA = 0x50;
//      cutReasonB = 0x50;
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
  Serial.println("INSIDE STATE SWTICH");
  // initialize all counters as static bytes that begin at zero
  static byte ascentCounter = 0,  slowAscentCounter = 0,  descentCounter = 0, slowDescentCounter = 0, recoveryCounter = 0, boundaryCounter = 0;// tempCounter = 0;
  static uint16_t floatCounter = 0;

  if(stateSwitched) {     // reset all state counters if the state was just switched
    ascentCounter = 0;  slowAscentCounter = 0;  descentCounter  = 0;  slowDescentCounter = 0;   floatCounter  = 0; recoveryCounter = 0;  boundaryCounter = 0;//, tempCounter = 0;
    stateSwitched = false;
  }

  if(ascentRate > MAX_SA_RATE && state != ASCENT) {
    ascentCounter++;
    thisHit = 2;
    if(ascentCounter >= 40) {
      state = ASCENT;
      ascentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate <= MAX_SA_RATE && ascentRate > MAX_FLOAT_RATE && state != SLOW_ASCENT) {
    slowAscentCounter++;
    thisHit = 2;
    if(slowAscentCounter >= 40) {
      state = SLOW_ASCENT;
      slowAscentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate <= MAX_FLOAT_RATE && ascentRate >= MIN_FLOAT_RATE && state != FLOAT) {
    floatCounter++;
    thisHit = 2;
    if(floatCounter >= 1800) {
      state = FLOAT;
      floatCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate < MIN_FLOAT_RATE && ascentRate >= MIN_SD_RATE && state != SLOW_DESCENT) {
    slowDescentCounter++;
    thisHit = 2;
    if(slowDescentCounter >= 40) {
      state = SLOW_DESCENT;
      slowDescentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(ascentRate < MIN_SD_RATE && state !=DESCENT) {
    descentCounter++;
    thisHit = 2;
    if(descentCounter >= 40) {
      state = DESCENT;
      descentCounter = 0;
      stateSwitched = true;
    }
  }
  else if(state != RECOVERY && (state == DESCENT || state == SLOW_DESCENT) && Altitude[0] < RECOVERY_ALTITUDE) {
    recoveryCounter++;
    thisHit = 2;
    if(recoveryCounter >= 40) {
      state = RECOVERY;
      recoveryCounter = 0;
      stateSwitched = true;
    }
  }

  // part of a separate series of if/else statements as criteria for this state is different
  if(boundaryCheck() && state != OUT_OF_BOUNDARY) {
    boundaryCounter++;
    thisHit = 1;
    if(boundaryCounter >= 40) {
      state = OUT_OF_BOUNDARY;
      boundaryCounter = 0;
      stateSwitched = true;
    }
  } 

 /* if(!((MIN_TEMP<tempInt_float)&&(tempInt_float<MAX_TEMP)) && state != TEMPERATURE_FAILURE)  {
    tempCounter++;
    thisHit = 2;
    if(tempCounter >= 40) {
      state = TEMPERATURE_FAILURE;
      tempCounter  = 0;
      stateSwitched = true;
    }
  } */
  
  // if two or more counters are greater than 0, reset all counters
  // resets counters if non-consecutive
  uint8_t statesHit = 0;
  if( ascentCounter > 0 ) statesHit++;
  if( slowAscentCounter > 0 ) statesHit++;
  if( descentCounter > 0 ) statesHit++;
  if( slowDescentCounter > 0 ) statesHit++;
  if( recoveryCounter > 0 ) statesHit++;
  if( floatCounter > 0 ) statesHit++;
  if( statesHit >= 2 ){
    ascentCounter = 0;
    slowAscentCounter = 0;
    descentCounter = 0;
    slowDescentCounter = 0;
    recoveryCounter = 0;
    floatCounter = 0;
  }
  
    if( prevHit != thisHit )
  {
    boundaryCounter = 0;
   // tempCounter = 0; 
  }

  prevHit = thisHit;
     
}

bool boundaryCheck() {
  // function to check if the payload is out of the flight boundaries
  if (longitude[0] > EASTERN_BOUNDARY) {
    cutReasonA = 0x10;
    cutReasonB = 0x10;
    return true;
  }
  else if (longitude[0] < WESTERN_BOUNDARY) {
    cutReasonA = 0x20;
    cutReasonB = 0x20;
    return true;
  }
  else if (latitude[0] > NORTHERN_BOUNDARY) {
    cutReasonA = 0x30;
    cutReasonB = 0x30;
    return true;
  }
  else if (latitude[0] < SOUTHERN_BOUNDARY) {
    cutReasonA = 0x40;
    cutReasonB = 0x40;
    return true; 
  }
  else {
    return false;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////******************* END OF STATE MACHINE FUNCTIONS *********************//////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
