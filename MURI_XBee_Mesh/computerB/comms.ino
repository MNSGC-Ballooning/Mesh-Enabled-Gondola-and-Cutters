void xbeeStartup()  {                               //Sets up the cutter to mesh with a network. Should be called during main setup()
  xbeeSerial.begin(XBEE_BAUD);
  xbee.init(xbeeChannel);
  xbee.enterATmode();
  xbee.atCommand("ATDL0");
 xbee.atCommand("ATMY1");;
 xbee.exitATmode();

  updateXbee();     //takes a first look for a nearby gondola
}

void updateXbee() {                                 //The main function to call during loop() after code integration. This manages incoming radio traffic
  while(xbee.available()>0)  {
    byte typebyte = xbee.read();
    switch(typebyte)  {
      case(0x00): { //ping command asking for all connected units to send identifiers
        xbee.write(0x01);
        xbee.write(cutterBIdentifier);
        break;
      }
      case(0x01): break; //response to ping command
      case(0x02):{ //packet from a gondola, check if it's sent to this cutter, if it is it records the data
        if(xbee.read() == cutterBIdentifier) {
          recieveTransmission();
          downtimeG = millis()/1000.0;
        }
        else {
          for(int i=0; i<7; i++) {
            xbee.read();
            delay(1);
          }
        }
        break;
      }
      case(0x03): break; //packet from a cutter A, ignored since this is a cutter
      case(0x04): break; //packet from a cutter B, ignored since this is a cutter
      case(0x05): { //request to assign a cutter A to a gondola
        break;
      }
      case(0x06): {
        if(cutterBIdentifier == 0x02) {
          cutterBIdentifier = xbee.read();
          downtimeG = millis()/1000.0;
        }break; //request to assign a cutter B to a gondola
      }
      default: while(xbee.available()>0) xbee.read();
    }
  }

  if(((millis()/1000.0)-downtimeG)>waitForReconnection){
    cutterBIdentifier = 0x02;
  }
}

bool sendMeshData(byte *meshdata) {                 //Sends data to gondola. meshadata should be a pointer to an array of size CDU_TX_SIZE that holds data to be sent
                                                    //Returns true if data is sent, false if cutter connection not yet established.
  if(cutterBIdentifier == 0x02) return false;
  xbee.write(0x04);
  xbee.write(cutterBIdentifier);
  for(int i=0;i<CDU_RX_SIZE;i++)  {
    xbee.write(*(meshdata+i));
  }
  return true;
}

void sendData(){
  // recording current data
  dataPacket.startByte = 0x42; 
  dataPacket.cutterTag = 'B';
  dataPacket.hrs = gps.getHour();
  dataPacket.mins = gps.getMinute();
  dataPacket.secs = gps.getSecond();
  dataPacket.latitude = latitude[0];
  dataPacket.longitude = longitude[0];
  dataPacket.Altitude = alt[0];
  dataPacket.t1 = t1;
  dataPacket.heatStatus = heatStatus;
  dataPacket.currentState = currentState;
  dataPacket.suggestedState = stateSuggest;
  dataPacket.autonomous = autonomousNow;
  dataPacket.cutStatus = cutStatusB;
  dataPacket.cutReason = cutReasonB;
  dataPacket.checksum = 0;
  dataPacket.stopByte = 0x53;

  byte dataHolder[30] = {0};              // define output array
  memcpy(&dataHolder, &dataPacket, 27);   // pass data packet to output array as bytes 

  for( uint8_t i=0; i<28; i++) dataPacket.checksum+=dataHolder[i];

  byte extraStuff[2] = {0};
  memcpy(&extraStuff, &dataPacket.checksum, 2);
  dataHolder[27] = extraStuff[0];
  dataHolder[28] = extraStuff[1];
  dataHolder[29] = 0x53;

  sendMeshData(dataHolder);        // write output array to main computer
  Serial.println();
  for(int i=0; i<29; i++){
    //XbeeSerial.print(dataHolder[i],HEX); // prints out sent data to serial
    Serial.print(dataHolder[i],HEX);
    Serial.print(F(" "));
  }
  Serial.println();
}

void requestCut(){
    if(autonomousNow && !cutterOnB) cutResistorOnB(); // cuts if in autonomous mode
 }

void fixXbeeLEDSchema() {
  // for LED2:
  // fix LED timing schema
  // if connection, solid light
  // if no connection, blink

  if(!autonomousNow && !LED2On){
    digitalWrite(LED2,HIGH);
    LED2On = true;
  }
  else if (autonomousNow && !LED2On && millis()-LED2OffStamp > LED_INTERVAL){
    digitalWrite(LED2,HIGH);
    LED2OnStamp = millis();
    LED2On = true;
  }
  else if (autonomousNow && LED2On && millis()-LED2OnStamp > LED_INTERVAL){
    digitalWrite(LED2,LOW);
    LED2OffStamp = millis();
    LED2On = false;
  }  
}

void recieveTransmission() {

          byte inputHolder[7] = {0};
          uint16_t checksumCheck = 0;
          Serial.print(F("DATA AVAILABLE!!!"));
          inputHolder[0] = xbee.read();
          inputHolder[1] = xbee.read();
          while(inputHolder[0] != 0x42 && inputHolder[1] != 0x42){ // reads data until it gets the correct start order
            inputHolder[0] = xbee.read();
            inputHolder[1] = xbee.read();
          }
          if(inputHolder[0] == 0x42 && inputHolder[1] == 0x42){ // if correct startByte and cutterTag, moves on
            Serial.print(inputHolder[0],HEX);
            Serial.print(F(" "));
            Serial.print(inputHolder[1],HEX);
            Serial.print(F(" "));
            for(int i=2; i<7; i++)
             {
             inputHolder[i] = xbee.read(); // saves each byte into byte array input
             Serial.print(inputHolder[i],HEX);
             Serial.print(F(" "));
             delay(1);
             }
          }
          Serial.println();
          memcpy(&inputPacket,&inputHolder,7); // copies input onto struct inputPacket, a readable format
       if((inputPacket.startByte != 0x42) || (inputPacket.stopByte != 0x53))
          {
            Serial.println(F("start or stop problem"));
            Serial.print(F("start: "));
            Serial.println(inputPacket.startByte,HEX);
            Serial.print(F("stop: "));
            Serial.println(inputPacket.stopByte,HEX);
            Serial.print(F("checksum: "));
            Serial.println(inputPacket.checksum);
            Serial.print(F("csc: "));
            Serial.println(checksumCheck);
            
            return false; 
          }
          if(inputPacket.command == 0x25 && inputPacket.cutterTag == 0x42)           // cut command
      cutResistorOnB();
    else if( inputPacket.command == 0x30 ){
      Serial.println(F("30"));
      counterZero++;
      counterOne = 0;
    }
    else if( inputPacket.command == 0x31){
      Serial.println(F("31"));
      counterOne++;
      counterZero = 0;
    }

    if ( counterZero >= 10 || counterOne >= 10 ){
      Serial.println(F("counter >10"));
      return false;}
    
    timeOut = 0;
    Serial.println(F("Autonomous Mode OFF"));
    autonomousNow = false;
    return true;
}
