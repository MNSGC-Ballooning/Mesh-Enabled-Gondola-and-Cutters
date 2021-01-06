void xbeeStartup()  {                               //Sets up the gondola to mesh with or create a network, and begins a catalog of radio contacts. Should be called during main setup()
  xbeeSerial.begin(XBEE_BAUD);
  xbee.init(xbeeChannel);
  xbee.enterATmode();
  xbee.atCommand("ATDL1");
  xbee.atCommand("ATMY0");
  xbee.exitATmode();

  xbee.write(0x00); //sends out ping looking for new contacts, in this case interested in other gondolas
  delay(1000);      //waits for a response
  updateXbee();     //logs responses and other radio traffic
  
  byte newName = 0x10;
  byte addName = 0x00;
  while(checkRegister(newName + addName)) {
    addName++;
    if(addName == 0x00) clearRegister();
  }
  gondolaIdentifier = newName + addName;
  radioEvents += "Gondola named " + String(gondolaIdentifier) + ",";
}

void sendMeshData(byte *meshdata) {                 //Sends data to both A and B cutters. meshadata should be a pointer to an array of size CDU_TX_SIZE that holds data to be sent
  sendMeshData(meshdata, false);
  sendMeshData(meshdata, true);
}
bool sendMeshData(byte *meshdata, bool aorb) {      //Sends data to A (aorb=0) or B (aorb=1) cutters. meshadata should be a pointer to an array of size CDU_TX_SIZE that holds data to be sent
  bool goodcheck = true;                                   //Returns true if data is sent, false if cutter connection not yet established.
  if(!aorb) {
    if(connectedA != 0x00)  {
      xbee.write(0x02);
      xbee.write(connectedA);
      for(int i=0;i<CDU_TX_SIZE;i++)  {
        xbee.write(*(meshdata+i));
      }
    }
    else goodcheck = false;
  }
  else {
    if(connectedB != 0x00)  {
      xbee.write(0x02);
      xbee.write(connectedB);
      for(int i=0;i<CDU_TX_SIZE;i++)  {
        xbee.write(*(meshdata+i));
      }
    }
    else goodcheck = false;
  }
  //for(int i=0; i<CDU_TX_SIZE; i++) {
//    Serial.println(" ");
//    Serial.println(meshdata[i]);
//    Serial.print(" ");
  //}
//  delay(1);
  return goodcheck;
}

void updateXbee() {                                 //The main function to call during loop() after code integration. This manages incoming radio traffic and contacts
  if(!keepRadioEvents) radioEvents = "";
  while(xbee.available()>0)  {
    byte typebyte = xbee.read();
    switch(typebyte)  {
      case(0x00): { //ping command asking for all connected units to send identifiers
        xbee.write(0x01);
        xbee.write(gondolaIdentifier);
        break;
      }
      case(0x01): { //response to ping command
        byte temp_name = xbee.read();
        if(searching && temp_name == 0x01 && (millis()/1000.0) - downtimeA > 3)sendName(false);
        else if(searching && temp_name == 0x02 && (millis()/1000.0) - downtimeB > 3)sendName(true);
        else checkRegister(temp_name);
        break;
      }
      case(0x02): break;; //packet from a gondola, ignored since this is a gondola
      case(0x03): { //packet from a cutter A, check if it's the A that this gondola is connected to, if it is it records the data
        if(xbee.read() == connectedA) {
          for(int i=0;i<CDU_RX_SIZE;i++)  {
            recoveredDataA[i]=xbee.read();
            delay(1);
          }
          downtimeA = millis()/1000.0;
        }
        else {
          for(int i=0;i<CDU_RX_SIZE;i++)  {
            xbee.read();
            delay(1);
          }
        }
        break;
      }
      case(0x04): { //packet from a cutter B, check if it's the B that this gondola is connected to, if it is it records the data
        if(xbee.read() == connectedB) {
          for(int i=0;i<CDU_RX_SIZE;i++)  {
            recoveredDataB[i]=xbee.read();
            delay(1);
          }
          downtimeB = millis()/1000.0;
        }
        else {
          for(int i=0;i<CDU_RX_SIZE;i++)  {
            xbee.read();
            delay(1);
          }
        }
        break;
      }
      case(0x05): break;//request to assign a cutter A to a gondola
      case(0x06): break;//request to assign a cutter B to a gondola
      default:{
        radioEvents += "Unknown data recieved";
        while(xbee.available()>0) xbee.read();
      }
    }
  }
  
  if((connectedA == 0x00 || connectedB == 0x00) && millis()-pingcool>500)  {
    pingcool = millis();
    searching = true;
    xbee.write(0x00); //sends out ping looking for unassigned XBees
  }
  if(((millis()/1000.0)-downtimeA)>waitForReconnection && connectedA != 0x00){
    radioEvents += "Connection lost with " + String(connectedA) + " at "+ String(millis()) + ",";
    connectedA = 0x00;
    searching = true;
  }
  else if(((millis()/1000.0)-downtimeB)>waitForReconnection && connectedB != 0x00){
    radioEvents += "Connection lost with " + String(connectedB) + " at "+ String(millis()) + ",";
    connectedB = 0x00;
    searching = true;
  }
  else if(connectedA != 0x00 && connectedB != 0x00) searching = false;
}

void sendName(bool aorb) {                          //Assigns a free name to an about-to-be-assigned contact. aorb is false for cutter A, true for cutter B
  byte newName;
  if(!aorb) newName = 0x30;
  else newName = 0x50;
  byte addName = 0x00;
  while(checkRegister(newName + addName)) {
    addName++;
    if(addName == 0x00) clearRegister();
  }
  if(!aorb){
    xbee.write(0x05);
    xbee.write(newName+addName);
    connectedA = newName+addName;
    radioEvents += "New Connection to A unit " + String(connectedA) + ",";
    downtimeA = millis()/1000.0;
  }
  else {
    xbee.write(0x06);
    xbee.write(newName+addName);
    connectedB = newName+addName;
    radioEvents += "New Connection to B unit " + String(connectedB) + ",";
    downtimeB = millis()/1000.0;
  }
}

void clearRegister()  {                             //Deletes all stored contacts. Called in event that contact storage is filled and no new names can be found
  delete[] knownIDs;
  contactsHeld = 0;
}

bool checkRegister(byte incoming) {                 //Checks an incoming ID against the known IDs; If ID is unknown, it is added to list. Returns true if ID is known.
  bool known = false;
  for(int i=0;i<contactsHeld;i++) {
    if(incoming==*(knownIDs+i)) known = true;
  }
  if(!known) addID(incoming);
  return(known);
}

void addID(byte newID)  {                           //Dynamically enlarges knownID array to incorporate a new XBee contact.
                                                    //This function is automatically called when a new contact is identified. There should be no need to call this function while incorporating
                                                      //this function set into a full code base
  byte * tempArray = new byte[contactsHeld + 1];
  for(int i=0;i<contactsHeld;i++) {
    *(tempArray+i) = *(knownIDs+i);
  }
  *(tempArray+contactsHeld) = newID;
  delete[] knownIDs;
  knownIDs = tempArray;

  contactsHeld ++;
  if(contactsHeld > tooManyContacts) clearRegister();
}
