void updateTemperatures() {
  // First, temperature
  t1 = analogRead(THERMISTOR_A);                                   // All of these calculations have to do with the Steinhart-Hart equations and setting them up properly
  t2 = analogRead(THERMISTOR_B);
  t1 = log(((ADC_MAX/t1)-1)*CONST_R);
  t2 = log(((ADC_MAX/t2)-1)*CONST_R);
  t1 = CONST_A+CONST_B*t1+CONST_C*t1*t1*t1;
  t2 = CONST_A+CONST_B*t2+CONST_C*t2*t2*t2;
  t1 = 1/t1-C2K;                                                  // The final temperatures for both transistors in Celsius
  t2 = 1/t2-C2K;
}


bool checkTempReading(float temp) {
  // function to check if a temp sensor's recorded temperatures is valid
  return !(temp < -120); //-127 degrees C designates a temp sensor issue, so check for that (with some room for error)
}


bool tempCheck() {
  // return true if temperatures are at critical levels
  // there's probably a better way to work through this logic
  if(checkTempReading(t1) && checkTempReading(t2)){
    // if both temp sensors are working
    if((t1 < MIN_TEMP) && (t2 < MIN_TEMP)) {
       cutReasonA = 0x50;
       return true;
    }
    if((t1 > MAX_TEMP) && (t2 > MAX_TEMP)) {
      cutReasonA = 0x60;
      return true;
    }
     return false ;       
  }

  if(checkTempReading(t1)){// && !checkTempReading(t2)){
    // if temp sensor 1 is working
    if(t1 < MIN_TEMP) {
       cutReasonA = 0x50;
       return true;
    }
    if(t1 > MAX_TEMP) {
      cutReasonA = 0x60;
      return true;
    }
      return false ;       
  }

  /*if(!checkTempReading(t1) && checkTempReading(t2)){
    // if temp sensor 2 is working
    if(t2 < MIN_TEMP) {
       cutReasonA = 0x50;
       return true;
    }
    if(t2 > MAX_TEMP) {
      cutReasonA = 0x60;
      return true;
    }
    return false;       
  }*/

  if(!checkTempReading(t1)){// && !checkTempReading(t2)){
    // neither temp sensor is working
    return false;
  }

}
