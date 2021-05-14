void updateTemperatures() {
 
  t1 = analogRead(THERMISTOR_A);                           // All of these calculations have to do with the Steinhart-Hart equations and setting them up properly

  t1 = log(((ADC_MAX/t1)-1)*CONST_R);

  t1 = CONST_A+CONST_B*t1+CONST_C*t1*t1*t1;
;
  t1 = 1/t1-C2K;                                                  // The final temperature in Celsius

}


bool checkTempReading(float temp) {
  // function to check if a temp sensor's recorded temperatures is valid
  return !(temp < -120); //-127 degrees C designates a temp sensor issue, so check for that (with some room for error)
}


bool tempCheck() {
  // return true if temperatures are at critical levels

  if(checkTempReading(t1)){
    // if temp sensor is working
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


  if(!checkTempReading(t1)){
    // temp sensor isn't working
    return false;
  }

}
