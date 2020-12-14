// Resistor Cutter functions

void cutResistorOnB() {   // char argument denotes what cutter is being cut
    if(!cutterOnB){
      digitalWrite(CUTTER_PIN1,HIGH);
      digitalWrite(CUTTER_PIN3,HIGH);
      sensorHeatRelay.setState(false);
      heatStatus = 0x00;
      cutStatusB = 0x02;
      cutterOnB = true;
      Serial.println(F("Cutting"));
      cutStampB = millis();
    }
}


void cutResistorOffB() {
    if(cutterOnB)
    {
      digitalWrite(CUTTER_PIN1,LOW);
      digitalWrite(CUTTER_PIN3,LOW);
      cutterOnB = false;
      Serial.println(F("Shutting cutter off"));
    }
}
