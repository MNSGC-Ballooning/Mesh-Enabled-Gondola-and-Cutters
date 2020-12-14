// Resistor Cutter functions

void cutResistorOnA() {   // char argument denotes what cutter is being cut
    if(!cutterOnA){
      digitalWrite(CUTTER_PIN1,HIGH);
      digitalWrite(CUTTER_PIN3,HIGH);
      sensorHeatRelay.setState(false);
      heatStatus = 0x00;
      cutStatusA = 0x02;
      cutterOnA = true;
      Serial.println(F("Cutting"));
      cutStampA = millis();
    }
}


void cutResistorOffA() {
    if(cutterOnA)
    {
      digitalWrite(CUTTER_PIN1,LOW);
      digitalWrite(CUTTER_PIN3,LOW);
      cutterOnA = false;
      Serial.println(F("Shutting cutter off"));
    }
}
