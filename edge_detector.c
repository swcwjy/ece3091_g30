// Edge detector to determine shape of object


const int locator = A1; // analogue input for locator

//within void setup 

pinMode(locator,INPUT);
  
  
//within void loop
int proximityADC = analogRead(locator);
  float proximityV = (float)proximityADC * 5.0 / 1023.0;
  Serial.println(proximityV);
  delay(100);
