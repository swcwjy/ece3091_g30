// Arduino code for Colour Sensing

//within void setup 
// code for colour sensor
   // resistance of LDR, analogue output
  pinMode(colourSense,OUTPUT);
  
  
//within void loop
ldr_output = analogRead(A0);
  ldr_output = ldr_output*5/1023;
  if(ldr_output>3.9 && ldr_output<=4)
  {
    Serial.println("red: ");
    Serial.println(ldr_output);
    };
  if(ldr_output>4.25 && ldr_output<=4.35)
  {
    Serial.println("blue: ");
    Serial.println(ldr_output);
  };
  delay(100);
