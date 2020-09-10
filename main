// Main Arduino code for robot

#include <Servo.h>

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int i;
int angle1, angle2, angle3;

Servo servo_9;
Servo servo_10;
Servo servo_11;

void setup()
{
  Serial.begin(9600);
  
  servo_9.attach(9);
  servo_10.attach(10);
  servo_11.attach(11);
  	

}

void loop()
{
  // Enter 3 angles corresponding to the 3 joints of 
  // the 3DOF Robot Arm
  //for(i = 0;i<3;i++)
  //
  Serial.println(" Please enter angle 1: ");// %d for Joint %d: \n",i,i);// + i + " for Joint " + i + ": \n");
  while(!Serial.available()){}
    angle1 = Serial.read();
  
  
   //check if angles are within range
  while(angle1 >180 || angle1<0)
  {
    Serial.println("Invalid angle, enter between 0-180: ");
    while(!Serial.available()) {}
      angle1 = Serial.read();
      
  }
  
  
  
  // Input angle for joint 2t
  Serial.println(" Please enter angle 2: ");// %d for Joint %d: \n",i,i);// + i + " for Joint " + i + ": \n");
      while(!Serial.available()){}
        angle2 = Serial.read();
   
  //check if angles are within range
  while(angle2 >180 || angle2<0)
  {
    Serial.println("Invalid angle, enter between 0-180: ");
    while(!Serial.available()) {}
      angle2 = Serial.read();
    
  }
  
  
  
  // Input angle for joint 3
  Serial.println(" Please enter angle 3: ");// %d for Joint %d: \n",i,i);// + i + " for Joint " + i + ": \n");
      while(!Serial.available()){}
  		angle3= Serial.read();
    
   //check if angles are within range 
  while(angle3 >180 || angle3<0)
  {
    Serial.println("Invalid angle, enter between 0-180: ");
    while(!Serial.available()) {}
      angle3 = Serial.read();
    
  }
  
 
 
  servo_9.write(angle1);
  delay(1000);
  servo_10.write(angle2);
  delay(1000);
  servo_11.write(angle3);
  delay(1000);
  
  /*
  // sweep the servo from 0 to 180 degrees in steps
  // of 1 degrees
  for (pos1 = angleArray[1]; pos1 <= 180; pos1 += 1) {
    // tell servo to go to position in variable 'pos'
    servo_9.write(pos1);
    // wait 15 ms for servo to reach the position
    delay(15); // Wait for 15 millisecond(s)
  }
  for (pos2 = angleArray[2]; pos2 <= 180; pos2 += 1) {
    // tell servo to go to position in variable 'pos'
    servo_10.write(pos2);
    // wait 15 ms for servo to reach the position
    delay(15); // Wait for 15 millisecond(s)
  }
  for (pos3 = angleArray[3]; pos3 <= 180; pos2 += 1) {
    // tell servo to go to position in variable 'pos'
    servo_11.write(pos3);
    // wait 15 ms for servo to reach the position
    delay(15); // Wait for 15 millisecond(s)
  }
  */
  
}
