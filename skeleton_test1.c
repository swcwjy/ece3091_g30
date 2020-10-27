

//#include <VarSpeedServo.h>

#include <TFT.h>
#include <math.h>
#include <stdio.h>
#define PI 3.1415926535897932384626433832795
#include "VarSpeedServo.h"
//#include "VarSpeedServo.cpp"

float jointAngles[3];
float endEffectorPos[3];
float proximityV;
const int L1 = 6;
const int L2 = 8;
const int L3 = 8;
const int L4 = 6;

const int colourSense = 0; // analogue input for colour sensor
const int locator = A1; // analogue input for locator
//int endEffectorPos[2]; //

// Default servo positions (angles)
const int pos1= 90;
const int pos2 = 90;
const int pos3 = 90;
const int pos4 = 90;

// servo motors
VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;


// input values from pots
int inputVal1;
int inputVal2;
int inputVal3;
int inputVal4;

void calc_FK(float the1,float the2, float the3)
{
  
  float d1,d2,d3,th5,th6;
  float rad = (PI/180);
  int L1 = 6;
  int L2 = 8;
  int L3 = 8;
  int L4 = 6;
  float th1 = the1;
  float th2 = the2;
  float th3 = the3;
  th5 = 180 - 90 - th2;
  th6 = 180 + th3 - th5;
  d1 = L2*sin(th2*rad);
  d2 = L3*cos(th6*rad);
  d3 = L2*cos(th2*rad) + L3*sin(th6*rad) + L4;

  
  endEffectorPos[2] = L1 + d1 - d2;   // z
  endEffectorPos[1] = d3*sin(th1*rad); // y
  endEffectorPos[0] = d3*cos(th1*rad); // x
  
 
}

void calc_IK(float endEffectorPos[])
{
  float d, r1, r2, r3;
  float phi1, phi2, phi3; // 
  float deg = (180/PI);
  float th2,th3;

  float x = endEffectorPos[0];
  float y = endEffectorPos[1];
  float z = endEffectorPos[2];

  d = sqrt(x*x + y*y);
  r2 = d - L4;
  r1 = z - L1;
  r3 = sqrt(r1*r1 + r2*r2);
  phi3 = acos((L2*L2 + L3*L3 - r3*r3)/(2*L2*L3));
  phi2 = asin((L3* sin(phi3))/(r3));
  phi1 = atan(r1/r2);
  
  
  // base
  ///*
  if(x<0 && y!=0){jointAngles[0] = atan(y/x)* deg + 180;}
  else if((y == 0 && x>0)|| (x ==0&&y>0)) {jointAngles[0] = 90-atan(x)* deg;}  
  else if((y == 0 && x<0) || (x ==0&&y<0)) {jointAngles[0] = atan(-x)* deg+90;}  
  else if((y==0 && x==0) || x==0) {jointAngles[0] = 90;}
  else jointAngles[0] = atan(y/x)* deg;
  //*/

  
  
  th2 = (phi1 + phi2) * deg;
  th3 = (phi3)*deg;

  
   //shoulder and elbow
  jointAngles[1] =   phi2*deg+(phi3 *deg) -75;//th2-75;//
  jointAngles[2] =   phi3 *deg + 25;//th2 +25;

  /*
  Serial.print("phi2: ");
  Serial.println(phi2*deg);
  Serial.print("phi3: ");
  Serial.println(phi3*deg);
  */
   //shoulder and elbow
  //jointAngles[2]=   phi3 *deg + 25;//th2 +25;
  //jointAngles[1] = 25+ phi3 *deg + phi2*deg -75;//th2-75;
}

void move_IK(float x, float y, float z)
{
  endEffectorPos[0] = x;
  endEffectorPos[1] = y; 
  endEffectorPos[2] = z; 
  calc_IK(endEffectorPos);
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  
}

void move_angle(float theta1,float theta2,float theta3)
{
  jointAngles[0] = theta1; //th1
  jointAngles[1] = theta2; //th2
  jointAngles[2] = theta3; //th3
  
  servo1.slowmove(jointAngles[0],20);
  servo2.slowmove(jointAngles[1],20);
  servo3.slowmove(jointAngles[2],20);
  
  /*
  Serial.println(endEffectorPos[0]);
  Serial.println(endEffectorPos[1]);
  Serial.println(endEffectorPos[2]);
  */
}

void proximity()
{
  int proximityADC = analogRead(locator);
  proximityV = (float)proximityADC * 5.0 / 1023.0;
  Serial.print("proximityV: ");
  Serial.println(proximityV);
  delay(100);
}

void to_colourSense_pos()
{
  move_IK(6,16,4);
  
}

void setup()
{
  //Code here:
  Serial.begin(9600);
  // set up motors 
  servo1.attach(9); // base motor output
  servo2.attach(8); // shoulder motor output
  servo3.attach(7); // elbow motor output
  servo4.attach(6); //  claw motor output
    
  //Serial.println(endEffectorPos[0]);
  //Serial.println(endEffectorPos[1]);
  //Serial.println(endEffectorPos[2]);

  // code for location
  pinMode(locator,INPUT);
  
  // code for colour sensor
   // resistance of LDR, analogue output
  pinMode(colourSense,OUTPUT);
  
}

void loop()
{
// Fully extended arm Angle[x,100,110]
  
  int crnt_pos1, crnt_pos2, crnt_pos3, crnt_pos4; // current position
  float ldr_output;
  int x,y,z,i,j,theta1,theta2,theta3;
  float r_reflect,g_reflect,b_reflect;

  
 
  
  // COLOUR SENSOR
  // when the arm is above colour sensor

  
  // flash each LED individually and record values reflected


  // compare values and go to respective location
  
  /*
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
  */

  // colour sensor location
  
 
  // LOCATION DETECTION
  
  // Read in the ADC and convert it to a voltage:
  ///*

  // problem is need to be constantly checking for <1, void loop doesn't allow
  // need function constantly checking and passing value of proximityV to be detected
 
  
  // if promixityV < 1 then stop and continue to colour sensor
  
  //*/
  
  // Limit switches size detection
  // if switch pressed
  /*
  if(digitalRead(10)==LOW){
    digitalWrite(13,HIGH);
  }else{
    digitalWrite(13,LOW);
  }
  */

  // PANNING WORKSPACE - DEFAULT STATE
  //
  ///*
  for(theta1 = -70;theta1<0;theta1++)
  {
    move_angle(90-theta1,90-75,90+25);
    calc_FK(theta1,90,90);
    Serial.print("x: ");
    Serial.println(endEffectorPos[0]);
    Serial.print("y: ");
  Serial.println(endEffectorPos[1]);
  Serial.print("z: ");
  Serial.println(endEffectorPos[2]);
    delay(200);
    //proximity();
    //if(proximityV<1){break;}
  }
  //delay(500);
  //*/
  
  /*
  

  // if nothing detected keep on panning
  
  for( x = -5;x<5;x++)
  {
   
    move_IK(x,16,0);

    delay(400);
  }
  
  for( x = 5;x>-5;x--)
  {
   
    move_IK(x,16,0);
    delay(400);
  }
  proximity();
  
  
  
  Serial.print("Base angle: ");
  Serial.println(jointAngles[0]);
  Serial.print("Shoulder angle: ");
  Serial.println(jointAngles[1]);
  Serial.print("Elbow angle: ");
  Serial.println(jointAngles[2]);
  
  */
  

  // Lab2 Forward kinematics b) (panning around workspace)
  // enter angles
  /*
  jointAngles[0] = -70; //th1
  jointAngles[1] = 15; //th2
  jointAngles[2] = 45; //th3
  
  calc_FK(jointAngles);
  
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(6000);
  
  jointAngles[0] = 70; //th1
  jointAngles[1] = 15; //th2
  jointAngles[2] = 45; //th3
  
  calc_FK(jointAngles);
  
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(6000);

  Serial.println(endEffectorPos[0]);
  Serial.println(endEffectorPos[1]);
  Serial.println(endEffectorPos[2]);
  */
  
  // Draw an arc
  /*
    servo4.slowmove(0,20);
  servo1.slowmove(70+90,10);
  servo2.slowmove(90-75,10);
  servo3.slowmove(90-15,10);
  delay(6000);
  servo1.slowmove(90-70,10);
  servo2.slowmove(90-75,10);
  servo3.slowmove(90-15,10);
  delay(6000);
  */
  
  // drawing a horizontal line
  /*
  servo4.slowmove(0,20);
 
  for(int x = -5; x<5; x ++)
  {
    move_IK(x,5,8);

  }
  
  for(int x2 = 5; x2>-5;x2 --)
  {
    move_IK(x2,5,8);
 
 
  }
  */

 
  
  // drawing a square
  /*
    servo4.slowmove(0,20);
    
  for(int x = -3; x<3; x ++)
  {
    move_IK(x,5,8);
  
  }
  delay(300);
  for(int y = 5; y<12; y ++)
  {
    move_IK(3,y,8);
 
  }
  delay(300);
  for(int x2 = 3; x2>-3;x2 --)
  {
    move_IK(x2,12,8);
  
  }
  delay(300);
  
  for(int y2 = 12; y2>5;y2 --)
  {
    move_IK(-3,y2,8);
 
  }
  delay(300);
  */
 
  // return to default
  /*
  servo1.slowmove(pos1,10);
  servo2.slowmove(pos2,10);
  servo3.slowmove(pos3,10);
  delay(7000);
  
  */

  
  
  // CONTROL SHOULDER
  ///*
  
  
  //delay(4000);
  
  //delay(4000);

 //*/
  
 

  // CONTROL ELBOW
  ///*
  
  
  //delay(4000);
  
  //delay(4000);
  
  //servo2.slowmove(jointAngles[2],50);
  //*/

  // control claw
  /*
 
  servo4.slowmove(0,20);
  delay(4000);
  //servo4.slowmove(pos4,10);
  //delay(4000);
  */
  /*
   for(pos4=90;pos4>=5;pos4-=1){
    servo4.slowmove(pos4,80);
    delay(15);
  }
  for (pos4 = 5; pos4 <= 90; pos4 += 1) {
    // tell servo to go to position in variable 'pos'
    servo4.slowmove(pos4,80);
    // wait 15 ms for servo to reach the position
    delay(15); // Wait for 15 millisecond(s)
  }
  */
  }
