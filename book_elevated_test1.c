

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
int light, i, cube_size;
int count_loop = 0;
int r = 0;
int g = 0;
int b = 0;
int reset = 0;
int flag = 0;
float prox = 3.5;
int prox_on_off = 0;
const int L1 = 6;
const int L2 = 8;
const int L3 = 8;
const int L4 = 6;

const int colourSense = 0; // analogue input for colour sensor
const int locator = A1; // analogue input for locator
const int Lswitch = 2;

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

void calc_IK(float x1,float y1,float z1)
{
  float d, r1, r2, r3;
  float phi1, phi2, phi3; // 
  float deg = (180/PI);
  float th2,th3;

  float x = x1;
  float y = y1;
  float z = z1;

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
  th3 =  (phi3)*deg ;


  // trying another way of IK
  
  /*
  r3 = (x*x + y*y + pow(z-L1,2)-L2*L2-L3*L3)/(2*L2*L3);
  d = sqrt(1-r3*r3);
  jointAngles[1] = atan2(z-L1,sqrt(x*x+y*y))-atan2(L3*d,L2+L3*r3)-75;
  jointAngles[2] = atan2(d,r3)+25;
  */
  
  //if (th2<90){jointAngles[1] =   th2 +90-75;}
  //else if(th2>90){jointAngles[1] =   180+th2 -75;}
  jointAngles[1] = th2 -75;
  if (jointAngles[1] < 0){jointAngles[1] = 90+jointAngles[1];}
  if (jointAngles[1] > 180){jointAngles[1] = jointAngles[1]-90;}
  //-75+90;//th2+th3-75-90;//th2 + (th3+35)-180 - 75;//
  
  jointAngles[2] =  th3 + 35 - (90 - th2);
  //   jointAngles[2] =   - jointAngles[1] + th3 +25; //th2 +35;
  //  if (jointAngles[2] >180 ){jointAngles[2] = jointAngles[2]-90;}
  //  if (jointAngles[2] <0 ){jointAngles[2] = jointAngles[2]+90;}

  Serial.print("joint1: ");
  Serial.println(jointAngles[1]);
  Serial.print("joint2: ");
  Serial.println(jointAngles[2]);
  ///*
  Serial.print("phi1: ");
  Serial.println(phi1*deg);
  Serial.print("phi2: ");
  Serial.println(phi2*deg);
  Serial.print("phi3: ");
  Serial.println(phi3*deg);
  Serial.print("th2: ");
  Serial.println(th2);
  Serial.print("th3: ");
  Serial.println(th3);
  //*/
}

void move_IK(float x, float y, float z)
{
  calc_IK(x,y,z);
  servo1.slowmove(jointAngles[0],9);
  servo2.slowmove(jointAngles[1],9);
  servo3.slowmove(jointAngles[2],9);
  delay(200);
}

void move_angle(float theta1,float theta2,float theta3)
{
  if(theta1 <0){ theta1 = 90-theta1;}
  theta2 = theta2 - 75;
  theta3 = theta3 + 25;
  servo1.slowmove(theta1,10);
  servo2.slowmove(theta2,10);
  servo3.slowmove(theta3,10);
  delay(200);
  ///*
  Serial.println(theta1);
  Serial.println(theta2);
  Serial.println(theta3);
  //*/
}

void panning_height_top()
{
  servo2.slowmove(90-75+90,10);
  servo3.slowmove(52+25,10);
  delay(500);
}
void panning_height_middle()
{
  servo2.slowmove(78-75+90,10);
  servo3.slowmove(40+25,10);
  delay(500);
}
void panning_height_bottom()
{
  servo2.slowmove(65-75+90,10);
  servo3.slowmove(10+25,10);
  delay(500);
}

void move_up()
{
  servo2.slowmove(90+45-75,10);
  servo3.slowmove(90+45+25,10);
  delay(500);
}

void move_down_top() // y = 20, z just above cube
{
  servo2.slowmove(95-75+90,10);
  servo3.slowmove(50+25,10);
  delay(500);
}
void move_down_middle() // y = 20, z just above cube
{
  servo2.slowmove(89-75+90,10);
  servo3.slowmove(45+25,10);
  delay(500);
}
void move_down_bottom() // y = 20, z just above cube
{
  servo2.slowmove(72-75+90,10);
  servo3.slowmove(4+25,10);
  delay(500);
}

void rotate_base(float x_angle)
{
  if(x_angle <0){ x_angle = 90-x_angle;}
  servo1.slowmove(x_angle,10);
}

void proximity()
{
  
  proximityV = (float)analogRead(locator) * 5.0 / 1023.0;
  Serial.print("proximityV: ");
  Serial.println(proximityV);
  //delay(100);
}

void colour_sensor_pos()
{
  // hardcode position of colour sensor
  move_IK(0,14,10);
  
}

void container_pos(int r,int g, int b,int cube_size)
{
  if(cube_size ==1 && r ==1)
  {
    servo1.slowmove(70,10);
    servo2.slowmove(88-75+90,10);
    servo3.slowmove(45+25,10);
  }
  if(cube_size ==0 && r ==1)
  {
    servo1.slowmove(70,10);
    servo2.slowmove(88-75+90,10);
    servo3.slowmove(45+25,10);
  }
  if(cube_size ==1 && g ==1)
  {
    servo1.slowmove(70,10);
    servo2.slowmove(88-75+90,10);
    servo3.slowmove(45+25,10);
  }
  if(cube_size ==0 && g ==1)
  {
    servo1.slowmove(70,10);
    servo2.slowmove(88-75+90,10);
    servo3.slowmove(45+25,10);
  }
  if(cube_size ==1 && b ==1)
  {
    servo1.slowmove(70,10);
    servo2.slowmove(88-75+90,10);
    servo3.slowmove(45+25,10);
  }
  if(cube_size ==0 && b ==1)
  {
    servo1.slowmove(70,10);
    servo2.slowmove(88-75+90,10);
    servo3.slowmove(45+25,10);
  }
  reset = 1;
}

void cube_size_detect()
{
  if( (digitalRead(Lswitch) == LOW) && (flag == 0) ) 
  {
    Serial.println("door is closed"); 
    flag = 1; 
    delay(20); 
  }
  
    if( (digitalRead(Lswitch) == HIGH) && (flag == 1) ) 
  {
    Serial.println("door is opened"); 
    flag = 0;
    delay(20); 
  }
  
  
}

void colour_sense()
{
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  delay(500);

  while(light<30){delay(500);light = analogRead(A5);Serial.println(light);}
  delay(1000);
  light = analogRead(A5);
  Serial.println(light);
  //blue = ~50 - 70 , 4
  //green =~150-170 , 3
  //red = ~100-120 , 2
  // nothing < 30
  
  switch(light)// flash each LED individually and record values reflected
  {
    // values below for low light conditions
    case 50 ... 70: //blue
    digitalWrite(2,LOW);
    digitalWrite(3,LOW);
    digitalWrite(4,HIGH);  
    while(light>50){light = analogRead(A5);Serial.println(light); delay(500);}
    break;
    
    case 120 ... 170: //green
    digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    digitalWrite(4,LOW);  
    while(light>50){light = analogRead(A5);Serial.println(light); delay(500);}
    break;
    break;
    
    case 90 ... 110: //red
     digitalWrite(2,HIGH);
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);  
    while(light>40){light = analogRead(A5); Serial.println(light);delay(500);}
    break;
    
    
    default:  
    break;
  }
  // compare values and go to respective location
  
  
  //calibrate
  //digitalWrite(2,HIGH);
  //digitalWrite(3,HIGH);
  //digitalWrite(4,HIGH);
  //delay(500);
  //light = analogRead(A5);
  //Serial.println(light);
  
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
  
  pinMode(locator,INPUT);
  pinMode(colourSense,OUTPUT);
  pinMode(Lswitch, INPUT);

  // JA1 = 15 -> th2 = 90
  // JA2 = 125 -> th3 = 90
  /*
  jointAngles[0] = 90; //th1
  jointAngles[1] = 90; //th2
  jointAngles[2] = 90; //th3
  
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(1000);
  
  move_IK(-7,15,13);
  delay(3000);
  move_IK(7,14,9);
  */
}

void loop()
{
  // Fully extended arm Angle[x,100,110]

  // BASE ELEVATED 2.3CM, CUBES ELEVATED 1CM
  
  int crnt_pos1, crnt_pos2, crnt_pos3, crnt_pos4; // current position
  float ldr_output;
  int x,y,z,i,j,theta1,theta2,theta3;
  float r_reflect,g_reflect,b_reflect;
  int pick_up = 0;
  

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

  
  servo4.slowmove(4,20);
  delay(1000);
  for(int x = -70; x<0; x = x+2)
  {
      
      panning_height_bottom();
      proximity();
      delay(100);
      // bend down pick up block
      if(proximityV<prox && prox_on_off ==1)
      {
        servo1.slowmove(90-(x),10);
        delay(2000);
        servo1.slowmove(90-(x-8),10);
        delay(2000);
        servo4.slowmove(80,20);  // open claw
        delay(2000);
        move_down_bottom();   // move downwards toward object (roughly z = 0, object on same plane as base of robot)
        delay(2000);
        servo4.slowmove(4,20);   // close claw and hopefully grab object
        delay(2000);
        move_up();
        pick_up = 1;
        prox_on_off = 0;
        break;
      }
      rotate_base(x);
      delay(100);
      prox_on_off =1;
    }
  prox_on_off = 0;
  if(pick_up != 1)
  {
    servo4.slowmove(4,20);
    for(int x = -70; x<0; x = x+2)
    {
      
      panning_height_middle();
      proximity();
      delay(100);
      // bend down pick up block
      if(proximityV<prox&& prox_on_off ==1)
      {
        servo1.slowmove(90-(x),10);
        delay(2000);
        servo1.slowmove(90-(x-8),10);
        delay(2000);
        servo4.slowmove(80,20);  // open claw
        delay(2000);
        move_down_middle();   // move downwards toward object (roughly z = 0, object on same plane as base of robot)
        delay(2000);
        servo4.slowmove(4,20);   // close claw and hopefully grab object
        delay(2000);
        move_up();
        pick_up = 1;
        prox_on_off = 0;
        break;
      }
      rotate_base(x);
      delay(100);
      prox_on_off =1;
     } 
  }
  prox_on_off = 0;
  if(pick_up != 1)
  {
      servo4.slowmove(4,20);
     for(int x = -70; x<0; x = x+2)
    {
      
      panning_height_top();
      proximity();
      delay(100);
      // bend down pick up block
      if(proximityV<prox&& prox_on_off ==1)
      {
        servo1.slowmove(90-(x),10);
        delay(2000);
        servo1.slowmove(90-(x-8),10);
        delay(2000);
        servo4.slowmove(80,20);  // open claw
        delay(2000);
        move_down_top();   // move downwards toward object (roughly z = 0, object on same plane as base of robot)
        delay(2000);
        servo4.slowmove(4,20);   // close claw and hopefully grab object
        delay(2000);
        move_up();
        pick_up = 1;
        prox_on_off = 0;
        break;
      }
      rotate_base(x);
      delay(100);
      prox_on_off =1;
     } 
  }
    

   if(pick_up ==1)
  {
    // Move to colour sensor + edge detector
    colour_sensor_pos();
  
    //container_pos();
    delay(4000);
    servo4.slowmove(90,20);  // open claw
    delay(2000);
    pick_up = 0;
  }
    
  
  
  //*/
  //move_IK(x,1,14);
  
  
  //move_up();
  //delay(4000);
  /*
  for(int x2 = -10; x2<0;x2 ++)
  {
    move_IK(x2,14,14);
    
    delay(500);
  }
  delay(500);
  for(int x3 = 0; x3<-10;x3 ++)
  {
    move_IK(x3,14,14);
    delay(500);
    
  }
  delay(500);
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

 
  }
