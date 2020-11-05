 

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
float prox = 2;
int light,i;
int cubesize = 0;
int count_loop = 0;
int pick_up = 0;
int r = 0;
int g = 0;
int b = 0;
int reset = 0;
int flag = 0;
int colour_determine = 0;
int prox_on_off = 0;
int claw_close = 14;
  int claw_open = 80;

const int L1 = 6;
const int L2 = 8;
const int L3 = 8;
const int L4 = 6;

const int colourSense = 0; // analogue input for colour sensor
const int locator = A1; // analogue input for locator
const int Lswitch = 2; // analogue input for limit switch

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
  /*
  Serial.println(theta1);
  Serial.println(theta2);
  Serial.println(theta3);
  */
}

void panning_height_top()
{
  servo2.slowmove(74-75+90,10);
  servo3.slowmove(54+25,10);
  delay(300);
}
void panning_height_middle()
{
  servo2.slowmove(65-75+90,10);
  servo3.slowmove(40+25,10);
  delay(300);
}
void panning_height_bottom()
{
  servo2.slowmove(55-75+90,10);
  servo3.slowmove(13+25,10);
  delay(300);
}

void move_up()
{
  servo1.slowmove(0,10);
  servo2.slowmove(90-75,10);
  servo3.slowmove(90+45+25,10);
  delay(300);
}

void move_down_top() // y = 20, z just above cube
{
  servo2.slowmove(80-75+90,10);
  servo3.slowmove(49+25,10);
  delay(300);
}
void move_down_middle() // y = 20, z just above cube
{
  servo2.slowmove(73-75+90,10);
  servo3.slowmove(37+25,10);
  delay(300);
}
void move_down_bottom() // y = 20, z just above cube
{
  servo2.slowmove(64-75+90,10);
  servo3.slowmove(15+25,10);
  delay(300);
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

void colour_sensor_pos(int cubesize)
{
  if(cubesize ==0)// for big cube
  {
    servo1.slowmove(44,10);
  servo2.slowmove(36 -75+90,10);
  servo3.slowmove(43+25,10);
  delay(300);
  }
  else 
  servo1.slowmove(44,10);
  servo2.slowmove(36 -75+90,10);
  servo3.slowmove(44+25,10);
  delay(300);
}

void container_pos(int r,int g, int b,int cube_size)
{
  // locations need to be changed
  if(cube_size ==1 && r ==1) // small red cube
  {
    servo1.slowmove(90-20,10);
    servo2.slowmove(45 -75+90,10);
    servo3.slowmove(36+25,10);
  }
  if(cube_size ==0 && r ==1) // big red cube
  {
    servo1.slowmove(90-10,10);
    servo2.slowmove(45-75+90,10);
    servo3.slowmove(36+25,10);
  }
  if(cube_size ==1 && g ==1) // small green cube
  {
    servo1.slowmove(90-20,10);
    servo2.slowmove(50-75+90,10);
    servo3.slowmove(47+25,10);
  }
  if(cube_size ==0 && g ==1) // big green cube
  {
    servo1.slowmove(90-10,10);
    servo2.slowmove(50-75+90,10);
    servo3.slowmove(47 +25,10);
  }
  if(cube_size ==1 && b ==1) // small blue cube
  {
    servo1.slowmove(90-20,10);
    servo2.slowmove(80-75+90,10);
    servo3.slowmove(82+25,10);
  }
  if(cube_size ==0 && b ==1) // big blue cube
  {
    servo1.slowmove(90-10,10);
    servo2.slowmove(80-75+90,10);
    servo3.slowmove(82+25,10);
  }
  reset = 1;
}


void colour_sense()
{
  int val = 8; // number of light readings
  int light_arr[val],median;
  
  int samples = 0;
  digitalWrite(10,HIGH); // red
  digitalWrite(11,HIGH); // green
  digitalWrite(12,HIGH); // blue
  delay(500);
  
  light = analogRead(A5);
  while(light>900)
  {
    delay(1000);
    light = analogRead(A5);
    Serial.println(light);
    samples = 0;
  }
  delay(1000);
  
  // delaying light reading 
  for(int i  = 0 ; i<8  ; i++)
  {
  light = analogRead(A5);
  //light_arr[i] = light;
  Serial.println(light);
  delay(500);
  }

  /*
  light_arr[] = {0,0,0,0,0,0,0,0};
  for(int i  = 0 ; i<val ; i++)
  {
  light = analogRead(A5);
  light_arr[i] = light;
  Serial.println(light);
  delay(200);


    // taking median of light voltages
    median = val;
    sort(light_arr,median);
    
    median = (median + 1)/2-1;
    Serial.print("Median: ");
    Serial.println(light_arr[median]);
*/

    switch(light) // flash each LED individually and record values reflected
    {
      // values below for low light conditions
      case 200 ... 520: //blue
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
      digitalWrite(12,HIGH);  
      b = 1;
      Serial.println("blue");
      colour_determine = 1;
      break;
      
      case 521 ... 620: //red
      digitalWrite(10,HIGH);
      digitalWrite(11,LOW);
      digitalWrite(12,LOW); 
      Serial.println("red"); 
      r = 1;
      colour_determine = 1;
      break;
      
      case 621 ... 730: //green
      digitalWrite(10,LOW);
      digitalWrite(11,HIGH);
      digitalWrite(12,LOW); 
      Serial.println("green"); 
      g = 1;
      colour_determine = 1;
  
      break;
      
      default:  
      break;
    }
  
  delay(1000);

}

void default_move(int layer)
{
  for(int x = -70; x<-4; x = x+2)
  {
      if(layer == 1){panning_height_bottom();}
      if(layer ==2){panning_height_middle();}
      if(layer ==3){panning_height_top();}
      
      
      // bend down pick up block
      if(proximityV<prox && prox_on_off ==1)
      {
        servo1.slowmove(90-(x),10);
        delay(2000);
        servo1.slowmove(90-(x-9),10);
        delay(2000);
        servo4.slowmove(claw_open,15);  // open claw
        delay(2000);
        if(layer == 1){move_down_bottom();}// move downwards toward object (roughly z = 0, object on same plane as base of robot)
        if(layer ==2){move_down_middle();}
        if(layer ==3){move_down_top();}
           
        delay(2000);
        servo4.slowmove(claw_close,15);    // close claw and hopefully grab object
        delay(2000);
        if(digitalRead(2) == LOW){cubesize = 1;Serial.print("small cube");}// if switch pressed
        else cubesize = 0;
        Serial.print("big cube");
        delay(1000);
        move_up();
        pick_up = 1;
        prox_on_off = 0;
        break;
      }
      rotate_base(x);
      delay(100);
      proximity();
      delay(100);
      prox_on_off =1;
    }
}

void claw(int open_close)
{
  
  if(open_close == 0)
  {
    servo4.slowmove(claw_open,15);
  }
  else
  servo4.slowmove(claw_close,15);
}

void setup()
{

  Serial.begin(9600);

  // initialise servo motor positions
  servo1.slowmove(90,10);
  servo2.slowmove(90-75,10);
  servo3.slowmove(90+25,10);
//  servo4.slowmove(80,10);

  
  // set up motors 
  servo1.attach(9); // base motor output
  servo2.attach(8); // shoulder motor output
  servo3.attach(7); // elbow motor output
  servo4.attach(5); //  claw motor output
  
  pinMode(locator,INPUT);
  pinMode(colourSense,OUTPUT);
  pinMode(Lswitch, INPUT);
  pinMode(2,INPUT);
  

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
 
  move_angle(90,90,90);
 delay(3000);
  servo4.slowmove(10,15);
  delay(4000);
  if(digitalRead(2) == LOW){cubesize = 1;Serial.print("small cube");}// if switch pressed
        else cubesize = 0;

    colour_determine = 0;
    while(colour_determine == 0)
    {
   colour_sensor_pos(cubesize);
    delay(3000);
  colour_sense();
 
    }
move_angle(90,90,90);
  delay(3000);
container_pos(r,g,b,cubesize);
      delay(4000);
      servo4.slowmove(80,20);  // open claw
     delay(4000);
move_angle(90,90,90);
 delay(4000);

 
  // Default panning state

  /*
  servo4.slowmove(claw_close,15); 
  delay(2000);
  default_move(1);
  prox_on_off = 0;
  
  if(pick_up != 1)
  {
    servo4.slowmove(claw_close,15); 
    default_move(2);
    if(pick_up != 1)
    {
       servo4.slowmove(claw_close,15); 
       default_move(3);
    }
  }
  
  */
    
   
  // Colour sensing and cube placement
  
  /*
   if(pick_up ==1)
    {
      
    // Move to colour sensor + edge detector
    colour_determine = 0;
    r = 0;b = 0; g = 0;
    while(colour_determine == 0)
    {
      colour_sensor_pos();
      delay(3000);
      colour_sense();
  
    }
//    Serial.println("Red");
//    Serial.println(r);
//    Serial.println("Green");
//    Serial.println(g);
//    Serial.println("Blue");
//    Serial.println(b);
//    Serial.println("size");
//    Serial.println(cubesize);
//    
    move_angle(90,90,90);
    delay(3000);
    container_pos(r,g,b,cubesize);
    delay(3000);
    claw(0); // open claw
    delay(4000);
     move_angle(90,90,90);
     delay(2000);
    pick_up = 0;
    
    
  }
  */
  
  
  

 
  }
