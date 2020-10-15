for(int x = -5; x<5; x ++)
  {
    endEffectorPos[0] = x;
    endEffectorPos[1] = 11; //z
  endEffectorPos[2] = 5; //y
  calc_IK(endEffectorPos);
  ;
  
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(300);
  }
  for(int y = 5; y<10; y ++)
  {
    endEffectorPos[0] = 5;
    endEffectorPos[1] = 11; //z
  endEffectorPos[2] = y; //y
  calc_IK(endEffectorPos);
 
  
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(300);
  }
  
  for(int x2 = 5; x2>-5;x2 --)
  {
    endEffectorPos[0] = x2;
    endEffectorPos[1] = 11; //z
  endEffectorPos[2] = 10; //y
  calc_IK(endEffectorPos);
  
  
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(300);
  }
  
  
  for(int y2 = 10; y2>5;y2 --)
  {
    endEffectorPos[0] = -5;
    endEffectorPos[1] = 11; //z
  endEffectorPos[2] = y2; //y
  calc_IK(endEffectorPos);
  
  
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(300);
  }
