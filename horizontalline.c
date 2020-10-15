// Drawing a horizontal line
  ///*
  //servo4.slowmove(0,20);
 
  for(int x = -5; x<5; x ++)
  {
    endEffectorPos[0] = x;
    endEffectorPos[1] = 12; //z
  endEffectorPos[2] = 7; //y
  calc_IK(endEffectorPos);
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  
  delay(300);
  }
  
  for(int x2 = 5; x2>-5;x2 --)
  {
    endEffectorPos[0] = x2;
    endEffectorPos[1] = 12; //z
  endEffectorPos[2] = 7; //y
  calc_IK(endEffectorPos);
 
  servo1.slowmove(jointAngles[0],10);
  servo2.slowmove(jointAngles[1],10);
  servo3.slowmove(jointAngles[2],10);
  delay(300);
  }
  //*/
