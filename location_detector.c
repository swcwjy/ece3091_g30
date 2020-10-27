// within the void loop



for(int x = -10; x<0; x ++)
  {

    proximity();
    //Serial.println(jointAngles[0]);
    //Serial.println(jointAngles[1]);
    //Serial.println(jointAngles[2]);

    // bend down pick up block
    ///*
    if(proximityV<2)
    {
      //delay(5000);
      Serial.print("STOP");
      servo4.slowmove(90,20);  // open claw
      delay(5000);
      move_IK(x,14,4);   // move downwards toward object
      delay(5000);
      
      servo4.slowmove(0,20);   // close claw and hopefully grab object
      delay(5000);

      // Move to colour sensor + edge detector
     }
     
    delay(500);
    move_IK(x,14,6);
    
    
  }
