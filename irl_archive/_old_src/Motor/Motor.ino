
/*

This code is the Arbotix-M Robocontroller Servo Controller Node.
It is designed to use an external power supply to power the servo motor. 
It merely utilizes the ax12 library to control the MX-64 Servo using
SetPosition to turn the motor to a specified integer. It will also
use RosSerial as a Subscriber to take inputs from ROS to move the Servo.
*/


#include <ax12.h>
#include <BioloidController.h>



void setup(){
  
  delay(1000);
  SetPosition(1,2050);
  delay(3000);
  
}

void loop(){
  SetPosition(1,random(100,2000));
  delay(2000);
  SetPosition(1, random(3000,4000));
  delay(2000);
}  

