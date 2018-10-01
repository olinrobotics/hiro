
/*

This code is the Arbotix-M Robocontroller Servo Controller Node.
It is designed to use an external power supply to power the servo motor. 
It merely utilizes the ax12 library to control the MX-64 Servo using
SetPosition to turn the motor to a specified integer. It will also
use RosSerial as a Subscriber to take inputs from ROS to move the Servo.
*/


#include <ax12.h>
#include <BioloidController.h>

//ROS stuff
#include <ros.h>
#include <std_msgs/String.h>


ros::NodeHandle edwin_head;
String attitude = "";

void inc_message( const std_msgs::String& mood){
  
  attitude = mood.data;
  if(attitude == "happy"){
    for(int i=0; i<3;i++){
      delay(300);
      SetPosition(1, 4000);
      delay(300);
      SetPosition(1, 2050);
    } 
    delay(1000);
  }  

    
  
}


ros::Subscriber<std_msgs::String> motor("edwin_emotion", &inc_message);



void setup(){
  edwin_head.getHardware() -> setBaud(9600);
  edwin_head.initNode();
  edwin_head.subscribe(motor);
  delay(1000);
  SetPosition(1,2050);

   
}

void loop(){
  edwin_head.spinOnce();
  delay(1);
}  

