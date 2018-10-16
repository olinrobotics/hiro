

/*
This code is for the Arbotix-M accelerometer. It is designed 
to be powered and grounded by the Arbotix-M, and has its Xo, Yo, 
and Zo pins connected to data pins on the Arbotix-M. This program 
will then read the values and then output a boolean when the
accelerometer detects that there is a sudden surge in movement,
i.e. when I tap it. It must be able to recognize tapping
even when the accelerator moves at a constant speed.
It also utilizes RosSerial as a Publisher to tell Edwin whether it moves or was touched.
*/


#include <ros.h>
#include <std_msgs/String.h>
#include <math.h>

ros::NodeHandle edwin_head;

std_msgs::String str_msg;
ros::Publisher accel("edwin_imu", &str_msg);

//String that keeps track of whether an arm is moving, and Boolean for the loop
String arm = "";
boolean moving = false;

void inc_message( const std_msgs::String& stat){
  
  arm = stat.data;
  if(arm == 1){
    moving = true;
  }
  else if(arm  == "STOPPING"){
    moving = false;
  }
}
    

ros::Subscriber<std_msgs::String> arm_status("arm_status", &inc_message);


//Pin A5 is the Z output from the accelerometer, Pin A4 is the Y 
// output, and Pin A3 is the X output.
int pointX = 3;
int pointY = 4;
int pointZ = 5;


//Positions
int current_x = 0;
int current_y = 0;
int current_z = 0;

//Differences in positions
int diff_z = 0;
int diff_y = 0;
int diff_x = 0;

//Records of the last five numbers
int old_x[5];
int old_y[5];
int old_z[5];

//increment for keeping track of the oldest number
int oldest = 0;


// The average

int last_avg_x = 0;
int last_avg_y = 0;
int last_avg_z = 0;

//Increment for initialization of first data set
int i = 0;


//Counter to determine whether a movement if a pat or a robot movement
int counter = 0;

void setup(){
  edwin_head.getHardware() -> setBaud(9600);
  edwin_head.initNode();
  edwin_head.advertise(accel);
  edwin_head.subscribe(arm_status);
  
  
  pinMode(pointZ, INPUT);
  pinMode(pointY, INPUT);
  pinMode(pointX, INPUT);
     
  Serial.begin(9600);
  
  
}  

void loop(){

  if(moving == false){  
  
    if(i < 5){
    
      if(i == 0){
        
        old_x[0] = analogRead(pointX);
        old_y[0] = analogRead(pointY);
        old_z[0] = analogRead(pointZ);
      }
      else if(i == 1){
        
        old_x[1] = analogRead(pointX);
        old_y[1] = analogRead(pointY);
        old_z[1] = analogRead(pointZ);
      }    
      else if(i == 2){
        
        old_x[2] = analogRead(pointX);
        old_y[2] = analogRead(pointY);
        old_z[2] = analogRead(pointZ);
      }
      else if(i == 3){
        
        old_x[3] = analogRead(pointX);
        old_y[3] = analogRead(pointY);
        old_z[3] = analogRead(pointZ);;
      }
      else if(i == 4){
        
        old_x[4] = analogRead(pointX);
        old_y[4] = analogRead(pointY);
        old_z[4] = analogRead(pointZ);
      }    
      i++;
    
    }
    else{  
  
      
        
      current_x = analogRead(pointX);
      current_y = analogRead(pointY);
      current_z = analogRead(pointZ);
      
      
      for(i = 0; i < 5; i++){
        last_avg_x += old_x[i];
        last_avg_y += old_y[i];
        last_avg_z += old_z[i];
      }
      
      last_avg_x = last_avg_x/5;
      last_avg_y = last_avg_y/5;
      last_avg_z = last_avg_z/5;
        
      
      diff_z = current_z - last_avg_z;
      diff_y = current_y - last_avg_y;
      diff_x = current_x - last_avg_x;
    
      if((abs(diff_z) <= 70 && abs(diff_z) > 25) || (abs(diff_y) <= 70 && abs(diff_y) > 25) ||
      (abs(diff_x) <= 70 && abs(diff_x) > 25)){
       
        str_msg.data = "IMU: pat";
        accel.publish( &str_msg );
        
        for(i = 0; i < 5; i++){
          old_x[i] = 0;
          old_y[i] = 0;
          old_z[i] = 0;
        }
        
        i = 0;
        delay(2000);
  
        
      }
      else if((abs(diff_z) > 70) || (abs(diff_y) > 70) || (abs(diff_x) > 70)){
       
        str_msg.data = "IMU: slap";
        accel.publish( &str_msg );
        
        for(i = 0; i < 5; i++){
          old_x[i] = 0;
          old_y[i] = 0;
          old_z[i] = 0;
        }
        
        i = 0;
        
        delay(2000);
  
        
      }
      else{
        
        str_msg.data = "IMU: notouch";
        accel.publish( &str_msg );
  
      }
      
      
      edwin_head.spinOnce();
      delay(200);
  
  
      if(oldest >= 5){
        oldest = 0;
      }
      
      old_x[oldest] = current_x;
      old_y[oldest] = current_y;
      old_z[oldest] = current_z;
      oldest++;
      
      last_avg_x = 0;
      last_avg_y = 0;
      last_avg_z = 0;
    
    }  
    
  }
    
  
  
  

}  
