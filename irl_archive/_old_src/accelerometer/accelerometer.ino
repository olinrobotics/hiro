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
#include <std_msgs/Int16.h>
#include <math.h>

ros::NodeHandle edwin_head;

std_msgs::String str_msg;
ros::Publisher accel("edwin_imu", &str_msg);


std_msgs:: Int16 stuff_msg;
ros::Publisher test("testing", &stuff_msg);


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


//Total Difference
int totaldist = 0;

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


void setup(){
  edwin_head.getHardware() -> setBaud(9600);
  edwin_head.initNode();
  edwin_head.advertise(accel);
  edwin_head.advertise(test);


  pinMode(pointZ, INPUT);
  pinMode(pointY, INPUT);
  pinMode(pointX, INPUT);


}

void loop(){
  

    if(i < 5){

        old_x[i] = analogRead(pointX);
        old_y[i] = analogRead(pointY);
        old_z[i] = analogRead(pointZ);

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
      
      totaldist = sqrt(sq(diff_x) + sq(diff_y) + sq(diff_z));
      
      stuff_msg.data = diff_x;
      test.publish( &stuff_msg);
      stuff_msg.data = diff_y;
      test.publish( &stuff_msg);
      stuff_msg.data = diff_z;
      test.publish( &stuff_msg);
      stuff_msg.data = totaldist;
      test.publish( &stuff_msg);

      if(totaldist > 30 && totaldist <= 100){

        str_msg.data = "IMU: pat";
        accel.publish( &str_msg );

        for(i = 0; i < 5; i++){
          old_x[i] = 0;
          old_y[i] = 0;
          old_z[i] = 0;
        }
        
        i = 0;
        delay(200);


      }
      else if(totaldist > 100){

        str_msg.data = "IMU: slap" ;
        accel.publish( &str_msg );

        for(i = 0; i < 5; i++){
          old_x[i] = 0;
          old_y[i] = 0;
          old_z[i] = 0;
        }
        

        i = 0;

        delay(200);


      }


      if(oldest >= 5){
        oldest = 0;
      }
      
      
      old_x[oldest] = current_x;
      old_y[oldest] = current_y;
      old_z[oldest] = current_z;
      oldest++;



    }
    
    last_avg_x = 0;
    last_avg_y = 0;
    last_avg_z = 0;
    
   edwin_head.spinOnce();
   delay(200);

}
