// Basic Edwin Hind Brain Test Code
// Version 1.0  04/12/17
// Basic code to blink lights, move jaw and make sound
// Please don't break Edwin. Ask before altering motor parameters

#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <Servo.h>        //include built in Arduino servo library



// Set up Arduino Ports, Pins and Global Variables to support Robot-------------------------------------------
int ledPinR = 13;          // robot alive red blinky light pin
int ledPinG = 10;          // robot alive green blinky light pin
int ledPinB = 9;           // robot alive blue blinky light pin
int eStopPin = 5;          // create name for E-Stop reading pin
int soundOut = 6;          // create name for sound out pin
int delayPeriod = 50;     // hindbrain loop delay Note: change over to timers not delays
int sharpDistance0 = 0;    // create name for sharp ir 0 analog input pin 0
int sharpDistance1 = 1;    // create name for sharp ir 1 analog input pin 1
int speakerPin = 6;        // create name for speaker pin
int numtones = 10;         // create number of tones
int tones[] = { 261, 277, 294, 311, 330, 349, 370, 392, 415, 440 };  //stored tones
int jaw = 60;             // variable to store jaw servo position in degrees
Servo jawServo;           // create jaw servo object to control a servo
char command = 'g' ;      //'g' is go command from midbrain,'s'is stop, '90' servo setpoint  
ros::NodeHandle nh;
void grip_cb(const std_msgs::Int16& msg){
  int grip = msg.data;
  if (grip == 1){
    jawServo.write(128);
  }
  else{
    jawServo.write(103);
  }
}
ros::Subscriber<std_msgs::Int16> grip_sub("/gripper", grip_cb);

void setup() {                         // put your setup code here, to run once:
  pinMode (ledPinR, OUTPUT);           //sets up Blinky "alive" light"
  pinMode (ledPinG, OUTPUT);           //sets up Blinky "alive" light"
  pinMode (ledPinB, OUTPUT);           //sets up Blinky "alive" light"  
  pinMode (eStopPin, INPUT);           //sets up Sense input of E-Stop button
  //jawServo.writeMicroseconds(1500);    //set initial servo position to 60 deg
  digitalWrite (ledPinR, HIGH);
  delay (delayPeriod);
  jawServo.attach(2);                  //attach the jaw servo to pin 3
  jawServo.write(125);
  
  nh.initNode();
  nh.subscribe(grip_sub);



  }



  

// Run Hindbrain loop until commanded to stop by Midbrain------------------------------------------------------------
void loop() {
  nh.spinOnce();
  
}


// Think Functions-tttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt

 
  
// Act Functions-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa


