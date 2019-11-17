/*
 * Edwin Jaw Actuation
 * Olin Robotics Lab Spring 2017
 * olinrobotics.github.io
 * 
 * Receives commands from a ROS node
 * Commands the servo that actuates Edwin's jaw (& tongue?)
 * 
 */


#include <Servo.h>

Servo servo1;
int x;

int t0;

void setup()
{
  int x = 0;
  pinMode(1, OUTPUT);
  servo1.attach(9);
  pinMode(10, OUTPUT);
  Serial.begin(9600); //initialize baud rate
  delay(10);
  t0 = millis();
 return; 
}

void loop()
{

  
}
