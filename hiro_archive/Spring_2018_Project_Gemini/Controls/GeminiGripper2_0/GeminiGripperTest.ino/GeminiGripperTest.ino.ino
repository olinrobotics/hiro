/*
  modified 29 April 2018
  by Hannah Kolano
  hannah.kolano@students.olin.edu
*/

//servo library
#include <Servo.h>

Servo myservo;  //creates servo object "myservo"
int pos = 90;    // variable to store the servo position
const int servoPin = 9;


void setup() {
  myservo.attach(servoPin);    // attaches the servo on pin 9 to the servo object
  pinMode(LED_BUILTIN, OUTPUT); //blinky builtin light

 
  delay(500);
  Serial.begin(9600);   //start communication with serial port

   // Toggle between these two blocks of code to custom move the gripper in/out

   // THIS CLOSES IT
//      rest_to_inwards(60);
//      delay(200);
//      slow_to_rest(60);
    

  //THIS OPENS IT
      rest_to_outwards(120);
      delay(1600);
      slow_to_rest(120);


  
    delay(300);
}

void loop() {

 
}

/*
  Functions
*/


//starting at rest, go to out_speed
void rest_to_outwards(int out_speed) {
  for (pos = 90; pos <= out_speed; pos += 1) {
    myservo.write(pos);
    delay(1);
  }
}

//starting at rest, go to in_speed
void rest_to_inwards(int in_speed) {
  for (pos = 90; pos >= in_speed; pos -= 1) {
    myservo.write(pos);
    delay(1);
  }
}

//starting at cur_speed, slow to rest
void slow_to_rest(int cur_speed) {
  if (cur_speed > 90) {
    for (pos = cur_speed; pos >= 90; pos -= 1) {
      myservo.write(pos);
      delay(1);
    }
  }
  else if (cur_speed < 90) {
    for (pos = cur_speed; pos <= 90; pos += 1) {
      myservo.write(pos);
      delay(1);
    }
  }
}




