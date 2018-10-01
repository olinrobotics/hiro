/*
  modified 29 April 2018
  by Hannah Kolano
  hannah.kolano@students.olin.edu
*/

//servo library
#include <Servo.h>

//ROS stuff
#include <ros.h>
#include <std_msgs/Int32.h>


Servo myservo;  //creates servo object "myservo"
int pos = 90;    // variable to store the servo position
const int servoPin = 9;

ros::NodeHandle gemini_claw;
int command = 0; //user input. Later make ROS input.


void callback( const std_msgs::Int32& pyMessage) { 
  command = pyMessage.data;
  Serial.print(command);

  digitalWrite(LED_BUILTIN, HIGH);
    
  change_grip();
    
  command = 0;
    
  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
}

ros::Subscriber<std_msgs::Int32> sub("/grab_cmd", &callback);

void setup() {
  myservo.attach(servoPin);    // attaches the servo on pin 9 to the servo object
  pinMode(LED_BUILTIN, OUTPUT); //blinky builtin light

  gemini_claw.getHardware() -> setBaud(9600);
  gemini_claw.initNode();
  gemini_claw.subscribe(sub);
  delay(500);
  Serial.begin(9600);   //start communication with serial port
}

void loop() {
  gemini_claw.spinOnce();
  delay(1);
}

/*
  Functions
*/

void change_grip() {
  if (command == 1) {
      rest_to_inwards(60);
      delay(1875);
      slow_to_rest(60);
    }
    if (command == 2) {
      rest_to_outwards(120);
      delay(1600);
      slow_to_rest(120);
    }
    myservo.write(90);
    delay(300);
}

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




