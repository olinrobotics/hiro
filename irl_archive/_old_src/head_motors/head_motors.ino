/*
Arduino code for the three servos in Edwin's head
2 are normal servos, 1 is an MX-64 dynamixel
*/ 

#include <Servo.h> 
 
Servo pen_servo;
Servo eye_servo;

int pen_pos = 0;    
int eye_pos = 0;    
int pos = 0;
int num = 0;

void setup() 
{ 
  //Attach servos to pin 21 and 19 of the arbotix digital pins. Subject to change
  pen_servo.attach(21);
  eye_servo.attach(19);
  Serial.begin(9600); 
} 
 
void loop() 
{ 
  Serial.println(num);
  num = num + 1;
  Serial.println("Forwards");
  for (pos = 0; pos <= 90; pos += 1)
  {
   pen_servo.write(pos);
   eye_servo.write(pos);
   delay(15);
  }
 Serial.println("Backwards");
 for (pos = 90; pos >= 0; pos -= 1)
  {
   pen_servo.write(pos);
   eye_servo.write(pos);
   delay(15);
  
  } 
} 
