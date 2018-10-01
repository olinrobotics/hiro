// Basic Edwin Hind Brain Test Code
// Version 1.0  04/12/17
// Basic code to blink lights, move jaw and make sound
// Please don't break Edwin. Ask before altering motor parameters

#include <Servo.h>        //include built in Arduino servo library

// Set up Arduino Ports, Pins and Global Variables to support Robot-------------------------------------------
int ledPinR = 11;          // robot alive red blinky light pin
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
String readString;        // create a string to store Midbrain commands in

void setup() {                         // put your setup code here, to run once:
  pinMode (ledPinR, OUTPUT);           //sets up Blinky "alive" light"
  pinMode (ledPinG, OUTPUT);           //sets up Blinky "alive" light"
  pinMode (ledPinB, OUTPUT);           //sets up Blinky "alive" light"  
  pinMode (eStopPin, INPUT);           //sets up Sense input of E-Stop button
  //jawServo.writeMicroseconds(1500);    //set initial servo position to 60 deg
  jawServo.write(60);
  jawServo.attach(3);                  //attach the jaw servo to pin 3
  Serial.begin(9600);                  //send and recieve at 9600 baud
  for (int i=0; i < numtones; i++)
  { tone(speakerPin, tones[i]);
    delay(100);
  }
  noTone(speakerPin);
  for (int i=0; i < numtones; i++)
  { tone(speakerPin, tones[numtones-i]);
    delay(100);
  }
  noTone(speakerPin);
  jawServo.write(60);              // command pan servo to new pan angle
  delay(1000);
  jawServo.write(70);              // command pan servo to new pan angle
  delay(1000);
  jawServo.write(60);              // command pan servo to new pan angle
  delay(1000);
  jawServo.write(70);              // command pan servo to new pan angle
  delay(1000);
  jawServo.write(60);              // command pan servo to new pan angle
  delay(1000);
  jawServo.write(70);              // command pan servo to new pan angle
  delay(1000);
  }
  

// Run Hindbrain loop until commanded to stop by Midbrain------------------------------------------------------------
void loop() {
  
  // Read Midbrain commands-rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
  while (Serial.available()) {    // checks to see if you have message in serial buffer
    command = Serial.read();      // read character command from serial monitor
    readString += command;        //makes the string readString to store commands
    delay(2);                     //slow loop to allow buffer to fill with next character
  }
  
    if (readString.length() >0) {
    Serial.println(readString);     //so you can see the captured string 
    if (command != 's') {      //during stop, ignore command to move jaw
    jaw = readString.toInt();       //convert readString into a jaw angle
    }     
    readString="";                  //empty readString for next input
    } 
    
 // Sense: Read and Process Robot Sensors-sssssssssssssssssssssssssssssssssssssssssssss    
  float SharpRange0= sharpRange(sharpDistance0);  // Read Sharp mouth range 0 on Pin A0
  float SharpRange1= sharpRange(sharpDistance1);  // Read Sharp nose range 1 on Pin A1
  delay(50);
  float SharpRange2= sharpRange(sharpDistance1);  // Read again
  SharpRange1=((SharpRange1+SharpRange2)/2);      //filter a little
  //Note: Sharps only work from 20-60mm and they invert closer than 20mm
  Serial.print ("MouthRange: ");
  Serial.println (SharpRange0);                //range from sharp 0 debug only
  Serial.print ("NoseRange: ");
  Serial.println (SharpRange1);                //range from sharp 0 debug only
  
  // Think: Run low level cognition and safety code-ttttttttttttttttttttttttttttttttttttt
   blink();                          // blink hindbrain running LED
   soundRange(SharpRange1);          // convert nose range to sound
  
   
  // Act: Run actuators and behavior lights-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  Serial.print("Commanding jaw angle: ");
  Serial.println(jaw);
  if (command == 's')   //if Stop command all motors to stop
    {        
     jaw=90;
     jawServo.write(60);   //60 degress is open
    } 
  else  // if hind brain running
  {
  /*if (jaw >= 90) jaw=90;             //set closed lmit on jaw
  if (jaw <= 70) jaw=70;             //set open limit on jaw */
  jawServo.write(jaw);              // command pan servo to new pan angle
  }
  
  // Write status data up to MidBrain-wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww 
   Serial.println ("----------------------------------------"); 
  if (command == 's') {
  Serial.println("Hind brain stopped....");   //print hindbrain status on serial monitor
  }
  if (command == 'g') {
  Serial.println("Hind brain running!!!!");   //print hindbrain status on serial monitor
  } 
   Serial.println("'g' is go command from midbrain,'s'is stop, '60'-open, '70'-closed' jaw servo setpoint ");
   Serial.print ("Last Midbrain command sent: ");
   Serial.println (command);      //print midbrain command in serial monitor     
 
}

// Hindbrain functions ********************************************************************************************
//
// Sense Functions-sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
float sharpRange(int sensornum) {   //Read Sharp range, return range in cm's
  int rawData = analogRead(sensornum);    // V is 0-1023
  float volts = rawData*0.0048828125;     //convert to volts
  float range = 65*pow(volts, -1.10);     // approximate exp data graph function
  return range;
}

float soundRange(int range1) { //play sounds to indicate range
   if (range1 > 60 )
  { noTone(speakerPin);              // no objects in sight, make no sound
  delay(50);}
  if ((range1 > 55) && (range1 < 60) )
  {tone(speakerPin, tones[1]);      // beep to show head active
  delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); }       
   if ((range1 > 50) && (range1 < 55) )
  {tone(speakerPin, tones[2]);      // beep to show head active
  delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); } 
   if ((range1 > 45) && (range1 < 50) )
  {tone(speakerPin, tones[3]);      // beep to show head active
   delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); } 
   if ((range1 > 40) && (range1 < 45) )   
  {tone(speakerPin, tones[4]);      // beep to show head active
  delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); } 
   if ((range1 > 35) && (range1 < 40) )   
  {tone(speakerPin, tones[4]);      // beep to show head active
 delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); } 
   if ((range1 > 30) && (range1 < 35) )   
  {tone(speakerPin, tones[5]);      // beep to show head active
  delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); } 
   if ((range1 > 25) && (range1 < 30) )   
  {tone(speakerPin, tones[6]);      // beep to show head active
  delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); } 
   if ((range1 > 20) && (range1 < 25) )   
  {tone(speakerPin, tones[7]);      // beep to show head active
 delay(100);
  noTone(speakerPin);                   // stop beep
  delay(300); } 
   if (range1 < 20 )
   {noTone(speakerPin);
   delay(50);}    // object too close, make no sound 
}

// Think Functions-tttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt
void blink (){                   //this function blinks the Hindbrain Alive LED
  digitalWrite (ledPinR, HIGH);
  delay (delayPeriod);
  digitalWrite (ledPinR, LOW);
  delay  (delayPeriod);
  digitalWrite (ledPinB, HIGH);
  delay (delayPeriod);
  digitalWrite (ledPinB, LOW);
  delay  (delayPeriod);
   digitalWrite (ledPinG, HIGH);
  delay (delayPeriod);
  digitalWrite (ledPinG, LOW);
  delay  (delayPeriod);
  }

 
  
// Act Functions-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa


