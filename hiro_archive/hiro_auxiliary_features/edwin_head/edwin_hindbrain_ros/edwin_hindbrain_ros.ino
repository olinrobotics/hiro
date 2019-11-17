// Basic Edwin Hind Brain Test Code
// Version 1.0  04/12/17
// Basic code to blink lights, move jaw and make sound
// Please don't break Edwin. Ask before altering motor parameters

#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "config.h"
#include "Gripper.h"
#include <Servo.h>        //include built in Arduino servo library

Gripper gripper(JAW_PIN, JAW_OPEN, JAW_CLOSED);

// ROS STUFF
ros::NodeHandle nh;
void grip_cb(const std_msgs::Int16& msg) {
  int grip_value = msg.data;
  gripper.write(grip_value);
}

ros::Subscriber<std_msgs::Int16> grip_sub("/grip", grip_cb);

// currently disabled mouth ir because it was reading gibberish
//std_msgs::Float32 m_ir_msg; //mouth ir
//ros::Publisher m_ir_pub("/m_ir", &m_ir_msg);

std_msgs::Float32 n_ir_msg; //nose ir
ros::Publisher n_ir_pub("/n_ir", &n_ir_msg);

int numtones = 10;         // create number of tones
int tones[] = { 261, 277, 294, 311, 330, 349, 370, 392, 415, 440 };  //stored tones

void setup_ros() {
  nh.initNode();
  nh.subscribe(grip_sub);
  //nh.advertise(m_ir_pub);
  nh.advertise(n_ir_pub);

}

void setup() {                         // put your setup code here, to run once:
  gripper.setup();
  setup_ros();

  pinMode (R_LED_PIN, OUTPUT);           //sets up Blinky "alive" light"
  pinMode (G_LED_PIN, OUTPUT);           //sets up Blinky "alive" light"
  pinMode (B_LED_PIN, OUTPUT);           //sets up Blinky "alive" light"

  pinMode (E_STOP_PIN, INPUT);           //sets up Sense input of E-Stop button
  //pinMode (MOUTH_IR_PIN, INPUT);
  pinMode (NOSE_IR_PIN, INPUT);

  for (int i = 0; i < numtones; i++)
  { tone(SPEAKER_PIN, tones[i]);
    delay(100);
  }
  noTone(SPEAKER_PIN);
  for (int i = 0; i < numtones; i++)
  { tone(SPEAKER_PIN, tones[numtones - i]);
    delay(100);
  }
  noTone(SPEAKER_PIN);
}

float read_ir(int pin) {
  int rawData = analogRead(pin);    // V is 0-1023
  float volts = rawData * 0.0048828125;   //convert to volts, equivalent to 5.0V / 1023
  float range = 65 * pow(volts, -1.10);   // approximate exp data graph function
  return range;
}

// Run Hindbrain loop until commanded to stop by Midbrain------------------------------------------------------------
void loop() {
  nh.spinOnce();

  //float mouth_ir_range = read_ir(MOUTH_IR_PIN);
  float nose_ir_range = read_ir(NOSE_IR_PIN);

  //m_ir_msg.data = mouth_ir_range * 0.01; // convert to m, standard ROS unit
  n_ir_msg.data = nose_ir_range * 0.01;

  //m_ir_pub.publish(&m_ir_msg);
  n_ir_pub.publish(&n_ir_msg);

  // Think: Run low level cognition and safety code-ttttttttttttttttttttttttttttttttttttt
  blink();                          // blink hindbrain running LED
  soundRange(nose_ir_range);          // convert nose range to sound
  delay(DELAY_PERIOD);
}

float soundRange(int range1) { //play sounds to indicate range
  if (range1 > 60 )
  { noTone(SPEAKER_PIN);              // no objects in sight, make no sound
    delay(50);
  }
  if ((range1 > 55) && (range1 < 60) )
  { tone(SPEAKER_PIN, tones[1]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if ((range1 > 50) && (range1 < 55) )
  { tone(SPEAKER_PIN, tones[2]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if ((range1 > 45) && (range1 < 50) )
  { tone(SPEAKER_PIN, tones[3]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if ((range1 > 40) && (range1 < 45) )
  { tone(SPEAKER_PIN, tones[4]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if ((range1 > 35) && (range1 < 40) )
  { tone(SPEAKER_PIN, tones[4]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if ((range1 > 30) && (range1 < 35) )
  { tone(SPEAKER_PIN, tones[5]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if ((range1 > 25) && (range1 < 30) )
  { tone(SPEAKER_PIN, tones[6]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if ((range1 > 20) && (range1 < 25) )
  { tone(SPEAKER_PIN, tones[7]);      // beep to show head active
    delay(100);
    noTone(SPEAKER_PIN);                   // stop beep
    delay(300);
  }
  if (range1 < 20 )
  { noTone(SPEAKER_PIN);
    delay(50);
  }    // object too close, make no sound
}

// Think Functions-tttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt
void blink () {                  //this function blinks the Hindbrain Alive LED
  digitalWrite (R_LED_PIN, HIGH);
  delay (DELAY_PERIOD);
  digitalWrite (R_LED_PIN, LOW);
  delay  (DELAY_PERIOD);
  digitalWrite (B_LED_PIN, HIGH);
  delay (DELAY_PERIOD);
  digitalWrite (B_LED_PIN, LOW);
  delay  (DELAY_PERIOD);
  digitalWrite (G_LED_PIN, HIGH);
  delay (DELAY_PERIOD);
  digitalWrite (G_LED_PIN, LOW);
  delay  (DELAY_PERIOD);
}



// Act Functions-aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa


