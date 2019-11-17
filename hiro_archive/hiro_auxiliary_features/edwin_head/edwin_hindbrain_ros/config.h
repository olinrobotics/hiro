#ifndef __CONFIG_H__
#define __CONFIG_H__

// Set up Arduino Ports, Pins and Global Variables to support Robot-------------------------------------------
#define R_LED_PIN 11          // robot alive red blinky light pin
#define G_LED_PIN 10          // robot alive green blinky light pin
#define B_LED_PIN 9           // robot alive blue blinky light pin
#define E_STOP_PIN 5          // create name for E-Stop reading pin
#define DELAY_PERIOD 50     // hindbrain loop delay Note: change over to timers not delays
#define MOUTH_IR_PIN A0    // create name for sharp ir 1 analog input pin 1
#define NOSE_IR_PIN A1    // create name for sharp ir 0 analog input pin 0
#define SPEAKER_PIN 6        // create name for speaker pin

// JAW
#define JAW_PIN 3
#define JAW_OPEN 60
#define JAW_CLOSED 90

#endif
