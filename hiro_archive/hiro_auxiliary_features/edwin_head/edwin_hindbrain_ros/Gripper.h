#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#define DEBUGGING

#include <Servo.h>

class Gripper {
    const int servo_pin;
    int val_open, val_close;
    int jaw_value;
    Servo gripper_servo;
  public:
    Gripper(const int pin, const int val_open, const int val_close):
      servo_pin(pin), val_open(val_open), val_close(val_close) {

    }
    void setup() {
      pinMode(servo_pin, OUTPUT);
      gripper_servo.attach(servo_pin);
      grip(false);
    }
    void grip(bool do_grip) {
      gripper_servo.write(do_grip ? val_close : val_open);
    }
#ifdef DEBUGGING
    void write(int value) {
      gripper_servo.write(value);
    }
#endif
};

#endif
