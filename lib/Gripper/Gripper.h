#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>
#include "Servo.h"

class Gripper{
private:
    ServoMotor Gr,  Lf;
public:
    void init(int p1, int ch1, int angle1, int p2, int ch2, int angle2){
        Gr.init(p1, ch1, angle1);
        Lf.init(p2, ch2, angle2);
    }
    void close(int angle){
        Gr.smooth(angle);
    }
    void open(){
        Gr.smooth(50);
    }
    void lift_up(){
        Lf.smooth(105);
    }
    void lift_down(){
        Lf.smooth(10);
    }
    void close_and_lift(int angle){
        close(angle);
        delay(500);
        lift_up();
    }
    void lift_and_open(){
        lift_down();
        delay(500);
        open();
    }
};

#endif