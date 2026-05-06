#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

class ServoMotor{
private:
    int pin;
    int channel;
    int minPulse = 500;
    int maxPulse = 2500;
    int freq = 50;
    int resolution = 16;
    int current_angle;
public:
    void init(int p, int ch, int current_angle){
        pin = p;
        channel = ch;

        ledcSetup(channel, freq, resolution);
        ledcAttachPin(pin, channel);
        smooth(current_angle);
    }
    void writeMicro(int micros){
        micros = constrain(micros, minPulse, maxPulse);
        int duty = (micros * ((1 << resolution) - 1)) / 20000;
        ledcWrite(channel, duty);
    }
    void run(int angle){    
        angle = constrain(angle, 0, 180);
        int pulse = minPulse + (angle * (maxPulse - minPulse)) / 180;
        writeMicro(pulse);
    }
    void smooth(int angle){
        if (angle > current_angle){
            for (int i = current_angle; i <= angle; i++){
                run(i);
                delay(30);
            }
        }
        else if (angle < current_angle){
            for (int i = current_angle; i >= angle; i--){
                run(i);
                delay(30);
            }
        }
        current_angle = angle;
    }
};

#endif