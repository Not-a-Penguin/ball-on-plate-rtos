#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <ESP32Servo.h>

float deg2rad(float degreeValue);
float rad2deg(float radValue);

class ServoControl{

  public:
    ServoControl(int servo1Pin, int servo2Pin);
    ~ServoControl();
    
    void startPosition();
    void moveServos(int angle1, int angle2);

    //Initial position of the servos
//    float offset1 = 79.8;
    float offset1 = 95.0;
//    float offset1 = 80;
    float offset2 = 71.0;
    
  private:

    Servo servo1;
    Servo servo2;
    Servo servos[2] = {servo1, servo2};
    
    //Safety limits
  
    int lowerAngleLimit = 50;
    int upperAngleLimit = 110;

    void checkAngleSafety(int* angle);
};

#endif
