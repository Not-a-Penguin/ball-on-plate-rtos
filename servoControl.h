#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <ESP32Servo.h>

float deg2rad(float degreeValue);
float rad2deg(float radValue);

class ServoControl{

  public:
    ServoControl(int servoPin);
    ~ServoControl();
    
    void startPosition();
    void moveServo(int angle);

    //Initial position of the servos
    float offset;
    // float offset1 = 95.0;
    // float offset2 = 71.0;

    void setOffset(int offset);
    float getOffset() { return this->offset; }
    
  private:

    Servo servo;
    // Servo servo1;
    // Servo servo2;
    // Servo servos[2] = {servo1, servo2};
    
    //Safety limits
  
    int lowerAngleLimit = 50;
    int upperAngleLimit = 110;
    void checkAngleSafety(int* angle);
};

#endif
