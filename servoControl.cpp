
#include "servoControl.h"

ServoControl::ServoControl(int servoPin){
  this->servo.attach(servoPin);
}

ServoControl::~ServoControl(){/* :^) */}

void ServoControl::startPosition(){
  this->moveServo(this->offset);
}

void ServoControl::moveServo(int angle){
  
  this->checkAngleSafety(&angle);
  this->servo.write(angle);
}

void ServoControl::setOffset(int offset) {
  this->offset = offset;
}

void ServoControl::checkAngleSafety(int* angle){
  if(*angle < this->lowerAngleLimit){
    *angle = this->lowerAngleLimit;
  }
  else if(*angle > this->upperAngleLimit){
    *angle = this->upperAngleLimit;
  }
}

float deg2rad(float degreeValue){
  return degreeValue * PI/180;
}

float rad2deg(float radValue){
  return radValue * 180/PI;
}
