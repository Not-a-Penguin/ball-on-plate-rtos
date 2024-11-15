
#include "servoControl.h"

ServoControl::ServoControl(int servo1Pin, int servo2Pin){
  this->servo1.attach(servo1Pin);
  this->servo2.attach(servo2Pin);
}

ServoControl::~ServoControl(){/* :^) */}

void ServoControl::startPosition(){
  this->moveServos(this->offset1, this->offset2);
}

void ServoControl::moveServos(int angle1, int angle2){
  
  this->checkAngleSafety(&angle1);
  this->checkAngleSafety(&angle2);

  this->servo1.write(angle1);
  this->servo2.write(angle2);
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
