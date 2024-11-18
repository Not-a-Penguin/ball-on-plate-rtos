#include "freertos/projdefs.h"
#include "freertos/portmacro.h"
#include <Arduino.h>
#include "touchScreen.h"

TouchScreen::TouchScreen(
  int upperLeft, 
  int upperRight, 
  int lowerLeft, 
  int lowerRight, 
  int sensorPin,
  QueueHandle_t *xInputQueue,
  QueueHandle_t *yInputQueue,
  QueueHandle_t *xInputOutputQueue,
  QueueHandle_t *yInputOutputQueue,
  EventGroupHandle_t *inputEventGroup,
  EventBits_t xIputEventBit,
  EventBits_t yInputEventBit,
  char* taskName
) : servos(23, 19) {

  //Set the pins according to the corners of the screen
  this->_upperLeftPin = upperLeft;
  this->_upperRightPin = upperRight;
  
  this->_lowerLeftPin = lowerLeft;
  this->_lowerRightPin = lowerRight;

  this->_sensorPin = sensorPin;

  this->setPins();

  this->servos.startPosition();

  //RTOS

  this->xControlInputQueue = xInputQueue;
  this->yControlInputQueue = yInputQueue;
  this->xInputOutputQueue = xInputOutputQueue;
  this->yInputOutputQueue = yInputOutputQueue;
  this->controlInputEvent = inputEventGroup;
  this->xIputEventBit = xIputEventBit;
  this->yInputEventBit = yInputEventBit;
  this->taskName = taskName;
  
};

TouchScreen::~TouchScreen(){/* 
  (づ｡◕‿‿◕｡)づ no destructor
*/}

void TouchScreen::run(){
  screenCoordinates coords;
  screenCoordinatesCm coordsCm;

  inputAndOutput xInputOutput;
  inputAndOutput yInputOutput;

  float xInput;
  float yInput;

  float uDegreeX;
  float uDegreeY;
  float angleX;
  float angleY;

  while(1){
    //Wait for control input to be updated
    EventBits_t eventBit = xEventGroupWaitBits(
      *(this->controlInputEvent),
      (this->yInputEventBit | this->xIputEventBit), 
      pdTRUE, 
      pdTRUE, 
      portMAX_DELAY);

    if(eventBit){
      EventsHandler::sendEvent(this->taskName, EventsHandler::EventType::START);

      if(xQueueReceive(*(this->xControlInputQueue), &xInput, 0) == pdFALSE) {
        xInput = 0.0;
      }
      if(xQueueReceive(*(this->yControlInputQueue), &yInput, 0) == pdFALSE) {
        yInput = 0.0;
      };

      //Convert to degree and saturate
      // uX = xController.controlLaw(statesX);   
      uDegreeX = rad2deg(xInput);
      saturate(&uDegreeX, -25, 25);
      xInput = deg2rad(uDegreeX);

      // uY = yController.controlLaw(statesY);   
      uDegreeY = rad2deg(yInput);
      saturate(&uDegreeY, -25, 25);
      yInput = deg2rad(uDegreeY);

      angleX = (uDegreeX) + servos.offset1;       
      angleY = (uDegreeY) + servos.offset2;
      // servos.moveServos(angleX, servos.offset2+90);
      this->servos.moveServos(angleX , angleY);

      //Read touchScreen
      coords = this->getCoordinates();
      coordsCm = this->getCoordinatesCm(coords.x, coords.y);

      xInputOutput.input = coords.x;
      xInputOutput.output = xInput;

      yInputOutput.input = coords.y;
      yInputOutput.output = yInput;

      // End Task
      EventsHandler::sendEvent(this->taskName, EventsHandler::EventType::END);
      //Send x data to x queue
      xQueueSend(*(this->xInputOutputQueue), &xInputOutput, portMAX_DELAY);
      //Send y data to y queue and 
      xQueueSend(*(this->yInputOutputQueue), &yInputOutput, portMAX_DELAY);
    }
  }

}

void TouchScreen::start(){
  xTaskCreatePinnedToCore(
    tsTask, 
    this->taskName, 
    6000,
    this, 
    tskIDLE_PRIORITY, 
    NULL,
    0
  );
}

void TouchScreen::tsTask(void *params){

  TouchScreen* self = static_cast<TouchScreen*> (params);
  self->run();

}

screenCoordinates TouchScreen::getCoordinates(){ 

  this->_currentScreenCoordinates.x = this->readCoordinate("x");
  this->_currentScreenCoordinates.y = this->readCoordinate("y");
  this->_newReading = true;

  return this->_currentScreenCoordinates; 
};

int TouchScreen::readCoordinate(String coordinate){

  /*  
   *  In order to read the coordinates, the corners of the screen must be set to HIGH or LOW 
   *  in a certain way to create a voltage gradient across the screen, then when a touch occurs
   *  and the voltage divider is made, the voltage across that gradient with change and can be read
   *  with the middle pin.
   *  
   *  The corner signals are explained in the table bellow
   *   
   *  ______________________________________________________________________
   * |  function      | upper-left | lower-left | upper-right | lower-right |
   * |read x-position |     Vss    |    Vss     |      Vdd    |     Vdd     |
   * |read y-position |     Vss    |    Vdd     |      Vss    |     Vdd     |
   * |______________________________________________________________________|
   */
   
  if(coordinate == "x"){
    digitalWrite(this->_lowerLeftPin, HIGH);
    digitalWrite(this->_upperRightPin, LOW);
  }
  
  else if(coordinate == "y"){
    digitalWrite(this->_lowerLeftPin, LOW);
    digitalWrite(this->_upperRightPin, HIGH); 
  }
  
  // delay(this->_interval);
  vTaskDelay(pdMS_TO_TICKS(this->_interval));
  return analogRead(this->_sensorPin);
};

screenCoordinatesCm TouchScreen::getCoordinatesCm(float xValue, float yValue){

  this->_currentScreenCoordiantesCm.xCm = 1.6298789841486278e-002 * xValue - 3.0054968467700697e+001 + 0.4;
  this->_currentScreenCoordiantesCm.yCm = 1.2133349039934032e-002 * yValue - 2.4024031099069383e+001 + 0.2 ;

  return this->_currentScreenCoordiantesCm;
  
};

bool TouchScreen::screenUpdated(){
  if(this->_newReading == true){
    this->_newReading = false;
    return true;
  } else return false;
}

void TouchScreen::setSamplingTime(int milliseconds){
  this->_interval = milliseconds >> 1;  
};

void TouchScreen::setPins(){

  //All corner pins are output
  pinMode(this->_upperLeftPin, OUTPUT);
  pinMode(this->_upperRightPin, OUTPUT);
  
  pinMode(this->_lowerLeftPin, OUTPUT);
  pinMode(this->_lowerRightPin, OUTPUT);

  pinMode(this->_sensorPin, INPUT);  

  digitalWrite(this->_upperLeftPin, HIGH);
  digitalWrite(this->_lowerRightPin, LOW);
};
