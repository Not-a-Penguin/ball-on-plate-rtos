#include "freertos/portmacro.h"
#include "controller.h"

Controller::Controller(Matrix<1, systemOrder> gains, QueueHandle_t statesQueue, QueueHandle_t inputQueue, EventBits_t inputEventBit){
   this->K = gains;
   this->statesQueue = statesQueue;
   this->inputQueue = inputQueue;
   this->inputEventBit = inputEventBit;
};

Controller::~Controller(){};

float Controller::controlLaw(Matrix<systemOrder,1> currentState){
  
  return (this->K * currentState)(0);
}

void saturate(float* input, float lowerLimit, float upperLimit){
  if(*input < lowerLimit) *input = lowerLimit;
  if(*input > upperLimit) *input = upperLimit;
}

void Controller::controllerTask(void *params){
  Controller* self = static_cast<Controller*> (params);
  self->run();
}

void Controller::run(){

  Matrix<2,1> states;

  while(1){
    //Wait for states 
    if(xQueueReceive(this->statesQueue, &states, portMAX_DELAY)){
      float controlInput = this->controlLaw(states);

      //Send to touchScreenQueue
      xQueueSend(this->inputQueue, &controlInput, portMAX_DELAY);
      //Set event bit
      // xEvent

    }
  }


}

void Controller::start(){
    xTaskCreate(
    controllerTask, 
    "controllerTask", 
    3000,
    this, 
    tskIDLE_PRIORITY+2, 
    NULL);
}