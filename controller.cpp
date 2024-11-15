#include "freertos/portmacro.h"
#include "controller.h"

Controller::Controller(Matrix<1, systemOrder> gains, QueueHandle_t statesQueue, QueueHandle_t inputQueue,
                       EventGroupHandle_t xEventGroup, EventBits_t inputEventBit, char* taskName){
   this->K = gains;
   this->statesQueue = statesQueue;
   this->inputQueue = inputQueue;
   this->inputEventBit = inputEventBit;
   this->taskName = taskName;
   this->xEventGroup = xEventGroup;
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
      // xEvent
      xEventGroupSetBits(this->xEventGroup, this->inputEventBit);
    }
  }
}

void Controller::start(){
    xTaskCreate(
    controllerTask, 
    this->taskName, 
    3000,
    this, 
    tskIDLE_PRIORITY+2, 
    NULL);
}
