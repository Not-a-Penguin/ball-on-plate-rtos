#include "freertos/portmacro.h"
#include "kalmanFilter.h"
#include "servoControl.h"

static const int systemOrder = 2;

KalmanFilter::KalmanFilter(BLA::Matrix<systemOrder, systemOrder> A, BLA::Matrix<systemOrder,1> B, BLA::Matrix<1, systemOrder> C, float qValue1, float qValue2, float rValue, 
                           QueueHandle_t *inputOutputQueue, QueueHandle_t *statesQueue, char* taskName){
  
  this->A = A;
  this->B = B;
  this->C = C;
  
  this->Q = {
    qValue1, 0,
    0, qValue2
  };
  this->R = {rValue};

  this->inputOutputQueue = inputOutputQueue;
  this->statesQueue = statesQueue;
  this->taskName = taskName;
};

KalmanFilter::~KalmanFilter(){};

void KalmanFilter::kfTask(void *params){
  KalmanFilter* self = static_cast<KalmanFilter*> (params);
  self->run();
}

void KalmanFilter::run(){

  inputAndOutput inputOutput;

  while(1){

    //Wait for inputOutputQueue
    if(xQueueReceive(*(this->inputOutputQueue), &inputOutput, portMAX_DELAY)){
      EventsHandler::sendEvent(this->taskName, EventsHandler::EventType::START);
      //Filter and estimate
      float position = inputOutput.input * 0.01; //meter
      float controlInput = inputOutput.output;
      BLA::Matrix<systemOrder,1> states = this->kalman(controlInput, position);

//      Serial.println(this->taskName);
//      Serial.println(inputOutput.input); // x
//      Serial.println(inputOutput.output); //u

      //Send data to controllerQueue and end task
//      Serial.printf("x:%f,x1:%f,x2:%f,u:%f\n", position*100, states(0)*100, states(1), rad2deg(controlInput));
      xQueueSend(*(this->statesQueue), &states, portMAX_DELAY);
      EventsHandler::sendEvent(this->taskName, EventsHandler::EventType::END);
    };

  }

}

void KalmanFilter::start(){
  xTaskCreatePinnedToCore(
    kfTask, 
    this->taskName, 
    6000,
    this, 
    tskIDLE_PRIORITY+3, 
    NULL,
    0
  );
}

BLA::Matrix<systemOrder, 1> KalmanFilter::kalman(float input, float output){

  //a priori
  this->x_priori = this->A * this->x_hat + this->B * input;
  this->P_priori = this->A * this->P_hat * ~this->A + this->U * this->Q * ~this->U;
  
  //kalman gain
  this->K_kalman = this->P_priori * ~this->C * BLA::Inverse(this->C * this->P_priori * ~this->C + this->R);

  //a posteriori
  this->x_hat = this->x_priori + this->K_kalman * (output - (this->C * this->x_priori)(0,0));
  this->P_hat = this->P_priori - this->K_kalman * this->C * this->P_priori;
  
  return this->x_hat;
};

float KalmanFilter::getPTrace(){
  float trace = P_hat(0) + P_hat(2);

  return trace;
}
