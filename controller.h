#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

void saturate(float* input, float lowerLimit, float upperLimit);

class Controller{

  private:
  char* taskName;
  static const int systemOrder = 2;
  Matrix<1, systemOrder> K; 
  public:  

  Controller(
    Matrix<1, systemOrder> gains,
    QueueHandle_t statesQueue,
    QueueHandle_t inputQueue, 
    EventGroupHandle_t xEventGroup,
    EventBits_t inputEventBit,
    char* taskName);

  ~Controller();
  float controlLaw(Matrix<systemOrder, 1> currentState);

  //RTOS
  QueueHandle_t statesQueue;
  QueueHandle_t inputQueue;
  EventBits_t inputEventBit;
  EventGroupHandle_t xEventGroup;
  static void controllerTask(void* params);
  void run();
  void start();

};
#endif
