#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <BasicLinearAlgebra.h>
#include "events.h"
#include "tiny_api.hpp"

#define NHORIZON 30
#define MAX_ITERATIONS 200

void saturate(float* input, float lowerLimit, float upperLimit);

enum ControllerType {
  STATE_FEEDBACK,
  MPC
};

class Controller{

private:
  char* taskName;
  static const int systemOrder = 2;
  BLA::Matrix<1, systemOrder> K;
  ControllerType controllerType;

  // MPC related
  TinySolver *solver;
  TinyWorkspace *work;

  typedef Matrix<tinytype, 1, NHORIZON-1> tiny_MatrixNuNhm1;
  typedef Matrix<tinytype, systemOrder, NHORIZON> tiny_MatrixNxNh;
  typedef Matrix<tinytype, systemOrder, 1> tiny_VectorNx;

  float rho_value = 1.0; // verificar o que é isso

  tinytype Adyn_data[systemOrder * systemOrder];
  tinytype Bdyn_data[systemOrder * 1];
  tinytype Q_data[systemOrder];
  tinytype R_data[systemOrder];

  tinyMatrix Adyn;
  tinyMatrix Bdyn;
  tinyVector Q;
  tinyVector R;

  tinyMatrix x_min = tiny_MatrixNxNh::Constant(-30);   // fix this values TODO: CHANGE TO VECTOR
  tinyMatrix x_max = tiny_MatrixNxNh::Constant(30);    // fix this values
  tinyMatrix u_min = tiny_MatrixNuNhm1::Constant(-25); // fix this values
  tinyMatrix u_max = tiny_MatrixNuNhm1::Constant(25);  // fix this values

  tiny_VectorNx x; // stores the states -> gets from the kalman filter

public:  

  Controller(
    BLA::Matrix<1, systemOrder> gains,
    QueueHandle_t *statesQueue,
    QueueHandle_t *inputQueue, 
    EventGroupHandle_t *xEventGroup,
    EventBits_t inputEventBit,
    char* taskName
  );

  Controller(
    BLA::Matrix<systemOrder, systemOrder> A, 
    BLA::Matrix<systemOrder,1> B, 
    tinytype Q_data[systemOrder], 
    tinytype R_data[systemOrder],
    QueueHandle_t *statesQueue,
    QueueHandle_t *inputQueue, 
    EventGroupHandle_t *xEventGroup,
    EventBits_t inputEventBit,
    char* taskName
  );

  ~Controller();
  float controlLaw(BLA::Matrix<systemOrder, 1> currentState);

  //RTOS
  QueueHandle_t *statesQueue;
  QueueHandle_t *inputQueue;
  EventBits_t inputEventBit;
  EventGroupHandle_t *xEventGroup;
  static void controllerTask(void* params);
  void run();
  void start();

};
#endif
