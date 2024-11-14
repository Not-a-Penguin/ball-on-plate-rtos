#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <BasicLinearAlgebra.h>
#include "touchScreen.h"
using namespace BLA;

class KalmanFilter{

  private:

  static const int systemOrder = 2;

  Matrix<systemOrder, systemOrder> A;
  Matrix<systemOrder,1> B;
  Matrix<1, systemOrder> C;
  
  Matrix<systemOrder, systemOrder> P_priori;
  Matrix<systemOrder, systemOrder> P_hat;
  
  Matrix<systemOrder, 1> x_hat;
  Matrix<systemOrder, 1> x_priori;

  Matrix<systemOrder, 1> K_kalman;
  
  //Identity matrix
  const Matrix<systemOrder, systemOrder> U = {
    1, 0,
    0, 1
  };

  Matrix<systemOrder, systemOrder> Q;
  Matrix<1,1> R;
  
  public:

  KalmanFilter(
    Matrix<systemOrder, systemOrder> A, 
    Matrix<systemOrder,1> B, 
    Matrix<1, systemOrder> C, 
    float qValue1, 
    float qValue2, 
    float rValue,
    QueueHandle_t inputOutputQueue,
    QueueHandle_t statesQueue
  );

  ~KalmanFilter();
  float getPTrace();

  Matrix<systemOrder, 1> kalman(float input, float output);  

  //RTOS
  static void kfTask(void* params);
  void start();
  void run();
  QueueHandle_t inputOutputQueue;
  QueueHandle_t statesQueue;

};

#endif
