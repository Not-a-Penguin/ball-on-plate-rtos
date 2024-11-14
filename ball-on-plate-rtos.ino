#include "touchScreen.h"
#include "kalmanFilter.h"
#include "controller.h"
#include "stateSpaceMatrices.h"

//Struct to hold both states and control input -> goes to Kalman Filter
struct statesInput{
  Matrix<2,1> states;
  float controlInput;
};

// X and Y measurement queue
// QueueHandle_t measurementQueue;

//X axis
QueueHandle_t xStatesQueue;
QueueHandle_t xControlInputQueue;
QueueHandle_t xInputOutputQueue;

//Y axis
QueueHandle_t yStatesQueue;
QueueHandle_t yControlInputQueue;
QueueHandle_t yInputOutputQueue;

//Event for when both X and Y control inputs are updated
EventGroupHandle_t controlInputEvent;
#define xControlInputBitReady (1 << 0)
#define yControlInputBitReady (1 << 1)
#define controlInputBitReady (xControlInputBitReady | yControlInputBitReady)

/*

depends on controlInputQueue of both X and Y axis - OK
1. Get measurements
  measurementQueue = 1
  send measurement x + input x to xInputOutputQueue - OK

depends on xInputOutputQueue 
2. Estimate states with KF
  statesQueue = 1

depends on statesQueue
3. Generate control input
  controlInputQueue = 1

*/

TouchScreen ts(27, 26, 32, 33, 25, 
  xControlInputQueue, 
  yControlInputQueue,
  xInputOutputQueue,
  yInputOutputQueue,
  controlInputEvent,
  xControlInputBitReady,
  yControlInputBitReady
);

//TODO: pass custom task name to class to differ between x and y

Matrix<1,2> hInfSatGains = {17.8954, 10.0515};
Matrix<1,2> lqrGains = {8.4460, 8.7225}; 
Matrix<1, 2> poleGains = {2.0527, 1.8320};

KalmanFilter xFilter(sys.A, sys.B, sys.C, 0.01, 200, 150, xInputOutputQueue, xStatesQueue, "xFilterTask");
KalmanFilter yFilter(sys.A, sys.B, sys.C, 0.01, 200, 150, yInputOutputQueue, yStatesQueue, "yFilterTask");

Controller xController(poleGains, xStatesQueue, xControlInputQueue, controlInputEvent,  xControlInputBitReady, "xControllerTask");
Controller yController(poleGains, yStatesQueue, yControlInputQueue, controlInputEvent, yControlInputBitReady, "yControllerTask");

void setup(){

  Serial.begin(1000000);
  Serial.println("Start");

  xControlInputQueue = xQueueCreate(1, sizeof(float));
  yControlInputQueue = xQueueCreate(1, sizeof(float));

  xStatesQueue = xQueueCreate(1, sizeof(Matrix<2,1>));
  yStatesQueue = xQueueCreate(1, sizeof(Matrix<2,1>));

  xInputOutputQueue = xQueueCreate(1, sizeof(inputAndOutput));
  yInputOutputQueue = xQueueCreate(1, sizeof(inputAndOutput));

  controlInputEvent = xEventGroupCreate();

  ts.start();

}

void loop() {
  // put your main code here, to run repeatedly:

}
