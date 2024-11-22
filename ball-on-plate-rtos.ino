#include "touchScreen.h"
#include "kalmanFilter.h"
#include "controller.h"
#include "stateSpaceMatrices.h"
#include "events.h"
#include "tiny_api.hpp"

#define USE_MPC

uint16_t failedMessageCounter;
char sendEventsTaskName[15] = "sendEventsTask";

//Struct to hold both states and control input -> goes to Kalman Filter
struct statesInput{
  BLA::Matrix<2,1> states;
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

// Events Queue
QueueHandle_t eventsQueue;

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
  &xControlInputQueue,
  &yControlInputQueue,
  &xInputOutputQueue,
  &yInputOutputQueue,
  &controlInputEvent,
  xControlInputBitReady,
  yControlInputBitReady,
  "TouchScreenTask"
);

ServoControl ServoX(23);
ServoControl ServoY(19);

KalmanFilter xFilter(sys.A, sys.B, sys.C, 0.01, 200, 150, &xInputOutputQueue, &xStatesQueue, "xFilterTask");
KalmanFilter yFilter(sys.A, sys.B, sys.C, 0.01, 200, 150, &yInputOutputQueue, &yStatesQueue, "yFilterTask");

#ifndef USE_MPC
BLA::Matrix<1,2> hInfSatGains = {17.8954, 10.0515};
BLA::Matrix<1,2> lqrGains = {8.4460, 8.7225}; 
BLA::Matrix<1, 2> poleGains = {2.0527, 1.8320};
#else
tinytype Q_data[] = {10.0, 1.0};
tinytype R_data[] = {1.0};
#endif

#ifndef USE_MPC
Controller xController(&ServoX, hInfSatGains, &xStatesQueue, &xControlInputQueue, &controlInputEvent,  xControlInputBitReady, "xControllerTask");
Controller yController(&ServoY, hInfSatGains, &yStatesQueue, &yControlInputQueue, &controlInputEvent, yControlInputBitReady, "yControllerTask");
#else
Controller xController(&ServoX, sys.A, sys.B, Q_data, R_data, &xStatesQueue, &xControlInputQueue, &controlInputEvent,  xControlInputBitReady, "xControllerTask");
Controller yController(&ServoY, sys.A, sys.B, Q_data, R_data, &yStatesQueue, &yControlInputQueue, &controlInputEvent, yControlInputBitReady, "yControllerTask");
#endif

void setup(){

  Serial.begin(1000000);
  Serial.println("Start");
  // servos.startPosition();

  xControlInputQueue = xQueueCreate(1, sizeof(float));
  yControlInputQueue = xQueueCreate(1, sizeof(float));

  xStatesQueue = xQueueCreate(1, sizeof(BLA::Matrix<2,1>));
  yStatesQueue = xQueueCreate(1, sizeof(BLA::Matrix<2,1>));

  xInputOutputQueue = xQueueCreate(1, sizeof(inputAndOutput));
  yInputOutputQueue = xQueueCreate(1, sizeof(inputAndOutput));

  eventsQueue = xQueueCreate(25, sizeof(EventsHandler::EventsMessage));

  controlInputEvent = xEventGroupCreate();
  xEventGroupSetBits(controlInputEvent, xControlInputBitReady);
  xEventGroupSetBits(controlInputEvent, yControlInputBitReady);

 xTaskCreatePinnedToCore(
   EventsHandler::sendEventsToSerial, 
   sendEventsTaskName, 
   10000,
   (void*) 10, 
   tskIDLE_PRIORITY+1, 
   NULL,
   0
 );
 
  ServoX.setOffset(95.0);
  ServoY.setOffset(71.0);  
  ServoX.startPosition();
  ServoY.startPosition();

  ts.setSamplingTime(35);
  xController.setSamplingTime(35);
  yController.setSamplingTime(35);
  
  ts.start();
  xFilter.start();
  yFilter.start();
  xController.start();
  yController.start();

  Serial.println("Headers");
  Serial.printf("task_name,event_type,time,x1,x2,u,mpc_time,message_counter,message_failed\n");

}

void loop() {}
