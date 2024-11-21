#include "freertos/portmacro.h"
#include "controller.h"
#include <iostream>

Controller::Controller(ServoControl *servo, BLA::Matrix<1, systemOrder> gains, QueueHandle_t *statesQueue, QueueHandle_t *inputQueue,
                       EventGroupHandle_t *xEventGroup, EventBits_t inputEventBit, char* taskName){
  this->servo = servo;
  this->controllerType = ControllerType::STATE_FEEDBACK;
  this->K = gains;
  this->statesQueue = statesQueue;
  this->inputQueue = inputQueue;
  this->inputEventBit = inputEventBit;
  this->taskName = taskName;
  this->xEventGroup = xEventGroup;
};

Controller::Controller(ServoControl *servo, BLA::Matrix<systemOrder, systemOrder> A, BLA::Matrix<systemOrder,1> B, 
                       tinytype Q_data[systemOrder], tinytype R_data[systemOrder],
                       QueueHandle_t *statesQueue, QueueHandle_t *inputQueue,
                       EventGroupHandle_t *xEventGroup, EventBits_t inputEventBit, char* taskName){
  this->servo = servo;
  this->controllerType = ControllerType::MPC;
  this->statesQueue = statesQueue;
  this->inputQueue = inputQueue;
  this->inputEventBit = inputEventBit;
  this->taskName = taskName;
  this->xEventGroup = xEventGroup;

  // MPC related
  this->controllerType = ControllerType::MPC;

  for(int i = 0; i < systemOrder; i++){
    for(int j = 0; j < systemOrder; j++){
      this->Adyn_data[i+j] = A(i, j);
    }
    this->Bdyn_data[i+0] = B(i, 0);
    this->Q_data[i] = Q_data[i];
  }
  
  this->R_data[0] = R_data[0];

  this->Adyn = Map<Matrix<tinytype, systemOrder, systemOrder, RowMajor>>(Adyn_data);
  this->Bdyn = Map<Matrix<tinytype, systemOrder, 1>>(Bdyn_data);
  this->Q = Map<Matrix<tinytype, systemOrder, 1>>(Q_data);
  this->R = Map<Matrix<tinytype, 1, 1>>(R_data);

  // this->x_min = tiny_MatrixNxNh::Constant(-30);
  // this->x_max = tiny_MatrixNxNh::Constant(30);

  this->x_min.block<1, NHORIZON>(0, 0) = tiny_Matrix1Nh::Constant(-15);  // x1 min
  this->x_min.block<1, NHORIZON>(1, 0) = tiny_Matrix1Nh::Constant(-300); // x2 min
  this->x_max.block<1, NHORIZON>(0, 0) = tiny_Matrix1Nh::Constant(15);   // x1 max
  this->x_max.block<1, NHORIZON>(1, 0) = tiny_Matrix1Nh::Constant(300);  // x2 max

  this->u_min = tiny_MatrixNuNhm1::Constant(-25);
  this->u_max = tiny_MatrixNuNhm1::Constant(25);

  int status = tiny_setup(&this->solver,
                          Adyn, Bdyn, Q.asDiagonal(), R.asDiagonal(),
                          rho_value, systemOrder, 1, NHORIZON,
                          x_min, x_max, u_min, u_max, 1);

  // Update whichever settings we'd like
  this->solver->settings->max_iter = MAX_ITERATIONS;

  // Alias this->solver->work for brevity
  work = this->solver->work;

  std::cout << x_min << std::endl;
  std::cout << x_max << std::endl;

  // Reference
  tiny_VectorNx ref;
  ref << 0.0, 0.0;
  work->Xref  << ref.replicate<1, NHORIZON>(); // TODO: check if this is really NHORIZON
};

Controller::~Controller(){};

float Controller::controlLaw(BLA::Matrix<systemOrder,1> currentState){
  
  if(this->controllerType == ControllerType::MPC) {  
    // 1. Update measurement
    this->x << currentState(0), currentState(1); // current measurement
    tiny_set_x0(this->solver, this->x);
    
    // 2. Update reference
    // work->Xref = Xref_total.block<systemOrder, NHORIZON>(0, k);

    // 3. Reset dual variables if needed
    this->work->y = tiny_MatrixNuNhm1::Zero();
    this->work->g = tiny_MatrixNxNh::Zero();
    
    // 4. Solve MPC problem
    tiny_solve(this->solver);

    return this->work->u.col(0)[0];
  }

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

  BLA::Matrix<2,1> states;

  while(1){
    //Wait for states 
    if(xQueueReceive(*(this->statesQueue), &states, portMAX_DELAY)){
      EventsHandler::sendEvent(this->taskName, EventsHandler::EventType::START);
      long timeBeforeMpc = micros();
      
      TickType_t xLastWakeTime = xTaskGetTickCount();
      float controlInput = this->controlLaw(states);
      // vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(this->_interval) );
      if(this->taskName == "xControllerTask")
        EventsHandler::vTaskDelayUntilWithEvent(this->taskName, &xLastWakeTime, pdMS_TO_TICKS(this->_interval)); // TODO: check if this is needed for the Y axis
      
      //Convert to degree and saturate
      float uDegree = rad2deg(controlInput);
      saturate(&uDegree, -25, 25);
      float angle = (uDegree) + this->servo->getOffset();
      this->servo->moveServo(angle);

      //Send to touchScreenQueue -> send
      xQueueSend(*(this->inputQueue), &controlInput, portMAX_DELAY);
      // xEvent
      xEventGroupSetBits(*(this->xEventGroup), this->inputEventBit);

      EventsHandler::MpcPayload payload;
      payload.u = uDegree;
      payload.cost = 0; // TODO: change this value after MPC insertion
      payload.computationTime = float(micros()-timeBeforeMpc)/1000.0; // conversion from micros to millis

      EventsHandler::sendEvent(this->taskName, EventsHandler::EventType::END, nullptr, &payload);
    }
  }
}

void Controller::start(){
    xTaskCreatePinnedToCore(
    controllerTask, 
    this->taskName, 
    6000,
    this, 
    tskIDLE_PRIORITY+2, 
    NULL,
    0
  );
}

void Controller::setSamplingTime(int milliseconds){
  this->_interval = milliseconds >> 1;
};
