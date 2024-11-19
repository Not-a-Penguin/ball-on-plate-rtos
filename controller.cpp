#include "freertos/portmacro.h"
#include "controller.h"
#include "servoControl.h"

Controller::Controller(BLA::Matrix<1, systemOrder> gains, QueueHandle_t *statesQueue, QueueHandle_t *inputQueue,
                       EventGroupHandle_t *xEventGroup, EventBits_t inputEventBit, char* taskName){
  this->controllerType = ControllerType::STATE_FEEDBACK;
  this->K = gains;
  this->statesQueue = statesQueue;
  this->inputQueue = inputQueue;
  this->inputEventBit = inputEventBit;
  this->taskName = taskName;
  this->xEventGroup = xEventGroup;
};

Controller::Controller(BLA::Matrix<systemOrder, systemOrder> A, BLA::Matrix<systemOrder,1> B, 
                       tinytype Q_data[systemOrder], tinytype R_data[systemOrder],
                       QueueHandle_t *statesQueue, QueueHandle_t *inputQueue,
                       EventGroupHandle_t *xEventGroup, EventBits_t inputEventBit, char* taskName){
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

  int status = tiny_setup(&this->solver,
                          Adyn, Bdyn, Q.asDiagonal(), R.asDiagonal(),
                          rho_value, systemOrder, 1, NHORIZON,
                          x_min, x_max, u_min, u_max, 1);

  // Update whichever settings we'd like
  this->solver->settings->max_iter = MAX_ITERATIONS;

  // Alias this->solver->work for brevity
  work = this->solver->work;

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
      
      float controlInput = this->controlLaw(states);

      //Send to touchScreenQueue -> send
      xQueueSend(*(this->inputQueue), &controlInput, portMAX_DELAY);
      // xEvent
      xEventGroupSetBits(*(this->xEventGroup), this->inputEventBit);

      EventsHandler::MpcPayload payload;
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
