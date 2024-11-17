#ifndef TOUCHSCREEN_H
#define TOUCHSCREEN_H

#include "servoControl.h"
#include "controller.h"
#include "events.h"
/*
 * Library for interfacing with a 5-wire resistive touchScreen. 
 */

//definition of struct where the current values of x and y will be stored:
struct screenCoordinates{
  float x = 0;
  float y = 0;  
};

struct screenCoordinatesCm{
  float xCm = 0;
  float yCm = 0;
};

struct inputAndOutput{
  float input;
  float output;
};

class TouchScreen{

  private:

    //Analog read of specified axis. Also takes care of setting the correct pins HIGH or LOW
    int readCoordinate(String coordinate);
    void alternateReading();
    void setPins();

    //variables for calculating when the screen should be read
    int _interval= 30; //in milliseconds -> can be set in setScreenDelay()
    unsigned long _previousMillis = 0;

    screenCoordinates _currentScreenCoordinates;
    screenCoordinatesCm _currentScreenCoordiantesCm;
  
    //Signals when a new reading of the coordinates has been made
    bool _newReading;

    //Variables for the digital pins corresponding to the four corners of the display.
    int _upperLeftPin;
    int _upperRightPin;
    
    int _lowerLeftPin;
    int _lowerRightPin;

    //ADC
    //The middle pin of the display which corresponds to the analog value of the voltage divider when there
    //is a voltage gradient in the screen
    int _sensorPin;

    char* taskName;

  public:

    TouchScreen(
      int upperLeft, 
      int upperRight, 
      int lowerLeft, 
      int lowerRight, 
      int sensorPin,
      QueueHandle_t *xInputQueue,
      QueueHandle_t *yInputQueue,
      QueueHandle_t *xInputOutputQueue,
      QueueHandle_t *yInputOutputQueue,
      EventGroupHandle_t *inputEventGroup,
      EventBits_t xIputEventBit,
      EventBits_t yInputEventBit,
      char* taskName
    );
    
    ~TouchScreen();

    screenCoordinates getCoordinates();
    screenCoordinatesCm getCoordinatesCm(float xValue, float yValue);
    void setSamplingTime(int milliseconds);
    bool screenUpdated();
    
    ServoControl servos;


    //RTOS

    //To get info
    QueueHandle_t *xControlInputQueue;
    QueueHandle_t *yControlInputQueue;

    QueueHandle_t *xInputOutputQueue;
    QueueHandle_t *yInputOutputQueue;

    //Event
    EventGroupHandle_t *controlInputEvent;
    EventBits_t xIputEventBit;
    EventBits_t yInputEventBit;

    static void tsTask(void* params);
    void run();
    void start();

};
#endif // !TOUCHSCREEN_H
