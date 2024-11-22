#ifndef EVENTS_H
#define EVENTS_H

#include <Arduino.h>

// TODO: handle controller ferformance data
// TODO: handle filter and controller information

extern QueueHandle_t eventsQueue;
extern uint16_t failedMessageCounter;
extern char sendEventsTaskName[15];

inline char EventNames[4][10] = {
    "START",
    "BLOCKED",
    "RESUMED",
    "END"
};

namespace EventsHandler{

enum EventType {
    START,
    BLOCKED,
    RESUMED,
    END
};

enum PayloadType {
    NONE,
    MPC,
    SEND_TASK
};

struct MpcPayload {
    float x1;
    float x2;
    float u;
    float cost;
    uint32_t computationTime; // ms
};

struct SendTaskPayload {
    int sentMessages;
    int failedMessages;
};

struct EventsMessage {
    char taskName[32];
    EventType type;
    uint32_t time; // ms
    PayloadType payloadType;
    union {
        MpcPayload mpc;
        SendTaskPayload sendTask;
    } payload;
};

void vTaskDelayWithEvent(char* taskName, const TickType_t xTimeIncrement);
void vTaskDelayUntilWithEvent(char* TaskName, TickType_t *pxPreviousWakeTime, const TickType_t xTimeIncrement);
void sendEvent(char* taskName, EventType event, PayloadType payloadType = PayloadType::NONE, MpcPayload* payloadMpc = nullptr, SendTaskPayload* payloadSendTask = nullptr);
void sendEventsToSerial(void* parameters);

}

#endif
