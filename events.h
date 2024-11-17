#ifndef EVENTS_H
#define EVENTS_H

#include <Arduino.h>

// TODO: handle controller ferformance data
// TODO: handle filter and controller information

extern QueueHandle_t eventsQueue;
extern long failedMessageCounter;

namespace EventsHandler{

enum EventType {
    START,
    END
};

enum PayloadType {
    NONE,
    FILTER,
    MPC
};

struct FilterPayload {
    float original;
    float filtered;
};

struct MpcPayload {
    float u;
    float cost;
    unsigned long computationTime; // ms
};

struct EventsMessage {
    char *taskName;
    EventType type;
    unsigned long time; // ms
    PayloadType payloadType;
    union {
        FilterPayload filter;
        MpcPayload mpc;
    } payload;
    long failedMessages; 
};

void sendEvent(char* taskName, EventType event, FilterPayload* filterPayload = nullptr, MpcPayload* mpcPayload = nullptr);
void sendEventsToSerial(void* parameters);

}

#endif
