#ifndef EVENTS_H
#define EVENTS_H

#include <Arduino.h>

// TODO: handle controller ferformance data
// TODO: handle filter and controller information

extern QueueHandle_t eventsQueue;
extern uint16_t failedMessageCounter;

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
    uint32_t computationTime; // ms
};

struct EventsMessage {
    char *taskName;
    EventType type;
    uint32_t time; // ms
    PayloadType payloadType;
    union {
        FilterPayload filter;
        MpcPayload mpc;
    } payload;
    uint16_t failedMessages; 
};

void sendEvent(char* taskName, EventType event, FilterPayload* filterPayload = nullptr, MpcPayload* mpcPayload = nullptr);
void sendEventsToSerial(void* parameters);

}

#endif
