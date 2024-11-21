#include "events.h"

#define DEBUG_EVENT // uncomment to enable human readable events

namespace EventsHandler {

void vTaskDelayWithEvent(char* taskName, const TickType_t xTimeIncrement) {
    sendEvent(taskName, EventType::BLOCKED);
    vTaskDelay(xTimeIncrement);
    sendEvent(taskName, EventType::RESUMED);
}

void vTaskDelayUntilWithEvent(char* taskName, TickType_t *pxPreviousWakeTime, const TickType_t xTimeIncrement ) {
    sendEvent(taskName, EventType::BLOCKED);
    vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement);
    sendEvent(taskName, EventType::RESUMED);
}

void sendEvent(char* taskName, EventType event, FilterPayload* filterPayload, MpcPayload* mpcPayload) {
    EventsMessage message;
    strncpy(&message.taskName[0], taskName, 32);
    message.type = event;
    message.time = millis(); // xTaskGetTickCount() * portTICK_PERIOD_MS;
    message.payloadType = PayloadType::NONE;
    message.failedMessages = failedMessageCounter;
    
    if(filterPayload) {
        message.payloadType = PayloadType::FILTER;
        message.payload.filter = *filterPayload;
    }
    else if(mpcPayload) {
        message.payloadType = PayloadType::MPC;
        message.payload.mpc = *mpcPayload;
    }

    if(xQueueSend(eventsQueue, (void*) &message, 0) != pdPASS) {
        /* Failed to post the message, do not wait any tick */
        failedMessageCounter++;
    }
}

void sendEventsToSerial(void* parameters) {
    int delayInMillis = int(parameters);
    
    EventsMessage message;
    #ifdef DEBUG_EVENT
    volatile long before = 0;
    bool sent = false;
    int counter = 0;
    #endif
    for(;;) {
        
        if(xQueueReceive(eventsQueue, &message, ( TickType_t ) 0)) {
            #ifdef DEBUG_EVENT
            before = micros();
            counter = 0;
            #endif
            do {
                #ifdef DEBUG_EVENT
                sent = true;
//                Serial.print("taks name/type/time: "); // temporary
//                Serial.print(message.taskName); // temporary
//                Serial.print("/"); // temporary
                char type[10];
                switch (message.type) {
                  case EventType::START:
                    strcpy(&type[0], "START");
                    break;
                  case EventType::BLOCKED:
                    strcpy(&type[0], "BLOCKED");
                    break;
                  case EventType::RESUMED:
                    strcpy(&type[0], "RESUMED");
                    break;
                  case EventType::END:
                    strcpy(&type[0], "END");
                    break;
                }
//                Serial.printf(message.type == EventType::START? "START":"END"); // temporary
//                Serial.print("/");
//                Serial.println(message.time);

                Serial.printf("taks name/type/time: %s/%s/%ld\n", message.taskName, type, message.time);
                
                counter++;
                #endif

                #ifndef DEBUG_EVENT
                Serial.write((byte*)&message, sizeof(EventsMessage));
                Serial.write('\r');
                Serial.write('\n');
                #endif
            
            } while(xQueueReceive(eventsQueue, &message, ( TickType_t ) 0));
        }

        #ifdef DEBUG_EVENT
        if(sent){
            Serial.print("time to sent: ");
            Serial.print(micros()-before);
            Serial.print(" counter: ");
            Serial.print(counter);
            Serial.print(" error: ");
            Serial.println(failedMessageCounter);
            sent = false;
        }
        #endif

        vTaskDelay(pdMS_TO_TICKS(delayInMillis));
    }
}

}
