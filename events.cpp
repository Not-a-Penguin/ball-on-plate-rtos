#include "events.h"

#define SEND_AS_STRING // uncomment to enable human readable events
// #define DEBUG_EVENT

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

void sendEvent(char* taskName, EventType event, PayloadType payloadType, MpcPayload* payloadMpc, SendTaskPayload* payloadSendTask) {
    EventsMessage message;
    strncpy(&message.taskName[0], taskName, 32);
    message.type = event;
    message.time = millis(); // xTaskGetTickCount() * portTICK_PERIOD_MS;
    message.payloadType = payloadType;
    
    switch (payloadType)
    {
        case PayloadType::MPC:
            message.payload.mpc = *payloadMpc;
            break;
        
        case PayloadType::SEND_TASK:
            message.payload.sendTask = *payloadSendTask;
            break;
    }

    if(xQueueSend(eventsQueue, (void*) &message, 0) != pdPASS) {
        /* Failed to post the message, do not wait any tick */
        failedMessageCounter++;
    }
}

void sendEventsToSerial(void* parameters) {
    int delayInMillis = int(parameters);
    
    EventsMessage message;
    int counter = 0;
    #ifdef DEBUG_EVENT
    volatile long before = 0;
    #endif

    for(;;) {
        
        if(!xQueueReceive(eventsQueue, &message, ( TickType_t ) 0)) {
            vTaskDelay(pdMS_TO_TICKS(delayInMillis >> 1));
            continue;
        }

        sendEvent(sendEventsTaskName, EventType::START);
        counter = 0;

        #ifdef DEBUG_EVENT
        before = micros();
        #endif
        do {
            #ifdef SEND_AS_STRING
            // char type[10];
            // switch (message.type) {
            //   case EventType::START:
            //     strcpy(&type[0], "START");
            //     break;
            //   case EventType::BLOCKED:
            //     strcpy(&type[0], "BLOCKED");
            //     break;
            //   case EventType::RESUMED:
            //     strcpy(&type[0], "RESUMED");
            //     break;
            //   case EventType::END:
            //     strcpy(&type[0], "END");
            //     break;
            // }

            switch (message.payloadType)
            {
            case PayloadType::NONE:
                Serial.printf("%s,%s,%ld,,,,,,\n", 
                    message.taskName, 
                    EventNames[message.type], 
                    message.time
                );
                break;

            case PayloadType::MPC:
                Serial.printf("%s,%s,%ld,%f,%f,%f,%ld,,\n", 
                    message.taskName, 
                    EventNames[message.type], 
                    message.time,
                    message.payload.mpc.x1,
                    message.payload.mpc.x2,
                    message.payload.mpc.u,
                    message.payload.mpc.computationTime
                );
                break;
            
            case PayloadType::SEND_TASK:
                Serial.printf("%s,%s,%ld,,,,,%d,%d\n", 
                    message.taskName, 
                    EventNames[message.type], 
                    message.time,
                    message.payload.sendTask.sentMessages,
                    message.payload.sendTask.failedMessages
                );
                break;
            
            }
            #endif

            #ifndef SEND_AS_STRING
            Serial.write((byte*)&message, sizeof(EventsMessage));
            Serial.write('\r');
            Serial.write('\n');
            #endif
        
            counter++;
        } while(xQueueReceive(eventsQueue, &message, ( TickType_t ) 0));

        SendTaskPayload payload;
        payload.sentMessages = counter;
        payload.failedMessages = failedMessageCounter;
        sendEvent(sendEventsTaskName, EventType::END, PayloadType::SEND_TASK, nullptr, &payload);

        #ifdef DEBUG_EVENT
        Serial.print("time to sent: ");
        Serial.print(micros()-before);
        Serial.print(" counter: ");
        Serial.print(counter);
        Serial.print(" error: ");
        Serial.println(failedMessageCounter);
        #endif

        vTaskDelay(pdMS_TO_TICKS(delayInMillis));
    }
}

}
