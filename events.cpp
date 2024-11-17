#include "events.h"

namespace EventsHandler {

void sendEvent(char* taskName, EventType event, FilterPayload* filterPayload, MpcPayload* mpcPayload) {
    EventsMessage message;
    message.taskName = taskName;
    message.type = event;
    message.time = xTaskGetTickCount() * (1000/configTICK_RATE_HZ);
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
    EventsMessage message;
    int delayInMillis = int(parameters);

    long before = 0;
    bool sent = false;
    for(;;) {
        before = micros();
        while(xQueueReceive(eventsQueue, &message, 0)) {
            sent = true;
            Serial.print("taks name: "); // temporary
            Serial.println(message.taskName); // temporary
            Serial.write((byte*)&message, sizeof(EventsMessage));
            Serial.println();
        }
        if(sent){
            Serial.print("time to sent: ");
            Serial.println(micros()-before);
            sent = false;
        }
        vTaskDelay(pdMS_TO_TICKS(delayInMillis));
    }
}

}
