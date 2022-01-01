#ifndef X_QUEUE_H
#define X_QUEUE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class Queue {
    public:
    Queue(uint32_t length, uint32_t size);
    bool Send(const void* data);
    bool Receive(void* data);
    uint32_t NumOfItems();
    bool ReceivePeek(void* data);
    private:
    QueueHandle_t x_queue_;
};

#endif