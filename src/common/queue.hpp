#ifndef X_QUEUE_H
#define X_QUEUE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class Queue {
    public:
    Queue(uint32_t length, uint32_t size);
    void Send(const void* data);
    void Receive(void* data);
    uint32_t NumOfItems();
    void ReceivePeek(void* data);
    private:
    QueueHandle_t x_queue_;
};

#endif