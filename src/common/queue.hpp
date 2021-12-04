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

Queue::Queue(uint32_t length, uint32_t size) {
    x_queue_ = xQueueCreate(length, size);
}

void Queue::Send(const void* data) {
    if (xQueueSendToBack(x_queue_, data, portMAX_DELAY) != pdTRUE) {
    }
}

void Queue::Receive(void* data) {
    if (xQueueReceive(x_queue_, data, portMAX_DELAY) != pdTRUE) {
    }
}

uint32_t Queue::NumOfItems() {
    return uxQueueMessagesWaiting(x_queue_);
}

void Queue::ReceivePeek(void* data) {
    if (xQueuePeek(x_queue_, data, portMAX_DELAY) != pdTRUE) {
    }
}

#endif