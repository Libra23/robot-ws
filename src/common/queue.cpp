#include "queue.hpp"

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