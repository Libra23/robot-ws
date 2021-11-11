#ifndef QUEUE_H
#define QUEUE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

template<typename T>
class Queue {
    public:
    Queue();
    void Write(const T& data);
    void Read(T& data);
    private:
    QueueHandle_t x_mutex_;
    T memory_;
};

template <typename T>
Queue<T>::Queue() {
    x_mutex_ = xSemaphoreCreateMutex();
    memory_ = T();
}

template <typename T>
void Queue<T>::Write(const T& data) {
    if (xSemaphoreTake(x_mutex_, portMAX_DELAY) == pdTRUE) {
        memory_ = data;
        xSemaphoreGive(x_mutex_);
    }
}

template <typename T>
void Queue<T>::Read(T& data) {
    if (xSemaphoreTake(x_mutex_, portMAX_DELAY) == pdTRUE) {
        data = memory_;
        xSemaphoreGive(x_mutex_);
    }
}

#endif