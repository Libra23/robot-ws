#ifndef MEMORY_H
#define MEMORY_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

template<typename T>
class ShareMemory {
    public:
    ShareMemory();
    void Write(const T& data);
    void Read(T& data);
    private:
    SemaphoreHandle_t x_mutex_;
    T memory_;
};

template <typename T>
ShareMemory<T>::ShareMemory() {
    x_mutex_ = xSemaphoreCreateMutex();
    memory_ = T();
}

template <typename T>
void ShareMemory<T>::Write(const T& data) {
    if (xSemaphoreTake(x_mutex_, portMAX_DELAY) == pdTRUE) {
        memory_ = data;
        xSemaphoreGive(x_mutex_);
    }
}

template <typename T>
void ShareMemory<T>::Read(T& data) {
    if (xSemaphoreTake(x_mutex_, portMAX_DELAY) == pdTRUE) {
        data = memory_;
        xSemaphoreGive(x_mutex_);
    }
}

#endif