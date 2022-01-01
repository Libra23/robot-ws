#ifndef X_MEMORY_H
#define X_MEMORY_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

template<typename T>
class ShareMemory {
    public:
    ShareMemory();
    bool Write(const T& data);
    bool Read(T& data);
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
bool ShareMemory<T>::Write(const T& data) {
    if (xSemaphoreTake(x_mutex_, portMAX_DELAY) == pdTRUE) {
        memory_ = data;
        xSemaphoreGive(x_mutex_);
        return true;
    } else {
        return false;
    }
}

template <typename T>
bool ShareMemory<T>::Read(T& data) {
    if (xSemaphoreTake(x_mutex_, portMAX_DELAY) == pdTRUE) {
        data = memory_;
        xSemaphoreGive(x_mutex_);
        return true;
    } else {
        return false;
    }
}

#endif