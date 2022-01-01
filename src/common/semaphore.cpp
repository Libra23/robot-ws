#include "semaphore.hpp"

Semaphore::Semaphore() {
    x_semaphore_ = xSemaphoreCreateBinary();
}

bool Semaphore::Take() {
    if (xSemaphoreTake(x_semaphore_, portMAX_DELAY) == pdTRUE) {
        return true;
    } else {
        return false;
    }
}

bool Semaphore::Give() {
    if (xSemaphoreGive(x_semaphore_) == pdTRUE) {
        return true;
    } else {
        return false;
    }
}

uint32_t Semaphore::GetCount() {
    return static_cast<uint32_t>(uxSemaphoreGetCount(x_semaphore_));
}