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

bool Semaphore::Take(uint32_t wait_ms) {
    if (xSemaphoreTake(x_semaphore_, wait_ms / portTICK_RATE_MS) == pdTRUE) {
        return true;
    } else {
        return false;
    }
}

void Semaphore::Give() {
    if (xSemaphoreGive(x_semaphore_) != pdTRUE) {
        // error
    }
}

int32_t Semaphore::GetCount() {
    return uxSemaphoreGetCount(x_semaphore_);
}