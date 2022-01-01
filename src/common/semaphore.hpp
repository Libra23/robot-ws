#ifndef X_SEMAPHORE_H
#define X_SEMAPHORE_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class Semaphore {
    public:
    Semaphore();
    bool Take();
    bool Give();
    uint32_t GetCount();
    private:
    SemaphoreHandle_t x_semaphore_;
};

#endif