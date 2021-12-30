#ifndef THREAD_H
#define THREAD_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>

class Thread {
    public:
    Thread();
    void Start(void (*function)(void*), const std::string& name, uint8_t priority, uint32_t stack_size, void* arg, uint8_t core);
    uint32_t GetStackMargin();
    private:
    xTaskHandle task_handle_;
};

class ThreadClock {
    public:
    ThreadClock(uint32_t ms);
    void Wait();
    void Reset();
    private:
    portTickType previous_wake_time_;
    uint32_t wait_ms_;
};

void delay(uint32_t ms);
int64_t get_time_us();
int64_t get_time_ms();

#endif